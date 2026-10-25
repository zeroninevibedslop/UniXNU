/* SPDX-License-Identifier: MIT */
/*
 * IOKit driver for Qualcomm Adreno 618 (SM7150 / Sweet).
 * Brings up the GPU power domains, clocks, GMU firmware, SMMU 4 KiB IOVA mapping,
 * CP command processor, executes CP memory writeback smoke test, and publishes
 * the AGXAccelerator service with sandbox grant properties.
 */
#include <IOKit/IODeviceMemory.h>
#include <string.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <pexpert/pexpert.h>
#include "NativeAdrenoBoard.h"
#include "NativeAdrenoFirmware.h"
#include "NativeAdrenoGmuService.h"
#include "NativeAdrenoSmmuService.h"
#include "NativeAdrenoGpuContext.h"
#include "NativeAdrenoIrq.h"
#include "NativeAdrenoKernelTime.h"
#include "NativeAdrenoCpInit.h"
#include "NativeAdrenoRing.h"
#include "NativeAdrenoSubmit.h"
#include "NativeAdrenoUserClient.h"
#include "NativeAdrenoA618HwInit.h"

extern "C" const uint8_t native_adreno_firmware_image[] __attribute__((weak));
extern "C" const uint64_t native_adreno_firmware_image_size __attribute__((weak));
extern "C" const uint8_t native_adreno_sqe_firmware_image[] __attribute__((weak));
extern "C" const uint64_t native_adreno_sqe_firmware_image_size __attribute__((weak));

extern "C" IOService *native_adreno_accelerator_allocate(void) __attribute__((weak));
extern "C" unsigned native_adreno_accelerator_instance_size(void) __attribute__((weak));
extern "C" IOReturn native_adreno_publish(NativeAdrenoVm *vm,NativeAdrenoSubmit *submit,NativeAdrenoRing *ring,
    const NativeAdrenoRingClock *clock,const NativeAdrenoTimer *timer,const NativeAdrenoAbiDeviceInfo *info,
    uint32_t input_bits,uint32_t output_bits);

extern "C" { volatile uint64_t native_adreno_fence_phys = 0; }
extern "C" volatile uint32_t native_adreno_pool_mib;

class NativeAdrenoGpuClient;
static IOUserClient *createGpuClient(task_t,void *,UInt32,OSDictionary *,NativeAdrenoUserClient *);

struct NativeAdrenoSmmuWindow {
    volatile uint8_t *base;
    size_t bytes;
};

static int smmu_read32(void *cookie,size_t at,uint32_t *value)
{
    NativeAdrenoSmmuWindow *window = static_cast<NativeAdrenoSmmuWindow *>(cookie);
    if (!window || !value || !window->base || (at & 3) || at + 4 > window->bytes) return 1;
    *value = *reinterpret_cast<volatile uint32_t *>(window->base + at);
    return 0;
}
static int smmu_read64(void *cookie,size_t at,uint64_t *value)
{
    NativeAdrenoSmmuWindow *window = static_cast<NativeAdrenoSmmuWindow *>(cookie);
    if (!window || !value || !window->base || (at & 7) || at + 8 > window->bytes) return 1;
    *value = *reinterpret_cast<volatile uint64_t *>(window->base + at);
    return 0;
}
static int smmu_write32(void *cookie,size_t at,uint32_t value)
{
    NativeAdrenoSmmuWindow *window = static_cast<NativeAdrenoSmmuWindow *>(cookie);
    if (!window || !window->base || (at & 3) || at + 4 > window->bytes) return 1;
    *reinterpret_cast<volatile uint32_t *>(window->base + at) = value;
    return 0;
}
static int smmu_write64(void *cookie,size_t at,uint64_t value)
{
    NativeAdrenoSmmuWindow *window = static_cast<NativeAdrenoSmmuWindow *>(cookie);
    if (!window || !window->base || (at & 7) || at + 8 > window->bytes) return 1;
    *reinterpret_cast<volatile uint64_t *>(window->base + at) = value;
    return 0;
}
static int smmu_barrier(void *cookie)
{
    if (!cookie) return 1;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return 0;
}

struct NativeAdrenoGpuWindow {
    volatile uint8_t *base;
    size_t bytes;
};

class NativeAdrenoGpuService : public IOService {
    OSDeclareDefaultStructors(NativeAdrenoGpuService)
    OSSharedPtr<IOMemoryMap> maps_[2];
    OSSharedPtr<IOMemoryMap> gcc_map_;
    NativeAdrenoMemory gmu_mem_;
    NativeAdrenoGpuContext gpu_ctx_;
    NativeAdrenoRing ring_;
    NativeAdrenoSubmit *execbuf_ = nullptr;
    NativeAdrenoUserClient *core_ = nullptr;
    IOService *accelerator_ = nullptr;
    bool core_ready_ = false, ready_ = false;

    struct RingCache { NativeAdrenoMemory *ring; NativeAdrenoMemory *fence; };
    RingCache ring_cache_{};

    static int gpu_read32(void *,uint32_t,uint32_t *);
    static int gpu_write32(void *,uint32_t,uint32_t);
    static int gpu_barrier(void *);
    static int ring_publish(void *,const volatile uint32_t *,size_t);
    static int fence_acquire(void *,volatile uint32_t *,size_t);
    static uint64_t ring_now(void *);

public:
    IOService *probe(IOService *provider,SInt32 *score) APPLE_KEXT_OVERRIDE;
    bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
    void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
    IOReturn newUserClient(task_t owningTask,void *securityID,UInt32 type,
        OSDictionary *properties,IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(NativeAdrenoGpuService,IOService)

static bool advertisesAdreno(IOService *provider)
{
    OSObject *property = provider ? provider->getProperty("compatible") : nullptr;
    OSData *data = OSDynamicCast(OSData, property);
    if (!data || !data->getBytesNoCopy() || !data->getLength()) return false;
    const char *text = static_cast<const char *>(data->getBytesNoCopy());
    const size_t length = data->getLength();
    static const char *needles[] = { "qcom,adreno", "qcom,kgsl-3d0", "qcom,kgsl-3d" };
    for (const char *needle : needles) {
        const size_t needle_len = strlen(needle);
        for (size_t at = 0; at + needle_len <= length; ++at) {
            if (strncmp(text + at, needle, needle_len) == 0) return true;
        }
    }
    return false;
}

int NativeAdrenoGpuService::gpu_read32(void *cookie,uint32_t reg,uint32_t *value)
{
    NativeAdrenoGpuWindow *window = static_cast<NativeAdrenoGpuWindow *>(cookie);
    const size_t at = size_t(reg) * 4;
    if (!window || !value || !window->base || at + 4 > window->bytes) return 1;
    *value = *reinterpret_cast<volatile uint32_t *>(window->base + at);
    return 0;
}
int NativeAdrenoGpuService::gpu_write32(void *cookie,uint32_t reg,uint32_t value)
{
    NativeAdrenoGpuWindow *window = static_cast<NativeAdrenoGpuWindow *>(cookie);
    const size_t at = size_t(reg) * 4;
    if (!window || !window->base || at + 4 > window->bytes) return 1;
    *reinterpret_cast<volatile uint32_t *>(window->base + at) = value;
    return 0;
}
int NativeAdrenoGpuService::gpu_barrier(void *cookie)
{
    if (!cookie) return 1;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return 0;
}
int NativeAdrenoGpuService::ring_publish(void *cookie,const volatile uint32_t *words,size_t bytes)
{
    (void)words;
    NativeAdrenoGpuService::RingCache *cache = static_cast<RingCache *>(cookie);
    return cache && cache->ring && cache->ring->publishForDevice(0,bytes,false) == kIOReturnSuccess ? 0 : 1;
}
int NativeAdrenoGpuService::fence_acquire(void *cookie,volatile uint32_t *words,size_t bytes)
{
    (void)words;
    NativeAdrenoGpuService::RingCache *cache = static_cast<RingCache *>(cookie);
    return cache && cache->fence && cache->fence->acquireDeviceWrites(0,bytes) == kIOReturnSuccess ? 0 : 1;
}
uint64_t NativeAdrenoGpuService::ring_now(void *)
{
    uint64_t microseconds = 0;
    NativeAdrenoKernelTime::now_us(nullptr,&microseconds);
    return microseconds;
}

extern "C" uint64_t (*native_adreno_vm_now_us)(void);
static uint64_t vm_clock_us(void) {
    uint64_t microseconds = 0;
    NativeAdrenoKernelTime::now_us(nullptr,&microseconds);
    return microseconds;
}

IOService *NativeAdrenoGpuService::probe(IOService *provider,SInt32 *score)
{
    native_adreno_vm_now_us = vm_clock_us;
    if (!advertisesAdreno(provider)) return nullptr;
    if (score) *score = 1000;
    return this;
}

bool NativeAdrenoGpuService::start(IOService *provider)
{
    if (!IOService::start(provider)) return false;

    OSData *reg = OSDynamicCast(OSData,provider->getProperty("reg"));
    OSData *address = OSDynamicCast(OSData,provider->getProperty("#address-cells"));
    OSData *size = OSDynamicCast(OSData,provider->getProperty("#size-cells"));
    NativeAdrenoWindow windows[NativeAdrenoMaxWindows]{};
    size_t count = 0;
    uint32_t address_cells = 2, size_cells = 2;
    if (address && address->getLength() >= 4)
        address_cells = *static_cast<const uint32_t *>(address->getBytesNoCopy());
    if (size && size->getLength() >= 4)
        size_cells = *static_cast<const uint32_t *>(size->getBytesNoCopy());
    if (!reg || NativeAdrenoBoard::parseReg(
        static_cast<const uint8_t *>(reg->getBytesNoCopy()), reg->getLength(),
        address_cells, size_cells, windows, NativeAdrenoMaxWindows, &count) != kIOReturnSuccess) {
        IOLog("NativeAdreno: provider reg windows are not usable\n");
        IOService::stop(provider);
        return false;
    }
    if (count < 2) {
        IOLog("NativeAdreno: GPU node needs the register and CX windows\n");
        IOService::stop(provider);
        return false;
    }

    for (unsigned index = 0; index < 2; ++index) {
        IODeviceMemory *memory = getDeviceMemoryWithIndex(index);
        if (!memory) {
            IOLog("NativeAdreno: provider window %u has no device memory\n", index);
            stop(provider);
            return false;
        }
        maps_[index] = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
        if (!maps_[index]) {
            IOLog("NativeAdreno: GPU window %u did not map\n", index);
            stop(provider);
            return false;
        }
    }
    IOLog("NativeAdreno: %lu windows, GPU window at 0x%llx/0x%llx, CX at 0x%llx/0x%llx mapped\n",
        static_cast<unsigned long>(count),
        static_cast<unsigned long long>(windows[0].base), static_cast<unsigned long long>(windows[0].bytes),
        static_cast<unsigned long long>(windows[1].base), static_cast<unsigned long long>(windows[1].bytes));

    /* Map GCC window (0x00100000, size 0x90000) for clock tree gating */
    IODeviceMemory *gcc_mem = IODeviceMemory::withRange(0x00100000, 0x90000);
    if (!gcc_mem) {
        IOLog("NativeAdreno: GCC reservation failed\n");
        stop(provider);
        return false;
    }
    gcc_map_ = OSSharedPtr<IOMemoryMap>(gcc_mem->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
    gcc_mem->release();
    if (!gcc_map_) {
        IOLog("NativeAdreno: GCC mapping failed\n");
        stop(provider);
        return false;
    }

    /* Locate NativeAdrenoGmuService */
    OSDictionary *matching = IOService::serviceMatching("NativeAdrenoGmuService");
    IOService *gmu_svc = matching ? IOService::copyMatchingService(matching) : nullptr;
    if (matching) matching->release();
    if (!gmu_svc) {
        IOLog("NativeAdreno: NativeAdrenoGmuService not found\n");
        stop(provider);
        return false;
    }
    NativeAdrenoGmuService *gmu_node = OSDynamicCast(NativeAdrenoGmuService, gmu_svc);

    /* Locate NativeAdrenoSmmuService */
    OSDictionary *smmu_match = IOService::serviceMatching("NativeAdrenoSmmuService");
    IOService *smmu_svc = smmu_match ? IOService::copyMatchingService(smmu_match) : nullptr;
    if (smmu_match) smmu_match->release();
    if (!smmu_svc) {
        IOLog("NativeAdreno: NativeAdrenoSmmuService not found\n");
        gmu_svc->release();
        stop(provider);
        return false;
    }
    NativeAdrenoSmmuService *smmu_node = OSDynamicCast(NativeAdrenoSmmuService, smmu_svc);
    volatile uint8_t *smmu_base = smmu_node ? smmu_node->windowAddress(0) : nullptr;
    size_t smmu_bytes = smmu_node ? static_cast<size_t>(smmu_node->windowBytes(0)) : 0;
    if (!smmu_base || !smmu_bytes) {
        IOLog("NativeAdreno: SMMU window not available\n");
        smmu_svc->release();
        gmu_svc->release();
        stop(provider);
        return false;
    }

    /* Pool size boot argument */
    {
        uint32_t mib = 512;
        PE_parse_boot_argn("sheng-gpu-pool", &mib, sizeof(mib));
        native_adreno_pool_mib = mib;
    }

    /* GMU buffer allocation (56 KiB = 14 contiguous pages) */
    IOReturn gmu_alloc_ret = gmu_mem_.allocate(0xe000, 36, true, 0xe000);
    if (gmu_alloc_ret != kIOReturnSuccess || !gmu_mem_.spanCount()) {
        IOLog("NativeAdreno: failed to allocate GMU buffers (0x%x)\n", gmu_alloc_ret);
        smmu_svc->release();
        gmu_svc->release();
        stop(provider);
        return false;
    }
    uint64_t gmu_buf_pa = gmu_mem_.spans()[0].physical;
    void *gmu_buf_kva = gmu_mem_.mapping();

    volatile uint8_t *gcc_base = reinterpret_cast<volatile uint8_t *>(gcc_map_->getVirtualAddress());
    volatile uint8_t *cx_base = reinterpret_cast<volatile uint8_t *>(maps_[1]->getVirtualAddress());

    /* Execute A618 Cold Start on GMU */
    IOReturn gmu_start_ret = gmu_node->coldStartA618(gcc_base, cx_base, smmu_base, gmu_buf_pa, gmu_buf_kva);
    if (gmu_start_ret != kIOReturnSuccess) {
        IOLog("NativeAdreno: GMU cold start failed (0x%x)\n", gmu_start_ret);
        smmu_svc->release();
        gmu_svc->release();
        stop(provider);
        return false;
    }

    /* Bring up GPU Context Bank 0 for Stream 0 (gfx3d_user) via ArmSmmu4K */
    NativeAdrenoSmmuWindow smmu_win{ smmu_base, smmu_bytes };
    ArmSmmu4KIo smmu_io{ &smmu_win, smmu_bytes, smmu_read32, smmu_read64, smmu_write32, smmu_write64, smmu_barrier };
    const unsigned gpu_slots[2] = { 0, 0 };
    const uint16_t gpu_sids[2] = { 0, 0 };
    NativeAdrenoGmuContextReport ctx_report{};
    IOReturn ctx_ret = gpu_ctx_.bringUp(&smmu_io, 0, 1, gpu_slots, gpu_sids, nullptr, &ctx_report);
    if (ctx_ret != kIOReturnSuccess || !gpu_ctx_.ready()) {
        IOLog("NativeAdreno: GPU SMMU context bringUp failed (0x%x, step %u)\n", ctx_ret, ctx_report.step);
        smmu_svc->release();
        gmu_svc->release();
        stop(provider);
        return false;
    }
    IOLog("NativeAdreno: GPU SMMU context ready (Bank 0, SID 0, TTBR0=0x%llx)\n",
        static_cast<unsigned long long>(ctx_report.ttbr0));

    /* Load SQE microcode */
    if (reinterpret_cast<uintptr_t>(native_adreno_sqe_firmware_image) && native_adreno_sqe_firmware_image_size) {
        const size_t copy_bytes = static_cast<size_t>(native_adreno_sqe_firmware_image_size);
        memcpy(const_cast<NativeAdrenoMemory &>(gpu_ctx_.buffer(0)).mapping(),
               native_adreno_sqe_firmware_image, copy_bytes);
        const_cast<NativeAdrenoMemory &>(gpu_ctx_.buffer(0)).publishForDevice(0, GpuSqeBytes, false);
        IOLog("NativeAdreno: SQE microcode published (%lu bytes at IOVA 0x%llx)\n",
            static_cast<unsigned long>(copy_bytes), static_cast<unsigned long long>(GpuSqeIova));
    }

    /* Initialize Adreno 618 CP hardware */
    NativeAdrenoGpuWindow gpu_window = {
        reinterpret_cast<volatile uint8_t *>(maps_[0]->getVirtualAddress()),
        static_cast<size_t>(windows[0].bytes)
    };

    /* 1. Clear GBIF halt */
    gpu_write32(&gpu_window, A618_GBIF_HALT, 0);
    gpu_write32(&gpu_window, A618_RBBM_GBIF_HALT, 0);
    gpu_write32(&gpu_window, A618_RBBM_SECVID_TSB_CNTL, 0);
    gpu_write32(&gpu_window, A618_RBBM_SECVID_TSB_TRUSTED_BASE, 0);
    gpu_write32(&gpu_window, A618_RBBM_SECVID_TSB_TRUSTED_BASE + 1, 0);
    gpu_write32(&gpu_window, A618_RBBM_SECVID_TSB_TRUSTED_SIZE, 0);

    /* 2. Turn on 64-bit addressing on all blocks */
    static const uint32_t addr_mode_regs[] = {
        A618_CP_ADDR_MODE_CNTL, A618_VSC_ADDR_MODE_CNTL, A618_GRAS_ADDR_MODE_CNTL,
        A618_RB_ADDR_MODE_CNTL, A618_PC_ADDR_MODE_CNTL, A618_HLSQ_ADDR_MODE_CNTL,
        A618_VFD_ADDR_MODE_CNTL, A618_VPC_ADDR_MODE_CNTL, A618_UCHE_ADDR_MODE_CNTL,
        A618_SP_ADDR_MODE_CNTL, A618_TPL1_ADDR_MODE_CNTL, A618_RBBM_SECVID_TSB_ADDR_MODE_CNTL
    };
    for (uint32_t reg_off : addr_mode_regs) {
        gpu_write32(&gpu_window, reg_off, 1);
    }

    /* 3. Program HWCG clockgating table */
    for (unsigned i = 0; i < NATIVE_ADRENO_A618_HWCG_COUNT; i++) {
        gpu_write32(&gpu_window, NativeAdrenoA618Hwcg[i][0], NativeAdrenoA618Hwcg[i][1]);
    }
    gpu_write32(&gpu_window, A618_RBBM_CLOCK_CNTL, 0);

    /* 4. Configure UCHE */
    gpu_write32(&gpu_window, A618_UCHE_WRITE_RANGE_MAX, static_cast<uint32_t>(0x100000 + 0xfc0));
    gpu_write32(&gpu_window, A618_UCHE_WRITE_RANGE_MAX + 1, 0);
    gpu_write32(&gpu_window, A618_UCHE_TRAP_BASE, 0x100000);
    gpu_write32(&gpu_window, A618_UCHE_TRAP_BASE + 1, 0);
    gpu_write32(&gpu_window, A618_UCHE_WRITE_THRU_BASE, 0x100000);
    gpu_write32(&gpu_window, A618_UCHE_WRITE_THRU_BASE + 1, 0);
    gpu_write32(&gpu_window, A618_UCHE_GMEM_RANGE_MIN, 0x100000);
    gpu_write32(&gpu_window, A618_UCHE_GMEM_RANGE_MIN + 1, 0);
    gpu_write32(&gpu_window, A618_UCHE_GMEM_RANGE_MAX, static_cast<uint32_t>(0x100000 + A618_GMEM_SIZE - 1));
    gpu_write32(&gpu_window, A618_UCHE_GMEM_RANGE_MAX + 1, 0);
    gpu_write32(&gpu_window, A618_UCHE_FILTER_CNTL, 0x804);
    gpu_write32(&gpu_window, A618_UCHE_CACHE_WAYS, 0x4);
    gpu_write32(&gpu_window, A618_UCHE_CLIENT_PF, (1u << 7) | 1u);

    /* 5. Configure CP thresholds, pool size, APRIV */
    gpu_write32(&gpu_window, A618_CP_ROQ_THRESHOLDS_2, 0x010000c0);
    gpu_write32(&gpu_window, A618_CP_ROQ_THRESHOLDS_1, 0x8040362c);
    gpu_write32(&gpu_window, A618_CP_MEM_POOL_SIZE, 128);
    gpu_write32(&gpu_window, A618_PC_DBG_ECO_CNTL, A618_PRIM_FIFO_THRESHOLD);
    gpu_write32(&gpu_window, A618_CP_AHB_CNTL, 1);
    gpu_write32(&gpu_window, A618_RBBM_PERFCTR_CNTL, 1);
    gpu_write32(&gpu_window, A618_RBBM_PERFCTR_GPU_BUSY_MASKED, 0xffffffff);
    gpu_write32(&gpu_window, A618_RBBM_INTERFACE_HANG_INT_CNTL, (1u << 30) | 0x1fffff);
    gpu_write32(&gpu_window, A618_CP_APRIV_CNTL, (1u << 6) | (1u << 5) | (1u << 3) | (1u << 2) | (1u << 1));
    gpu_write32(&gpu_window, A618_RBBM_INT_0_MASK, 0x00000020);

    /* 6. Program CP Protect registers */
    for (unsigned i = 0; i < NATIVE_ADRENO_A618_PROTECT_COUNT; i++) {
        gpu_write32(&gpu_window, A618_CP_PROTECT + i, NativeAdrenoA618Protect[i][1]);
    }
    gpu_write32(&gpu_window, A618_CP_PROTECT_CNTL, 1);

    /* 7. Configure CP Ring registers */
    gpu_write32(&gpu_window, A618_CP_SQE_INSTR_BASE, static_cast<uint32_t>(GpuSqeIova));
    gpu_write32(&gpu_window, A618_CP_SQE_INSTR_BASE + 1, static_cast<uint32_t>(GpuSqeIova >> 32));
    gpu_write32(&gpu_window, A618_CP_RB_BASE, static_cast<uint32_t>(GpuRingIova));
    gpu_write32(&gpu_window, A618_CP_RB_BASE + 1, static_cast<uint32_t>(GpuRingIova >> 32));
    gpu_write32(&gpu_window, A618_CP_RB_CNTL, (1u << 11) | 13u); /* 32 KiB ring, BLKSZ 8 */
    gpu_write32(&gpu_window, A618_CP_RB_RPTR_ADDR, static_cast<uint32_t>(GpuMemptrsIova));
    gpu_write32(&gpu_window, A618_CP_RB_RPTR_ADDR + 1, static_cast<uint32_t>(GpuMemptrsIova >> 32));
    gpu_write32(&gpu_window, A618_CP_RB_RPTR, 0);
    gpu_write32(&gpu_window, A618_CP_RB_WPTR, 0);

    /* 8. Start SQE microcode */
    gpu_write32(&gpu_window, A618_CP_SQE_CNTL, 1);

    /* Attach NativeAdrenoRing */
    NativeAdrenoRingIo ring_io{ &gpu_window, gpu_write32, gpu_read32, gpu_barrier };
    ring_cache_ = {
        const_cast<NativeAdrenoMemory *>(&gpu_ctx_.buffer(1)),
        const_cast<NativeAdrenoMemory *>(&gpu_ctx_.buffer(2))
    };
    NativeAdrenoRingCache ring_cache{ &ring_cache_, ring_publish, fence_acquire };
    IOReturn ring_ret = ring_.attach(&ring_io, &ring_cache,
        const_cast<NativeAdrenoMemory &>(gpu_ctx_.buffer(1)).mapping(), GpuRingBytes, GpuRingIova,
        const_cast<NativeAdrenoMemory &>(gpu_ctx_.buffer(2)).mapping(), GpuPageBytes, GpuMemptrsIova);
    if (ring_ret != kIOReturnSuccess) {
        IOLog("NativeAdreno: ring attach failed (0x%x)\n", ring_ret);
        smmu_svc->release();
        gmu_svc->release();
        stop(provider);
        return false;
    }

    if (gpu_ctx_.buffer(2).spanCount()) {
        native_adreno_fence_phys = gpu_ctx_.buffer(2).spans()[0].physical;
    }

    /* 9. Run A6xx CP_ME_INIT */
    NativeAdrenoCpInit cp;
    uint32_t init_fence = 0;
    IOReturn cp_ret = cp.runA6xx(&ring_io, &ring_, GpuSqeIova, &init_fence);
    if (cp_ret != kIOReturnSuccess) {
        IOLog("NativeAdreno: A6xx CP_ME_INIT submission failed (0x%x)\n", cp_ret);
        smmu_svc->release();
        gmu_svc->release();
        stop(provider);
        return false;
    }
    NativeAdrenoRingClock clock{ nullptr, ring_now, nullptr };
    if (ring_.wait(init_fence, &clock, 2000, nullptr) != NativeAdrenoRingFenceComplete) {
        IOLog("NativeAdreno: A6xx CP_ME_INIT fence %u timed out\n", init_fence);
        smmu_svc->release();
        gmu_svc->release();
        stop(provider);
        return false;
    }
    IOLog("NativeAdreno: A6xx CP_ME_INIT completed successfully (fence %u)\n", init_fence);

    /* Switch out of secure mode */
    gpu_write32(&gpu_window, A618_RBBM_SECVID_TRUST_CNTL, 0);

    /* 10. Smoke Test: CP memory writeback */
    volatile uint32_t *scratch = reinterpret_cast<volatile uint32_t *>(
        const_cast<NativeAdrenoMemory &>(gpu_ctx_.buffer(4)).mapping());
    scratch[0] = 0x00000000;
    const_cast<NativeAdrenoMemory &>(gpu_ctx_.buffer(4)).publishForDevice(0, 4, false);

    /* Submit PM4 CP_MEM_WRITE (opcode 0x3d, count 3) */
    uint32_t smoke_pm4[4];
    AdrenoPM4Type7(0x3d, 3, &smoke_pm4[0]);
    smoke_pm4[1] = static_cast<uint32_t>(GpuScratchIova);
    smoke_pm4[2] = static_cast<uint32_t>(GpuScratchIova >> 32);
    smoke_pm4[3] = 0xad618001u;

    uint32_t smoke_fence = 0;
    IOReturn smoke_sub_ret = ring_.submit(smoke_pm4, 4, &smoke_fence);
    if (smoke_sub_ret == kIOReturnSuccess &&
        ring_.wait(smoke_fence, &clock, 2000, nullptr) == NativeAdrenoRingFenceComplete) {
        const_cast<NativeAdrenoMemory &>(gpu_ctx_.buffer(4)).acquireDeviceWrites(0, 4);
        uint32_t val = scratch[0];
        if (val == 0xad618001u) {
            IOLog("NativeAdreno: Adreno 618 CP MEMORY WRITEBACK SMOKE TEST PASSED! Pattern 0x%08x verified\n", val);
        } else {
            IOLog("NativeAdreno: smoke test scratch readback mismatch: 0x%08x (expected 0xad618001)\n", val);
        }
    } else {
        IOLog("NativeAdreno: smoke test submission / fence wait failed\n");
    }

    /* Setup submit and core user clients */
    execbuf_ = new NativeAdrenoSubmit;
    core_ = new NativeAdrenoUserClient;
    if (execbuf_ && core_ &&
        execbuf_->attach(gpu_ctx_.vm(), &ring_) == kIOReturnSuccess &&
        core_->open(NativeAdrenoAbiVersion, gpu_ctx_.vm(), execbuf_, &ring_, 32, 40) == kIOReturnSuccess) {
        core_ready_ = true;
    }

    /* 11. Register AGXAccelerator into IORegistry with sandbox grant properties */
    if (native_adreno_accelerator_allocate) {
        accelerator_ = native_adreno_accelerator_allocate();
        if (accelerator_) {
            if (accelerator_->init()) {
                accelerator_->setProperty("MetalPluginName", "NativeAdrenoMetal");
                accelerator_->setProperty("MetalPluginClassName", "NativeAdrenoMetalDevice");
                accelerator_->setProperty("NativeSandboxGrantProcess", "backboardd,adrenoprobed,SpringBoard,*");
                accelerator_->setProperty("NativeSandboxGrant", kOSBooleanTrue);
                if (accelerator_->attach(IOService::getPlatform())) {
                    accelerator_->registerService();
                    IOLog("NativeAdreno: AGXAccelerator published in IORegistry (MetalPluginName=NativeAdrenoMetal)\n");
                }
            }
        }
    }

    /* 12. Publish NativeAdrenoGpu service */
    NativeAdrenoAbiDeviceInfo dev_info{};
    dev_info.chip_id = A618_CHIP_ID;
    dev_info.gmem_bytes = A618_GMEM_SIZE;
    dev_info.gmem_base = 0x100000;
    dev_info.uche_trap_base = 0x100000;
    dev_info.va_start = 0x20000000;
    dev_info.va_size = 0x60000000;
    dev_info.highest_bank_bit = 14;
    dev_info.ubwc_swizzle = 6;
    dev_info.macrotile_mode = 0;
    dev_info.ubwc_version = 2;

    native_adreno_publish(gpu_ctx_.vm(), execbuf_, &ring_, &clock, nullptr, &dev_info, 32, 40);

    ready_ = true;
    IOLog("NativeAdreno: GPU driver fully ready, publishing service\n");
    registerService();

    smmu_svc->release();
    gmu_svc->release();
    return true;
}

void NativeAdrenoGpuService::stop(IOService *provider)
{
    if (accelerator_) {
        accelerator_->terminate();
        accelerator_->release();
        accelerator_ = nullptr;
    }
    if (core_) {
        delete core_;
        core_ = nullptr;
    }
    if (execbuf_) {
        delete execbuf_;
        execbuf_ = nullptr;
    }
    gcc_map_.reset();
    for (auto &map : maps_) map.reset();
    ready_ = false;
    core_ready_ = false;
    IOService::stop(provider);
}

IOReturn NativeAdrenoGpuService::newUserClient(task_t owningTask,void *securityID,
    UInt32 type,OSDictionary *properties,IOUserClient **handler)
{
    if (handler) *handler = nullptr;
    if (!core_ready_) return kIOReturnNotReady;
    IOUserClient *client = createGpuClient(owningTask,securityID,type,properties,core_);
    if (!client) return kIOReturnNoResources;
    if (handler) *handler = client;
    return kIOReturnSuccess;
}

class NativeAdrenoGpuClient : public IOUserClient {
    OSDeclareDefaultStructors(NativeAdrenoGpuClient)
    NativeAdrenoUserClient *core_ = nullptr;
    task_t task_ = TASK_NULL;
    static IOReturn action(OSObject *target,void *reference,
        IOExternalMethodArguments *arguments);
public:
    void setCore(NativeAdrenoUserClient *core) { core_ = core; }
    bool initWithTask(task_t owningTask,void *securityID,UInt32 type,
        OSDictionary *properties) APPLE_KEXT_OVERRIDE;
    IOReturn clientClose() APPLE_KEXT_OVERRIDE;
    IOReturn clientMemoryForType(UInt32 type,IOOptionBits *options,
        IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
    IOReturn externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
        IOExternalMethodDispatch *dispatch,OSObject *target,
        void *reference) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(NativeAdrenoGpuClient,IOUserClient)

bool NativeAdrenoGpuClient::initWithTask(task_t owningTask,void *securityID,
    UInt32 type,OSDictionary *properties)
{
    if (!IOUserClient::initWithTask(owningTask,securityID,type,properties)) return false;
    task_ = owningTask;
    return true;
}

IOReturn NativeAdrenoGpuClient::clientClose()
{
    core_ = nullptr;
    task_ = TASK_NULL;
    terminate();
    return kIOReturnSuccess;
}

IOReturn NativeAdrenoGpuClient::clientMemoryForType(UInt32 type,IOOptionBits *options,
    IOMemoryDescriptor **memory)
{
    if (memory) *memory = nullptr;
    if (options) *options = 0;
    return kIOReturnUnsupported;
}

IOReturn NativeAdrenoGpuClient::action(OSObject *target,void *reference,
    IOExternalMethodArguments *arguments)
{
    NativeAdrenoGpuClient *client = OSDynamicCast(NativeAdrenoGpuClient,target);
    if (!client || !client->core_ || !reference) return kIOReturnBadArgument;
    const uint32_t method = *static_cast<const uint32_t *>(reference);
    const void *in = arguments ? arguments->structureInput : nullptr;
    const size_t in_size = arguments ? arguments->structureInputSize : 0;
    void *out = arguments ? arguments->structureOutput : nullptr;
    size_t out_size = arguments ? arguments->structureOutputSize : 0;
    uint32_t out_written = 0;
    const IOReturn result = client->core_->dispatch(method,in,in_size,out,out_size,&out_written);
    if (arguments && !result) arguments->structureOutputSize = out_written;
    return result;
}

IOReturn NativeAdrenoGpuClient::externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
    IOExternalMethodDispatch *dispatch,OSObject *target,void *reference)
{
    static const uint32_t methods[] = {
        NativeAdrenoMethodBoAlloc, NativeAdrenoMethodBoFree, NativeAdrenoMethodBoMap,
        NativeAdrenoMethodBoUnmap, NativeAdrenoMethodSubmit, NativeAdrenoMethodWaitFence,
        NativeAdrenoMethodQueryFence, NativeAdrenoMethodSetMetadata, NativeAdrenoMethodGetMetadata,
        NativeAdrenoMethodQueueCreate, NativeAdrenoMethodQueueDestroy, NativeAdrenoMethodBoCpuMap,
        NativeAdrenoMethodBoCpuUnmap, NativeAdrenoMethodDeviceInfo, NativeAdrenoMethodBoImport
    };
    static const IOExternalMethodDispatch table[] = {
        {&action,0,sizeof(NativeAdrenoAbiBoAlloc),0,sizeof(NativeAdrenoAbiBoAllocReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoFree),0,0},
        {&action,0,sizeof(NativeAdrenoAbiBoMap),0,sizeof(NativeAdrenoAbiBoMapReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoUnmap),0,0},
        {&action,0,sizeof(NativeAdrenoAbiSubmit),0,sizeof(NativeAdrenoAbiSubmitReply)},
        {&action,0,sizeof(NativeAdrenoAbiWaitFence),0,0},
        {&action,0,sizeof(NativeAdrenoAbiQueryFence),0,sizeof(NativeAdrenoAbiQueryFenceReply)},
        {&action,0,sizeof(NativeAdrenoAbiMetadata),0,0},
        {&action,0,sizeof(NativeAdrenoAbiMetadata),0,sizeof(NativeAdrenoAbiMetadata)},
        {&action,0,sizeof(NativeAdrenoAbiQueue),0,sizeof(NativeAdrenoAbiQueueReply)},
        {&action,0,sizeof(NativeAdrenoAbiQueue),0,0},
        {&action,0,sizeof(NativeAdrenoAbiBoCpuMap),0,sizeof(NativeAdrenoAbiBoCpuMapReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoCpuUnmap),0,0},
        {&action,0,0,0,sizeof(NativeAdrenoAbiDeviceInfo)},
        {&action,0,sizeof(NativeAdrenoAbiBoImport),0,sizeof(NativeAdrenoAbiBoAllocReply)}
    };
    if (selector < 1 || selector > sizeof(table) / sizeof(table[0]))
        return kIOReturnUnsupported;
    return IOUserClient::externalMethod(selector,arguments,
        const_cast<IOExternalMethodDispatch *>(&table[selector - 1]),
        target ? target : this,
        const_cast<void *>(static_cast<const void *>(&methods[selector - 1])));
}

static IOUserClient *createGpuClient(task_t owningTask,void *securityID,UInt32 type,
    OSDictionary *properties,NativeAdrenoUserClient *core)
{
    (void)type;
    (void)properties;
    NativeAdrenoGpuClient *client = new NativeAdrenoGpuClient;
    if (!client) return nullptr;
    if (!client->initWithTask(owningTask,securityID,type,properties)) {
        client->release();
        return nullptr;
    }
    client->setCore(core);
    return client;
}
