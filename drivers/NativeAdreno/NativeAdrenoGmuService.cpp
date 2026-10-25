/* SPDX-License-Identifier: MIT */
/*
 * NativeAdreno GMU service for Adreno 618 (SM7150 / Sweet).
 * Discovers the GMU, PDC, and PDC SEQ register windows, maps them,
 * and executes the full verified A618 cold-start hardware sequence:
 * clocks, RPMh ARC rails, GDSC, TCM firmware load, PDC/RSCC table,
 * SMMU CB3 GMU stream routing, CM3 unreset, SPTPRAC, and HFI.
 */
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <string.h>
#include "NativeAdrenoGmuService.h"

OSDefineMetaClassAndStructors(NativeAdrenoGmuService, IOService)

extern "C" const uint8_t native_adreno_firmware_image[] __attribute__((weak));
extern "C" const uint64_t native_adreno_firmware_image_size __attribute__((weak));

extern "C" int sweet_rpmh_arc_set(const char *res, unsigned corner, unsigned *level) __attribute__((weak));

#define GPU_RSCC_BASE           0x00023000
#define GPU_LOW_SVS             64u
#define GPU_CX_GDSCR            0x106c
#define GPU_GX_GDSCR            0x100c
#define GPU_BRANCH_ENABLE       (1u << 0)
#define GPU_BRANCH_OFF          (1u << 31)
#define GPU_DOMAIN_ON           (1u << 31)
#define GPU_GDSC_SW_COLLAPSE    (1u << 0)

#define GPU_GCC_GPU_GPLL_VOTE   0x52004
#define GPU_GPLL0_DIV_GATE      (1u << 16)
#define GPU_GMU_RCG_CMD         0x1120
#define GPU_GMU_RCG_CFG         0x1124
#define GPU_GMU_RCG_200MHZ      ((6u << 8) | 2u)
#define GPU_CC_AHB              0x1078
#define GPU_CC_CX_GMU           0x1098
#define GPU_CC_CXO              0x109c
#define GPU_CBCR_WAKE_SLEEP     ((0xfu << 8) | (0xfu << 4))
#define GPU_GCC_MEMNOC_GFX      0x7100c
#define GPU_GCC_DDRSS_GPU_AXI   0x4452c

#define GPU_SMMU_SMR            0x800
#define GPU_SMMU_S2CR           0xc00
#define GPU_SMMU_CBAR           0x000
#define GPU_SMMU_CBA2R          0x800
#define GPU_SMMU_CB0            0x8000
#define GPU_SMMU_CB_SCTLR       0x00
#define GPU_SMMU_CB_TCR2        0x10
#define GPU_SMMU_CB_TTBR0       0x20
#define GPU_SMMU_CB_TCR         0x30
#define GPU_SMMU_CB_MAIR0       0x38
#define GPU_SMMU_CB_MAIR1       0x3c
#define GPU_SMMU_CB_TLB_SYNC    0x7f4
#define GPU_SMMU_CB_TLB_STATUS  0x7f8
#define GPU_SMMU_CB_TLB_INVALL  0x618

#define GPU_SMMU_GMU_SID        5u
#define GPU_SMMU_GMU_CB         3u
#define GPU_PTE_TABLE           0x3ull
#define GPU_PTE_PAGE            ((1ull << 10) | (3ull << 8) | (2ull << 6) | 3ull)

static const struct { unsigned char region; uint32_t off; uint32_t value; } gpu_rpmh_init[] = {
    { 0, GPU_RSCC_BASE + 0x0004 * 4, 1u << 24 },          /* disable SDE clock gating */
    { 0, GPU_RSCC_BASE + 0x000b * 4, 1 },                 /* PDC_SLAVE_ID_DRV0 */
    { 0, GPU_RSCC_BASE + 0x000e * 4, 0 },                 /* HIDDEN_TCS_CMD0_DATA */
    { 0, GPU_RSCC_BASE + 0x000d * 4, 0 },                 /* HIDDEN_TCS_CMD0_ADDR */
    { 0, GPU_RSCC_BASE + 0x0010 * 4, 0 },
    { 0, GPU_RSCC_BASE + 0x000f * 4, 0 },
    { 0, GPU_RSCC_BASE + 0x0012 * 4, 0x80000000u },
    { 0, GPU_RSCC_BASE + 0x0011 * 4, 0 },
    { 0, GPU_RSCC_BASE + 0x0100 * 4, 0 },                 /* OVERRIDE_START_ADDR */
    { 0, GPU_RSCC_BASE + 0x0008 * 4, 0x4520 },            /* PDC_SEQ_START_ADDR */
    { 0, GPU_RSCC_BASE + 0x0009 * 4, 0x4510 },            /* PDC_MATCH_VALUE_LO */
    { 0, GPU_RSCC_BASE + 0x000a * 4, 0x4514 },            /* PDC_MATCH_VALUE_HI */
    { 0, GPU_RSCC_BASE + 0x0180 * 4, 0xa7a506a0u },       /* RSC sequencer uCode */
    { 0, GPU_RSCC_BASE + 0x0181 * 4, 0xa1e6a6e7u },
    { 0, GPU_RSCC_BASE + 0x0182 * 4, 0xa2e081e1u },
    { 0, GPU_RSCC_BASE + 0x0183 * 4, 0xe9a982e2u },
    { 0, GPU_RSCC_BASE + 0x0184 * 4, 0x0020e8a8u },
    { 2, 0 * 4, 0xfebea1e1u },                            /* PDC sequencer uCode */
    { 2, 1 * 4, 0xa5a4a3a2u },
    { 2, 2 * 4, 0x8382a6e0u },
    { 2, 3 * 4, 0xbce3e284u },
    { 2, 4 * 4, 0x002081fcu },
    { 1, 0x1573 * 4, 7 }, { 1, 0x1574 * 4, 0 }, { 1, 0x1572 * 4, 0 },   /* TCS1 (sleep) */
    { 1, 0x1575 * 4, 0x10108 }, { 1, 0x1576 * 4, 0x30010 }, { 1, 0x1577 * 1 },
    { 1, 0x1579 * 4, 0x10108 }, { 1, 0x157a * 4, 0x30000 }, { 1, 0x157b * 4, 0 },
    { 1, 0x157d * 4, 0x10108 }, { 1, 0x157e * 4, 0x30090 }, { 1, 0x157f * 4, 0 },
    { 1, 0x15d7 * 4, 7 }, { 1, 0x15d8 * 4, 0 }, { 1, 0x15d6 * 4, 0 },   /* TCS3 (wake), A618 data */
    { 1, 0x15d9 * 4, 0x10108 }, { 1, 0x15da * 4, 0x30010 }, { 1, 0x15db * 4, 2 },
    { 1, 0x15dd * 4, 0x10108 }, { 1, 0x15de * 4, 0x30000 }, { 1, 0x15df * 4, 2 },
    { 1, 0x15e1 * 4, 0x10108 }, { 1, 0x15e2 * 4, 0x30090 }, { 1, 0x15e3 * 4, 3 },
    { 1, 0x1148 * 4, 0 },                                 /* GPU_SEQ_START_ADDR */
    { 1, 0x1140 * 4, 0x80000001u },                       /* GPU_ENABLE_PDC */
};
#define GPU_RPMH_INIT_COUNT (sizeof(gpu_rpmh_init) / sizeof(gpu_rpmh_init[0]))

static inline uint32_t io_read32(volatile uint8_t *base, uint32_t off) {
    return *reinterpret_cast<volatile uint32_t *>(base + off);
}
static inline void io_write32(volatile uint8_t *base, uint32_t off, uint32_t val) {
    *reinterpret_cast<volatile uint32_t *>(base + off) = val;
}
static inline void io_write64(volatile uint8_t *base, uint32_t off, uint64_t val) {
    *reinterpret_cast<volatile uint64_t *>(base + off) = val;
}

static bool advertisesGmu(IOService *provider)
{
    OSObject *property = provider ? provider->getProperty("compatible") : nullptr;
    OSData *data = OSDynamicCast(OSData, property);
    if (!data || !data->getBytesNoCopy() || !data->getLength()) return false;
    const char *text = static_cast<const char *>(data->getBytesNoCopy());
    const size_t length = data->getLength();
    static const char *needles[] = { "qcom,adreno-gmu", "qcom,gpu-gmu" };
    for (const char *needle : needles) {
        const size_t needle_len = strlen(needle);
        for (size_t at = 0; at + needle_len <= length; ++at) {
            if (strncmp(text + at, needle, needle_len) == 0) return true;
        }
    }
    return false;
}

IOService *NativeAdrenoGmuService::probe(IOService *provider, SInt32 *score)
{
    if (!advertisesGmu(provider)) return nullptr;
    if (score) *score = 1000;
    return this;
}

bool NativeAdrenoGmuService::start(IOService *provider)
{
    if (!IOService::start(provider)) return false;
    OSData *reg = OSDynamicCast(OSData, provider->getProperty("reg"));
    OSData *address = OSDynamicCast(OSData, provider->getProperty("#address-cells"));
    OSData *size = OSDynamicCast(OSData, provider->getProperty("#size-cells"));
    uint32_t address_cells = 2, size_cells = 2;
    if (address && address->getLength() >= 4)
        address_cells = *static_cast<const uint32_t *>(address->getBytesNoCopy());
    if (size && size->getLength() >= 4)
        size_cells = *static_cast<const uint32_t *>(size->getBytesNoCopy());
    if (!reg || NativeAdrenoBoard::parseReg(
        static_cast<const uint8_t *>(reg->getBytesNoCopy()), reg->getLength(),
        address_cells, size_cells, windows_, NativeAdrenoMaxWindows, &count_) != kIOReturnSuccess) {
        IOLog("NativeAdrenoGmu: reg windows are not usable\n");
        count_ = 0;
        IOService::stop(provider);
        return false;
    }
    for (size_t index = 0; index < count_; ++index) {
        IODeviceMemory *memory = getDeviceMemoryWithIndex(static_cast<unsigned>(index));
        if (!memory) {
            IOLog("NativeAdrenoGmu: window %lu has no device memory\n",
                static_cast<unsigned long>(index));
            stop(provider);
            return false;
        }
        maps_[index] = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache),
            OSNoRetain);
        if (!maps_[index]) {
            IOLog("NativeAdrenoGmu: window %lu did not map\n",
                static_cast<unsigned long>(index));
            stop(provider);
            return false;
        }
    }

    OSData *bases = OSData::withCapacity(static_cast<UInt32>(count_ * sizeof(uint64_t)));
    if (!bases) { stop(provider); return false; }
    for (size_t index = 0; index < count_; ++index) {
        const uint64_t base = windows_[index].base;
        bases->appendBytes(&base, sizeof(base));
    }
    setProperty("gmu-window-bases", bases);
    bases->release();

    IOLog("NativeAdrenoGmu: %lu windows mapped (GMU=0x%llx, PDC=0x%llx, SEQ=0x%llx)\n",
        static_cast<unsigned long>(count_),
        count_ > 0 ? static_cast<unsigned long long>(windows_[0].base) : 0,
        count_ > 1 ? static_cast<unsigned long long>(windows_[1].base) : 0,
        count_ > 2 ? static_cast<unsigned long long>(windows_[2].base) : 0);
    registerService();
    return true;
}

void NativeAdrenoGmuService::stop(IOService *provider)
{
    for (auto &map : maps_) map.reset();
    count_ = 0;
    booted_ = false;
    IOService::stop(provider);
}

IOReturn NativeAdrenoGmuService::coldStartA618(volatile uint8_t *gcc, volatile uint8_t *cc, volatile uint8_t *smmu,
                                              uint64_t gmu_buf_pa, void *gmu_buf_kva)
{
    if (!gcc || !cc || !smmu || !gmu_buf_pa || !gmu_buf_kva) {
        IOLog("NativeAdrenoGmu: coldStartA618 invalid arguments\n");
        return kIOReturnBadArgument;
    }
    if (count_ < 3 || !windowAddress(0) || !windowAddress(1) || !windowAddress(2)) {
        IOLog("NativeAdrenoGmu: coldStartA618 GMU/PDC/SEQ windows not mapped\n");
        return kIOReturnNotReady;
    }

    volatile uint8_t *gmu = windowAddress(0);
    volatile uint8_t *pdc = windowAddress(1);
    volatile uint8_t *seq = windowAddress(2);

    IOLog("NativeAdrenoGmu: starting A618 cold start sequence\n");

    /* 1. RPMh ARC rail votes: MX and CX at LOW_SVS (64) */
    if (sweet_rpmh_arc_set) {
        unsigned mx_level = 0, cx_level = 0;
        int err_mx = sweet_rpmh_arc_set("mx.lvl", GPU_LOW_SVS, &mx_level);
        int err_cx = sweet_rpmh_arc_set("cx.lvl", GPU_LOW_SVS, &cx_level);
        if (err_mx || err_cx) {
            IOLog("NativeAdrenoGmu: RPMh ARC rail vote failed (mx=%d cx=%d)\n", err_mx, err_cx);
            return kIOReturnIOError;
        }
    }

    /* 2. CX GDSC power domain uncollapse */
    uint32_t cx_val = io_read32(cc, GPU_CX_GDSCR);
    io_write32(cc, GPU_CX_GDSCR, cx_val & ~GPU_GDSC_SW_COLLAPSE);
    IODelay(1);
    bool cx_on = false;
    for (unsigned i = 0; i < 1000; i++) {
        if (io_read32(cc, GPU_CX_GDSCR) & GPU_DOMAIN_ON) {
            cx_on = true;
            break;
        }
        IODelay(1);
    }
    if (!cx_on) {
        IOLog("NativeAdrenoGmu: CX PWR_ON did not assert\n");
        return kIOReturnTimeout;
    }
    IODelay(5);

    /* 3. GPLL0 divider gate vote */
    uint32_t gate = io_read32(gcc, GPU_GCC_GPU_GPLL_VOTE);
    io_write32(gcc, GPU_GCC_GPU_GPLL_VOTE, gate | GPU_GPLL0_DIV_GATE);
    IODelay(5);

    /* 4. GMU RCG 200 MHz */
    uint32_t rcg_cfg = io_read32(cc, GPU_GMU_RCG_CFG);
    io_write32(cc, GPU_GMU_RCG_CFG, (rcg_cfg & ~((7u << 8) | 0x1fu)) | GPU_GMU_RCG_200MHZ);
    io_write32(cc, GPU_GMU_RCG_CMD, 1u);
    bool rcg_ok = false;
    for (unsigned i = 0; i < 500; i++) {
        if (!(io_read32(cc, GPU_GMU_RCG_CMD) & 1u)) {
            rcg_ok = true;
            break;
        }
        IODelay(1);
    }
    if (!rcg_ok) {
        IOLog("NativeAdrenoGmu: GMU RCG update timed out\n");
        return kIOReturnTimeout;
    }

    /* 5. GPU CC branches */
    io_write32(cc, GPU_CC_AHB, GPU_BRANCH_ENABLE);
    io_write32(cc, GPU_CC_CX_GMU, GPU_BRANCH_ENABLE | GPU_CBCR_WAKE_SLEEP);
    io_write32(cc, GPU_CC_CXO, GPU_BRANCH_ENABLE);
    for (unsigned i = 0; i < 500; i++) {
        uint32_t b0 = io_read32(cc, GPU_CC_AHB);
        uint32_t b1 = io_read32(cc, GPU_CC_CX_GMU);
        uint32_t b2 = io_read32(cc, GPU_CC_CXO);
        if (!(b0 & GPU_BRANCH_OFF) && !(b1 & GPU_BRANCH_OFF) && !(b2 & GPU_BRANCH_OFF)) break;
        IODelay(1);
    }

    /* 6. GCC bus branches */
    io_write32(gcc, GPU_GCC_MEMNOC_GFX, GPU_BRANCH_ENABLE);
    io_write32(gcc, GPU_GCC_DDRSS_GPU_AXI, GPU_BRANCH_ENABLE);
    for (unsigned i = 0; i < 500; i++) {
        uint32_t b3 = io_read32(gcc, GPU_GCC_MEMNOC_GFX);
        uint32_t b4 = io_read32(gcc, GPU_GCC_DDRSS_GPU_AXI);
        if (!(b3 & GPU_BRANCH_OFF) && !(b4 & GPU_BRANCH_OFF)) break;
        IODelay(1);
    }

    /* 7. Load firmware into GMU TCM at +0x3000 */
    if (!reinterpret_cast<uintptr_t>(native_adreno_firmware_image) ||
        native_adreno_firmware_image_size < 32768) {
        IOLog("NativeAdrenoGmu: embedded GMU firmware missing or truncated\n");
        return kIOReturnNoResources;
    }
    const uint32_t *fw_words = reinterpret_cast<const uint32_t *>(native_adreno_firmware_image);
    const size_t fw_dwords = 32768 / 4;
    for (size_t i = 0; i < fw_dwords; i++) {
        io_write32(gmu, 0x3000 + static_cast<uint32_t>(i * 4), fw_words[i]);
    }

    /* 8. Program 48-entry RSCC / PDC table */
    volatile uint8_t *bases[3] = { gmu, pdc, seq };
    for (unsigned i = 0; i < GPU_RPMH_INIT_COUNT; i++) {
        io_write32(bases[gpu_rpmh_init[i].region], gpu_rpmh_init[i].off, gpu_rpmh_init[i].value);
    }
    /* RSCC stop handshake */
    io_write32(gmu, 0x9307 * 4, 1u);
    bool stop_done = false;
    for (unsigned i = 0; i < 1000; i++) {
        if (io_read32(gmu, GPU_RSCC_BASE + 0x0004 * 4) & (1u << 16)) {
            stop_done = true;
            break;
        }
        IODelay(1);
    }
    io_write32(gmu, 0x9307 * 4, 0u);
    if (!stop_done) {
        IOLog("NativeAdrenoGmu: RSCC stop handshake timed out\n");
        return kIOReturnTimeout;
    }

    /* 9. Format GMU SMMU page tables at gmu_buf_kva */
    uint8_t *kva = static_cast<uint8_t *>(gmu_buf_kva);
    memset(kva, 0, 0xe000);
    // L1 at 0: entry 1 -> L2 at pa + 0x1000
    reinterpret_cast<uint64_t *>(kva)[1] = (gmu_buf_pa + 0x1000) | GPU_PTE_TABLE;
    // L2 at +0x1000: entry 256 -> L3 at pa + 0x2000
    reinterpret_cast<uint64_t *>(kva + 0x1000)[256] = (gmu_buf_pa + 0x2000) | GPU_PTE_TABLE;
    // L3 at +0x2000: 13 pages (0x60000000 .. 0x6000d000)
    for (unsigned i = 0; i < 13; i++) {
        reinterpret_cast<uint64_t *>(kva + 0x2000)[i] = (gmu_buf_pa + 0x3000 + 0x1000 * i) | GPU_PTE_PAGE;
    }
    // HFI queues table at +0xc000 (offset from kva)
    uint32_t *hfi = reinterpret_cast<uint32_t *>(kva + 0xc000);
    hfi[0] = 0; hfi[1] = 120; hfi[2] = 6; hfi[3] = 12; hfi[4] = 2; hfi[5] = 2;
    for (unsigned i = 0; i < 2; i++) {
        uint32_t *q = hfi + 6 + 12 * i;
        q[0] = 1;
        q[1] = 0x6000a000u + 0x1000 * i;
        q[2] = (10u << 8) | (i ? 4 : 0);
        q[3] = 1024;
        q[6] = 1; q[7] = 1; q[8] = 1;
    }
    __builtin_arm_dsb(0xf);

    /* Program SMMU CB3 */
    const uint32_t n = GPU_SMMU_GMU_CB;
    const uint32_t cb = GPU_SMMU_CB0 + n * 0x1000u;
    io_write32(smmu, GPU_SMMU_CBA2R + 4 * n, 1);
    io_write32(smmu, GPU_SMMU_CBAR + 4 * n, 1u << 16);
    io_write32(smmu, cb + GPU_SMMU_CB_TCR2, 1);
    io_write32(smmu, cb + GPU_SMMU_CB_TCR, 0x802020u);
    io_write64(smmu, cb + GPU_SMMU_CB_TTBR0, gmu_buf_pa);
    io_write32(smmu, cb + GPU_SMMU_CB_MAIR0, 0x44);
    io_write32(smmu, cb + GPU_SMMU_CB_MAIR1, 0);

    /* TLB invalidate & sync */
    io_write32(smmu, cb + GPU_SMMU_CB_TLB_INVALL, 0);
    io_write32(smmu, cb + GPU_SMMU_CB_TLB_SYNC, 0);
    for (unsigned i = 0; i < 500; i++) {
        if (!(io_read32(smmu, cb + GPU_SMMU_CB_TLB_STATUS) & 1u)) break;
        IODelay(1);
    }
    io_write32(smmu, cb + GPU_SMMU_CB_SCTLR, 0x27);
    io_write32(smmu, GPU_SMMU_S2CR + 4 * n, n);
    io_write32(smmu, GPU_SMMU_SMR + 4 * n, (1u << 31) | GPU_SMMU_GMU_SID);

    /* 10. Mask interrupt sources */
    io_write32(gmu, 0x24c18, ~0u);
    io_write32(gmu, 0x1464c, ~0u);

    /* 11. GMU boot configuration */
    io_write32(gmu, 0x14070, 0);
    io_write32(gmu, 0x14004, 0x02);
    io_write32(gmu, 0x14614, 0x60009000);
    io_write32(gmu, 0x14610, 1);
    io_write32(gmu, 0x24c44, 0x802800a0);
    io_write32(gmu, 0x140b4, 0x4052);
    io_write32(gmu, 0x14608, 0x06018000);
    io_write32(gmu, 0x14400, 0x60005003);
    io_write32(gmu, 0x1303c, 1);
    io_write32(gmu, 0x13000, 1);
    io_write32(gmu, 0x13004, 1);
    io_write32(gmu, 0x14300, 0x09c40400);
    io_write32(gmu, 0x143a0, io_read32(gmu, 0x143a0) | 0x00000f11);
    io_write32(gmu, 0x14404, 0);

    /* 12. Release CM3 reset & poll FW_INIT_RESULT */
    IOLog("NativeAdrenoGmu: releasing CM3 processor from reset\n");
    io_write32(gmu, 0x14000, 0);

    bool cm3_ready = false;
    uint32_t init_res = 0;
    for (unsigned i = 0; i < 1000; i++) {
        init_res = io_read32(gmu, 0x14070);
        if (init_res == 0xbabeface || (init_res & 0x1ff) == 0x100) {
            cm3_ready = true;
            break;
        }
        IODelay(100);
    }
    if (!cm3_ready) {
        IOLog("NativeAdrenoGmu: CM3 FW_INIT_RESULT timed out (0x%08x)\n", init_res);
        return kIOReturnTimeout;
    }
    IOLog("NativeAdrenoGmu: CM3 FW initialized successfully (result 0x%08x)\n", init_res);

    /* 13. GX Rail vote, OOB BOOT_SLUMBER, SPTPRAC, and HFI */
    io_write32(gmu, 0x8fe0, 0);
    io_write32(gmu, 0x8fe4, 7); // GX Turbo = 7
    io_write32(gmu, 0x8fe8, 3); // MX Nominal = 3

    /* Request OOB BOOT_SLUMBER (bit 22), poll ACK (bit 30) */
    io_write32(gmu, 0x14650, 1u << 22);
    bool oob_done = false;
    for (unsigned i = 0; i < 2000; i++) {
        if (io_read32(gmu, 0x14648) & (1u << 30)) {
            oob_done = true;
            break;
        }
        IODelay(100);
    }
    io_write32(gmu, 0x14644, 1u << 30); // acknowledge
    if (!oob_done) {
        IOLog("NativeAdrenoGmu: OOB BOOT_SLUMBER handshake timed out\n");
        return kIOReturnTimeout;
    }

    /* Enable SPTPRAC shader core cluster power */
    io_write32(gmu, 0x204, 0x778000);
    bool sptprac_done = false;
    for (unsigned i = 0; i < 1000; i++) {
        if ((io_read32(gmu, 0x14340) & 0x38) == 0x28) {
            sptprac_done = true;
            break;
        }
        IODelay(10);
    }
    if (!sptprac_done) {
        IOLog("NativeAdrenoGmu: SPTPRAC enable timed out\n");
        return kIOReturnTimeout;
    }

    /* Start HFI */
    io_write32(gmu, 0x14618, 1);
    bool hfi_done = false;
    for (unsigned i = 0; i < 1000; i++) {
        if (io_read32(gmu, 0x14600) & 1u) {
            hfi_done = true;
            break;
        }
        IODelay(100);
    }
    if (!hfi_done) {
        IOLog("NativeAdrenoGmu: HFI start timed out\n");
        return kIOReturnTimeout;
    }

    booted_ = true;
    IOLog("NativeAdrenoGmu: A618 cold start SUCCESS! GX/SPTPRAC powered, HFI ready\n");
    return kIOReturnSuccess;
}
