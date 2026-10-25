/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoCpInit.h"
#include "AdrenoPM4.h"
#include <IOKit/IOLib.h>

/* Pinned definitions: CP_SQE_INSTR_BASE is the 64-bit register at dword offset
 * 0x0830 for A6XX..A7XX (a6xx.xml). CP_THREAD_CONTROL is opcode 0x17 with
 * CONCURRENT_BIN_DISABLE at bit 27, and CP_ME_INIT is opcode 0x48 with seven
 * payload words on A7xx and eight on A6xx (adreno_pm4.xml). */
enum : uint32_t {
    CpSqeInstrBase = 0x0830,
    CpThreadControlOpcode = 0x17,
    CpMeInitOpcode = 0x48,
    CpConcurrentBinDisable = 1u << 27,
    CpInitWords = 10
};

IOReturn NativeAdrenoCpInit::poison()
{
    poisoned_ = true;
    return kIOReturnIOError;
}

uint32_t NativeAdrenoCpInit::meInitMask()
{
    return (1u << 0) | (1u << 1) | (1u << 3) | (1u << 6) | (1u << 8);
}

IOReturn NativeAdrenoCpInit::run(const NativeAdrenoRingIo *io, NativeAdrenoRing *ring,
    const NativeAdrenoCpInitConfig *config, uint32_t *fence)
{
    if (!fence) return kIOReturnBadArgument;
    *fence = 0;
    if (done_ || poisoned_) return kIOReturnNotReady;
    if (!io || !io->write32 || !ring || ring->poisoned()) return kIOReturnBadArgument;
    if (!config || !config->sqe_iova || (config->sqe_iova & 4095) ||
        config->sqe_iova >= UINT64_C(1) << 48 || !config->pwrup_reglist_iova ||
        (config->pwrup_reglist_iova & 3) || config->pwrup_reglist_iova >= UINT64_C(1) << 48)
        return kIOReturnBadArgument;
    if (config->sqe_base_programmed) {
        uint32_t lo = 0, hi = 0;
        if (!io->read32 || io->read32(io->cookie, CpSqeInstrBase, &lo) || io->read32(io->cookie, CpSqeInstrBase + 1, &hi) ||
            (((uint64_t)hi << 32) | lo) != config->sqe_iova) return poison();
    } else if (io->write32(io->cookie, CpSqeInstrBase, (uint32_t)config->sqe_iova) ||
        io->write32(io->cookie, CpSqeInstrBase + 1, (uint32_t)(config->sqe_iova >> 32)))
        return poison();
    uint32_t words[CpInitWords] = {0};
    if (AdrenoPM4Type7(CpThreadControlOpcode, 1, &words[0]) != AdrenoPM4Success ||
        AdrenoPM4Type7(CpMeInitOpcode, 7, &words[2]) != AdrenoPM4Success) return poison();
    words[1] = CpConcurrentBinDisable;
    words[3] = meInitMask();
    words[4] = 0x00000003;      /* hardware contexts */
    words[5] = 0x20000000;      /* error detection */
    words[6] = 0x00000002;      /* operation mode */
    words[7] = (uint32_t)config->pwrup_reglist_iova;
    words[8] = (uint32_t)(config->pwrup_reglist_iova >> 32);
    words[9] = 1u << 31;          /* read registers from the list */
    uint32_t value = 0;
    const IOReturn status = ring->submit(words, CpInitWords, &value);
    if (status) return poison();
    done_ = true;
    *fence = value;
    return kIOReturnSuccess;
}

IOReturn NativeAdrenoCpInit::runA6xx(const NativeAdrenoRingIo *io, NativeAdrenoRing *ring,
    uint64_t sqe_iova, uint32_t *fence)
{
    if (!fence) return kIOReturnBadArgument;
    *fence = 0;
    if (done_ || poisoned_) return kIOReturnNotReady;
    if (!io || !ring || ring->poisoned()) return kIOReturnBadArgument;
    if (!sqe_iova || (sqe_iova & 4095) || sqe_iova >= (UINT64_C(1) << 48))
        return kIOReturnBadArgument;

    /* On A6xx, a6xx_cp_init submits CP_ME_INIT with 8 words (total 9 PM4 words) */
    uint32_t words[9] = {0};
    if (AdrenoPM4Type7(CpMeInitOpcode, 8, &words[0]) != AdrenoPM4Success)
        return poison();
    words[1] = 0x0000002f;
    words[2] = 0x00000003; /* hardware contexts */
    words[3] = 0x20000000; /* error detection */
    words[4] = 0;          /* don't enable header dump */
    words[5] = 0;
    words[6] = 0;          /* no workarounds */
    words[7] = 0;          /* padding */
    words[8] = 0;

    uint32_t value = 0;
    const IOReturn status = ring->submit(words, 9, &value);
    if (status) return poison();
    done_ = true;
    *fence = value;
    return kIOReturnSuccess;
}
