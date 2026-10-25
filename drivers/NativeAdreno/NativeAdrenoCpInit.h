/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_CPINIT_H
#define NATIVE_ADRENO_CPINIT_H
#include "NativeAdrenoRing.h"

struct NativeAdrenoCpInitConfig {
    uint64_t sqe_iova;             /* CP_SQE_INSTR_BASE, 4 KiB aligned, nonzero */
    uint64_t pwrup_reglist_iova;   /* Power-up register list, 4-byte aligned */
    /* Nonzero when the caller already wrote CP_SQE_INSTR_BASE before CP_SQE_CNTL, as a6xx hw_init does: the base is
     * then read back and compared instead of rewritten after the SQE started. */
    uint32_t sqe_base_programmed;
};

/*
 * One-shot Adreno command-processor start sequence:
 * - A7xx: CP_THREAD_CONTROL (concurrent binning disable) + CP_ME_INIT (7 words with pwrup list)
 * - A6xx (A618): CP_ME_INIT opcode 0x48 (8 words)
 */
class NativeAdrenoCpInit final {
    bool done_ = false, poisoned_ = false;
    IOReturn poison();
public:
    NativeAdrenoCpInit() = default;
    ~NativeAdrenoCpInit() = default;
    NativeAdrenoCpInit(const NativeAdrenoCpInit &) = delete;
    NativeAdrenoCpInit &operator=(const NativeAdrenoCpInit &) = delete;

    /* A7xx initialization sequence */
    IOReturn run(const NativeAdrenoRingIo *io, NativeAdrenoRing *ring,
                 const NativeAdrenoCpInitConfig *config, uint32_t *fence);

    /* A6xx / A618 initialization sequence */
    IOReturn runA6xx(const NativeAdrenoRingIo *io, NativeAdrenoRing *ring,
                     uint64_t sqe_iova, uint32_t *fence);

    bool completed() const { return done_; }
    bool poisoned() const { return poisoned_; }
    static uint32_t meInitMask();
};

#endif
