/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_GMU_SERVICE_H
#define NATIVE_ADRENO_GMU_SERVICE_H
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include "NativeAdrenoBoard.h"

class NativeAdrenoGmuService : public IOService {
    OSDeclareDefaultStructors(NativeAdrenoGmuService)
    OSSharedPtr<IOMemoryMap> maps_[NativeAdrenoMaxWindows];
    NativeAdrenoWindow windows_[NativeAdrenoMaxWindows]{};
    size_t count_ = 0;
    bool booted_ = false;

public:
    IOService *probe(IOService *provider, SInt32 *score) APPLE_KEXT_OVERRIDE;
    bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
    void stop(IOService *provider) APPLE_KEXT_OVERRIDE;

    uint64_t windowBase(size_t index) const {
        return index < count_ ? windows_[index].base : 0;
    }
    uint64_t windowBytes(size_t index) const {
        return index < count_ ? windows_[index].bytes : 0;
    }
    size_t windowCount() const { return count_; }

    /* Mapped window address for a caller that already trusts this service. */
    volatile uint8_t *windowAddress(size_t index) const {
        return index < count_ && maps_[index] ?
            reinterpret_cast<volatile uint8_t *>(maps_[index]->getVirtualAddress()) : nullptr;
    }

    bool isBooted() const { return booted_; }

    /*
     * A618 Cold-Start Hardware Bring-up:
     * - GPLL0/GPLL2 vote and GMU RCG 200 MHz clock tree configuration
     * - RPMh ARC rail voting (CX, MX to LOW_SVS 64)
     * - CX GDSC power domain uncollapse
     * - GMU firmware (a630_gmu.bin) loaded into TCM at +0x3000
     * - 48-entry RSCC / PDC initialization table and stop handshake
     * - SMMU Context Bank 3 setup for GMU Stream 5 with buffers at 0x60000000
     * - CM3 reset release and FW_INIT_RESULT polling (0xbabeface)
     * - GX Turbo=7, MX Nominal=3 rail votes
     * - OOB BOOT_SLUMBER handshake (bit 22/30)
     * - SPTPRAC shader core cluster power-up (0x778000 -> status 0x28)
     * - HFI initialization
     */
    IOReturn coldStartA618(volatile uint8_t *gcc, volatile uint8_t *cc, volatile uint8_t *smmu,
                           uint64_t gmu_buf_pa, void *gmu_buf_kva);
};

#endif
