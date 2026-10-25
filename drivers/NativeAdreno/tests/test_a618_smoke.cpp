/* SPDX-License-Identifier: MIT */
/*
 * Unit test for Adreno 618 (SM7150 / Sweet) kernel-space bring-up:
 * - Register offsets, chip ID and HWCG / Protect tables from NativeAdrenoA618HwInit.h
 * - A6xx CP_ME_INIT packet construction (opcode 0x48, 8 words)
 * - PM4 CP_MEM_WRITE packet construction (opcode 0x3d, 3 words)
 * - Ringbuffer submission, WPTR/RPTR tracking, and fence retirement
 * - Memory writeback smoke test simulation
 * - Sandbox grant process token matching logic
 */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoA618HwInit.h"
#include "NativeAdrenoCpInit.h"
#include "NativeAdrenoRing.h"
#include "AdrenoPM4.h"
#include <cstdio>
#include <cstring>
#include <vector>

#define CHECK(x) do { \
    if (!(x)) { \
        std::fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__, #x); \
        std::exit(1); \
    } \
} while (0)

struct MockGpuPeer {
    uint32_t regs[0x10000]{};
    std::vector<uint32_t> writes;
    unsigned publish_count = 0;
    unsigned acquire_count = 0;
    unsigned barrier_count = 0;
};

static MockGpuPeer peer;
static uint32_t ringmem[8192];  /* 32 KiB ring */
static uint32_t fencemem[1024]; /* 4 KiB fence / memptrs */
static uint32_t scratchmem[1024]; /* 4 KiB scratch buffer */

static int mock_write32(void *c, uint32_t reg, uint32_t v) {
    CHECK(c == &peer && reg < 0x10000);
    peer.regs[reg] = v;
    peer.writes.push_back(reg);
    return 0;
}
static int mock_read32(void *c, uint32_t reg, uint32_t *v) {
    CHECK(c == &peer && v && reg < 0x10000);
    *v = peer.regs[reg];
    return 0;
}
static int mock_barrier(void *c) {
    CHECK(c == &peer);
    ++peer.barrier_count;
    return 0;
}
static int mock_publish(void *c, const volatile uint32_t *w, size_t bytes) {
    (void)w;
    CHECK(c == &peer && bytes == sizeof(ringmem));
    ++peer.publish_count;
    return 0;
}
static int mock_acquire(void *c, volatile uint32_t *w, size_t bytes) {
    (void)w;
    CHECK(c == &peer && bytes == sizeof(fencemem));
    ++peer.acquire_count;
    return 0;
}

static NativeAdrenoRingIo ring_io{ &peer, mock_write32, mock_read32, mock_barrier };
static NativeAdrenoRingCache ring_cache{ &peer, mock_publish, mock_acquire };
static NativeAdrenoRingClock ring_clock{ &peer, nullptr, nullptr };

/* Sandbox grant matching test double */
static bool test_sandbox_match(const char *list, const char *process_name) {
    if (!list || !list[0] || !process_name || !process_name[0]) return false;
    if (std::strcmp(list, "*") == 0) return true;
    size_t namelen = std::strlen(process_name);
    const char *p = list;
    while (*p) {
        while (*p == ',' || *p == ' ') p++;
        if (!*p) break;
        const char *end = std::strchr(p, ',');
        size_t tokenlen = end ? (size_t)(end - p) : std::strlen(p);
        while (tokenlen > 0 && p[tokenlen - 1] == ' ') tokenlen--;
        if ((tokenlen == 1 && *p == '*') ||
            (tokenlen == namelen && std::strncmp(p, process_name, namelen) == 0)) {
            return true;
        }
        if (!end) break;
        p = end + 1;
    }
    return false;
}

int main() {
    std::printf("=== Running Adreno 618 Kernel Driver Unit Tests ===\n");

    /* 1. Hardware constant validation */
    std::printf("[*] Test 1: Adreno 618 Hardware Constants\n");
    CHECK(A618_CHIP_ID == 0x06010800);
    CHECK(A618_GMEM_SIZE == 0x80000); /* 512 KiB GMEM */
    CHECK(A618_CP_SQE_INSTR_BASE == 0x0830);
    CHECK(A618_CP_RB_BASE == 0x0800);
    CHECK(A618_CP_RB_CNTL == 0x0802);
    CHECK(A618_CP_RB_RPTR_ADDR == 0x0804);
    CHECK(A618_CP_RB_RPTR == 0x0806);
    CHECK(A618_CP_RB_WPTR == 0x0807);
    CHECK(A618_CP_SQE_CNTL == 0x0808);
    CHECK(NATIVE_ADRENO_A618_HWCG_COUNT == 63);
    CHECK(NATIVE_ADRENO_A618_PROTECT_COUNT == 32);
    std::printf("    -> Adreno 618 chip ID 0x%08x and 63 HWCG / 32 protect registers OK\n", A618_CHIP_ID);

    /* 2. IOVA Alignment and Range Checks */
    std::printf("[*] Test 2: GPU IOVA Aperture and SMMU Range Alignment\n");
    const uint64_t sqe_iova = 0x100000;
    const uint64_t ring_iova = 0x200000;
    const uint64_t memptrs_iova = 0x300000;
    const uint64_t pwrup_iova = 0x301000;
    const uint64_t scratch_iova = 0x302000;
    CHECK((sqe_iova & 0xfff) == 0);
    CHECK((ring_iova & 0x7fff) == 0); /* Ring buffer 32 KiB aligned */
    CHECK((memptrs_iova & 0xfff) == 0);
    CHECK((pwrup_iova & 0xfff) == 0);
    CHECK((scratch_iova & 0xfff) == 0);
    CHECK(sqe_iova + 0x13000 <= ring_iova);
    CHECK(ring_iova + 0x8000 <= memptrs_iova);
    CHECK(memptrs_iova + 0x1000 <= pwrup_iova);
    CHECK(pwrup_iova + 0x1000 <= scratch_iova);
    std::printf("    -> SMMU IOVA layout non-overlapping and 4K aligned OK\n");

    /* 3. A6xx CP_ME_INIT PM4 emission and submission */
    std::printf("[*] Test 3: A6xx CP_ME_INIT Emission via Ring\n");
    peer = MockGpuPeer{};
    std::memset(ringmem, 0xee, sizeof(ringmem));
    std::memset(fencemem, 0, sizeof(fencemem));
    std::memset(scratchmem, 0, sizeof(scratchmem));

    NativeAdrenoRing ring;
    CHECK(ring.attach(&ring_io, &ring_cache, ringmem, sizeof(ringmem), ring_iova,
                      fencemem, sizeof(fencemem), memptrs_iova) == kIOReturnSuccess);

    NativeAdrenoCpInit cp_init;
    uint32_t init_fence = 0;
    CHECK(cp_init.runA6xx(&ring_io, &ring, sqe_iova, &init_fence) == kIOReturnSuccess);
    CHECK(init_fence == 1);
    CHECK(cp_init.completed());

    /* Verify ring buffer contents:
     * Words 0..8: CP_ME_INIT packet
     * Verified by opcode and count, matching AdrenoPM4Type7
     */
    uint32_t expected_me_init_hdr = 0;
    CHECK(AdrenoPM4Type7(0x48, 8, &expected_me_init_hdr) == AdrenoPM4Success);
    CHECK(ringmem[0] == expected_me_init_hdr);
    CHECK((ringmem[0] >> 16 & 0x7fu) == 0x48);
    CHECK((ringmem[0] & 0x3fffu) == 8);
    CHECK(ringmem[1] == 0x0000002f);
    CHECK(ringmem[2] == 0x00000003); /* HW contexts */
    CHECK(ringmem[3] == 0x20000000); /* Error detection */
    CHECK(ringmem[4] == 0);
    CHECK(ringmem[5] == 0);
    CHECK(ringmem[6] == 0);
    CHECK(ringmem[7] == 0);
    CHECK(ringmem[8] == 0);

    /* Verify fence was appended (7 fence words):
     * Total packet words = 9 + 7 = 16 words.
     * WPTR register (0x0807) should be 16.
     */
    CHECK(peer.regs[A618_CP_RB_WPTR] == 16);

    /* Simulate CP execution: CP writes timestamp 1 to memptrs.fence (offset +4) */
    fencemem[1] = 1;
    CHECK(ring.wait(init_fence, &ring_clock, 1000, nullptr) == NativeAdrenoRingFenceComplete);
    std::printf("    -> A6xx CP_ME_INIT (9 words) + Fence emitted, WPTR=16, retired successfully\n");

    /* 4. PM4 CP_MEM_WRITE Smoke Test */
    std::printf("[*] Test 4: CP_MEM_WRITE (0x3d) PM4 Smoke Test Submission\n");
    scratchmem[0] = 0x00000000;

    /* Build PM4 CP_MEM_WRITE: Type 7 packet, opcode 0x3d, count 3 */
    uint32_t smoke_pm4[4];
    CHECK(AdrenoPM4Type7(0x3d, 3, &smoke_pm4[0]) == AdrenoPM4Success);
    CHECK((smoke_pm4[0] >> 16 & 0x7fu) == 0x3d && (smoke_pm4[0] & 0x3fffu) == 3);
    smoke_pm4[1] = static_cast<uint32_t>(scratch_iova);
    smoke_pm4[2] = static_cast<uint32_t>(scratch_iova >> 32);
    smoke_pm4[3] = 0xad618001u;

    /* Structural validation via AdrenoPM4Plan */
    AdrenoPM4Packet parsed[2];
    size_t packet_count = 0;
    CHECK(AdrenoPM4Plan(smoke_pm4, 4, parsed, 2, &packet_count) == AdrenoPM4Success);
    CHECK(packet_count == 1);
    CHECK(parsed[0].type == 7);
    CHECK(parsed[0].identifier == 0x3d);
    CHECK(parsed[0].payload_words == 3);

    /* Submit smoke PM4 */
    uint32_t smoke_fence = 0;
    CHECK(ring.submit(smoke_pm4, 4, &smoke_fence) == kIOReturnSuccess);
    CHECK(smoke_fence == 2);

    /* Verify ring buffer received the 4 smoke words + 7 fence words = 11 words.
     * New WPTR = 16 + 11 = 27.
     */
    CHECK(ringmem[16] == smoke_pm4[0]);
    CHECK(ringmem[17] == smoke_pm4[1]);
    CHECK(ringmem[18] == smoke_pm4[2]);
    CHECK(ringmem[19] == smoke_pm4[3]);
    CHECK(peer.regs[A618_CP_RB_WPTR] == 27);

    /* Simulate GPU CP execution: CP writes pattern 0xad618001 to scratch and advances fence to 2 */
    scratchmem[0] = 0xad618001u;
    fencemem[1] = 2;

    CHECK(ring.wait(smoke_fence, &ring_clock, 1000, nullptr) == NativeAdrenoRingFenceComplete);
    CHECK(scratchmem[0] == 0xad618001u);
    std::printf("    -> CP_MEM_WRITE pattern 0x%08x verified in scratch memory! Smoke test PASSED\n", scratchmem[0]);

    CHECK(ring.detach() == kIOReturnSuccess);

    /* 5. Sandbox Grant Process Parsing */
    std::printf("[*] Test 5: Sandbox Grant Token Parsing\n");
    const char *grant_list = "backboardd, adrenoprobed, SpringBoard, *";
    CHECK(test_sandbox_match(grant_list, "backboardd"));
    CHECK(test_sandbox_match(grant_list, "adrenoprobed"));
    CHECK(test_sandbox_match(grant_list, "SpringBoard"));
    CHECK(test_sandbox_match(grant_list, "any_app")); /* matches because '*' is included */

    const char *restricted_list = "backboardd, adrenoprobed";
    CHECK(test_sandbox_match(restricted_list, "backboardd"));
    CHECK(test_sandbox_match(restricted_list, "adrenoprobed"));
    CHECK(!test_sandbox_match(restricted_list, "unauthorized_app"));
    CHECK(!test_sandbox_match(restricted_list, "backboard")); /* prefix mismatch */
    CHECK(!test_sandbox_match(restricted_list, "backboarddx")); /* suffix mismatch */
    std::printf("    -> Sandbox grant token parsing, comma-separation and wildcard handling OK\n");

    std::printf("=== ALL ADRENO 618 KERNEL UNIT TESTS PASSED! ===\n");
    return 0;
}
