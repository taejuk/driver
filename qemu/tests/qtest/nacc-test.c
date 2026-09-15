/*
 * QTest for the nacc virtual accelerator — 명세서 v0.3
 *
 * 단계 1: PCI 식별, BAR0, 레지스터 파일
 * 단계 2: 상태 머신 (T1/T2/T3/T5/T8/T9/T10) + "전이표에 없는 조합"
 * 단계 3: 커맨드 링 (T4/T6/T7/T11/T12), NOP, MEMCPY, writeback
 *
 * 검증 환경 전제:
 *   -machine virt,highmem-ecam=off
 *       기본값이면 ECAM 이 RAM 뒤 동적 주소로 올라가 0x3f000000 으로
 *       접근할 수 없다 (hw/arm/virt.c: vms->highmem_ecam = true).
 *   -net none
 *       기본 virtio-net 이 슬롯 1 을 차지한다.
 *   BAR 설정 시 BME(0x04) 필수 — 없으면 pci_dma_read 가 실패한다.
 *
 * step_one_desc() 는 정확히 EXEC_DELAY_MS 만큼만 전진시켜야 한다.
 * +1 을 더하면 매 스텝 1ms 씩 남아 누적되고, 어느 스텝에서 두 개가
 * 한꺼번에 처리된다. 콜백이 데드라인 시점의 클럭을 기준으로 다음
 * 타이머를 걸기 때문이다.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest-single.h"

/* ------------------------------------------------------------------
 * arm virt 메모리 맵 (hw/arm/virt.c base_memmap)
 * ------------------------------------------------------------------ */
#define VIRT_ECAM_BASE      0x3f000000
#define VIRT_PCIE_MMIO_BASE 0x10000000

#define NACC_SLOT           1
#define NACC_BDF_ECAM       (VIRT_ECAM_BASE + (NACC_SLOT << 15))
#define NACC_BAR0_ADDR      VIRT_PCIE_MMIO_BASE

#define NACC_QEMU_ARGS \
    "-machine virt,highmem-ecam=off -net none -device nacc,addr=1.0"

/* PCI config space */
#define PCI_CFG_VENDOR      0x00
#define PCI_CFG_DEVICE      0x02
#define PCI_CFG_COMMAND     0x04
#define PCI_CFG_REVISION    0x08
#define PCI_CFG_CLASS       0x0A
#define PCI_CFG_CAP_PTR     0x34
#define PCI_CFG_BAR0        0x10

#define PCI_CMD_MEM_SPACE   0x0002
#define PCI_CMD_BUS_MASTER  0x0004

/* §2.1 */
#define NACC_VENDOR_ID      0x1234
#define NACC_DEVICE_ID      0x11EA
#define NACC_REVISION_ID    0x01
#define NACC_CLASS_CODE     0x1200
#define NACC_BAR0_SIZE      0x1000

/* §5.1 */
#define R_ID                0x0000
#define R_CTRL              0x0008
#define R_STATUS            0x000C
#define R_IRQ_STATUS        0x0010
#define R_IRQ_MASK          0x0014
#define R_RING_BASE_LO      0x0020
#define R_RING_BASE_HI      0x0024
#define R_RING_SIZE         0x0028
#define R_RING_PROD         0x002C
#define R_RING_CONS         0x0030
#define R_ERR_CODE          0x0040

#define NACC_ID_VALUE       0x4E410001u

/* §3.1 상태 인코딩 */
#define ST_RESET            0
#define ST_INIT             1
#define ST_READY            2
#define ST_BUSY             3
#define ST_HALTING          4
#define ST_ERROR            5

/* §5.3 CTRL */
#define CTRL_ENABLE         (1u << 0)

/* §5.3 IRQ */
#define IRQ_CMD_DONE        (1u << 0)
#define IRQ_CMD_ERR         (1u << 1)
#define IRQ_ALL             (IRQ_CMD_DONE | IRQ_CMD_ERR)

/* §8.1 오류 코드 */
#define ERR_NONE            0x00
#define ERR_BAD_CONFIG      0x01
#define ERR_BAD_PROD        0x02
#define ERR_BAD_OPCODE      0x03
#define ERR_BAD_LENGTH      0x04
#define ERR_BAD_ALIGN       0x05
#define ERR_DMA_FAULT       0x06

/* §6.1 디스크립터 */
#define DESC_SIZE           32
#define DESC_OFF_STATUS     0x1C

/* §6.5 opcode */
#define OP_NOP              0x00
#define OP_MEMCPY           0x01

/* §6.1 NACC-REQ-035 — MEMCPY 제약 */
#define MAX_XFER            0x100000u   /* 1 MiB */
#define XFER_CHUNK          4096        /* 디바이스의 바운스 버퍼 크기 */

/* 유효한 링 설정 — 32바이트 정렬, 2의 거듭제곱 */
#define RING_ADDR           0x40000000ULL
#define RING_SZ             8

/* MEMCPY 버퍼용 영역. 링과 겹치지 않게 */
#define SRC_ADDR            (RING_ADDR + 0x10000)
#define DST_ADDR            (RING_ADDR + 0x20000)

/* RAM 밖 주소. DMA_FAULT 유발용 */
#define BAD_ADDR            0xFFFF000000000000ULL

/*
 * 디바이스 구현의 지연 상수와 맞춰야 한다.
 * 한쪽만 바꾸면 테스트가 깨진다.
 *   NACC_HALT_DELAY_MS = 20
 *   NACC_EXEC_DELAY_MS = 2
 */
#define HALT_DELAY_MS       20
#define EXEC_DELAY_MS       2
#define MS_IN_NS            1000000LL

/* writeback 이 실제로 일어났는지 보려고 미리 심어두는 값 */
#define STATUS_POISON       0xA5A5A5A5u

/* 목적지 버퍼를 미리 채워 두는 값. 침범을 검출한다 */
#define DST_POISON          0xEE

/* §5.1 정의되지 않은 오프셋 (NACC-REQ-021) */
static const uint32_t undefined_offsets[] = {
    0x0004, 0x0018, 0x001C, 0x0034, 0x0038, 0x003C, 0x0044, 0x0800, 0x0FFC,
};

typedef struct QEMU_PACKED TestDesc {
    uint32_t opcode;
    uint32_t flags;
    uint64_t src_addr;
    uint64_t dst_addr;
    uint32_t length;
    uint32_t status;
} TestDesc;

/* ------------------------------------------------------------------
 * 헬퍼
 * ------------------------------------------------------------------ */

static uint32_t nacc_readl(uint32_t off)
{
    return readl(NACC_BAR0_ADDR + off);
}

static void nacc_writel(uint32_t off, uint32_t val)
{
    writel(NACC_BAR0_ADDR + off, val);
}

static uint32_t nacc_state(void)
{
    return nacc_readl(R_STATUS) & 0x7;
}

/*
 * qtest 는 펌웨어를 실행하지 않으므로 BAR 가 미할당 상태다.
 * 테스트가 직접 BAR 를 프로그래밍하고 MSE / BME 를 켠다.
 * BME 가 없으면 pci_dma_read 가 실패해 전부 DMA_FAULT 가 난다.
 */
static void nacc_setup_bar(void)
{
    uint16_t cmd;

    writel(NACC_BDF_ECAM + PCI_CFG_BAR0, NACC_BAR0_ADDR);
    cmd = readw(NACC_BDF_ECAM + PCI_CFG_COMMAND);
    writew(NACC_BDF_ECAM + PCI_CFG_COMMAND,
           cmd | PCI_CMD_MEM_SPACE | PCI_CMD_BUS_MASTER);
}

/* RESET 상태로 시작 */
static void nacc_start(void)
{
    qtest_start(NACC_QEMU_ARGS);
    nacc_setup_bar();
    g_assert_cmpuint(nacc_state(), ==, ST_RESET);
}

/* T1: RESET -> INIT. 유효한 주소를 래치한다 */
static void nacc_to_init(void)
{
    nacc_writel(R_RING_BASE_LO, (uint32_t)RING_ADDR);
    nacc_writel(R_RING_BASE_HI, (uint32_t)(RING_ADDR >> 32));
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);
}

/* T2: INIT -> READY */
static void nacc_to_ready(void)
{
    nacc_to_init();
    nacc_writel(R_RING_SIZE, RING_SZ);
    nacc_writel(R_CTRL, CTRL_ENABLE);
    g_assert_cmpuint(nacc_state(), ==, ST_READY);
}

/* 가상 클럭을 ms 단위로 전진시킨다 */
static void nacc_clock_step_ms(int64_t ms)
{
    qtest_clock_step(global_qtest, ms * MS_IN_NS);
}

/* 디스크립터 하나가 처리될 만큼만 전진. 파일 상단 주석 참조 */
static void step_one_desc(void)
{
    nacc_clock_step_ms(EXEC_DELAY_MS);
}

static uint64_t ring_slot_addr(uint32_t slot)
{
    return RING_ADDR + (uint64_t)slot * DESC_SIZE;
}

/* 한 슬롯에 디스크립터를 쓴다. status 는 포이즌으로 채워 둔다 */
static void ring_put(uint32_t slot, uint32_t opcode,
                     uint64_t src, uint64_t dst, uint32_t len)
{
    TestDesc d = {
        .opcode   = cpu_to_le32(opcode),
        .flags    = 0,                      /* NACC-REQ-032 */
        .src_addr = cpu_to_le64(src),
        .dst_addr = cpu_to_le64(dst),
        .length   = cpu_to_le32(len),
        .status   = cpu_to_le32(STATUS_POISON),
    };

    qtest_memwrite(global_qtest, ring_slot_addr(slot), &d, sizeof(d));
}

static uint32_t ring_status(uint32_t slot)
{
    uint32_t st;

    qtest_memread(global_qtest, ring_slot_addr(slot) + DESC_OFF_STATUS,
                  &st, sizeof(st));
    return le32_to_cpu(st);
}

/* 링 전체를 NOP 으로 채운다 */
static void ring_fill_nop(uint32_t count)
{
    for (uint32_t i = 0; i < count; i++) {
        ring_put(i, OP_NOP, 0, 0, 0);
    }
}

/* ---- MEMCPY 용 메모리 헬퍼 ---------------------------------------- */

/* addr 부터 len 바이트를 (seed + i) 패턴으로 채운다 */
static void mem_fill_pattern(uint64_t addr, uint32_t len, uint8_t seed)
{
    g_autofree uint8_t *buf = g_malloc(len);

    for (uint32_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(seed + i);
    }
    qtest_memwrite(global_qtest, addr, buf, len);
}

/* addr 부터 len 바이트가 (seed + i) 패턴인지 확인한다 */
static void mem_check_pattern(uint64_t addr, uint32_t len, uint8_t seed)
{
    g_autofree uint8_t *buf = g_malloc(len);

    qtest_memread(global_qtest, addr, buf, len);
    for (uint32_t i = 0; i < len; i++) {
        if (buf[i] != (uint8_t)(seed + i)) {
            g_error("pattern mismatch at +0x%x: got 0x%02x want 0x%02x",
                    i, buf[i], (uint8_t)(seed + i));
        }
    }
}

/* addr 부터 len 바이트를 같은 값으로 채운다 */
static void mem_fill_byte(uint64_t addr, uint32_t len, uint8_t v)
{
    g_autofree uint8_t *buf = g_malloc(len);

    memset(buf, v, len);
    qtest_memwrite(global_qtest, addr, buf, len);
}

/* addr 부터 len 바이트가 전부 v 인지 확인한다 */
static void mem_check_byte(uint64_t addr, uint32_t len, uint8_t v)
{
    g_autofree uint8_t *buf = g_malloc(len);

    qtest_memread(global_qtest, addr, buf, len);
    for (uint32_t i = 0; i < len; i++) {
        if (buf[i] != v) {
            g_error("unexpected write at +0x%x: got 0x%02x want 0x%02x",
                    i, buf[i], v);
        }
    }
}

/*
 * MEMCPY 하나를 제출하고 완료될 때까지 돌린다.
 * 반환값은 최종 상태 (ST_READY 또는 ST_ERROR).
 */
static uint32_t run_one_memcpy(uint64_t src, uint64_t dst, uint32_t len,
                               uint32_t max_steps)
{
    ring_put(0, OP_MEMCPY, src, dst, len);
    nacc_to_ready();
    nacc_writel(R_RING_PROD, 1);

    for (uint32_t i = 0; i < max_steps; i++) {
        if (nacc_state() != ST_BUSY) {
            break;
        }
        step_one_desc();
    }
    return nacc_state();
}

/* ==================================================================
 * 단계 1 — PCI / 레지스터 파일
 * ================================================================== */

static void test_pci_identity(void)
{
    qtest_start(NACC_QEMU_ARGS);

    g_assert_cmphex(readw(NACC_BDF_ECAM + PCI_CFG_VENDOR), ==, NACC_VENDOR_ID);
    g_assert_cmphex(readw(NACC_BDF_ECAM + PCI_CFG_DEVICE), ==, NACC_DEVICE_ID);
    g_assert_cmphex(readb(NACC_BDF_ECAM + PCI_CFG_REVISION), ==,
                    NACC_REVISION_ID);
    g_assert_cmphex(readw(NACC_BDF_ECAM + PCI_CFG_CLASS), ==, NACC_CLASS_CODE);

    /* NACC-REQ-004: MSI capability 존재 */
    g_assert_cmpuint(readb(NACC_BDF_ECAM + PCI_CFG_CAP_PTR), !=, 0);

    qtest_end();
}

static void test_bar0_size(void)
{
    uint32_t bar, mask, size;

    qtest_start(NACC_QEMU_ARGS);

    writel(NACC_BDF_ECAM + PCI_CFG_BAR0, 0xFFFFFFFF);
    bar = readl(NACC_BDF_ECAM + PCI_CFG_BAR0);

    mask = bar & 0xFFFFFFF0u;
    size = ~mask + 1;
    g_assert_cmphex(size, ==, NACC_BAR0_SIZE);

    /* NACC-REQ-002: memory space, 32bit, non-prefetchable */
    g_assert_cmphex(bar & 0x1, ==, 0);
    g_assert_cmphex((bar >> 1) & 0x3, ==, 0);
    g_assert_cmphex((bar >> 3) & 0x1, ==, 0);

    qtest_end();
}

static void test_id_register(void)
{
    nacc_start();
    g_assert_cmphex(nacc_readl(R_ID), ==, NACC_ID_VALUE);
    qtest_end();
}

static void test_reset_values(void)
{
    nacc_start();

    g_assert_cmphex(nacc_readl(R_ID),           ==, NACC_ID_VALUE);
    g_assert_cmphex(nacc_readl(R_CTRL),         ==, 0);
    g_assert_cmphex(nacc_readl(R_STATUS),       ==, ST_RESET);
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS),   ==, 0);
    /* 리셋 시 전부 마스킹 — 유일하게 0 이 아닌 레지스터 */
    g_assert_cmphex(nacc_readl(R_IRQ_MASK),     ==, IRQ_ALL);
    g_assert_cmphex(nacc_readl(R_RING_BASE_LO), ==, 0);
    g_assert_cmphex(nacc_readl(R_RING_BASE_HI), ==, 0);
    g_assert_cmphex(nacc_readl(R_RING_SIZE),    ==, 0);
    g_assert_cmphex(nacc_readl(R_RING_PROD),    ==, 0);
    g_assert_cmphex(nacc_readl(R_RING_CONS),    ==, 0);
    g_assert_cmphex(nacc_readl(R_ERR_CODE),     ==, 0);

    qtest_end();
}

/*
 * NACC-REQ-019 — 쓰기가 허용된 레지스터는 예약 비트까지 보존한다.
 *
 * CTRL 은 여기서 다루지 않는다. RESET 에서는 §5.2 에 따라 값이 저장되지
 * 않고, ENABLE 비트가 켜진 값을 쓰면 T10 이 발동한다.
 */
static void test_rw_registers_preserve_reserved(void)
{
    const uint32_t pattern = 0xDEADBEEFu;

    nacc_start();

    nacc_writel(R_IRQ_MASK, pattern);
    g_assert_cmphex(nacc_readl(R_IRQ_MASK), ==, pattern);

    /* RESET 에서 RING_* 는 쓰기 가능 (§5.2) */
    nacc_writel(R_RING_BASE_LO, pattern);
    g_assert_cmphex(nacc_readl(R_RING_BASE_LO), ==, pattern);

    nacc_writel(R_RING_SIZE, pattern);
    g_assert_cmphex(nacc_readl(R_RING_SIZE), ==, pattern);

    qtest_end();
}

/*
 * CTRL 의 예약 비트 [31:1] 보존.
 * INIT 상태에서 ENABLE=0 인 값을 쓴다 — 전이를 유발하지 않는 패턴.
 */
static void test_ctrl_preserves_reserved(void)
{
    const uint32_t pattern = 0xDEADBEEEu;    /* bit0 = 0 */

    nacc_start();
    nacc_to_init();

    nacc_writel(R_CTRL, pattern);
    g_assert_cmphex(nacc_readl(R_CTRL), ==, pattern);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);

    qtest_end();
}

static void test_ro_registers_ignore_writes(void)
{
    nacc_start();

    nacc_writel(R_ID, 0xDEADBEEF);
    g_assert_cmphex(nacc_readl(R_ID), ==, NACC_ID_VALUE);

    nacc_writel(R_STATUS, 0xDEADBEEF);
    g_assert_cmphex(nacc_readl(R_STATUS), ==, ST_RESET);

    nacc_writel(R_RING_CONS, 0xDEADBEEF);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 0);

    nacc_writel(R_ERR_CODE, 0xDEADBEEF);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, 0);

    qtest_end();
}

static void test_undefined_offsets(void)
{
    nacc_start();

    for (size_t i = 0; i < ARRAY_SIZE(undefined_offsets); i++) {
        uint32_t off = undefined_offsets[i];

        g_assert_cmphex(nacc_readl(off), ==, 0);
        nacc_writel(off, 0xDEADBEEF);
        g_assert_cmphex(nacc_readl(off), ==, 0);
    }

    /* 미매핑 쓰기가 정의된 레지스터를 오염시키지 않았는지 */
    g_assert_cmphex(nacc_readl(R_ID), ==, NACC_ID_VALUE);
    g_assert_cmphex(nacc_readl(R_CTRL), ==, 0);
    g_assert_cmphex(nacc_readl(R_IRQ_MASK), ==, IRQ_ALL);
    g_assert_cmpuint(nacc_state(), ==, ST_RESET);

    qtest_end();
}

/* NACC-REQ-048: RW1C. 이벤트가 없는 상태에서 1 을 써도 비트가 서지 않는다 */
static void test_irq_status_rw1c_shape(void)
{
    nacc_start();

    nacc_writel(R_IRQ_STATUS, 0xFFFFFFFF);
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS), ==, 0);

    nacc_writel(R_IRQ_STATUS, 0);
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS), ==, 0);

    qtest_end();
}

/* ==================================================================
 * 단계 2 — 상태 머신
 * ================================================================== */

/*
 * T1 — RESET -> INIT 는 RING_BASE_HI 쓰기로만 일어난다.
 * NACC-REQ-011: LO 는 섀도에만 반영되므로 전이를 유발하지 않는다.
 */
static void test_t1_latch_on_hi(void)
{
    nacc_start();

    nacc_writel(R_RING_BASE_LO, (uint32_t)RING_ADDR);
    g_assert_cmpuint(nacc_state(), ==, ST_RESET);
    g_assert_cmphex(nacc_readl(R_RING_BASE_LO), ==, (uint32_t)RING_ADDR);

    nacc_writel(R_RING_BASE_HI, (uint32_t)(RING_ADDR >> 32));
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);        /* T1 */

    qtest_end();
}

/* INIT 에서 RING_BASE_HI 를 다시 써도 전이는 없다 */
static void test_t1_rewrite_in_init(void)
{
    nacc_start();
    nacc_to_init();

    nacc_writel(R_RING_BASE_LO, 0x50000000);
    nacc_writel(R_RING_BASE_HI, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);
    g_assert_cmphex(nacc_readl(R_RING_BASE_LO), ==, 0x50000000);

    qtest_end();
}

/* T2 — 유효 설정. §3.2 부수효과까지 확인 */
static void test_t2_enable_valid(void)
{
    nacc_start();
    nacc_to_init();

    nacc_writel(R_RING_SIZE, RING_SZ);
    nacc_writel(R_CTRL, CTRL_ENABLE);

    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 0);
    g_assert_cmphex(nacc_readl(R_RING_PROD), ==, 0);
    g_assert_cmphex(nacc_readl(R_ERR_CODE),  ==, ERR_NONE);

    qtest_end();
}

/* §3.5 / NACC-REQ-036 — RING_SIZE 경계값 */
static void test_ring_size_boundaries(void)
{
    const struct {
        uint32_t size;
        bool valid;
        const char *why;
    } cases[] = {
        { 0,   false, "0" },
        { 1,   false, "하한 미만" },
        { 2,   true,  "하한" },
        { 3,   false, "2의 거듭제곱 아님" },
        { 4,   true,  "정상" },
        { 127, false, "2의 거듭제곱 아님" },
        { 128, true,  "상한" },
        { 129, false, "상한 초과" },
        { 256, false, "[7:0] 밖으로 넘침 -> 0 으로 읽힘" },
    };

    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        nacc_start();
        nacc_to_init();

        nacc_writel(R_RING_SIZE, cases[i].size);
        nacc_writel(R_CTRL, CTRL_ENABLE);

        if (cases[i].valid) {
            g_assert_cmpuint(nacc_state(), ==, ST_READY);
        } else {
            g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
            g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);
        }

        qtest_end();
    }
}

/* T3 — RING_BASE 가 32바이트 미정렬 (NACC-REQ-037) */
static void test_t3_unaligned_base(void)
{
    nacc_start();

    nacc_writel(R_RING_BASE_LO, (uint32_t)RING_ADDR | 0x10);
    nacc_writel(R_RING_BASE_HI, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);

    nacc_writel(R_RING_SIZE, RING_SZ);
    nacc_writel(R_CTRL, CTRL_ENABLE);

    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);

    /* §3.3 INIT 항목 3: 설정값은 지워지지 않는다 */
    g_assert_cmphex(nacc_readl(R_RING_SIZE), ==, RING_SZ);

    qtest_end();
}

/* T3 — RING_BASE 가 0 */
static void test_t3_zero_base(void)
{
    nacc_start();

    nacc_writel(R_RING_BASE_LO, 0);
    nacc_writel(R_RING_BASE_HI, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);        /* T1 은 일어난다 */

    nacc_writel(R_RING_SIZE, RING_SZ);
    nacc_writel(R_CTRL, CTRL_ENABLE);

    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);

    qtest_end();
}

/*
 * NACC-REQ-098 — 유효성 판정은 래치된 값을 본다.
 *
 * LO 에 미정렬 주소를 쓰되 HI 를 쓰지 않으면 래치는 갱신되지 않는다.
 * 섀도를 보는 구현이면 여기서 ERROR 가 나므로 걸린다.
 */
static void test_validation_uses_latched_base(void)
{
    nacc_start();
    nacc_to_init();

    nacc_writel(R_RING_BASE_LO, (uint32_t)RING_ADDR | 0x1F);

    nacc_writel(R_RING_SIZE, RING_SZ);
    nacc_writel(R_CTRL, CTRL_ENABLE);

    g_assert_cmpuint(nacc_state(), ==, ST_READY);

    qtest_end();
}

/* T10 — RESET 에서 ENABLE<-1 */
static void test_t10_enable_in_reset(void)
{
    nacc_start();

    nacc_writel(R_CTRL, CTRL_ENABLE);

    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);
    /* §5.2 각주 1: RESET 에서 CTRL 값은 저장되지 않는다 */
    g_assert_cmphex(nacc_readl(R_CTRL), ==, 0);

    qtest_end();
}

/*
 * T8 + T9 — 정지 시퀀스.
 * HALTING 에 실제로 지연이 있어야 한다. 지연이 0 이면 드라이버의
 * 폴링 경로가 한 번도 실행되지 않는다 (부록 C 단계 5).
 */
static void test_t8_t9_halting(void)
{
    nacc_start();
    nacc_to_ready();

    nacc_writel(R_CTRL, 0);                              /* T8 */
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);

    /* 절반만 전진시키면 아직 HALTING */
    nacc_clock_step_ms(HALT_DELAY_MS / 2);
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);

    nacc_clock_step_ms(HALT_DELAY_MS);                   /* T9 */
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);

    /* §3.2 T9: 링 설정값은 보존된다 */
    g_assert_cmphex(nacc_readl(R_RING_SIZE), ==, RING_SZ);
    g_assert_cmphex(nacc_readl(R_RING_BASE_LO), ==, (uint32_t)RING_ADDR);

    qtest_end();
}

/* HALTING 중에는 ENABLE<-1 이 무시된다 (§3.3 HALTING 항목 3) */
static void test_halting_ignores_enable(void)
{
    nacc_start();
    nacc_to_ready();

    nacc_writel(R_CTRL, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);

    nacc_writel(R_CTRL, CTRL_ENABLE);
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);      /* 무시 */

    nacc_clock_step_ms(HALT_DELAY_MS + 1);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);

    qtest_end();
}

/*
 * §8.2 NACC-REQ-051 — 오류 복구 전체 경로.
 * NACC-REQ-093: ERR_CODE 는 T2 가 일어날 때까지 보존된다.
 */
static void test_error_recovery_path(void)
{
    nacc_start();

    nacc_writel(R_CTRL, CTRL_ENABLE);                    /* T10 */
    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);

    nacc_writel(R_CTRL, 0);                              /* T8 */
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);

    nacc_clock_step_ms(HALT_DELAY_MS + 1);               /* T9 */
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);

    nacc_writel(R_RING_BASE_LO, (uint32_t)RING_ADDR);
    nacc_writel(R_RING_BASE_HI, (uint32_t)(RING_ADDR >> 32));
    nacc_writel(R_RING_SIZE, RING_SZ);
    nacc_writel(R_CTRL, CTRL_ENABLE);                    /* T2 */

    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_NONE);

    qtest_end();
}

/*
 * T5 — READY 에서 RING_PROD 범위 초과.
 * NACC-REQ-075: >= 비교. RING_SZ=8 이면 유효 범위는 0..7.
 */
static void test_t5_bad_prod(void)
{
    nacc_start();
    nacc_to_ready();

    nacc_writel(R_RING_PROD, RING_SZ);                   /* 경계값 */

    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_PROD);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 0);

    qtest_end();
}

/* RING_SZ-1 은 유효하다. T4 가 일어나 BUSY 가 된다 */
static void test_prod_max_valid(void)
{
    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    nacc_writel(R_RING_PROD, RING_SZ - 1);

    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);
    g_assert_cmphex(nacc_readl(R_RING_PROD), ==, RING_SZ - 1);

    qtest_end();
}

/* §5.2 — READY 에서 링 설정 레지스터는 R */
static void test_ring_config_locked_in_ready(void)
{
    nacc_start();
    nacc_to_ready();

    nacc_writel(R_RING_SIZE, 64);
    g_assert_cmphex(nacc_readl(R_RING_SIZE), ==, RING_SZ);

    nacc_writel(R_RING_BASE_LO, 0xCAFE0000);
    g_assert_cmphex(nacc_readl(R_RING_BASE_LO), ==, (uint32_t)RING_ADDR);

    nacc_writel(R_RING_BASE_HI, 0xCAFE);
    g_assert_cmphex(nacc_readl(R_RING_BASE_HI), ==,
                    (uint32_t)(RING_ADDR >> 32));

    g_assert_cmpuint(nacc_state(), ==, ST_READY);

    qtest_end();
}

/* §3.2 "전이표에 없는 조합" — RESET / INIT 에서 doorbell 은 무시된다 */
static void test_doorbell_ignored_before_ready(void)
{
    nacc_start();

    nacc_writel(R_RING_PROD, 1);
    g_assert_cmphex(nacc_readl(R_RING_PROD), ==, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_RESET);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_NONE);

    nacc_to_init();

    nacc_writel(R_RING_PROD, 1);
    g_assert_cmphex(nacc_readl(R_RING_PROD), ==, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_NONE);

    qtest_end();
}

/* RESET / INIT 에서 ENABLE<-0 은 무시된다 */
static void test_disable_ignored_when_inactive(void)
{
    nacc_start();

    nacc_writel(R_CTRL, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_RESET);

    nacc_to_init();
    nacc_writel(R_CTRL, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);

    qtest_end();
}

/*
 * 이미 활성인 상태에서 ENABLE<-1 을 또 써도 무시된다.
 *
 * 단계 3 에서 doorbell 이 T4 를 유발하게 되었으므로 BUSY 에서 확인한다.
 * T2 가 다시 일어났다면 prod/cons 가 0 으로 밀리고 READY 가 되었을 것이다.
 */
static void test_redundant_enable_ignored(void)
{
    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 3);                         /* T4 -> BUSY */
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);

    nacc_writel(R_CTRL, CTRL_ENABLE);                    /* 무시되어야 함 */

    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);
    g_assert_cmphex(nacc_readl(R_RING_PROD), ==, 3);

    qtest_end();
}

/* NACC-REQ-023 — ID 는 모든 상태에서 같은 값 */
static void test_id_stable_across_states(void)
{
    nacc_start();
    g_assert_cmphex(nacc_readl(R_ID), ==, NACC_ID_VALUE);

    nacc_to_init();
    g_assert_cmphex(nacc_readl(R_ID), ==, NACC_ID_VALUE);

    nacc_to_ready();
    g_assert_cmphex(nacc_readl(R_ID), ==, NACC_ID_VALUE);

    nacc_writel(R_CTRL, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);
    g_assert_cmphex(nacc_readl(R_ID), ==, NACC_ID_VALUE);

    qtest_end();
}

/* ==================================================================
 * 단계 3 — 커맨드 링 (NOP)
 * ================================================================== */

/*
 * RING_ADDR 이 실제 RAM 인지 먼저 확인한다.
 * 여기가 실패하면 이후 링 테스트가 전부 DMA_FAULT 로 무너진다.
 */
static void test_ram_sanity(void)
{
    qtest_start(NACC_QEMU_ARGS);

    qtest_writel(global_qtest, RING_ADDR, 0xDEADBEEF);
    g_assert_cmphex(qtest_readl(global_qtest, RING_ADDR), ==, 0xDEADBEEF);

    qtest_writel(global_qtest, SRC_ADDR, 0xCAFEBABE);
    g_assert_cmphex(qtest_readl(global_qtest, SRC_ADDR), ==, 0xCAFEBABE);

    qtest_writel(global_qtest, DST_ADDR, 0x12345678);
    g_assert_cmphex(qtest_readl(global_qtest, DST_ADDR), ==, 0x12345678);

    qtest_end();
}

/*
 * T4 -> T6 왕복. 디스크립터 하나.
 *
 * ST_BUSY 단언이 핵심이다. 디바이스가 doorbell 핸들러 안에서
 * 즉시 처리해 버리면 여기서 걸린다.
 */
static void test_ring_nop_single(void)
{
    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 1);                        /* T4 */
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 0);    /* 아직 처리 전 */

    step_one_desc();

    g_assert_cmpuint(nacc_state(), ==, ST_READY);       /* T6 */
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 1);
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS), ==, IRQ_CMD_DONE);
    /* NACC-REQ-033: status writeback */
    g_assert_cmphex(ring_status(0), ==, ERR_NONE);

    qtest_end();
}

/*
 * 여러 개를 제출하고 한 칸씩 처리되는지 본다.
 * 완료 인터럽트는 마지막에 한 번만 나야 한다 (§3.2 T6).
 */
static void test_ring_nop_stepwise(void)
{
    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 3);
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);

    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 1);
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);
    /* 디스크립터마다 인터럽트가 나면 안 된다 */
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS), ==, 0);

    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 2);
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS), ==, 0);

    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 3);
    g_assert_cmpuint(nacc_state(), ==, ST_READY);       /* T6 */
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS), ==, IRQ_CMD_DONE);

    /* 처리한 슬롯만 writeback */
    g_assert_cmphex(ring_status(0), ==, ERR_NONE);
    g_assert_cmphex(ring_status(1), ==, ERR_NONE);
    g_assert_cmphex(ring_status(2), ==, ERR_NONE);
    g_assert_cmphex(ring_status(3), ==, STATUS_POISON); /* 미처리 */

    qtest_end();
}

/*
 * wrap-around. 인덱스 마스크 연산이 맞는지 확인한다.
 * RING_SZ=8 에서 cons 6 -> 7 -> 0 -> 1 -> 2 로 0 을 통과한다.
 */
static void test_ring_wraparound(void)
{
    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    /* 1라운드: 슬롯 0..5 */
    nacc_writel(R_RING_PROD, 6);
    for (int i = 0; i < 6; i++) {
        step_one_desc();
    }
    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 6);

    /* 슬롯을 다시 채우고 2라운드 */
    ring_fill_nop(RING_SZ);
    nacc_writel(R_IRQ_STATUS, IRQ_CMD_DONE);            /* ack */

    nacc_writel(R_RING_PROD, 2);                        /* T4 */
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);

    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 7);

    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 0);    /* 여기서 순환 */

    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 1);

    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 2);
    g_assert_cmpuint(nacc_state(), ==, ST_READY);

    g_assert_cmphex(ring_status(7), ==, ERR_NONE);
    g_assert_cmphex(ring_status(0), ==, ERR_NONE);
    g_assert_cmphex(ring_status(1), ==, ERR_NONE);

    qtest_end();
}

/*
 * NACC-REQ-045 — NOP 은 src/dst/length 를 검증하지 않는다.
 * 검증 루틴을 opcode 판별 앞에 두면 여기서 걸린다 (NACC-REQ-055).
 */
static void test_nop_ignores_fields(void)
{
    nacc_start();

    /* 정렬도 안 맞고 길이도 범위 밖인 쓰레기 값 */
    ring_put(0, OP_NOP, 0x1, 0x3, 0xFFFFFFFF);

    nacc_to_ready();
    nacc_writel(R_RING_PROD, 1);
    step_one_desc();

    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_NONE);
    g_assert_cmphex(ring_status(0), ==, ERR_NONE);

    qtest_end();
}

/*
 * T11 — BUSY 중에 prod 를 늘리면 이어서 처리된다.
 * 매 콜백이 새 prod 를 보므로 별도 처리 없이 성립한다.
 */
static void test_t11_append_during_busy(void)
{
    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 2);
    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 1);
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);

    nacc_writel(R_RING_PROD, 5);                        /* T11 */
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);

    for (int i = 0; i < 4; i++) {
        step_one_desc();
    }

    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 5);
    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    g_assert_cmphex(ring_status(4), ==, ERR_NONE);

    qtest_end();
}

/* T12 — BUSY 중에 범위 밖 prod */
static void test_t12_bad_prod_during_busy(void)
{
    uint32_t cons;

    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 4);
    step_one_desc();
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);

    nacc_writel(R_RING_PROD, RING_SZ);                  /* 무효 */

    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_PROD);

    /* ERROR 진입 후에는 처리가 멈춰야 한다 */
    cons = nacc_readl(R_RING_CONS);
    step_one_desc();
    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, cons);

    qtest_end();
}

/*
 * T7 — 정의되지 않은 opcode.
 *
 * NACC-REQ-078: RING_CONS 가 실패한 디스크립터를 가리킨 채 고정된다.
 * NACC-REQ-050: 실패한 슬롯의 status 에 오류 코드가 기록된다.
 *               BAD_OPCODE 분기도 nacc_fail_descriptor() 를 써야 한다.
 */
static void test_t7_bad_opcode(void)
{
    nacc_start();
    ring_fill_nop(RING_SZ);
    ring_put(2, 0x99, 0, 0, 0);                         /* 슬롯 2 만 불량 */
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 5);

    step_one_desc();                                    /* 슬롯 0 */
    step_one_desc();                                    /* 슬롯 1 */
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);

    step_one_desc();                                    /* 슬롯 2 -> T7 */
    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_OPCODE);
    /* 실패한 슬롯을 가리킨 채 고정 — 다음 슬롯이 아니다 */
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 2);
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS) & IRQ_CMD_ERR, ==, IRQ_CMD_ERR);
    /* NACC-REQ-050 */
    g_assert_cmphex(ring_status(2), ==, ERR_BAD_OPCODE);

    /* 이후 슬롯은 건드리지 않는다 */
    g_assert_cmphex(ring_status(3), ==, STATUS_POISON);
    g_assert_cmphex(ring_status(4), ==, STATUS_POISON);

    /* 더 이상 진행하지 않는다 */
    step_one_desc();
    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 2);

    qtest_end();
}

/*
 * T8 during BUSY — 진행 중 명령 폐기.
 *
 * NACC-REQ-090: HALTING 진입 후 새 메모리 요청을 발행하지 않는다.
 *               exec_timer 를 끄지 않으면 cons 가 계속 오른다.
 * NACC-REQ-094: 폐기된 명령에 완료 인터럽트가 없다.
 * NACC-REQ-095: HALTING 진입 시점의 cons 가 폐기 경계다.
 */
static void test_t8_abort_during_busy(void)
{
    uint32_t boundary;

    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 6);
    step_one_desc();
    step_one_desc();
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);

    boundary = nacc_readl(R_RING_CONS);
    g_assert_cmphex(boundary, ==, 2);

    nacc_writel(R_CTRL, 0);                             /* T8 */
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);

    /* HALTING 동안 처리가 이어지면 안 된다 */
    nacc_clock_step_ms(EXEC_DELAY_MS * 4);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, boundary);
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);

    nacc_clock_step_ms(HALT_DELAY_MS + 1);              /* T9 */
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);

    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, boundary);
    /* 폐기된 명령에 완료 인터럽트가 없다 */
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS) & IRQ_CMD_DONE, ==, 0);
    /* 폐기된 슬롯은 writeback 되지 않았다 */
    g_assert_cmphex(ring_status(boundary), ==, STATUS_POISON);

    /* T9 이후에도 진행되지 않는다 */
    nacc_clock_step_ms(EXEC_DELAY_MS * 4);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, boundary);

    qtest_end();
}

/* 폐기 후 재활성화. T2 가 cons/prod 를 0 으로 되돌린다 (NACC-REQ-076) */
static void test_restart_after_abort(void)
{
    nacc_start();
    ring_fill_nop(RING_SZ);
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 6);
    step_one_desc();
    nacc_writel(R_CTRL, 0);
    nacc_clock_step_ms(HALT_DELAY_MS + 1);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);

    nacc_writel(R_CTRL, CTRL_ENABLE);                   /* T2 */
    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 0);
    g_assert_cmphex(nacc_readl(R_RING_PROD), ==, 0);

    /* 다시 정상 동작하는지 */
    ring_fill_nop(RING_SZ);
    nacc_writel(R_RING_PROD, 1);
    step_one_desc();
    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 1);

    qtest_end();
}

/*
 * DMA_FAULT — 링 베이스를 RAM 밖으로 잡는다.
 *
 * §3.5 유효성 판정은 정렬과 0 여부만 보므로 READY 까지는 간다.
 * doorbell 을 치는 순간 페치가 실패해야 한다 (§6.6 1번).
 */
static void test_dma_fault_out_of_ram(void)
{
    nacc_start();

    nacc_writel(R_RING_BASE_LO, (uint32_t)BAD_ADDR);
    nacc_writel(R_RING_BASE_HI, (uint32_t)(BAD_ADDR >> 32));
    nacc_writel(R_RING_SIZE, RING_SZ);
    nacc_writel(R_CTRL, CTRL_ENABLE);
    g_assert_cmpuint(nacc_state(), ==, ST_READY);       /* 주소는 검증 못 함 */

    nacc_writel(R_RING_PROD, 1);
    step_one_desc();

    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_DMA_FAULT);

    qtest_end();
}

/* ==================================================================
 * 단계 3 — MEMCPY
 * ================================================================== */

/* 기본 복사. 64바이트 */
static void test_memcpy_basic(void)
{
    const uint32_t len = 64;

    nacc_start();

    mem_fill_pattern(SRC_ADDR, len, 0x10);
    mem_fill_byte(DST_ADDR, len, DST_POISON);

    g_assert_cmpuint(run_one_memcpy(SRC_ADDR, DST_ADDR, len, 8),
                     ==, ST_READY);

    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 1);
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS), ==, IRQ_CMD_DONE);
    g_assert_cmphex(ring_status(0), ==, ERR_NONE);

    mem_check_pattern(DST_ADDR, len, 0x10);

    qtest_end();
}

/*
 * 청크 경계를 넘는 전송.
 * 디바이스는 XFER_CHUNK(4096) 단위 바운스 버퍼로 나눠 옮긴다.
 * 경계 계산이 틀리면 여기서 걸린다.
 */
static void test_memcpy_multi_chunk(void)
{
    const uint32_t len = XFER_CHUNK * 2;    /* 8 KiB */

    nacc_start();

    mem_fill_pattern(SRC_ADDR, len, 0x37);
    mem_fill_byte(DST_ADDR, len, DST_POISON);

    g_assert_cmpuint(run_one_memcpy(SRC_ADDR, DST_ADDR, len, 8),
                     ==, ST_READY);

    mem_check_pattern(DST_ADDR, len, 0x37);

    qtest_end();
}

/* 청크 경계에 딱 걸치지 않는 길이 */
static void test_memcpy_odd_chunk(void)
{
    const uint32_t len = XFER_CHUNK + 4;

    nacc_start();

    mem_fill_pattern(SRC_ADDR, len, 0x5A);
    mem_fill_byte(DST_ADDR, len, DST_POISON);

    g_assert_cmpuint(run_one_memcpy(SRC_ADDR, DST_ADDR, len, 8),
                     ==, ST_READY);

    mem_check_pattern(DST_ADDR, len, 0x5A);

    qtest_end();
}

/*
 * 목적지 무결성 — 요청한 길이 뒤쪽을 침범하지 않는지.
 *
 * 길이 계산이 틀리면 인접 메모리를 덮어쓴다.
 * 실제 드라이버에서는 커널 메모리 손상이 되는 종류다.
 */
static void test_memcpy_dest_integrity(void)
{
    const uint32_t len   = 64;
    const uint32_t guard = 64;

    nacc_start();

    mem_fill_pattern(SRC_ADDR, len, 0x20);
    mem_fill_byte(DST_ADDR, len + guard, DST_POISON);

    g_assert_cmpuint(run_one_memcpy(SRC_ADDR, DST_ADDR, len, 8),
                     ==, ST_READY);

    mem_check_pattern(DST_ADDR, len, 0x20);
    /* 뒤쪽 guard 바이트는 그대로여야 한다 */
    mem_check_byte(DST_ADDR + len, guard, DST_POISON);

    qtest_end();
}

/*
 * NACC-REQ-035 — length 제약.
 * 4의 배수, 4 <= len <= 1 MiB.
 */
static void test_memcpy_bad_length(void)
{
    const struct {
        uint32_t len;
        const char *why;
    } cases[] = {
        { 0,             "0" },
        { 1,             "4의 배수 아님" },
        { 3,             "4의 배수 아님" },
        { 5,             "4의 배수 아님" },
        { MAX_XFER + 4,  "상한 초과" },
    };

    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        nacc_start();

        mem_fill_byte(DST_ADDR, 64, DST_POISON);

        g_assert_cmpuint(run_one_memcpy(SRC_ADDR, DST_ADDR, cases[i].len, 8),
                         ==, ST_ERROR);
        g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_LENGTH);
        /* NACC-REQ-050 */
        g_assert_cmphex(ring_status(0), ==, ERR_BAD_LENGTH);
        /* NACC-REQ-067: 검증 오류는 전송을 시작하지 않는다 */
        mem_check_byte(DST_ADDR, 64, DST_POISON);
        /* NACC-REQ-078: cons 는 실패한 슬롯을 가리킨다 */
        g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 0);

        qtest_end();
    }
}

/* MAX_XFER 는 유효해야 한다 — 상한 경계 */
static void test_memcpy_max_length_valid(void)
{
    nacc_start();

    /* 실제로 1 MiB 를 옮기면 느리므로 상태만 확인한다.
     * SRC/DST 가 1 MiB 씩 떨어져 있지 않으므로 겹칠 수 있으나
     * (§6.5 NACC-REQ-046 미정의), 여기서 보는 것은 길이 검증뿐이다. */
    ring_put(0, OP_MEMCPY, SRC_ADDR, SRC_ADDR + MAX_XFER, MAX_XFER);
    nacc_to_ready();
    nacc_writel(R_RING_PROD, 1);

    for (int i = 0; i < 8 && nacc_state() == ST_BUSY; i++) {
        step_one_desc();
    }

    g_assert_cmpuint(nacc_state(), !=, ST_ERROR);

    qtest_end();
}

/* NACC-REQ-035 — src / dst 4바이트 정렬 */
static void test_memcpy_bad_align(void)
{
    const struct {
        uint64_t src;
        uint64_t dst;
        const char *why;
    } cases[] = {
        { SRC_ADDR | 1, DST_ADDR,     "src 미정렬" },
        { SRC_ADDR | 2, DST_ADDR,     "src 미정렬" },
        { SRC_ADDR,     DST_ADDR | 1, "dst 미정렬" },
        { SRC_ADDR,     DST_ADDR | 3, "dst 미정렬" },
    };

    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        nacc_start();

        mem_fill_byte(DST_ADDR, 64, DST_POISON);

        g_assert_cmpuint(run_one_memcpy(cases[i].src, cases[i].dst, 64, 8),
                         ==, ST_ERROR);
        g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_ALIGN);
        g_assert_cmphex(ring_status(0), ==, ERR_BAD_ALIGN);
        /* 전송이 시작되지 않았다 */
        mem_check_byte(DST_ADDR, 64, DST_POISON);

        qtest_end();
    }
}

/*
 * §6.6 NACC-REQ-055 — 검증 순서.
 * length 와 정렬이 동시에 틀리면 BAD_LENGTH 가 먼저 나야 한다.
 */
static void test_memcpy_length_checked_first(void)
{
    nacc_start();

    /* 길이도 3, src 도 미정렬 */
    g_assert_cmpuint(run_one_memcpy(SRC_ADDR | 1, DST_ADDR, 3, 8),
                     ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_LENGTH);

    qtest_end();
}

/* 소스가 RAM 밖 — 전송 도중 DMA_FAULT */
static void test_memcpy_dma_fault_src(void)
{
    nacc_start();

    mem_fill_byte(DST_ADDR, 64, DST_POISON);

    g_assert_cmpuint(run_one_memcpy(BAD_ADDR, DST_ADDR, 64, 8),
                     ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_DMA_FAULT);
    g_assert_cmphex(ring_status(0), ==, ERR_DMA_FAULT);

    qtest_end();
}

/* 목적지가 RAM 밖 */
static void test_memcpy_dma_fault_dst(void)
{
    nacc_start();

    mem_fill_pattern(SRC_ADDR, 64, 0x42);

    g_assert_cmpuint(run_one_memcpy(SRC_ADDR, BAD_ADDR, 64, 8),
                     ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_DMA_FAULT);

    qtest_end();
}

/*
 * NOP 과 MEMCPY 를 섞어서 제출한다.
 * 링이 opcode 에 관계없이 순서대로 돌아가는지.
 */
static void test_memcpy_mixed_with_nop(void)
{
    const uint32_t len = 64;

    nacc_start();

    mem_fill_pattern(SRC_ADDR, len, 0x77);
    mem_fill_byte(DST_ADDR, len, DST_POISON);

    ring_put(0, OP_NOP, 0, 0, 0);
    ring_put(1, OP_MEMCPY, SRC_ADDR, DST_ADDR, len);
    ring_put(2, OP_NOP, 0, 0, 0);

    nacc_to_ready();
    nacc_writel(R_RING_PROD, 3);

    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 1);
    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 2);
    step_one_desc();
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 3);

    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    mem_check_pattern(DST_ADDR, len, 0x77);

    g_assert_cmphex(ring_status(0), ==, ERR_NONE);
    g_assert_cmphex(ring_status(1), ==, ERR_NONE);
    g_assert_cmphex(ring_status(2), ==, ERR_NONE);

    qtest_end();
}

/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    /* 단계 1 — PCI / 레지스터 파일 */
    qtest_add_func("/nacc/pci-identity",      test_pci_identity);
    qtest_add_func("/nacc/bar0-size",         test_bar0_size);
    qtest_add_func("/nacc/id-register",       test_id_register);
    qtest_add_func("/nacc/reset-values",      test_reset_values);
    qtest_add_func("/nacc/rw-reserved",       test_rw_registers_preserve_reserved);
    qtest_add_func("/nacc/ctrl-reserved",     test_ctrl_preserves_reserved);
    qtest_add_func("/nacc/ro-ignore",         test_ro_registers_ignore_writes);
    qtest_add_func("/nacc/undefined",         test_undefined_offsets);
    qtest_add_func("/nacc/irq-status-rw1c",   test_irq_status_rw1c_shape);

    /* 단계 2 — 전이 */
    qtest_add_func("/nacc/t1-latch-on-hi",    test_t1_latch_on_hi);
    qtest_add_func("/nacc/t1-rewrite-init",   test_t1_rewrite_in_init);
    qtest_add_func("/nacc/t2-enable-valid",   test_t2_enable_valid);
    qtest_add_func("/nacc/ring-size-bounds",  test_ring_size_boundaries);
    qtest_add_func("/nacc/t3-unaligned",      test_t3_unaligned_base);
    qtest_add_func("/nacc/t3-zero-base",      test_t3_zero_base);
    qtest_add_func("/nacc/latched-base",      test_validation_uses_latched_base);
    qtest_add_func("/nacc/t10-enable-reset",  test_t10_enable_in_reset);
    qtest_add_func("/nacc/t8-t9-halting",     test_t8_t9_halting);
    qtest_add_func("/nacc/halting-ignores",   test_halting_ignores_enable);
    qtest_add_func("/nacc/error-recovery",    test_error_recovery_path);
    qtest_add_func("/nacc/t5-bad-prod",       test_t5_bad_prod);
    qtest_add_func("/nacc/prod-max-valid",    test_prod_max_valid);

    /* 단계 2 — 접근 제한 및 무시 조합 */
    qtest_add_func("/nacc/ring-locked-ready", test_ring_config_locked_in_ready);
    qtest_add_func("/nacc/doorbell-ignored",  test_doorbell_ignored_before_ready);
    qtest_add_func("/nacc/disable-ignored",   test_disable_ignored_when_inactive);
    qtest_add_func("/nacc/redundant-enable",  test_redundant_enable_ignored);
    qtest_add_func("/nacc/id-stable",         test_id_stable_across_states);

    /* 단계 3 — 커맨드 링 (NOP) */
    qtest_add_func("/nacc/ram-sanity",        test_ram_sanity);
    qtest_add_func("/nacc/ring-nop-single",   test_ring_nop_single);
    qtest_add_func("/nacc/ring-stepwise",     test_ring_nop_stepwise);
    qtest_add_func("/nacc/ring-wraparound",   test_ring_wraparound);
    qtest_add_func("/nacc/nop-ignores",       test_nop_ignores_fields);
    qtest_add_func("/nacc/t11-append",        test_t11_append_during_busy);
    qtest_add_func("/nacc/t12-bad-prod",      test_t12_bad_prod_during_busy);
    qtest_add_func("/nacc/t7-bad-opcode",     test_t7_bad_opcode);
    qtest_add_func("/nacc/t8-abort-busy",     test_t8_abort_during_busy);
    qtest_add_func("/nacc/restart-abort",     test_restart_after_abort);
    qtest_add_func("/nacc/dma-fault",         test_dma_fault_out_of_ram);

    /* 단계 3 — MEMCPY */
    qtest_add_func("/nacc/memcpy-basic",      test_memcpy_basic);
    qtest_add_func("/nacc/memcpy-multi-chunk", test_memcpy_multi_chunk);
    qtest_add_func("/nacc/memcpy-odd-chunk",  test_memcpy_odd_chunk);
    qtest_add_func("/nacc/memcpy-integrity",  test_memcpy_dest_integrity);
    qtest_add_func("/nacc/memcpy-bad-length", test_memcpy_bad_length);
    qtest_add_func("/nacc/memcpy-max-length", test_memcpy_max_length_valid);
    qtest_add_func("/nacc/memcpy-bad-align",  test_memcpy_bad_align);
    qtest_add_func("/nacc/memcpy-order",      test_memcpy_length_checked_first);
    qtest_add_func("/nacc/memcpy-fault-src",  test_memcpy_dma_fault_src);
    qtest_add_func("/nacc/memcpy-fault-dst",  test_memcpy_dma_fault_dst);
    qtest_add_func("/nacc/memcpy-mixed",      test_memcpy_mixed_with_nop);

    return g_test_run();
}