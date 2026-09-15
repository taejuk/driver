/*
 * QTest for the nacc virtual accelerator — 명세서 v0.3
 *
 * 단계 1: PCI 식별, BAR0, 레지스터 파일
 * 단계 2: 상태 머신 (T1/T2/T3/T5/T8/T9/T10) + "전이표에 없는 조합"
 *
 * 미검증 (단계 3): T4/T6/T7/T11/T12 — 커맨드 링 처리가 필요하다.
 *
 * 검증 환경 전제:
 *   -machine virt,highmem-ecam=off
 *       기본값이면 ECAM 이 RAM 뒤 동적 주소로 올라가 0x3f000000 으로
 *       접근할 수 없다 (hw/arm/virt.c: vms->highmem_ecam = true).
 *   -net none
 *       기본 virtio-net 이 슬롯 1 을 차지한다.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
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
#define NACC_EXEC_DELAY_MS 2
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

/* 유효한 링 설정 — 32바이트 정렬, 2의 거듭제곱 */
#define RING_ADDR           0x40000000ULL
#define RING_SZ             8

/* 디바이스 구현의 HALTING 지연 (NACC_HALT_DELAY_MS) 보다 넉넉히 */
#define HALT_DELAY_MS       20
#define MS_IN_NS            1000000LL

/* §5.1 정의되지 않은 오프셋 (NACC-REQ-021) */
static const uint32_t undefined_offsets[] = {
    0x0004, 0x0018, 0x001C, 0x0034, 0x0038, 0x003C, 0x0044, 0x0800, 0x0FFC,
};

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
 * 테스트가 직접 BAR 를 프로그래밍하고 Memory Space Enable 을 켠다.
 */
static void nacc_setup_bar(void)
{
    uint16_t cmd;

    writel(NACC_BDF_ECAM + PCI_CFG_BAR0, NACC_BAR0_ADDR);
    cmd = readw(NACC_BDF_ECAM + PCI_CFG_COMMAND);
    writew(NACC_BDF_ECAM + PCI_CFG_COMMAND, cmd | PCI_CMD_MEM_SPACE | PCI_CMD_BUS_MASTER);
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
 * CTRL 의 예약 비트 보존은 test_ctrl_preserves_reserved 에서 확인한다.
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
    /* ENABLE=0 이므로 INIT 에서 전이가 없어야 한다 */
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
    g_assert_cmpuint(nacc_state(), ==, ST_RESET);       /* 아직 RESET */
    g_assert_cmphex(nacc_readl(R_RING_BASE_LO), ==, (uint32_t)RING_ADDR);

    nacc_writel(R_RING_BASE_HI, (uint32_t)(RING_ADDR >> 32));
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);        /* T1 */

    qtest_end();
}

/* INIT 에서 RING_BASE_HI 를 다시 써도 전이는 없다 (INIT -> INIT) */
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
    /* T2 부수효과: cons, prod, err_code 초기화 */
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

    nacc_writel(R_RING_BASE_LO, (uint32_t)RING_ADDR | 0x10);  /* 하위 5비트 */
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

    /* LO=0, HI=0 을 써서 래치. 주소가 0 이므로 무효 */
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
 * 따라서 이전에 래치된 (정렬된) 주소로 판정되어 T2 가 일어나야 한다.
 * 섀도를 보는 구현이면 여기서 ERROR 가 나므로 걸린다.
 */
static void test_validation_uses_latched_base(void)
{
    nacc_start();
    nacc_to_init();                      /* 정렬된 주소가 래치됨 */

    /* 섀도만 미정렬로 오염시킨다 */
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

    /* 지연이 실제로 걸려 있는지 — 절반만 전진시키면 아직 HALTING */
    nacc_clock_step_ms(HALT_DELAY_MS / 2);
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);

    /* 지연을 넘기면 INIT (T9) */
    nacc_clock_step_ms(HALT_DELAY_MS);
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
 * ERROR -> (ENABLE<-0) HALTING -> INIT -> (ENABLE<-1) READY
 *
 * NACC-REQ-093: ERR_CODE 는 T2 가 일어날 때까지 보존된다.
 */
static void test_error_recovery_path(void)
{
    nacc_start();

    /* T10 으로 ERROR 진입 */
    nacc_writel(R_CTRL, CTRL_ENABLE);
    g_assert_cmpuint(nacc_state(), ==, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);

    /* T8 */
    nacc_writel(R_CTRL, 0);
    g_assert_cmpuint(nacc_state(), ==, ST_HALTING);
    /* HALTING 중에도 ERR_CODE 보존 */
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);

    /* T9 */
    nacc_clock_step_ms(HALT_DELAY_MS + 1);
    g_assert_cmpuint(nacc_state(), ==, ST_INIT);
    /* INIT 에서도 아직 보존 */
    g_assert_cmphex(nacc_readl(R_ERR_CODE), ==, ERR_BAD_CONFIG);

    /* 링을 설정하고 재활성화 */
    nacc_writel(R_RING_BASE_LO, (uint32_t)RING_ADDR);
    nacc_writel(R_RING_BASE_HI, (uint32_t)(RING_ADDR >> 32));
    nacc_writel(R_RING_SIZE, RING_SZ);
    nacc_writel(R_CTRL, CTRL_ENABLE);                    /* T2 */

    g_assert_cmpuint(nacc_state(), ==, ST_READY);
    /* T2 에서 비로소 클리어 */
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
    /* §3.2 T5: RING_CONS 불변 */
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 0);

    qtest_end();
}

/* RING_SZ-1 은 유효하므로 ERROR 가 아니어야 한다 */
static void test_prod_max_valid(void)
{
    nacc_start();
    nacc_to_ready();

    nacc_writel(R_RING_PROD, RING_SZ - 1);

    g_assert_cmpuint(nacc_state(), !=, ST_ERROR);
    g_assert_cmphex(nacc_readl(R_RING_PROD), ==, RING_SZ - 1);

    qtest_end();
}

/*
 * §5.2 접근 가능 행렬 — READY 에서 링 설정 레지스터는 R.
 * 쓰기는 무시되고 값이 바뀌지 않아야 한다.
 */
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

/*
 * §3.2 "전이표에 없는 조합" — RESET / INIT 에서 doorbell 은 무시된다.
 * 쓰기 무시이므로 RING_PROD 값도 바뀌지 않는다.
 */
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

/* 이미 READY 인데 ENABLE<-1 을 또 써도 무시된다 */
// static void test_redundant_enable_ignored(void)
// {
//     nacc_start();
//     nacc_to_ready();

//     nacc_writel(R_RING_PROD, 3);
//     nacc_writel(R_CTRL, CTRL_ENABLE);

//     g_assert_cmpuint(nacc_state(), ==, ST_READY);
//     /* T2 가 다시 일어났다면 prod 가 0 으로 밀렸을 것이다 */
//     g_assert_cmphex(nacc_readl(R_RING_PROD), ==, 3);

//     qtest_end();
// }

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

static void test_nop_single(void)
{
    uint8_t desc[32] = { 0 };       /* opcode = NOP(0) */

    nacc_start();
    qtest_memwrite(global_qtest, RING_ADDR, desc, sizeof(desc));
    nacc_to_ready();

    nacc_writel(R_RING_PROD, 1);                  /* T4 */
    g_assert_cmpuint(nacc_state(), ==, ST_BUSY);  /* 즉시 완료되면 실패 */

    nacc_clock_step_ms(NACC_EXEC_DELAY_MS + 1);

    g_assert_cmpuint(nacc_state(), ==, ST_READY); /* T6 */
    g_assert_cmphex(nacc_readl(R_RING_CONS), ==, 1);
    g_assert_cmphex(nacc_readl(R_IRQ_STATUS), ==, IRQ_CMD_DONE);

    qtest_end();
}

/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    /* 단계 1 */
    qtest_add_func("/nacc/pci-identity",     test_pci_identity);
    qtest_add_func("/nacc/bar0-size",        test_bar0_size);
    qtest_add_func("/nacc/id-register",      test_id_register);
    qtest_add_func("/nacc/reset-values",     test_reset_values);
    qtest_add_func("/nacc/rw-reserved",      test_rw_registers_preserve_reserved);
    qtest_add_func("/nacc/ctrl-reserved",    test_ctrl_preserves_reserved);
    qtest_add_func("/nacc/ro-ignore",        test_ro_registers_ignore_writes);
    qtest_add_func("/nacc/undefined",        test_undefined_offsets);
    qtest_add_func("/nacc/irq-status-rw1c",  test_irq_status_rw1c_shape);

    /* 단계 2 — 전이 */
    qtest_add_func("/nacc/t1-latch-on-hi",   test_t1_latch_on_hi);
    qtest_add_func("/nacc/t1-rewrite-init",  test_t1_rewrite_in_init);
    qtest_add_func("/nacc/t2-enable-valid",  test_t2_enable_valid);
    qtest_add_func("/nacc/ring-size-bounds", test_ring_size_boundaries);
    qtest_add_func("/nacc/t3-unaligned",     test_t3_unaligned_base);
    qtest_add_func("/nacc/t3-zero-base",     test_t3_zero_base);
    qtest_add_func("/nacc/latched-base",     test_validation_uses_latched_base);
    qtest_add_func("/nacc/t10-enable-reset", test_t10_enable_in_reset);
    qtest_add_func("/nacc/t8-t9-halting",    test_t8_t9_halting);
    qtest_add_func("/nacc/halting-ignores",  test_halting_ignores_enable);
    qtest_add_func("/nacc/error-recovery",   test_error_recovery_path);
    qtest_add_func("/nacc/t5-bad-prod",      test_t5_bad_prod);
    qtest_add_func("/nacc/prod-max-valid",   test_prod_max_valid);

    /* 단계 2 — 접근 제한 및 무시 조합 */
    qtest_add_func("/nacc/ring-locked-ready", test_ring_config_locked_in_ready);
    qtest_add_func("/nacc/doorbell-ignored", test_doorbell_ignored_before_ready);
    qtest_add_func("/nacc/disable-ignored",  test_disable_ignored_when_inactive);
    //qtest_add_func("/nacc/redundant-enable", test_redundant_enable_ignored);
    qtest_add_func("/nacc/id-stable",        test_id_stable_across_states);

    /*descriptor*/
    qtest_add_func("/nacc/descriptor-nop-single", test_nop_single);
    return g_test_run();
}