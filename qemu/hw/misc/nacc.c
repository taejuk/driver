/*
 * nacc - virtual PCIe accelerator
 *
 * nacc 디바이스 명세서 v0.3 구현.
 *
 * 단계 1: PCI 식별 + BAR0 + MSI + 레지스터 파일
 * 단계 2: 상태 머신 (T1/T2/T3/T5/T8/T9/T10) + 인터럽트 전달
 *
 * 미구현 (단계 3): 커맨드 링 처리. T4/T6/T7/T11/T12.
 *                  RING_PROD 유효값 쓰기가 BUSY 로 전이하지 않는다.
 *
 * 구조는 hw/misc/edu.c (MIT) 를 참고했다.
 * SPDX-License-Identifier: MIT
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/host-utils.h"    /* is_power_of_2 */
#include "qemu/timer.h"
#include "qemu/module.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "qom/object.h"

#define TYPE_PCI_NACC_DEVICE "nacc"
typedef struct NaccState NaccState;
DECLARE_INSTANCE_CHECKER(NaccState, NACC, TYPE_PCI_NACC_DEVICE)

/* -------------------------------------------------------------------------
 * §2.1 PCI 식별자
 * ------------------------------------------------------------------------- */
#define NACC_VENDOR_ID      0x1234      /* == PCI_VENDOR_ID_QEMU */
#define NACC_DEVICE_ID      0x11EA
#define NACC_REVISION_ID    0x01
#define NACC_CLASS_CODE     0x1200      /* Processing Accelerator, §9 Q6 미확인 */

/* §2.2 BAR0 */
#define NACC_BAR0_SIZE      (4 * KiB)

/* -------------------------------------------------------------------------
 * §5.1 레지스터 오프셋
 * ------------------------------------------------------------------------- */
#define NACC_REG_ID             0x0000  /* RO   */
#define NACC_REG_CTRL           0x0008  /* RW   */
#define NACC_REG_STATUS         0x000C  /* RO   */
#define NACC_REG_IRQ_STATUS     0x0010  /* RW1C */
#define NACC_REG_IRQ_MASK       0x0014  /* RW   */
#define NACC_REG_RING_BASE_LO   0x0020  /* RW   */
#define NACC_REG_RING_BASE_HI   0x0024  /* RW   */
#define NACC_REG_RING_SIZE      0x0028  /* RW   */
#define NACC_REG_RING_PROD      0x002C  /* RW   */
#define NACC_REG_RING_CONS      0x0030  /* RO   */
#define NACC_REG_ERR_CODE       0x0040  /* RO   */

/* §5.3 ID: [31:16] MAGIC='NA', [15:8] RSVD, [7:0] REV */
#define NACC_ID_MAGIC       0x4E41
#define NACC_ID_VALUE       ((NACC_ID_MAGIC << 16) | NACC_REVISION_ID)

/* §5.3 CTRL */
#define NACC_CTRL_ENABLE    (1u << 0)

/* §5.3 IRQ_STATUS / IRQ_MASK */
#define NACC_IRQ_CMD_DONE   (1u << 0)
#define NACC_IRQ_CMD_ERR    (1u << 1)
#define NACC_IRQ_ALL        (NACC_IRQ_CMD_DONE | NACC_IRQ_CMD_ERR)

/* §5.3 RING_SIZE / RING_PROD / RING_CONS 는 [7:0] 만 유효 필드다 */
#define NACC_RING_IDX_MASK  0xFFu

/* §6.2 NACC-REQ-036 */
#define NACC_RING_SIZE_MIN  2
#define NACC_RING_SIZE_MAX  128

/* §6.2 NACC-REQ-037: 링 베이스는 32바이트 정렬 */
#define NACC_RING_BASE_ALIGN_MASK  0x1FULL
#define NACC_EXEC_DELAY_MS 2

#define NACC_DESC_SIZE          32
#define NACC_DESC_OFF_OPCODE    0x00
#define NACC_DESC_OFF_STATUS    0x1C

#define NACC_MAX_XFER       0x100000u
#define NACC_XFER_CHUNK     4096

/*
 * §3.3 HALTING 체류 시간.
 *
 * 스펙은 "유한하다. 구체적 상한은 구현 정의"라고만 정한다.
 * 0 으로 두면 ENABLE←0 직후 곧바로 INIT 이 되어 드라이버의 폴링 루프와
 * 타임아웃 경로가 한 번도 실행되지 않은 채 통과한다. 실제 하드웨어에서만
 * 드러나는 버그를 만들게 되므로 의도적으로 지연을 넣는다.
 *
 * 20ms 는 드라이버가 readl_poll_timeout() 으로 여러 번 폴링하게 되는
 * 정도로 골랐다. 너무 짧으면 검증 효과가 없고 너무 길면 테스트가 느리다.
 */
#define NACC_HALT_DELAY_MS  20

/* -------------------------------------------------------------------------
 * §3.1 상태 인코딩
 * 값이 STATUS.STATE 로 그대로 노출되므로 순서를 바꾸면 안 된다.
 * ------------------------------------------------------------------------- */
typedef enum NaccStateId {
    NACC_ST_RESET   = 0,
    NACC_ST_INIT    = 1,
    NACC_ST_READY   = 2,
    NACC_ST_BUSY    = 3,
    NACC_ST_HALTING = 4,
    NACC_ST_ERROR   = 5,
    /* 6, 7 은 M2 펌웨어 상태용 예약 (§1.2) */
} NaccStateId;

#define NACC_STATUS_STATE_MASK  0x7     /* [2:0] */

/* §8.1 오류 코드 */
enum {
    NACC_ERR_NONE       = 0x00,
    NACC_ERR_BAD_CONFIG = 0x01,
    NACC_ERR_BAD_PROD   = 0x02,
    NACC_ERR_BAD_OPCODE = 0x03,     /* 단계 3 */
    NACC_ERR_BAD_LENGTH = 0x04,     /* 단계 3 */
    NACC_ERR_BAD_ALIGN  = 0x05,     /* 단계 3 */
    NACC_ERR_DMA_FAULT  = 0x06,     /* 단계 3 */
};

/* ------------------------------------------------------------------------- */

struct NaccState {
    PCIDevice pdev;
    MemoryRegion mmio;

    /* 현재 상태. STATUS 로 노출된다 (§5.3) */
    NaccStateId state;

    /* 레지스터 백업 저장소.
     * 쓰기가 허용된 경우 예약 비트까지 통째로 보존한다 (NACC-REQ-019) */
    uint32_t ctrl;
    uint32_t irq_status;
    uint32_t irq_mask;
    uint32_t ring_base_lo;      /* 섀도 (NACC-REQ-011) */
    uint32_t ring_base_hi;      /* 섀도 */
    uint32_t ring_size;
    uint32_t ring_prod;
    uint32_t ring_cons;
    uint32_t err_code;

    /* NACC-REQ-011: RING_BASE_HI 쓰기 시점에 확정되는 유효 주소.
     * 섀도와 다를 수 있다 — LO 만 쓰고 HI 를 쓰지 않은 경우.
     * NACC-REQ-098: 설정 유효성 판정은 이 값을 본다. */
    uint64_t ring_base_latched;

    /* NACC-REQ-064: MSI 는 (IRQ_STATUS & ~IRQ_MASK) 가 0 -> 비0 으로
     * 전이할 때만 발행한다. 직전 상태를 기억해야 전이를 판정할 수 있다. */
    bool irq_asserted;

    /* T9: HALTING 탈출용. §3.3 HALTING 항목 4 */
    QEMUTimer halt_timer;
    QEMUTimer exec_timer;
};

enum {
    NACC_OP_NOP    = 0x00,
    NACC_OP_MEMCPY = 0x01,
};

typedef struct QEMU_PACKED NaccDesc {
    uint32_t opcode;
    uint32_t flags;
    uint64_t src_addr;
    uint64_t dst_addr;
    uint32_t length;
    uint32_t status;
} NaccDesc;

/* -------------------------------------------------------------------------
 * 인터럽트 — §7.3
 * ------------------------------------------------------------------------- */

/*
 * IRQ_STATUS 또는 IRQ_MASK 가 바뀔 때마다 호출한다.
 *
 * NACC-REQ-064: (IRQ_STATUS & ~IRQ_MASK) 가 0 -> 비0 전이일 때 MSI 발행.
 *               이미 비0 인 상태에서 비트가 추가되는 것은 발행하지 않는다.
 * NACC-REQ-065: IRQ_MASK 쓰기로 같은 전이가 일어나도 발행한다.
 *               (마스크를 풀면 밀려 있던 인터럽트가 즉시 뜬다)
 */
static void nacc_irq_update(NaccState *s)
{
    bool now = (s->irq_status & ~s->irq_mask) != 0;

    if (now && !s->irq_asserted) {
        if (msi_enabled(&s->pdev)) {
            msi_notify(&s->pdev, 0);
        }
    }
    s->irq_asserted = now;
}

/*
 * NACC-REQ-049: 디바이스의 set 과 드라이버의 클리어가 겹치면 set 이 이긴다.
 * 여기서는 항상 OR 이므로 자연히 만족된다. 절대 대입(=)하지 말 것.
 */
static void nacc_irq_raise(NaccState *s, uint32_t bits)
{
    s->irq_status |= bits;
    nacc_irq_update(s);
}

/* -------------------------------------------------------------------------
 * 상태 전이 헬퍼
 *
 * 전이마다 부수효과를 손으로 반복하면 언젠가 하나 빠뜨린다.
 * §3.2 전이표의 각 행이 아래 함수 하나에 대응한다.
 * ------------------------------------------------------------------------- */

/* T3 / T5 / T7 / T10 / T12 — ERROR 진입 */
static void nacc_goto_error(NaccState *s, uint32_t code)
{
    /*
     * NACC-REQ-054: ERR_CODE 와 STATE 를 먼저 갱신하고,
     * 그 다음에 CMD_ERR 을 세운다.
     * 순서가 반대면 ISR 이 옛 ERR_CODE 를 읽는 경쟁이 난다.
     */
    s->err_code = code;
    s->state    = NACC_ST_ERROR;
    nacc_irq_raise(s, NACC_IRQ_CMD_ERR);
}

/* T1 — RESET -> INIT */
static void nacc_goto_init_from_reset(NaccState *s)
{
    s->state = NACC_ST_INIT;
    /* 부수효과 없음. 링 메모리는 읽지 않는다 (§3.2 T1) */
}

/* T2 — INIT -> READY */
static void nacc_goto_ready(NaccState *s)
{
    /*
     * §3.2 T2 부수효과.
     * RING_PROD 를 디바이스가 0 으로 미는 것은 §6.3 소유권 규칙의
     * 유일한 예외다 (NACC-REQ-076). 진행 중 명령이 없으므로 안전하다.
     */
    s->ring_cons = 0;
    s->ring_prod = 0;
    s->err_code  = NACC_ERR_NONE;
    s->state     = NACC_ST_READY;
    /* 링 메모리는 읽지 않는다 */
}

/* T9 — HALTING -> INIT. 타이머 콜백이므로 드라이버 개입이 없다 */
static void nacc_halt_complete(void *opaque)
{
    NaccState *s = opaque;

    if (s->state != NACC_ST_HALTING) {
        return;
    }

    /*
     * §3.2 T9: ERR_CODE, IRQ_STATUS, 링 설정값을 모두 보존한다.
     * NACC-REQ-091: 이 시점에 미처리 DMA 가 존재하지 않아야 한다.
     * 단계 2 에는 DMA 자체가 없으므로 자동으로 성립한다.
     * 단계 3 에서는 진행 중이던 전송을 여기서 회수해야 한다.
     */
    s->state = NACC_ST_INIT;
}

/* T8 — READY/BUSY/ERROR -> HALTING */
static void nacc_goto_halting(NaccState *s)
{
    s->state = NACC_ST_HALTING;

    /*
     * NACC-REQ-094: 진행 중 명령은 완료를 기다리지 않고 폐기된다.
     * 완료 인터럽트도 발생하지 않고 RING_CONS 도 전진하지 않는다.
     * NACC-REQ-095: 이 시점의 RING_CONS 가 폐기 경계다. 그래서 건드리지 않는다.
     *
     * 단계 3: 여기서 진행 중인 디스크립터 처리를 중단시켜야 한다.
     */

    timer_mod(&s->halt_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + NACC_HALT_DELAY_MS);
    timer_del(&s->exec_timer);
}

/* -------------------------------------------------------------------------
 * §3.5 설정 유효성 판정 (NACC-REQ-097, NACC-REQ-098)
 * ------------------------------------------------------------------------- */
static bool nacc_config_valid(NaccState *s)
{
    uint32_t size = s->ring_size & NACC_RING_IDX_MASK;   /* [7:0] 만 유효 */

    /* 현재 상태가 INIT 이어야 한다 (RESET 이면 무조건 무효) */
    if (s->state != NACC_ST_INIT) {
        return false;
    }
    /* 2의 거듭제곱, 2 <= size <= 128 */
    if (size < NACC_RING_SIZE_MIN || size > NACC_RING_SIZE_MAX) {
        return false;
    }
    if (!is_power_of_2(size)) {
        return false;
    }
    /* NACC-REQ-098: 섀도가 아니라 래치된 값을 본다 */
    if (s->ring_base_latched == 0) {
        return false;
    }
    if (s->ring_base_latched & NACC_RING_BASE_ALIGN_MASK) {
        return false;
    }
    return true;
}

/* -------------------------------------------------------------------------
 * CTRL 쓰기 처리
 *
 * 값이 아니라 "현재 상태 + 쓰려는 ENABLE 값" 조합으로 분기한다.
 * 엣지 검출은 필요 없다 — §3.2 전이표가 이미 상태별로 결과를 정해 두었고,
 * "이미 ENABLE=1 인데 1 을 쓰는" 경우는 READY/BUSY/ERROR 분기에서
 * 무시로 흡수된다.
 * ------------------------------------------------------------------------- */

static void nacc_ctrl_enable_set(NaccState *s)
{
    switch (s->state) {
    case NACC_ST_RESET:
        /* T10: 링 주소가 미설정이므로 무조건 설정 무효 */
        nacc_goto_error(s, NACC_ERR_BAD_CONFIG);
        break;

    case NACC_ST_INIT:
        if (nacc_config_valid(s)) {
            nacc_goto_ready(s);                         /* T2 */
        } else {
            nacc_goto_error(s, NACC_ERR_BAD_CONFIG);    /* T3 */
        }
        break;

    default:
        /* 이미 활성이거나 HALTING. §3.2 "전이표에 없는 조합" — 무시 */
        break;
    }
}

static void nacc_ctrl_enable_clear(NaccState *s)
{
    switch (s->state) {
    case NACC_ST_READY:
    case NACC_ST_BUSY:
    case NACC_ST_ERROR:
        nacc_goto_halting(s);                           /* T8 */
        break;

    default:
        /* RESET / INIT / HALTING — 무시 */
        break;
    }
}

/* -------------------------------------------------------------------------
 * MMIO
 * ------------------------------------------------------------------------- */

static uint64_t nacc_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    NaccState *s = opaque;

    switch (addr) {
    case NACC_REG_ID:
        /* NACC-REQ-023: 모든 상태에서 같은 값 */
        return NACC_ID_VALUE;

    case NACC_REG_CTRL:
        return s->ctrl;

    case NACC_REG_STATUS:
        /* [31:3] RO 예약은 항상 0 */
        return s->state & NACC_STATUS_STATE_MASK;

    case NACC_REG_IRQ_STATUS:
        return s->irq_status;

    case NACC_REG_IRQ_MASK:
        return s->irq_mask;

    case NACC_REG_RING_BASE_LO:
        /* NACC-REQ-011: 마지막으로 쓴 값(섀도)을 그대로 반환 */
        return s->ring_base_lo;

    case NACC_REG_RING_BASE_HI:
        return s->ring_base_hi;

    case NACC_REG_RING_SIZE:
        return s->ring_size;

    case NACC_REG_RING_PROD:
        return s->ring_prod;

    case NACC_REG_RING_CONS:
        return s->ring_cons;

    case NACC_REG_ERR_CODE:
        return s->err_code;

    default:
        /* NACC-REQ-021: 미매핑 오프셋 읽기는 0.
         * 0xFFFFFFFF 는 링크 다운 신호로 예약되어 있으므로 쓰지 않는다. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "nacc: read from undefined offset 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static bool nacc_do_memcpy(NaccState *s, uint64_t src, uint64_t dst, uint32_t len)
{
    uint8_t buf[NACC_XFER_CHUNK];

    while (len) {
        uint32_t n = MIN(len, sizeof(buf));

        if (pci_dma_read(&s->pdev, src, buf, n) != MEMTX_OK) {
            return false;
        }
        if (pci_dma_write(&s->pdev, dst, buf, n) != MEMTX_OK) {
            return false;
        }
        src += n;
        dst += n;
        len -= n;
    }
    return true;
}

/* T7 — 실패한 디스크립터의 status 에 오류 코드를 기록하고 ERROR 로 간다 */
static void nacc_fail_descriptor(NaccState *s, uint64_t desc_addr,
                                 uint32_t code)
{
    uint32_t st = cpu_to_le32(code);

    /* NACC-REQ-050. 페치 자체가 실패한 경우엔 이 쓰기도 실패할 수 있으나
     * 추가 오류를 발생시키지 않는다 (§8.2 예외 조항) */
    pci_dma_write(&s->pdev, desc_addr + NACC_DESC_OFF_STATUS,
                  &st, sizeof(st));

    nacc_goto_error(s, code);
}

static bool nacc_exec_descriptor(NaccState *s)
{
    NaccDesc desc;
    uint64_t addr = s->ring_base_latched +
                    (uint64_t)s->ring_cons * NACC_DESC_SIZE;
    uint32_t st;

    /* §6.6 1. 페치 */
    if (pci_dma_read(&s->pdev, addr, &desc, sizeof(desc)) != MEMTX_OK) {
        nacc_goto_error(s, NACC_ERR_DMA_FAULT);
        return false;
    }

    /* §6.6 2. opcode 판별. 검증은 opcode 뒤에 한다 (REQ-045/055) */
    switch (le32_to_cpu(desc.opcode)) {
    case NACC_OP_NOP:
        break;                      /* 필드 검증 없음 */

    case NACC_OP_MEMCPY: {
        /* TODO: length / 정렬 검증 후 전송 */
        uint64_t src = le64_to_cpu(desc.src_addr);
        uint64_t dst = le64_to_cpu(desc.dst_addr);
        uint32_t len = le32_to_cpu(desc.length);

        if (len == 0 || len > NACC_MAX_XFER || (len & 0x3)) {
            nacc_fail_descriptor(s, addr, NACC_ERR_BAD_LENGTH);
            return false;
        }
        if ((src & 0x3) || (dst & 0x3)) {
            nacc_fail_descriptor(s, addr, NACC_ERR_BAD_ALIGN);
            return false;
        }
        if (!nacc_do_memcpy(s, src, dst, len)) {
            nacc_fail_descriptor(s, addr, NACC_ERR_DMA_FAULT);
            return false;
        }
        break;
    }
    default:
        nacc_fail_descriptor(s, addr, NACC_ERR_BAD_OPCODE);
        return false;
    }

    /* §6.6 5. status writeback — 4바이트만 */
    st = cpu_to_le32(NACC_ERR_NONE);
    if (pci_dma_write(&s->pdev, addr + NACC_DESC_OFF_STATUS,
                      &st, sizeof(st)) != MEMTX_OK) {
        nacc_goto_error(s, NACC_ERR_DMA_FAULT);
        return false;
    }

    return true;
}


static void nacc_sched_next(NaccState *s)
{
    timer_mod(&s->exec_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + NACC_EXEC_DELAY_MS);
}

static void nacc_process_one(void *opaque)
{
    NaccState *s = opaque;

    if (s->state != NACC_ST_BUSY) {
        return;
    }

    if (!nacc_exec_descriptor(s)) {
        return;                      
    }
    uint32_t size = s->ring_size & NACC_RING_IDX_MASK;
    s->ring_cons = (s->ring_cons + 1) & (size - 1);

    if (s->ring_cons == (s->ring_prod & 0xFF)) {
        s->state = NACC_ST_READY;
        nacc_irq_raise(s, NACC_IRQ_CMD_DONE);
    } else {
        nacc_sched_next(s);
    }
}

static void nacc_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    NaccState *s = opaque;
    uint32_t v = (uint32_t)val;

    switch (addr) {

    /* ------------------------------------------------------------------
     * CTRL — §5.2 에서 RESET 과 HALTING 은 R, 나머지는 RW.
     * RESET 은 각주 1: 값은 저장되지 않지만 ENABLE<-1 이 T10 을 유발한다.
     * ------------------------------------------------------------------ */
    case NACC_REG_CTRL:
        switch (s->state) {
        case NACC_ST_RESET:
            /* 값 저장 없음. ENABLE<-1 만 T10 을 유발 */
            if (v & NACC_CTRL_ENABLE) {
                nacc_ctrl_enable_set(s);
            }
            break;

        case NACC_ST_HALTING:
            /* 쓰기 전체 무시 (§5.2) */
            qemu_log_mask(LOG_GUEST_ERROR,
                          "nacc: CTRL write ignored in HALTING\n");
            break;
        default:
            /* INIT / READY / BUSY / ERROR: 예약 비트까지 저장 (REQ-019) */
            
            s->ctrl = v;
            if (v & NACC_CTRL_ENABLE) {
                nacc_ctrl_enable_set(s);
            } else {
                nacc_ctrl_enable_clear(s);
            }
            break;
        }
        break;

    /* ------------------------------------------------------------------
     * IRQ_STATUS — RW1C. 전 상태에서 RW (§5.2)
     * ------------------------------------------------------------------ */
    case NACC_REG_IRQ_STATUS:
        /* NACC-REQ-048: 1 을 쓴 비트만 클리어. 0 은 변화 없음 */
        s->irq_status &= ~v;
        nacc_irq_update(s);
        break;

    /* ------------------------------------------------------------------
     * IRQ_MASK — 전 상태에서 RW
     * ------------------------------------------------------------------ */
    case NACC_REG_IRQ_MASK:
        s->irq_mask = v;
        /* NACC-REQ-065: 마스크 해제로 전이가 생기면 여기서 MSI 가 나간다 */
        nacc_irq_update(s);
        break;

    /* ------------------------------------------------------------------
     * RING_BASE_LO — RESET / INIT 에서만 W (§5.2)
     * ------------------------------------------------------------------ */
    case NACC_REG_RING_BASE_LO:
        if (s->state == NACC_ST_RESET || s->state == NACC_ST_INIT) {
            /* NACC-REQ-011: 섀도에만 반영. 유효 주소는 바뀌지 않는다 */
            s->ring_base_lo = v;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "nacc: RING_BASE_LO write ignored in state %d\n",
                          s->state);
        }
        break;

    /* ------------------------------------------------------------------
     * RING_BASE_HI — RESET / INIT 에서만 W. RESET 에서는 T1 을 유발 (§5.2 각주 2)
     * ------------------------------------------------------------------ */
    case NACC_REG_RING_BASE_HI:
        if (s->state == NACC_ST_RESET || s->state == NACC_ST_INIT) {
            bool was_reset = (s->state == NACC_ST_RESET);

            s->ring_base_hi = v;
            /* NACC-REQ-011: 여기서 {HI,LO} 가 래치된다 */
            s->ring_base_latched = ((uint64_t)v << 32) | s->ring_base_lo;

            if (was_reset) {
                nacc_goto_init_from_reset(s);           /* T1 */
            }
            /* 이미 INIT 이면 주소만 갱신. 전이 없음 */
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "nacc: RING_BASE_HI write ignored in state %d\n",
                          s->state);
        }
        break;

    /* ------------------------------------------------------------------
     * RING_SIZE — RESET / INIT 에서만 W (§5.2)
     * ------------------------------------------------------------------ */
    case NACC_REG_RING_SIZE:
        if (s->state == NACC_ST_RESET || s->state == NACC_ST_INIT) {
            /* 값 검증은 여기서 하지 않는다. §3.5 에 따라 ENABLE<-1 시점에
             * 몰아서 판정한다. 잘못 쓴 값도 그대로 읽혀야 드라이버가
             * 원인을 확인할 수 있다 (NACC-REQ-071 과 같은 취지) */
            s->ring_size = v;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "nacc: RING_SIZE write ignored in state %d\n",
                          s->state);
        }
        break;

    /* ------------------------------------------------------------------
     * RING_PROD — READY / BUSY 에서만 W. 쓰기가 곧 doorbell (NACC-REQ-074)
     * ------------------------------------------------------------------ */
    case NACC_REG_RING_PROD:
        if (s->state != NACC_ST_READY && s->state != NACC_ST_BUSY) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "nacc: RING_PROD write ignored in state %d\n",
                          s->state);
            break;
        }

        /* NACC-REQ-075: >= 비교. RING_SIZE=128 이면 유효 범위는 0..127 */
        if ((v & NACC_RING_IDX_MASK) >=
            (s->ring_size & NACC_RING_IDX_MASK)) {
            /* T5 (READY) / T12 (BUSY) */
            nacc_goto_error(s, NACC_ERR_BAD_PROD);
            break;
        }

        s->ring_prod = v;

        /*
         * TODO(단계 3): 여기서 T4 / T11 이 일어나야 한다.
         *   READY 이고 prod != cons  -> BUSY 로 전이, 디스크립터 페치 시작
         *   BUSY                     -> 대기열에 추가, 상태 유지
         *   READY 이고 prod == cons  -> 무시 (지금과 동일)
         *
         * 단계 2 에는 디스크립터 처리가 없으므로 전이를 만들지 않는다.
         * BUSY 로 보내 놓고 나올 방법이 없으면 디바이스가 멈춰 버린다.
         */
        // 상태를 바꾸고 난 뒤,
        if((s->state == NACC_ST_READY) && ((s->ring_prod & NACC_RING_IDX_MASK)  != s->ring_cons)) {
            s->state = NACC_ST_BUSY;
            nacc_sched_next(opaque);
            // status에 완료되었다고 쓴다.
            // 그리고 cons값을 1 증가시킨다.
            // 그리고 만약 pr
        }
        break;

    /* ------------------------------------------------------------------
     * RO 레지스터 — 쓰기 무시.
     * default 로 흘려보내지 않고 명시적으로 나열한다.
     * 미매핑 오프셋과 구분되어야 오프셋 오타를 잡을 수 있다.
     * ------------------------------------------------------------------ */
    case NACC_REG_ID:
    case NACC_REG_STATUS:
    case NACC_REG_RING_CONS:
    case NACC_REG_ERR_CODE:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "nacc: write to RO register 0x%" HWADDR_PRIx "\n", addr);
        break;

    default:
        /* NACC-REQ-021: 미매핑 오프셋 쓰기는 무시. 오류가 아니다 */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "nacc: write to undefined offset 0x%" HWADDR_PRIx "\n",
                      addr);
        break;
    }
}

static const MemoryRegionOps nacc_mmio_ops = {
    .read       = nacc_mmio_read,
    .write      = nacc_mmio_write,
    /* NACC-REQ-013: MMIO 는 리틀엔디안 */
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* NACC-REQ-010: 32비트 정렬 접근만 허용.
     * valid 와 impl 을 모두 4 로 고정해야 8/16비트 접근이
     * 콜백까지 도달하지 않는다. */
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned       = false,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* -------------------------------------------------------------------------
 * 리셋 / realize
 * ------------------------------------------------------------------------- */

static void nacc_reset_regs(NaccState *s)
{
    s->state             = NACC_ST_RESET;
    s->ctrl              = 0;
    s->irq_status        = 0;
    /* 리셋값은 전부 마스킹. 드라이버가 핸들러를 등록하기 전에
     * 인터럽트가 전달되면 안 된다 (§5.3 IRQ_MASK) */
    s->irq_mask          = NACC_IRQ_ALL;
    s->ring_base_lo      = 0;
    s->ring_base_hi      = 0;
    s->ring_size         = 0;
    s->ring_prod         = 0;
    s->ring_cons         = 0;
    s->err_code          = NACC_ERR_NONE;
    s->ring_base_latched = 0;
    s->irq_asserted      = false;
}

static void nacc_qdev_reset(DeviceState *dev)
{
    NaccState *s = NACC(dev);

    timer_del(&s->halt_timer);
    timer_del(&s->exec_timer);
    nacc_reset_regs(s);
}

static void pci_nacc_realize(PCIDevice *pdev, Error **errp)
{
    NaccState *nacc = NACC(pdev);

    /* NACC-REQ-004: MSI 1벡터. INTx 는 지원하지 않으므로
     * pci_config_set_interrupt_pin() 을 호출하지 않는다. */
    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    /* QEMU_CLOCK_VIRTUAL 을 쓰면 qtest 에서 클럭을 점프시켜
     * 실시간을 기다리지 않고 T9 를 검증할 수 있다 */
    timer_init_ms(&nacc->halt_timer, QEMU_CLOCK_VIRTUAL,
                  nacc_halt_complete, nacc);
    timer_init_ms(&nacc->exec_timer, QEMU_CLOCK_VIRTUAL,
              nacc_process_one, nacc);
    nacc_reset_regs(nacc);

    memory_region_init_io(&nacc->mmio, OBJECT(nacc), &nacc_mmio_ops, nacc,
                          "nacc-mmio", NACC_BAR0_SIZE);
    /* NACC-REQ-002: MMIO, non-prefetchable, 32bit */
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &nacc->mmio);
}

static void pci_nacc_uninit(PCIDevice *pdev)
{
    NaccState *nacc = NACC(pdev);

    timer_del(&nacc->halt_timer);
    timer_del(&nacc->exec_timer);
    msi_uninit(pdev);
}

static void nacc_class_init(ObjectClass *class, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize   = pci_nacc_realize;
    k->exit      = pci_nacc_uninit;
    k->vendor_id = NACC_VENDOR_ID;
    k->device_id = NACC_DEVICE_ID;
    k->revision  = NACC_REVISION_ID;
    k->class_id  = NACC_CLASS_CODE;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_legacy_reset(dc, nacc_qdev_reset);
}

static const TypeInfo nacc_types[] = {
    {
        .name          = TYPE_PCI_NACC_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(NaccState),
        .class_init    = nacc_class_init,
        .interfaces    = (const InterfaceInfo[]) {
            { INTERFACE_CONVENTIONAL_PCI_DEVICE },
            { },
        },
    }
};

DEFINE_TYPES(nacc_types)