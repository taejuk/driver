// SPDX-License-Identifier: GPL-2.0
/*
 * nacc — virtual PCIe accelerator driver
 *
 * nacc 디바이스 명세서 v0.3 의 [SW] 요구사항을 구현한다.
 * 하드웨어 쪽 절반은 QEMU 디바이스 모델(hw/misc/nacc.c)에 있다.
 *
 * M1 범위: 커맨드 링 제출/완료, MSI, 정지 시퀀스, debugfs 테스트 인터페이스.
 * char device 와 accel 서브시스템 편입은 M2.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

/* -------------------------------------------------------------------------
 * §2.1 PCI 식별자
 * ------------------------------------------------------------------------- */
#define NACC_VENDOR_ID          0x1234
#define NACC_DEVICE_ID          0x11EA
#define NACC_REVISION_KNOWN     0x01

/* -------------------------------------------------------------------------
 * §5.1 레지스터 맵
 * ------------------------------------------------------------------------- */
#define NACC_REG_ID             0x0000
#define NACC_REG_CTRL           0x0008
#define NACC_REG_STATUS         0x000C
#define NACC_REG_IRQ_STATUS     0x0010
#define NACC_REG_IRQ_MASK       0x0014
#define NACC_REG_RING_BASE_LO   0x0020
#define NACC_REG_RING_BASE_HI   0x0024
#define NACC_REG_RING_SIZE      0x0028
#define NACC_REG_RING_PROD      0x002C
#define NACC_REG_RING_CONS      0x0030
#define NACC_REG_ERR_CODE       0x0040

/* §5.3 ID */
#define NACC_ID_MAGIC_SHIFT     16
#define NACC_ID_MAGIC           0x4E41          /* 'NA' */
#define NACC_ID_REV_MASK        0xFF

/* §5.3 CTRL */
#define NACC_CTRL_ENABLE        BIT(0)

/* §5.3 STATUS */
#define NACC_STATE_MASK         0x7

/* §3.1 상태 인코딩 */
enum nacc_state {
	NACC_ST_RESET   = 0,
	NACC_ST_INIT    = 1,
	NACC_ST_READY   = 2,
	NACC_ST_BUSY    = 3,
	NACC_ST_HALTING = 4,
	NACC_ST_ERROR   = 5,
};

/* §5.3 IRQ_STATUS / IRQ_MASK */
#define NACC_IRQ_CMD_DONE       BIT(0)
#define NACC_IRQ_CMD_ERR        BIT(1)
#define NACC_IRQ_ALL            (NACC_IRQ_CMD_DONE | NACC_IRQ_CMD_ERR)

/* §5.3 링 인덱스는 [7:0] 만 유효 */
#define NACC_IDX_MASK           0xFFu

/* §6.2 NACC-REQ-036 */
#define NACC_RING_SIZE_MIN      2
#define NACC_RING_SIZE_MAX      128

/* §6.1 NACC-REQ-030 */
#define NACC_DESC_SIZE          32

/* §6.5 opcode */
#define NACC_OP_NOP             0x00
#define NACC_OP_MEMCPY          0x01

/* §6.1 NACC-REQ-035 */
#define NACC_MAX_XFER           0x100000u       /* 1 MiB */
#define NACC_XFER_ALIGN         4

/* §8.1 오류 코드 */
enum nacc_err {
	NACC_ERR_NONE       = 0x00,
	NACC_ERR_BAD_CONFIG = 0x01,
	NACC_ERR_BAD_PROD   = 0x02,
	NACC_ERR_BAD_OPCODE = 0x03,
	NACC_ERR_BAD_LENGTH = 0x04,
	NACC_ERR_BAD_ALIGN  = 0x05,
	NACC_ERR_DMA_FAULT  = 0x06,
};

static const char *nacc_err_name(u32 code)
{
	switch (code) {
	case NACC_ERR_NONE:		return "NONE";
	case NACC_ERR_BAD_CONFIG:	return "BAD_CONFIG";
	case NACC_ERR_BAD_PROD:		return "BAD_PROD";
	case NACC_ERR_BAD_OPCODE:	return "BAD_OPCODE";
	case NACC_ERR_BAD_LENGTH:	return "BAD_LENGTH";
	case NACC_ERR_BAD_ALIGN:	return "BAD_ALIGN";
	case NACC_ERR_DMA_FAULT:	return "DMA_FAULT";
	default:			return "UNKNOWN";
	}
}

static const char *nacc_state_name(u32 st)
{
	static const char * const names[] = {
		"RESET", "INIT", "READY", "BUSY", "HALTING", "ERROR",
	};

	return st < ARRAY_SIZE(names) ? names[st] : "RESERVED";
}

/*
 * 드라이버가 디스크립터에 심어두는 값.
 * 디바이스가 완료 시 덮어쓴다(NACC-REQ-033). 이 값이 그대로면
 * 그 슬롯은 처리되지 않은 것이다.
 */
#define NACC_STATUS_PENDING     0xFFFFFFFFu

/* §6.1 디스크립터 포맷. 엔디안은 NACC-REQ-014 */
struct nacc_desc {
	__le32 opcode;
	__le32 flags;
	__le64 src_addr;
	__le64 dst_addr;
	__le32 length;
	__le32 status;
} __packed;

static_assert(sizeof(struct nacc_desc) == NACC_DESC_SIZE);

/* -------------------------------------------------------------------------
 * 타임아웃
 *
 * NACC-REQ-092: HALTING 탈출은 하드웨어가 유한 시간을 보장한다.
 *               디바이스 모델의 지연이 20ms 이므로 넉넉히 50ms.
 * NACC-REQ-096: BUSY 체류는 하드웨어가 상한을 보장하지 않는다.
 *               드라이버가 상한을 소유한다. 디스크립터당 2ms 지연 +
 *               1 MiB 전송 여유를 감안해 넉넉히 잡는다.
 * ------------------------------------------------------------------------- */
#define NACC_HALT_TIMEOUT_US    50000
#define NACC_HALT_POLL_US       100
#define NACC_CMD_TIMEOUT_MS     2000

/* MEMCPY 테스트용 버퍼 크기 */
#define NACC_TEST_BUF_SIZE      (64 * 1024)

/* ------------------------------------------------------------------------- */

struct nacc_dev {
	struct pci_dev		*pdev;
	void __iomem		*bar0;

	/* 링. dma_alloc_coherent 로 잡으므로 버스 주소 공간에서 연속이다
	 * (NACC-REQ-038) */
	struct nacc_desc	*ring;
	dma_addr_t		ring_dma;
	u32			ring_size;

	/*
	 * 링 상태 전체를 이 락 하나로 보호한다.
	 * ISR 에서도 잡으므로 프로세스 문맥에서는 _irqsave 를 쓴다.
	 */
	spinlock_t		lock;

	/* NACC-REQ-076: T2 시점에 디바이스가 0 으로 밀므로 섀도도 맞춘다 */
	u32			prod;

	struct completion	done;
	u32			last_err;       /* 락 보호 */

	/* MEMCPY 테스트용 버퍼 */
	void			*src;
	dma_addr_t		src_dma;
	void			*dst;
	dma_addr_t		dst_dma;

	struct dentry		*dbg;

	u64			n_submitted;
	u64			n_completed;
	u64			n_errors;
};

static unsigned int ring_size = 8;
module_param(ring_size, uint, 0444);
MODULE_PARM_DESC(ring_size, "command ring entries (power of 2, 2..128)");

/* -------------------------------------------------------------------------
 * 레지스터 접근
 *
 * NACC-REQ-053: doorbell 과 RING_CONS 에 relaxed 변형을 쓰지 않는다.
 *   writel() 은 선행 메모리 쓰기의 완료를 먼저 기다린다 → NACC-REQ-016
 *   readl() 은 후속 메모리 읽기보다 먼저 완료된다     → NACC-REQ-018
 * relaxed 변형은 이 두 보장을 잃는다.
 * ------------------------------------------------------------------------- */

static u32 nacc_rd(struct nacc_dev *nacc, u32 off)
{
	return readl(nacc->bar0 + off);
}

static void nacc_wr(struct nacc_dev *nacc, u32 off, u32 val)
{
	writel(val, nacc->bar0 + off);
}

static u32 nacc_state(struct nacc_dev *nacc)
{
	return nacc_rd(nacc, NACC_REG_STATUS) & NACC_STATE_MASK;
}

/*
 * NACC-REQ-020: CTRL 의 예약 비트 [31:1] 을 보존해야 한다.
 * 값을 통째로 쓰지 말고 read-modify-write 한다.
 */
static void nacc_set_enable(struct nacc_dev *nacc, bool on)
{
	u32 val = nacc_rd(nacc, NACC_REG_CTRL);

	if (on)
		val |= NACC_CTRL_ENABLE;
	else
		val &= ~NACC_CTRL_ENABLE;

	nacc_wr(nacc, NACC_REG_CTRL, val);
}

/* -------------------------------------------------------------------------
 * 링 조작 — 전부 nacc->lock 을 잡은 상태에서만 호출한다
 * ------------------------------------------------------------------------- */

/*
 * NACC-REQ-039: 한 슬롯을 항상 비워둔다.
 * 하드웨어는 링 full 을 검출하지 않는다(§3.3 READY 항목 3).
 * 이 규약을 어기면 조용히 데이터가 깨진다.
 */
// 남은 ring free 수
static u32 nacc_ring_free(struct nacc_dev *nacc, u32 cons)
{
	lockdep_assert_held(&nacc->lock);

	return (cons - nacc->prod - 1) & (nacc->ring_size - 1);
}

static int nacc_submit_locked(struct nacc_dev *nacc, u32 opcode,
			      dma_addr_t src, dma_addr_t dst, u32 len)
{
	struct nacc_desc *d;
	u32 cons;

	lockdep_assert_held(&nacc->lock);

	cons = nacc_rd(nacc, NACC_REG_RING_CONS) & NACC_IDX_MASK;
	if (nacc_ring_free(nacc, cons) == 0)
		return -ENOSPC;

	/*
	 * NACC-REQ-040: 이 슬롯은 RING_CONS 가 지나간 뒤라 드라이버 소유다.
	 * 디스크립터 본문 [0x00:0x1C) 만 쓴다. status 는 디바이스 소유이지만
	 * 아직 제출 전이므로 포이즌을 심어 처리 여부를 관측한다.
	 */
	d = &nacc->ring[nacc->prod];
	d->opcode   = cpu_to_le32(opcode);
	d->flags    = 0;                        /* NACC-REQ-032 */
	d->src_addr = cpu_to_le64(src);
	d->dst_addr = cpu_to_le64(dst);
	d->length   = cpu_to_le32(len);
	d->status   = cpu_to_le32(NACC_STATUS_PENDING);

	nacc->prod = (nacc->prod + 1) & (nacc->ring_size - 1);
	nacc->n_submitted++;

	return 0;
}

/*
 * NACC-REQ-074: RING_PROD 쓰기가 곧 doorbell 이다.
 * NACC-REQ-016: 디스크립터 쓰기가 디바이스에 보여야 한다.
 *               writel() 이 선행 메모리 쓰기 완료를 기다리므로
 *               별도 dma_wmb() 는 필요 없다 (명세서 §9 Q5).
 */
//
static void nacc_doorbell_locked(struct nacc_dev *nacc)
{
	lockdep_assert_held(&nacc->lock);

	nacc_wr(nacc, NACC_REG_RING_PROD, nacc->prod);
}

/* -------------------------------------------------------------------------
 * 인터럽트 — §7.2
 * ------------------------------------------------------------------------- */

static irqreturn_t nacc_isr(int irq, void *data)
{
	struct nacc_dev *nacc = data;
	u32 stat;

	stat = nacc_rd(nacc, NACC_REG_IRQ_STATUS);
	if (!stat)
		return IRQ_NONE;

	/*
	 * NACC-REQ-062: 읽은 값을 그대로 되써서 ack 한다.
	 * val &= ~BIT 방식의 read-modify-write 는 그 사이 발생한
	 * 이벤트를 지워버린다.
	 *
	 * ack 을 처리보다 먼저 하는 이유: 처리 도중 발생한 새 이벤트가
	 * 나중의 ack 에 휩쓸리지 않게 한다.
	 */
	nacc_wr(nacc, NACC_REG_IRQ_STATUS, stat);

	spin_lock(&nacc->lock);

	if (stat & NACC_IRQ_CMD_ERR) {
		/*
		 * NACC-REQ-054 가 ERR_CODE 와 STATE 를 CMD_ERR 보다 먼저
		 * 갱신하도록 보장하므로 여기서 읽는 값은 유효하다.
		 */
		nacc->last_err = nacc_rd(nacc, NACC_REG_ERR_CODE);
		nacc->n_errors++;
	}
	if (stat & NACC_IRQ_CMD_DONE)
		nacc->n_completed++;

	spin_unlock(&nacc->lock);

	complete(&nacc->done);

	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------
 * 제출 + 완료 대기
 * ------------------------------------------------------------------------- */

/*
 * NACC-REQ-096: BUSY 체류 상한은 드라이버가 소유한다.
 * 타임아웃 시 CTRL.ENABLE <- 0 으로 T8 을 유발하고 정지 시퀀스를 수행한다.
 */
static int nacc_wait_batch(struct nacc_dev *nacc)
{
	unsigned long flags;
	u32 err;

	if (!wait_for_completion_timeout(&nacc->done,
					 msecs_to_jiffies(NACC_CMD_TIMEOUT_MS))) {
		dev_err(&nacc->pdev->dev,
			"command timeout, state=%s — aborting\n",
			nacc_state_name(nacc_state(nacc)));
		return -ETIMEDOUT;
	}

	spin_lock_irqsave(&nacc->lock, flags);
	err = nacc->last_err;
	nacc->last_err = NACC_ERR_NONE;
	spin_unlock_irqrestore(&nacc->lock, flags);

	if (err != NACC_ERR_NONE) {
		dev_err(&nacc->pdev->dev, "command failed: %s (0x%02x)\n",
			nacc_err_name(err), err);
		return -EIO;
	}

	if (nacc_state(nacc) == NACC_ST_ERROR) {
		dev_err(&nacc->pdev->dev, "device in ERROR after batch\n");
		return -EIO;
	}

	return 0;
}

/* 디스크립터 배열을 한 배치로 제출하고 완료를 기다린다 */
static int nacc_run_batch(struct nacc_dev *nacc, u32 opcode,
			  dma_addr_t src, dma_addr_t dst, u32 len, u32 count)
{
	unsigned long flags;
	u32 i;
	int ret = 0;

	if (count == 0)
		return 0;
	if (count > nacc->ring_size - 1)
		return -EINVAL;

	reinit_completion(&nacc->done);

	spin_lock_irqsave(&nacc->lock, flags);
	for (i = 0; i < count; i++) {
		ret = nacc_submit_locked(nacc, opcode, src, dst, len);
		if (ret)
			break;
	}
	// 쓴게 있으니깐 doorbell로 깨운다.
	if (i > 0)
		nacc_doorbell_locked(nacc);
	spin_unlock_irqrestore(&nacc->lock, flags);

	if (i == 0)
		return ret;

	return nacc_wait_batch(nacc);
}

/* -------------------------------------------------------------------------
 * §3.4 정지(quiesce) 시퀀스
 * ------------------------------------------------------------------------- */

static int nacc_quiesce(struct nacc_dev *nacc)
{
	unsigned long flags;
	u32 val, cons, pending;
	int ret;

	/* 1. 인터럽트 전달 차단.
	 *    3번 폴링 중에 완료 인터럽트가 도착해 해제 중인 자료구조를
	 *    핸들러가 건드리는 것을 막는다. */
	nacc_wr(nacc, NACC_REG_IRQ_MASK, NACC_IRQ_ALL);

	/* 2. T8 */
	nacc_set_enable(nacc, false);

	/* 3. NACC-REQ-092: HALTING 을 벗어난 것을 확인한 뒤에만
	 *    DMA 버퍼를 해제할 수 있다. */
	ret = readl_poll_timeout(nacc->bar0 + NACC_REG_STATUS, val,
				 (val & NACC_STATE_MASK) != NACC_ST_HALTING,
				 NACC_HALT_POLL_US, NACC_HALT_TIMEOUT_US);
	if (ret) {
		/*
		 * 하드웨어 고장으로 간주한다. 버퍼를 해제하지 않고
		 * 누수시키는 편이 안전하다 — 회수되지 않은 DMA 가
		 * 해제된 페이지를 덮어쓰는 것보다 낫다.
		 */
		dev_err(&nacc->pdev->dev,
			"stuck in HALTING; leaking ring to avoid corruption\n");
		return ret;
	}

	/* 4. NACC-REQ-095: 이 시점의 RING_CONS 가 폐기 경계다.
	 *    [0, cons) 는 완료, [cons, prod) 는 폐기되었다. */
	spin_lock_irqsave(&nacc->lock, flags);
	cons = nacc_rd(nacc, NACC_REG_RING_CONS) & NACC_IDX_MASK;
	pending = (nacc->prod - cons) & (nacc->ring_size - 1);
	spin_unlock_irqrestore(&nacc->lock, flags);

	if (pending)
		dev_warn(&nacc->pdev->dev,
			 "%u command(s) discarded at cons=%u\n", pending, cons);

	/* 5. NACC-REQ-056: 마스크를 해제하기 전에 IRQ_STATUS 를 클리어한다.
	 *    NACC-REQ-065 에 따라 마스크 해제 시 밀린 인터럽트가 즉시 뜨므로,
	 *    클리어하지 않으면 재초기화 직후 유령 인터럽트를 받는다. */
	val = nacc_rd(nacc, NACC_REG_IRQ_STATUS);
	nacc_wr(nacc, NACC_REG_IRQ_STATUS, val);

	return 0;
}

/* -------------------------------------------------------------------------
 * 초기화 시퀀스
 * ------------------------------------------------------------------------- */

static int nacc_hw_start(struct nacc_dev *nacc)
{
	struct device *dev = &nacc->pdev->dev;
	u32 st;

	/*
	 * NACC-REQ-012: LO 를 먼저, HI 를 나중에 쓴다.
	 * HI 쓰기 시점에 {HI,LO} 가 래치되고 RESET 이면 T1 이 일어난다.
	 * NACC-REQ-017 에 따라 두 쓰기 사이에 배리어는 필요 없다.
	 */
	nacc_wr(nacc, NACC_REG_RING_BASE_LO, lower_32_bits(nacc->ring_dma));
	nacc_wr(nacc, NACC_REG_RING_BASE_HI, upper_32_bits(nacc->ring_dma));

	st = nacc_state(nacc);
	if (st != NACC_ST_INIT) {
		dev_err(dev, "expected INIT after ring base write, got %s\n",
			nacc_state_name(st));
		return -EIO;
	}

	nacc_wr(nacc, NACC_REG_RING_SIZE, nacc->ring_size);

	/* NACC-REQ-056 의 짝: 해제 전에 클리어 */
	nacc_wr(nacc, NACC_REG_IRQ_STATUS,
		nacc_rd(nacc, NACC_REG_IRQ_STATUS));
	/* 핸들러가 등록된 뒤에만 마스크를 푼다 */
	nacc_wr(nacc, NACC_REG_IRQ_MASK, 0);

	/* T2 */
	nacc_set_enable(nacc, true);

	st = nacc_state(nacc);
	if (st != NACC_ST_READY) {
		u32 err = nacc_rd(nacc, NACC_REG_ERR_CODE);

		dev_err(dev, "enable failed: state=%s err=%s\n",
			nacc_state_name(st), nacc_err_name(err));
		return -EIO;
	}

	/* NACC-REQ-076: T2 가 RING_PROD 를 0 으로 밀었다. 섀도를 맞춘다 */
	nacc->prod = 0;

	return 0;
}

/* -------------------------------------------------------------------------
 * debugfs
 * ------------------------------------------------------------------------- */

/* NOP 을 count 개 실행한다. 링 용량을 넘으면 배치로 나눈다 */
static int nacc_run_nops(struct nacc_dev *nacc, u64 count)
{
	u32 per_batch = nacc->ring_size - 1;   /* NACC-REQ-039 */
	int ret;

	while (count) {
		u32 n = min_t(u64, count, per_batch);

		ret = nacc_run_batch(nacc, NACC_OP_NOP, 0, 0, 0, n);
		if (ret)
			return ret;
		count -= n;
	}
	return 0;
}

static int nacc_dbg_nop_set(void *data, u64 val)
{
	return nacc_run_nops(data, val);
}
DEFINE_DEBUGFS_ATTRIBUTE(nacc_dbg_nop_fops, NULL, nacc_dbg_nop_set, "%llu\n");

/* MEMCPY 한 번. 패턴을 채우고 결과를 검증한다 */
static int nacc_dbg_memcpy_set(void *data, u64 val)
{
	struct nacc_dev *nacc = data;
	struct device *dev = &nacc->pdev->dev;
	u32 len = (u32)val;
	u32 i;
	int ret;

	if (len == 0 || len > NACC_TEST_BUF_SIZE)
		return -EINVAL;
	if (len % NACC_XFER_ALIGN)
		return -EINVAL;         /* NACC-REQ-035 */

	for (i = 0; i < len; i++)
		((u8 *)nacc->src)[i] = (u8)(i + 0x10);
	memset(nacc->dst, 0xEE, len);

	ret = nacc_run_batch(nacc, NACC_OP_MEMCPY,
			     nacc->src_dma, nacc->dst_dma, len, 1);
	if (ret)
		return ret;

	/*
	 * NACC-REQ-018: RING_CONS 를 readl() 로 읽은 뒤이므로
	 * 별도 배리어 없이 결과를 읽을 수 있다.
	 * (완료를 인터럽트로 받았고, ISR 이 MMIO 를 읽었다)
	 */
	for (i = 0; i < len; i++) {
		u8 got = ((u8 *)nacc->dst)[i];
		u8 want = (u8)(i + 0x10);

		if (got != want) {
			dev_err(dev, "memcpy mismatch at +0x%x: %02x != %02x\n",
				i, got, want);
			return -EIO;
		}
	}

	dev_info(dev, "memcpy %u bytes verified\n", len);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(nacc_dbg_memcpy_fops, NULL, nacc_dbg_memcpy_set,
			 "%llu\n");

/*
 * M1 완료 조건 — 1000개 디스크립터를 연속 제출하고 전부 완료 통지를 받는다.
 * 링이 8칸이면 wrap 이 140회 넘게 일어난다.
 */
static int nacc_dbg_stress_set(void *data, u64 val)
{
	struct nacc_dev *nacc = data;
	u64 before = nacc->n_submitted;
	ktime_t t0 = ktime_get();
	int ret;

	ret = nacc_run_nops(nacc, val);
	if (ret)
		return ret;

	dev_info(&nacc->pdev->dev,
		 "stress: %llu descriptors in %lld us\n",
		 nacc->n_submitted - before,
		 ktime_us_delta(ktime_get(), t0));
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(nacc_dbg_stress_fops, NULL, nacc_dbg_stress_set,
			 "%llu\n");

static int nacc_regs_show(struct seq_file *s, void *unused)
{
	struct nacc_dev *nacc = s->private;
	u32 id = nacc_rd(nacc, NACC_REG_ID);
	u32 st = nacc_rd(nacc, NACC_REG_STATUS);
	u32 err = nacc_rd(nacc, NACC_REG_ERR_CODE);
	unsigned long flags;
	u32 prod_shadow;

	spin_lock_irqsave(&nacc->lock, flags);
	prod_shadow = nacc->prod;
	spin_unlock_irqrestore(&nacc->lock, flags);

	seq_printf(s, "ID           0x%08x (magic %04x rev %02x)\n",
		   id, id >> NACC_ID_MAGIC_SHIFT, id & NACC_ID_REV_MASK);
	seq_printf(s, "CTRL         0x%08x\n", nacc_rd(nacc, NACC_REG_CTRL));
	seq_printf(s, "STATUS       0x%08x (%s)\n",
		   st, nacc_state_name(st & NACC_STATE_MASK));
	seq_printf(s, "IRQ_STATUS   0x%08x\n",
		   nacc_rd(nacc, NACC_REG_IRQ_STATUS));
	seq_printf(s, "IRQ_MASK     0x%08x\n",
		   nacc_rd(nacc, NACC_REG_IRQ_MASK));
	seq_printf(s, "RING_BASE    0x%08x%08x\n",
		   nacc_rd(nacc, NACC_REG_RING_BASE_HI),
		   nacc_rd(nacc, NACC_REG_RING_BASE_LO));
	seq_printf(s, "RING_SIZE    %u\n", nacc_rd(nacc, NACC_REG_RING_SIZE));
	seq_printf(s, "RING_PROD    %u (shadow %u)\n",
		   nacc_rd(nacc, NACC_REG_RING_PROD), prod_shadow);
	seq_printf(s, "RING_CONS    %u\n", nacc_rd(nacc, NACC_REG_RING_CONS));
	seq_printf(s, "ERR_CODE     0x%02x (%s)\n", err, nacc_err_name(err));
	seq_puts(s, "\n");
	seq_printf(s, "submitted    %llu\n", nacc->n_submitted);
	seq_printf(s, "completed    %llu\n", nacc->n_completed);
	seq_printf(s, "errors       %llu\n", nacc->n_errors);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(nacc_regs);

static void nacc_debugfs_init(struct nacc_dev *nacc)
{
	nacc->dbg = debugfs_create_dir(pci_name(nacc->pdev), NULL);

	debugfs_create_file("regs", 0400, nacc->dbg, nacc, &nacc_regs_fops);
	debugfs_create_file_unsafe("nop", 0200, nacc->dbg, nacc,
				   &nacc_dbg_nop_fops);
	debugfs_create_file_unsafe("memcpy", 0200, nacc->dbg, nacc,
				   &nacc_dbg_memcpy_fops);
	debugfs_create_file_unsafe("stress", 0200, nacc->dbg, nacc,
				   &nacc_dbg_stress_fops);
}

/* -------------------------------------------------------------------------
 * probe / remove
 * ------------------------------------------------------------------------- */

static int nacc_check_id(struct nacc_dev *nacc)
{
	struct device *dev = &nacc->pdev->dev;
	u32 id = nacc_rd(nacc, NACC_REG_ID);
	u32 magic = id >> NACC_ID_MAGIC_SHIFT;
	u32 rev = id & NACC_ID_REV_MASK;

	/*
	 * NACC-REQ-024: probe 시 MAGIC 을 확인한다.
	 * 전체가 0xFFFFFFFF 이면 대개 링크 다운이다.
	 * §4.5 에 따라 정의되지 않은 오프셋은 0 을 반환하므로
	 * 0xFFFFFFFF 는 링크 다운 전용 신호로 해석할 수 있다.
	 */
	if (id == 0xFFFFFFFF) {
		dev_err(dev, "device not responding (link down?)\n");
		return -ENODEV;
	}
	if (magic != NACC_ID_MAGIC) {
		dev_err(dev, "bad magic 0x%04x (expected 0x%04x)\n",
			magic, NACC_ID_MAGIC);
		return -ENODEV;
	}

	/* NACC-REQ-007 */
	if (rev < NACC_REVISION_KNOWN) {
		dev_err(dev, "revision %u is older than %u\n",
			rev, NACC_REVISION_KNOWN);
		return -ENODEV;
	}
	if (rev > NACC_REVISION_KNOWN)
		dev_warn(dev, "revision %u is newer than %u; assuming compatible\n",
			 rev, NACC_REVISION_KNOWN);

	return 0;
}

static int nacc_alloc_buffers(struct nacc_dev *nacc)
{
	struct device *dev = &nacc->pdev->dev;
	size_t ring_bytes = (size_t)nacc->ring_size * NACC_DESC_SIZE;

	/*
	 * NACC-REQ-038: 링 전체가 버스 주소 공간에서 연속이어야 한다.
	 * dma_alloc_coherent 한 번의 호출이면 조건이 만족된다.
	 * NACC-REQ-037: 32바이트 정렬 — coherent 할당은 페이지 정렬이므로
	 * 자동으로 만족된다.
	 *
	 * 링 메모리를 0 으로 초기화할 필요는 없다. §3.2 T2 에 따라
	 * 디바이스는 첫 doorbell 전까지 링을 읽지 않는다.
	 */
	nacc->ring = dma_alloc_coherent(dev, ring_bytes, &nacc->ring_dma,
					GFP_KERNEL);
	if (!nacc->ring)
		return -ENOMEM;

	nacc->src = dma_alloc_coherent(dev, NACC_TEST_BUF_SIZE,
				       &nacc->src_dma, GFP_KERNEL);
	if (!nacc->src)
		goto err_ring;

	nacc->dst = dma_alloc_coherent(dev, NACC_TEST_BUF_SIZE,
				       &nacc->dst_dma, GFP_KERNEL);
	if (!nacc->dst)
		goto err_src;

	return 0;

err_src:
	dma_free_coherent(dev, NACC_TEST_BUF_SIZE, nacc->src, nacc->src_dma);
	nacc->src = NULL;
err_ring:
	dma_free_coherent(dev, ring_bytes, nacc->ring, nacc->ring_dma);
	nacc->ring = NULL;
	return -ENOMEM;
}

static void nacc_free_buffers(struct nacc_dev *nacc)
{
	struct device *dev = &nacc->pdev->dev;
	size_t ring_bytes = (size_t)nacc->ring_size * NACC_DESC_SIZE;

	if (nacc->dst)
		dma_free_coherent(dev, NACC_TEST_BUF_SIZE, nacc->dst,
				  nacc->dst_dma);
	if (nacc->src)
		dma_free_coherent(dev, NACC_TEST_BUF_SIZE, nacc->src,
				  nacc->src_dma);
	if (nacc->ring)
		dma_free_coherent(dev, ring_bytes, nacc->ring, nacc->ring_dma);
}

static int nacc_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct nacc_dev *nacc;
	int ret, irq;

	if (ring_size < NACC_RING_SIZE_MIN || ring_size > NACC_RING_SIZE_MAX ||
	    !is_power_of_2(ring_size)) {
		dev_err(dev, "ring_size must be a power of 2 in [%u, %u]\n",
			NACC_RING_SIZE_MIN, NACC_RING_SIZE_MAX);
		return -EINVAL;
	}

	nacc = devm_kzalloc(dev, sizeof(*nacc), GFP_KERNEL);
	if (!nacc)
		return -ENOMEM;

	nacc->pdev = pdev;
	nacc->ring_size = ring_size;
	spin_lock_init(&nacc->lock);
	init_completion(&nacc->done);

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	/* NACC-REQ-052: 기본 I/O 속성으로 매핑한다.
	 * write-combining 등 비기본 속성은 §4.3 의 순서 보장을 잃는다. */
	ret = pcim_iomap_regions(pdev, BIT(0), KBUILD_MODNAME);
	if (ret)
		return ret;
	nacc->bar0 = pcim_iomap_table(pdev)[0];

	ret = nacc_check_id(nacc);
	if (ret)
		return ret;

	/* NACC-REQ-072: 링 주소는 버스 주소(IOMMU 환경에서는 IOVA)다.
	 * RING_BASE 가 64비트이므로 64비트 마스크를 시도한다. */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(dev, "no usable DMA mask\n");
			return ret;
		}
	}

	/*
	 * NACC-REQ-006: MSI 는 호스트 메모리로 향하는 posted write 이므로
	 * DMA 와 같은 upstream 경로를 쓴다. BME 가 0 이면 인터럽트도
	 * 디스크립터 페치도 되지 않는다.
	 */
	pci_set_master(pdev);

	ret = nacc_alloc_buffers(nacc);
	if (ret)
		return ret;

	/* NACC-REQ-004: MSI 1벡터. INTx 는 지원하지 않으므로 폴백하지 않는다 */
	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
	if (ret < 0) {
		dev_err(dev, "failed to allocate MSI vector\n");
		goto err_buffers;
	}

	irq = pci_irq_vector(pdev, 0);
	ret = request_irq(irq, nacc_isr, 0, KBUILD_MODNAME, nacc);
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", ret);
		goto err_vectors;
	}

	ret = nacc_hw_start(nacc);
	if (ret)
		goto err_irq;

	pci_set_drvdata(pdev, nacc);
	nacc_debugfs_init(nacc);

	dev_info(dev, "ready: ring %u entries at %pad, irq %d, msi=%d\n",
		 nacc->ring_size, &nacc->ring_dma, irq, pdev->msi_enabled);

	return 0;

err_irq:
	free_irq(irq, nacc);
err_vectors:
	pci_free_irq_vectors(pdev);
err_buffers:
	nacc_free_buffers(nacc);
	return ret;
}

static void nacc_remove(struct pci_dev *pdev)
{
	struct nacc_dev *nacc = pci_get_drvdata(pdev);
	int ret;

	debugfs_remove_recursive(nacc->dbg);

	/* §3.4 표준 정지 시퀀스 */
	ret = nacc_quiesce(nacc);

	free_irq(pci_irq_vector(pdev, 0), nacc);
	pci_free_irq_vectors(pdev);

	/*
	 * NACC-REQ-092: HALTING 을 벗어난 것을 확인한 뒤에만 해제한다.
	 * 실패했다면 회수되지 않은 DMA 가 남아 있을 수 있으므로
	 * 누수를 감수한다.
	 */
	if (!ret)
		nacc_free_buffers(nacc);

	dev_info(&pdev->dev, "removed (submitted=%llu completed=%llu errors=%llu)\n",
		 nacc->n_submitted, nacc->n_completed, nacc->n_errors);
}

static const struct pci_device_id nacc_ids[] = {
	{ PCI_DEVICE(NACC_VENDOR_ID, NACC_DEVICE_ID) },
	{ }
};
MODULE_DEVICE_TABLE(pci, nacc_ids);

static struct pci_driver nacc_driver = {
	.name     = KBUILD_MODNAME,
	.id_table = nacc_ids,
	.probe    = nacc_probe,
	.remove   = nacc_remove,
};
module_pci_driver(nacc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for the nacc virtual PCIe accelerator");