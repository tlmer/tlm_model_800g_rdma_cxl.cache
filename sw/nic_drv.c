//============================================================================
// nic_drv.c — nic 平台设备驱动【骨架】  [2026-10-04 建]
//
// ★ 覆盖:平台设备 probe/remove + MMIO(2MB 窗)+ **wire 中断**(电平;读-清 = INTR_STS **W1C**)
//   + 最小字符设备(wait 队列,给用户态等中断)✓
// ⚠ **明确是骨架**:数据面(QP 建/门铃/CQE 轮询/用户态 mmap)只留接口 + TODO ✗
//   完整面随 P4.3/P4.4 与客户联调补(客户 B2 的 Linux 版本号待抄)
//
// ★ 序列口径**不在此处发明**:寄存器/位域/门铃/中断全走
//   `nic_regs.h` + `nic_hw.h`(逐条带规格出处;已被模型 TB 实测对拍)✓
//   —— 即:本驱动与模型自测走**同一套序列代码**,差异只落在 readl/writel 与模型直连两种原语上 ✓
//
// 设备树契约:`compatible = "rnl,nic"`(⚠ 与 `nic.dtsi` 草案同串;正式串待定)
//============================================================================
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/mm.h>

#include "nic_regs.h"
#include "nic_hw.h"

#define NIC_DRV_NAME "nic"

struct nic_dev {
	struct device *dev;
	void __iomem *base;			/* 2MB 配置窗 ✓ */
	phys_addr_t    phys;			/* 窗物理基(用户态 mmap 用)✓ */
	int irq;
	nic_io_t io;
	struct miscdevice miscdev;		/* /dev/nic(用户态面)✓ */
	wait_queue_head_t wq;			/* 中断等待队列(poll/read 用)✓ */
	nic_u32 intr_cnt, read_cnt;		/* 中断序号 / 已读序号 ⇒ "有没有新中断" ✓ */
	nic_u32 last_sts;			/* 最近一次读到的 INTR_STS(读给用户态)✓ */
};

/* ---- 原语:内核侧 = readl/writel(窗口内偏移)✓ ---- */
static nic_u32 nic_krd32(void *ctx, nic_u32 off)
{
	return readl((nic_u8 *)ctx + off);
}
static void nic_kwr32(void *ctx, nic_u32 off, nic_u32 v)
{
	writel(v, (nic_u8 *)ctx + off);
}

/* ---- 中断:读状态 ⇒ W1C 清 ⇒ 唤醒(照 §10.1:一根电平线,多源 OR)✓ ---- */
static irqreturn_t nic_isr(int irq, void *p)
{
	struct nic_dev *d = p;
	nic_u32 sts = nic_intr_status(&d->io);

	if (!sts)
		return IRQ_NONE;			/* 共享线/毛刺:不是我们的 ⚠ 线式独占时可省 */

	d->last_sts = sts;			/* 留给用户态读(骨架)⚠ */
	/* ★ 非 CQ 位:W1C 按位清(照 §10.3);**CQ 位(bit4)写它不清** —— 照 RTL 死路(/)✗ */
	nic_intr_ack(&d->io, sts & ~NIC_INTR_WQE_CMPL);
	if (sts & NIC_INTR_WQE_CMPL)
		nic_cq_intr_ack(&d->io, 0xFFFFFFFFu);	/* ⚠ 骨架:全 QP 拍平;真面按"已处理到哪个 QP"给掩码 ✗ */
	d->intr_cnt++;
	wake_up_interruptible(&d->wq);		/* ⚠ 骨架:真面按源分流(§10.2 九源)✗ */
	return IRQ_HANDLED;
}

/*============================================================================
 * 用户态面(⚠ **骨架**;mmap 面的**形态待 A7 答复** ⇒ 现按"平台直连"占位)✗
 *   ① read/poll = 等中断(驱动 ISR 唤醒)⇒ 用户态做"中断驱动完成"✓
 *   ② mmap = 把配置窗映射给用户态 ⇒ **门铃直写**(SQ_PI_DB/RQ_CI_DB)、不必逐次进内核 ✓
 *      ⚠ 暴露面要**收窄**(只映射门铃/QP 段,不整段;段边界见 `nic_regs.h`)—— 骨架先整段 ✗
 *============================================================================*/
static int nic_open(struct inode *ino, struct file *f)
{
	struct nic_dev *d =
		container_of(f->private_data, struct nic_dev, miscdev);

	f->private_data = d;
	return 0;
}
static int nic_release(struct inode *ino, struct file *f) { return 0; }

static ssize_t nic_read(struct file *f, char __user *ub, size_t n, loff_t *off)
{
	struct nic_dev *d = f->private_data;
	nic_u32 sts;

	if (n < sizeof(sts))
		return -EINVAL;
	if (wait_event_interruptible(d->wq, d->intr_cnt != d->read_cnt))	/* 等"有新中断" ✓ */
		return -ERESTARTSYS;
	d->read_cnt = d->intr_cnt;
	sts = d->last_sts;			/* ISR 已 W1C 应答 ⇒ 给快照 ✓ */
	if (copy_to_user(ub, &sts, sizeof(sts)))
		return -EFAULT;
	return (ssize_t)sizeof(sts);
}

static unsigned int nic_poll(struct file *f, poll_table *pt)
{
	struct nic_dev *d = f->private_data;

	poll_wait(f, &d->wq, pt);
	return (d->intr_cnt != d->read_cnt) ? (POLLIN | POLLRDNORM) : 0;
}

static int nic_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct nic_dev *d = f->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	if (off >= NIC_SIZE)
		return -EINVAL;
	/* ⚠ A7 占位:整窗映射(收窄到门铃/QP 段 = TODO)✗ */
	return remap_pfn_range(vma, vma->vm_start,
			       (d->phys + off) >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot);
}

static const struct file_operations nic_fops = {
	.owner   = THIS_MODULE,
	.open    = nic_open,
	.release = nic_release,
	.read    = nic_read,
	.poll    = nic_poll,
	.mmap    = nic_mmap,
};

/* ---- 探针:MMIO + 中断(设备树 reg/interrupts 契约见 nic.dtsi)✓ ---- */
static int nic_probe(struct platform_device *pdev)
{
	struct nic_dev *d;
	struct resource *res;
	int ret;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = &pdev->dev;
	platform_set_drvdata(pdev, d);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	if (resource_size(res) < NIC_SIZE) {	/* ⚠ 窗必须 ≥ 2MB(§8.1 21 位译码)✗ */
		dev_err(d->dev, "reg 窗 %pa 小于 2MB(§8.1)\n", &res->start);
		return -EINVAL;
	}
	d->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(d->base))
		return PTR_ERR(d->base);
	d->phys = res->start;			/* mmap 用 ✓ */

	d->io.ctx  = d->base;
	d->io.rd32 = nic_krd32;
	d->io.wr32 = nic_kwr32;

	d->irq = platform_get_irq(pdev, 0);		/* ★ wire 中断(单条;B4)✓ */
	if (d->irq < 0)
		return d->irq;
	init_waitqueue_head(&d->wq);
	ret = devm_request_irq(d->dev, d->irq, nic_isr, IRQF_SHARED,
			       NIC_DRV_NAME, d);
	if (ret)
		return ret;

	/* 复位后中断全屏蔽(§10.3)⇒ 驱动按需开 ---- ⚠ 骨架:只开 CQ/RNR 两源,余按场景 ✗ */
	nic_intr_enable(&d->io, NIC_INTR_WQE_CMPL | NIC_INTR_RNR_NAK);

	d->miscdev.minor = MISC_DYNAMIC_MINOR;
	d->miscdev.name  = NIC_DRV_NAME;
	d->miscdev.fops  = &nic_fops;
	d->miscdev.parent = d->dev;
	ret = misc_register(&d->miscdev);	/* ⇒ /dev/nic ✓ */
	if (ret)
		return ret;

	dev_info(d->dev, NIC_DRV_NAME " ready: reg=%pa irq=%d version=0x%08x\n",
		 &res->start, d->irq, nic_rd(&d->io, NIC_GLB(NIC_VERSION)));
	/* TODO(P4.3):QP 建/用户态面(mmap 门铃与 QP 窗)/验收序列 ✗ */
	return 0;
}

static int nic_remove(struct platform_device *pdev)
{
	struct nic_dev *d = platform_get_drvdata(pdev);

	misc_deregister(&d->miscdev);
	nic_intr_enable(&d->io, 0);			/* 关中断再走(电平线)✓ */
	dev_info(d->dev, NIC_DRV_NAME " removed(intr=%u)\n", d->intr_cnt);
	return 0;
}

static const struct of_device_id nic_of_match[] = {
	{ .compatible = "rnl,nic" },		/* ⚠ 与 nic.dtsi 草案同串;正式串待定 ✗ */
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, nic_of_match);

static struct platform_driver nic_driver = {
	.probe  = nic_probe,
	.remove = nic_remove,
	.driver = {
		.name           = NIC_DRV_NAME,
		.of_match_table = nic_of_match,
	},
};
module_platform_driver(nic_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("nic platform driver (skeleton; NIC MMIO + wire IRQ)");
