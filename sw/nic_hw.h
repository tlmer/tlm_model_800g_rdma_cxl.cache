/*============================================================================
 * nic_hw.h — 硬件访问层(OS 无关;header-only)  [2026-10-04 建]
 *
 * ★ 用途:**驱动(内核/用户态)与模型自测(TB)走同一套序列** —— 序列写一次、两处验证 ✓
 *   使用方只提供两个原语(窗口内偏移 ⇒ 32 位读写):
 *     · 内核:  readl(base + off) / writel(v, base + off)     (base = ioremap 的 2MB 窗)
 *     · TB:    DeviceTlm::reg_read / reg_write(经 `sw/` 同款地址换算)✓
 *   ⚠ 本文件**不发明**任何寄存器/位域:全部来自 `nic_regs.h`(逐条带规格出处)✓
 *
 * ★ 门铃/中断的序列口径(规格 §8.5 / §10.3):
 *   · SQ 门铃 = **写 per-QP `SQ_PI_DB`**(off 0x38,值 = 生产者指针)✓
 *   · RQ 门铃 = **写 per-QP `RQ_CI_DB`**(off 0x34,值 = 消费者指针)✓
 *   · 中断 = 读 `INTR_STS` → **W1C 按位清**(写回读到的位;⚠ **bit4 例外** —— 写它不清 CQ 中断,
 *     照 RTL 死路)⇒ **CQ 完成中断的应答 = 写 `CQ_INTR_STS1` 掩码**(见 `nic_cq_intr_ack`)✓
 *   · RQ 生产指针(SW 可见面)= **读 RO `STAT_RQ_PI_DB`**(off 0x9C)✓(模型 v1.0 起引擎真源)
 *============================================================================*/
#ifndef NIC_HW_H
#define NIC_HW_H

/* ⚠ 双环境类型:内核空间没有 <stdint.h>(用 linux/types.h)⇒ 统一走 nic_u32/nic_u8 ✓ */
#ifdef __KERNEL__
#include <linux/types.h>
typedef u32    nic_u32;
typedef u16    nic_u16;
typedef u8     nic_u8;
typedef u64    nic_u64;
#else
#include <stdint.h>
typedef uint32_t nic_u32;
typedef uint16_t nic_u16;
typedef uint8_t  nic_u8;
typedef uint64_t nic_u64;
#endif
#include "nic_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 原语接口:窗口内偏移(0..0x1FFFFF)⇒ 32 位读写 ✓ ---- */
typedef struct {
	void     *ctx;                                  /* 使用方上下文(基址/对象) */
	nic_u32 (*rd32)(void *ctx, nic_u32 off);
	void     (*wr32)(void *ctx, nic_u32 off, nic_u32 v);
} nic_io_t;

#define NIC_WOFF(a)   ((nic_u32)((a) - NIC_BASE))   /* 全地址 ⇒ 窗口内偏移 ✓ */

/* ---- 寄存器访问(全地址宏 ⇒ 窗口偏移)✓ ---- */
static inline nic_u32 nic_rd(nic_io_t *io, nic_u32 abs_addr)
{
	return io->rd32(io->ctx, NIC_WOFF(abs_addr));
}
static inline void nic_wr(nic_io_t *io, nic_u32 abs_addr, nic_u32 v)
{
	io->wr32(io->ctx, NIC_WOFF(abs_addr), v);
}

/* ---- 中断:读状态 / W1C 清(★ 写回**读到的位**)✓ ---- */
static inline nic_u32 nic_intr_status(nic_io_t *io)
{
	return nic_rd(io, NIC_GLB(NIC_INTR_STS));
}
/* ★ CQ 完成中断应答(★ /D14 定案路径:写 `CQ_INTR_STS1`(bank0 = QP0..31);mask 1 = 清对应 QP)✓ */
static inline void nic_cq_intr_ack(nic_io_t *io, nic_u32 qp_mask)
{
	nic_wr(io, NIC_GLB_CQ_INTR_STS(1), qp_mask);
}

static inline void nic_intr_ack(nic_io_t *io, nic_u32 bits)
{
	nic_wr(io, NIC_GLB(NIC_INTR_STS), bits);        /* W1C ✓ */
}
static inline void nic_intr_enable(nic_io_t *io, nic_u32 bits)
{
	nic_wr(io, NIC_GLB(NIC_INTR_EN), bits);         /* RW;复位全屏蔽 ✓ */
}

/* ---- 门铃(写触发;照 §8.5)✓ ---- */
static inline void nic_doorbell_sq(nic_io_t *io, nic_u32 qp_idx, nic_u16 pi)
{
	nic_wr(io, NIC_QP(qp_idx) + NIC_QP_SQ_PI_DB, pi);
}
static inline void nic_doorbell_rq(nic_io_t *io, nic_u32 qp_idx, nic_u16 ci)
{
	nic_wr(io, NIC_QP(qp_idx) + NIC_QP_RQ_CI_DB, ci);
}
/* RQ 生产指针(SW 可见面;RO 回读)✓ */
static inline nic_u32 nic_stat_rq_pi(nic_io_t *io, nic_u32 qp_idx)
{
	return nic_rd(io, NIC_QP(qp_idx) + NIC_QP_STAT_RQ_PI);
}

/* ---- QP 配置(驱动 init 序列;字段 = 寄存器原生布局 ✓)---- */
typedef struct {
	nic_u32 qp_idx;        /* 地址索引(★ 厂商 QPn = 索引+1)✓ */
	nic_u32 sq_ba;         /* 字节地址([31:5];64B 对齐)✓ */
	nic_u32 cq_ba;         /* 字节地址([31:5])✓ */
	nic_u32 rq_ba;         /* 字节地址([31:8];**256B 粒度**)✓ */
	nic_u32 sq_depth;      /* 1..65535(指针 1..depth→1)✓ */
	nic_u32 rq_depth;      /* 同上(容量 = depth−1)✓ */
	nic_u32 rq_buf_sz;     /* RQ 项步长(**单位 256B**;QP_CONF[31:24])✓ */
	nic_u32 pmtu;          /* QP_CONF[10:8] ✓ */
} nic_qp_cfg_t;

static inline void nic_qp_init(nic_io_t *io, const nic_qp_cfg_t *c)
{
	nic_u32 base = NIC_QP(c->qp_idx);
	nic_u32 conf = NIC_QPC_QP_EN
	              | ((c->pmtu & 0x7u) << 8)
	              | ((c->rq_buf_sz & 0xFFu) << 24);
	nic_wr(io, base + NIC_QP_RQ_BUF_BA, c->rq_ba);       /* 256B 粒度 ✓ */
	nic_wr(io, base + NIC_QP_SQ_BA,     c->sq_ba);
	nic_wr(io, base + NIC_QP_CQ_BA,     c->cq_ba);
	nic_wr(io, base + NIC_QP_Q_DEPTH,   (c->rq_depth << 16) | (c->sq_depth & 0xFFFFu));
	nic_wr(io, base + NIC_QP_CONF,      conf);           /* ★ QP_EN 最后写(照惯例)✓ */
}

/* ---- ★ CM 协商结果落表(规格文档 §2.4;⚠ 直配 nic_qp_init 逐字节不动 = 双路并存)----
 * 调用方(TB/驱动 CM 回调)把 CM 协商值换算好后传入:
 *   lpsn   = 本侧起始 PSN(同 REP 报出值)                ⇒ QP_SQ_PSN(0x40)
 *   rqpn   = 对端 QPN(REQ/REP 的 Local QPN)             ⇒ QP_DEST_QP(0x48)
 *   r_ipv4 = 对端 IPv4(GID[12..15],**小端装载**;
 *            用 `cm_gid_to_ipv4_reg()`(nic_cm.h)换算) ⇒ QP_IP_REMOTE1(0x60)
 *   pkey   = P_Key(REQ 携带,**线上形态** 0x666f;函数内做字节反序) ⇒ QP_ADV_CONF[31:16](0x04)
 * ⚠ MTU:协商值由 c->pmtu 带(两侧 = REQ 提议值)✓
 * ⚠ P_Key 寄存器形态 = **线上字节反序**(实证:tb_roce_engine_m1 写 reg[31:16]=0x6f66 ⇒
 *    线上 BTH 字节 0x66,0x6f = **0x666f** ✓)—— 函数内自动换序 ✓ */
static inline void nic_qp_init_cm(nic_io_t *io, const nic_qp_cfg_t *c,
                                   nic_u32 lpsn, nic_u32 rqpn,
                                   nic_u32 r_ipv4, nic_u16 pkey)
{
	nic_u32 base = NIC_QP(c->qp_idx);
	nic_u32 pkey_reg = ((nic_u32)((pkey >> 8) | (pkey << 8))) << 16;   /* ★ 字节反序 ✓ */
	nic_qp_init(io, c);                                      /* 基配(与直配同源) */
	nic_wr(io, base + NIC_QP_SQ_PSN,     lpsn & 0xFFFFFFu);
	nic_wr(io, base + NIC_QP_DEST_QP,    rqpn & 0xFFFFFFu);
	nic_wr(io, base + NIC_QP_IP_REMOTE1, r_ipv4);
	nic_wr(io, base + NIC_QP_ADV_CONF,   pkey_reg);
}

/* ---- ★ M2:verbs 三段(INIT→RTR→RTS;逐段读回见 规格文档 §2.2)----
 *   INIT:本端参数(SQ/CQ/深度 + **SQ 起始 PSN**)⇒ CONF=0(**未使能**)
 *   RTR :对端参数(DEST_QP / IP_REMOTE / P_Key)+ MTU ⇒ CONF 仍**未使能**
 *   RTS :**RMW 置 EN**(读 QP_CONF |= QP_EN 写回)⇒ 此后才发 WQE ✓ */
static inline void nic_qp_to_init(nic_io_t *io, nic_u32 qp_idx, nic_u32 sq_ba,
                                   nic_u32 cq_ba, nic_u32 q_depth, nic_u32 lpsn)
{
	nic_u32 base = NIC_QP(qp_idx);
	nic_wr(io, base + NIC_QP_SQ_BA,   sq_ba);
	nic_wr(io, base + NIC_QP_CQ_BA,   cq_ba);
	nic_wr(io, base + NIC_QP_Q_DEPTH, q_depth);
	nic_wr(io, base + NIC_QP_SQ_PSN,  lpsn & 0xFFFFFFu);
	nic_wr(io, base + NIC_QP_CONF,    0);                    /* ★ EN=0 ✓ */
}

static inline void nic_qp_to_rtr(nic_io_t *io, nic_u32 qp_idx, nic_u32 rqpn,
                                  nic_u32 r_ipv4, nic_u16 pkey, nic_u32 pmtu)
{
	nic_u32 base = NIC_QP(qp_idx);
	nic_wr(io, base + NIC_QP_DEST_QP,    rqpn & 0xFFFFFFu);
	nic_wr(io, base + NIC_QP_IP_REMOTE1, r_ipv4);
	nic_wr(io, base + NIC_QP_ADV_CONF,   ((nic_u32)((pkey >> 8) | (pkey << 8))) << 16);
	nic_wr(io, base + NIC_QP_CONF,       ((pmtu & 7u) << 8));  /* ★ EN 仍 0 ✓ */
}

static inline void nic_qp_to_rts(nic_io_t *io, nic_u32 qp_idx)
{
	nic_u32 base = NIC_QP(qp_idx);
	nic_wr(io, base + NIC_QP_CONF, nic_rd(io, base + NIC_QP_CONF) | NIC_QPC_QP_EN);
}

/* ---- WQE 装填(↔ 模型解析同一布局宏)✓ ---- */
static inline void nic_wqe_put32(nic_u8 *wqe, nic_u32 off_bytes, nic_u32 v)
{
	wqe[off_bytes + 0] = (nic_u8)(v & 0xFF);
	wqe[off_bytes + 1] = (nic_u8)((v >> 8) & 0xFF);
	wqe[off_bytes + 2] = (nic_u8)((v >> 16) & 0xFF);
	wqe[off_bytes + 3] = (nic_u8)((v >> 24) & 0xFF);
}
static inline void nic_wqe_build(nic_u8 *wqe /* 64B,先清零 */,
	nic_u16 wrid, nic_u8 opcode, nic_u64 local_off, nic_u64 remote_off, nic_u32 dma_len)
{
	wqe[NIC_WQE_WRID_OFF + 0] = (nic_u8)(wrid & 0xFF);
	wqe[NIC_WQE_WRID_OFF + 1] = (nic_u8)((wrid >> 8) & 0xFF);
	nic_wqe_put32(wqe, NIC_WQE_LOFF_OFF, (nic_u32)local_off);
	nic_wqe_put32(wqe, NIC_WQE_LOFF_OFF + 4, (nic_u32)(local_off >> 32));
	nic_wqe_put32(wqe, NIC_WQE_LEN_OFF, dma_len);
	wqe[NIC_WQE_OPC_OFF] = opcode;
	nic_wqe_put32(wqe, NIC_WQE_ROFF_OFF, (nic_u32)remote_off);
	nic_wqe_put32(wqe, NIC_WQE_ROFF_OFF + 4, (nic_u32)(remote_off >> 32));
}

/* ---- CQE 取用(驱动 poll 自己的 CQ 缓冲;布局 ⇔ §9.4)✓ ---- */
static inline const nic_u8 *nic_cqe_at(const nic_u8 *cq_base, nic_u32 idx)
{
	return cq_base + (idx * NIC_CQE_BYTES);   /* cq_ba + idx*4 ✓ */
}

/*============================================================================
 * ★ 数据面辅助(队列指针口径,照 §9.2 —— **模型 / 驱动 / TB 同一套**)✓
 *   · 指针 1-based:`1..depth`,到 depth 回 1 ✓
 *   · **地址索引 = (ptr == depth) ? 0 : ptr**(即 `depth` 映射到物理 0)✓
 *   · ★ **CQ 深度 = SQ 深度**(RTL 实证 o_qp_cq_depth = o_qp_sq_depth`)✓
 *============================================================================*/
static inline nic_u32 nic_ring_idx(nic_u32 ptr, nic_u32 depth)      /* 指针 ⇒ 物理索引 ✓ */
{
	return (ptr == depth) ? 0u : ptr;
}
static inline nic_u16 nic_ring_next(nic_u32 ptr, nic_u32 depth)     /* 指针推进(回绕)✓ */
{
	return (nic_u16)((ptr == depth) ? 1u : ptr + 1u);
}
/* SQ 门铃(按指针口径;内部换算到物理无关 —— 门铃值就是指针本身)✓ */
static inline void nic_sq_post(nic_io_t *io, nic_u32 qp_idx, nic_u16 pi)
{
	nic_doorbell_sq(io, qp_idx, pi);
}
/* 按**指针**取 WQE 行(驱动侧地址计算 = `sq_ba + idx*64`,idx = (ptr==depth)?0:ptr)✓ */
static inline const nic_u8 *nic_wqe_at(const nic_u8 *sq_base, nic_u32 depth, nic_u16 pi)
{
	return sq_base + (nic_ring_idx(pi, depth) * NIC_WQE_BYTES);
}
/* 按**指针**取 CQE(cq_idx = (hd==depth)?0:hd ⇒ cq_ba + idx*4)✓ */
static inline const nic_u8 *nic_cqe_at_ptr(const nic_u8 *cq_base, nic_u32 depth, nic_u16 hd)
{
	return nic_cqe_at(cq_base, nic_ring_idx(hd, depth));
}
/* CQ 头指针(SW 侧消费者索引 / 可 override;⚠ 覆盖语义待开卷)✓ */
static inline void nic_cq_head_set(nic_io_t *io, nic_u32 qp_idx, nic_u16 hd)
{
	nic_wr(io, NIC_QP(qp_idx) + NIC_QP_CQ_HEAD, hd);
}

#ifdef __cplusplus
}
#endif
#endif /* NIC_HW_H */
