/*============================================================================
 * nic_regs.h — nic(NIC 配置面)寄存器 / 位域 / 描述符定义  [2026-10-04 建]
 *
 * ★★ 本文件 = **模型 / 驱动 / app / 脚本 的共同基准**(照 `规格文档` §8)——
 *    每条都带出处(§8.x 或 `RTL 源:行号`);⚠ **改这里必须同步规格册,反之亦然** ✓
 *
 * ★ 地址口径(照规格 §8.1,RTL 实证):
 *    · 只译**低 21 位** ⇒ 整个寄存器窗 = **2MB**
 *    · 段选择 = addr[20:19]:`00/01` = PD 段 / `10` = 全局段 / `11` = **per-QP 段**
 *    · per-QP **块 = 0x100**,索引 = `addr[15:8]`;**厂商 QPn = 索引 + 1** ✓
 *    · 全局段只选 `addr[10:2]` ⇒ 每 **0x800 别名** ✓
 *
 * ★ SoC 侧基址 = **我方分配**(客户 B3:"总线地址与中断号由你们分配")⇒
 *   定案 `0x5000_0000` + 2MB(与 RTL 台现用一致;设备树草案 ⇒ 本目录 `nic.dtsi`)✓
 *============================================================================*/
#ifndef NIC_REGS_H
#define NIC_REGS_H

/* ---- 窗口 ---- */
#define NIC_BASE        0x50000000u          /* SoC 分配(我方定案)✓ */
#define NIC_SIZE        0x00200000u          /* 21 位译码 ⇒ 2MB ✓(§8.1) */

/* ---- PD 段:`0x5000_0000 + n*0x100`,n = addr[15:8](规格 §8.2;厂商列 n=0..5)---- */
#define NIC_PD(n)           (NIC_BASE + ((n) << 8))
#define NIC_PD_VIRT_ADDR0   0x04             /* PD 虚拟地址低位 ✓(本模型已用) */
#define NIC_PD_VIRT_ADDR1   0x08             /* PD 虚拟地址高位 ✓ */

/* ---- 全局段:`0x5010_0000 + off`(仅 [10:2] ⇒ 每 0x800 别名,§8.1)---- */
/* ★ D16 开卷补齐(2026-10-05;出处 `RTL 源` 全局表  解码)✓ */
#define NIC_NIC_CONF      0x000   /* [0]en [1]ipver [2]bypass [4:3]tx_ack_gen [5]err_buf_en
                                           [7:6]flow_credits **[23:8]udp_src_port** ✓ */
#define NIC_MAC_NIC_LSB   0x010   /* 本机 MAC 低 32(头字节低先;LSB/MSB 拼接家族)✓ */
#define NIC_MAC_NIC_MSB   0x014   /* 本机 MAC 高 16 ✓ */
#define NIC_IPV4_NIC_ADDR 0x070   /* 本机 IPv4 源地址/IPV4_NIC_ADDR_REG)✓ */
#define NIC_GLB(off)        (NIC_BASE + 0x00100000u + ((off) & 0x7FFu))
#define NIC_VERSION         0x01C            /* 识别/版本类寄存器 —— 【模型刻意不实现】:地址可读但恒 0、写忽略(交付口径:不出 IP 识别面)✓ */
#define NIC_INTR_EN         0x180            /* INTR_EN_REG(RW;复位 0 = 全屏蔽)✓ §8.3 */
#define NIC_INTR_STS        0x184            /* INTR_STS_REG(**W1C**;按位)✓ §8.3 */

/* ---- 中断位(规格 §10.2;一根电平线 `nic_intr`)---- */
#define NIC_INTR_HDR_ERR        (1u << 0)    /* 入站包头校验错 */
#define NIC_INTR_MAD            (1u << 1)    /* MAD 包(QP1) */
#define NIC_INTR_BYPASS         (1u << 2)    /* bypass 包 */
#define NIC_INTR_RNR_NAK        (1u << 3)    /* RQ 满发 RNR-NAK ✓(模型 v1.0 已实现) */
#define NIC_INTR_WQE_CMPL       (1u << 4)    /* **CQ 完成**(wqe_cmpl) ✓(模型已实现) */
/* bit5 = ill_opc_in_sq **源已禁用(恒 0)** */
#define NIC_INTR_QP_PKT         (1u << 6)    /* admin QP 收包 */
#define NIC_INTR_FATAL          (1u << 7)    /* QP fatal / 入站错误 */
#define NIC_INTR_CNP_SCHED      (1u << 8)    /* CNP 调度超时 */

/* ---- 中断状态组(全局段;★ 2026-10-04 /定案,出处 = `RTL 源`)----
 *   · `CQ_INTR_STS1..64`(0x290..0x38C,步长 4):**CQ 完成中断的**真清路** —— 写 32 位掩码(1=清对应 QP);
 *     RTL:写译� ⇒ 活清路 `resp_handler_top.i_clr_wqe_cmpl_sts_set_valid`→ ✓
 *   · `RQ_INTR_STS1..64`(0x190..0x28C,步长 4):RQ 侧同型(读/写=按位清)✓
 *   · ⚠ **写全局 `INTR_STS` bit4 不清 CQ 中断**(RTL 频闪 整链到 `resp_handler_fsm`
 *     后**未被使用** = 死路)⇒ **应答请用 `NIC_GLB_CQ_INTR_STS(1)`** ✓
 *   · ⚠ `C_NUM_QP=32` ⇒ 只 bank1(`CQ_INTR_STS1`,QP0..31)有效 ✓ */
#define NIC_GLB_RQ_INTR_STS(n)  (NIC_BASE + 0x00100000u + 0x190u + (((n)-1u) << 2))
#define NIC_GLB_CQ_INTR_STS(n)  (NIC_BASE + 0x00100000u + 0x290u + (((n)-1u) << 2))

/* ---- per-QP 段:`0x5018_0000 + (索引<<8) + off`(块 = 0x100;规格 §8.4)---- */
#define NIC_QP(idx)         (NIC_BASE + 0x00180000u + ((idx) << 8))
#define NIC_QP_QPN(idx)     ((idx) + 1u)     /* ★ 厂商 QPn = 地址索引 + 1 ✓(§8.1) */

/* per-QP 寄存器偏移(★ 全表 37 条见规格 §8.4;此处列驱动/模型要用的子集) */
#define NIC_QP_CONF         0x00             /* QP_CONF_QPN(位域见下)✓ */
#define NIC_QP_RQ_BUF_BA    0x08             /* RQ_BUF_BA_QPN:[31:8],**256B 粒度** ✓ */
#define NIC_QP_SQ_BA        0x10             /* SQ_BA_QPN:[31:5] ✓ */
#define NIC_QP_CQ_BA        0x18             /* CQ_BA_QPN:[31:5] ✓ */
#define NIC_QP_RQ_WRPTR_DB  0x20             /* RQ_WRPTR_DB_ADD_QPN(+0x24 MSB)= 主机内存门铃地址 ⚠ */
#define NIC_QP_SQ_CMPL_DB   0x28             /* SQ_CMPL_DB_ADD_QPN(+0x2C)= CQ 门铃落点 ⚠ */
#define NIC_QP_CQ_HEAD      0x30             /* CQ_HEAD_QPN:[15:0](需 sw_override)✓ */
#define NIC_QP_RQ_CI_DB     0x34             /* **RQ_CI_DB_QPN:[15:0] 写 = 门铃**(实跑 ✓) */
#define NIC_QP_SQ_PI_DB     0x38             /* **SQ_PI_DB_QPN:[15:0] 写 = 门铃**(实跑 ✓) */
#define NIC_QP_Q_DEPTH      0x3C             /* [15:0]SQ depth / [31:16]RQ depth ✓ */
#define NIC_QP_SQ_PSN       0x40             /* [23:0] ✓ */
#define NIC_QP_ADV_CONF     0x04             /* ★ D16:[31:16]P_Key [15:8]TTL [5:0]DSCP)✓ */
#define NIC_QP_DEST_QP      0x48             /* [23:0] Dest QP ID(★ D16:包 DestQP 真源)✓ */
#define NIC_QP_MAC_REM_LSB  0x50             /* ★ D16:远端 MAC 低 32(拼接 `{MSB[15:0],LSB}`)✓ */
#define NIC_QP_MAC_REM_MSB  0x54             /* 远端 MAC 高 16 ✓ */
#define NIC_QP_IP_REMOTE1   0x60             /* ★ D16:远端 IP(IPv4 = 本寄存器低 32;)✓ */
#define NIC_QP_IP_REMOTE2   0x64             /* (IPv6 用;v4 不用)✓ */
#define NIC_QP_IP_REMOTE3   0x68
#define NIC_QP_IP_REMOTE4   0x6C
#define NIC_QP_TIMEOUT      0x4C             /* [4:0]TMO [10:8]RETRY [13:11]RNR_RETRY [20:16]RNR_TVAL ✓ */
#define NIC_QP_PD_NUMBER    0xB0             /* [23:0] ✓ */
#define NIC_QP_STAT_RQ_BUF_CA 0x94           /* RO ✓ */
#define NIC_QP_STAT_WQE_CNT 0x98             /* RO(WQE_CNT_POP)✓ */
#define NIC_QP_STAT_RQ_PI   0x9C             /* RO(RQ_PI;引擎真源 ✓) */
#define NIC_QP_STAT_RET_PSN 0xA0             /* RO ✓ */

/* QP_CONF 位域(规格 §8.4;RTL 声明) */
#define NIC_QPC_QP_EN       (1u << 0)        /* [0] ✓ */
#define NIC_QPC_ACK_COALSC  (1u << 1)        /* [1] ✓ */
#define NIC_QPC_RQ_INTR_EN  (1u << 2)        /* [2] ✓ */
#define NIC_QPC_CQ_INTR_EN  (1u << 3)        /* [3] ✓ */
#define NIC_QPC_HW_HNDSHK_DIS (1u << 4)      /* [4] ✓ */
#define NIC_QPC_PMTU_MASK   0x00000700u      /* [10:8] ✓ */
#define NIC_QPC_MAXRD_MASK  0x00FF0000u      /* [23:16] ✓ */
#define NIC_QPC_RQBUFSZ_MASK 0xFF000000u     /* [31:24] **RQ 项步长(单位 256B)** ✓ */

/* ---- WQE(64B)字节偏移(规格 §9.1;bit0 = 字节 0,小端)---- */
#define NIC_WQE_BYTES       64
#define NIC_WQE_WRID_OFF    0                /* [15:0]  wrid(CQE 原样回带)✓ */
#define NIC_WQE_LOFF_OFF    4                /* [63:0]  local_offset ✓ */
#define NIC_WQE_LEN_OFF     12               /* [31:0]  dma_len ✓ */
#define NIC_WQE_OPC_OFF     16               /* [7:0]   wqe_opcode ✓ */
#define NIC_WQE_QID_OFF     17               /* ★ qp_id = [143](**8b**;C_MAX_QID_WIDTH=C_QP_INDX_WIDTH=8)
                                                     —— 订正:原先记 5b [140] ✗✓(规格 §9.1 已改) */
#define NIC_WQE_ROFF_OFF    20               /* [63:0]  remote_offset ✓ */
#define NIC_WQE_RKEY_OFF    28               /* [31:0]  r_key ✓ */
#define NIC_WQE_IMM_OFF     48               /* [31:0]  imm_data ✓ */
#define NIC_WQE_RETRY_OFF   2                /* [16] retried / [23:17] retried_buf_id ✓ */

/* wqe_opcode(规格 §9.1;`opcode[0]` = IMMDT 有效位)✓ */
#define NIC_OPC_WRITE          0x00
#define NIC_OPC_WRITE_IMMDT    0x01          /* = WRITE + imm_data ✓ */
#define NIC_OPC_SEND           0x02
#define NIC_OPC_SEND_IMMDT     0x03
#define NIC_OPC_READ           0x04
#define NIC_OPC_SEND_INV       0x0C          /* SEND + Invalidate(⚠ 语义未开卷) */

/* ---- CQE(4B;`{7'h0,fatal,opcode[7:0],wrid[15:0]}`,小端 —— 规格 §9.4)---- */
#define NIC_CQE_BYTES       4
#define NIC_CQE_WRID(b)     ((unsigned)((b)[0]) | ((unsigned)((b)[1]) << 8))  /* ✓ */
#define NIC_CQE_OPCODE(b)   ((b)[2])                                          /* = WQE opcode 原样 ✓ */
#define NIC_CQE_FATAL(b)    ((b)[3] & 1)                                      /* {7'h0,fatal} ✓ */

#endif /* NIC_REGS_H */
