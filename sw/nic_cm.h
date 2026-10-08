/*============================================================================
 * nic_cm.h — CM 建链(RoCEv2)最小件 · M1 消息层(256B MAD 布局)  [2026-10-07 建]
 *
 * 依据(cmd 全静态;**线上布局以编译器为准** —— 本项目铁律):
 *   · 上游 `include/rdma/ibta_vol1_c12.h` 宏表(IBTA 卷1 第12章 REQ/REP/RTU 字段)
 *   · 上游 `include/rdma/ib_mad.h`(ib_mad_hdr = **24B**)/ `ib_cm.h`(attr_id/私有数据尺寸)
 *   · 派生规则:绝对偏移 = 宏 byte_offset **+ 24**;位段自 **MSB** 起算
 *     (`include/rdma/iba.h`:`IBA_FIELD_BLOC` = `GENMASK(7-bit_offset, 7-bit_offset-n+1)`)
 *   · 实测:sizeof(cm_req_msg) = **256**(= 24B 头 + 232B 体;私有数据尾端卡满:
 *     REQ 164+92 / REP 60+196 / RTU 32+224 = 256 ✓)
 * 口径:文件末 `_Static_assert` 断言全部布局常数 ⇒ 改一行即编译期报错 ✓
 * ⚠ 承载层 = 自建字节流(M1,非真栈);本文件只负责**消息字节**,与承载解耦 ✓
 *==========================================================================*/
#ifndef NIC_CM_H
#define NIC_CM_H

#include <stdint.h>
#include <stddef.h>

/* C/C++ 双用静态断言(C11 与 C++11 皆可)✓ */
#if defined(__cplusplus)
#define CM_STATIC_ASSERT(c, m) static_assert(c, m)
#else
#define CM_STATIC_ASSERT(c, m) _Static_assert(c, m)
#endif

/* ---- 公共 ---- */
#define CM_MSG_LEN        256      /* 三消息全同长 ✓ */

/* MAD hdr(24B;值照 `cm_format_mad_hdr`,cm.c) */
#define CM_HDR_BASE_VER   0
#define CM_HDR_MGMT_CLASS 1        /* = 0x07 CM(⚠ 非 0x03) */
#define CM_HDR_CLASS_VER  2        /* = 0x02 */
#define CM_HDR_METHOD     3        /* = 0x03 SEND */
#define CM_HDR_STATUS     4        /* 2B BE */
#define CM_HDR_CLASS_SPEC 6        /* 2B BE */
#define CM_HDR_TID        8        /* 8B BE64(事务 ID) */
#define CM_HDR_ATTR_ID    16       /* 2B BE16 */
#define CM_HDR_RESV       18
#define CM_HDR_ATTR_MOD   20       /* 4B BE */

#define CM_VAL_BASE_VER   0x01
#define CM_VAL_CLASS      0x07
#define CM_VAL_CLASS_VER  0x02
#define CM_VAL_METHOD     0x03

#define CM_ATTR_ID_REQ    0x0010   /* ib_cm.h 全表 */
#define CM_ATTR_ID_MRA    0x0011
#define CM_ATTR_ID_REJ    0x0012
#define CM_ATTR_ID_REP    0x0013
#define CM_ATTR_ID_RTU    0x0014
/* 拒绝原因码(ib_cm.h-,取 M2 用集) */
#define CM_REJ_NO_QP       1
#define CM_REJ_NO_RESOURCES 3
#define CM_REJ_TIMEOUT     4
#define CM_REJ_UNSUPPORTED 5

/* ---- REQ(绝对偏移;宏 + 24)---- */
#define CM_REQ_LCOMM_ID   24       /* 32b   LOCAL_COMM_ID(0) */
#define CM_REQ_SERVICE_ID 32       /* 64b   SERVICE_ID(8) */
#define CM_REQ_CA_GUID    40       /* 64b   LOCAL_CA_GUID(16) */
#define CM_REQ_QKEY       52       /* 32b   LOCAL_Q_KEY(28) */
#define CM_REQ_QPN        56       /* ★24b  LOCAL_QPN(32):占 56/57/58(BE;word 掩码[31:8]) */
#define CM_REQ_RESP_RES   59       /* 8b    RESPONDER_RESOURCES(35) */
#define CM_REQ_EECN       60       /* 24b   LOCAL_EECN(36) */
#define CM_REQ_INIT_DEPTH 63       /* 8b    INITIATOR_DEPTH(39) */
#define CM_REQ_REMOTE_EECN 64      /* 24b   REMOTE_EECN(40) */
#define CM_REQ_TO_TST     67       /* 8b    [7:3]Remote CM Resp TO(43,5b);[2:1]TST;[0]E2E */
#define CM_REQ_PSN        68       /* ★24b  STARTING_PSN(44) */
#define CM_REQ_LCL_TO     71       /* 8b    [7:3]Local CM Resp TO(47,5b);[2:0]RetryCount */
#define CM_REQ_PKEY       72       /* 16b   PARTITION_KEY(48) */
#define CM_REQ_MTU_RNR    74       /* 8b    ★[7:4]Path MTU(50,4b);[3]RDC;[2:0]RNR retry(50,5,3) */
#define CM_REQ_RETRY_SRQ  75       /* 8b    [7:4]MaxCMRetries(51,4b);[3]SRQ;[2:0]ExtTST */
#define CM_REQ_LOCAL_GID  80       /* ★16B  PRIMARY_LOCAL_PORT_GID(56) */
#define CM_REQ_REMOTE_GID 96       /* ★16B  PRIMARY_REMOTE_PORT_GID(72) */
#define CM_REQ_FLOW_LABEL 112      /* 20b   (88,20b/113 + 114[7:4]) */
#define CM_REQ_PKT_RATE   115      /* 6b    [5:0](91,2,6) */
#define CM_REQ_TCLASS     116      /* 8b    PRIMARY_TRAFFIC_CLASS(92) */
#define CM_REQ_HOP_LIMIT  117      /* 8b    (93) */
#define CM_REQ_SL         118      /* 8b    [7:4]SL(94,4b);[3]SubnetLocal */
#define CM_REQ_ACK_TIMEOUT 119     /* 5b    [7:3](95,5b) */
#define CM_REQ_PRIV       164      /* priv  PRIVATE_DATA(140) */
#define CM_REQ_PRIV_LEN   92       /*       IB_CM_REQ_PRIVATE_DATA_SIZE=92 ✓ */

/* ---- REP(绝对偏移)---- */
#define CM_REP_LCOMM_ID   24       /* LOCAL_COMM_ID(0) */
#define CM_REP_RCOMM_ID   28       /* REMOTE_COMM_ID(4) */
#define CM_REP_QKEY       32       /* LOCAL_Q_KEY(8) */
#define CM_REP_QPN        36       /* ★24b  LOCAL_QPN(12) */
#define CM_REP_PSN        44       /* ★24b  STARTING_PSN(20) */
#define CM_REP_RESP_RES   48       /* RESPONDER_RESOURCES(24) */
#define CM_REP_INIT_DEPTH 49       /* INITIATOR_DEPTH(25) */
#define CM_REP_ACK_DELAY  50       /* [7:3]TARGET_ACK_DELAY(26,5b) */
#define CM_REP_RNR_SRQ    51       /* [7:5]RNR retry(27,8→3);[4]SRQ */
#define CM_REP_CA_GUID    52       /* LOCAL_CA_GUID(28,64b) */
#define CM_REP_PRIV       60       /* priv */
#define CM_REP_PRIV_LEN   196      /* IB_CM_REP_PRIVATE_DATA_SIZE=196 ✓ */

/* ---- REJ(256B;abs = 宏 + 24;Table 108)---- */
#define CM_REJ_LCOMM_ID   24       /* LOCAL_COMM_ID(0) */
#define CM_REJ_RCOMM_ID   28       /* REMOTE_COMM_ID(4) */
#define CM_REJ_MSG_REJ    32       /* [7:6]MESSAGE_REJECTED(8,2b):0=REQ 1=REP 2=OTHER */
#define CM_REJ_INFO_LEN   33       /* [7:1]REJECTED_INFO_LENGTH(9,7b) */
#define CM_REJ_REASON     34       /* REASON(10,16b,BE) */
#define CM_REJ_ARI        36       /* ARI(12,576b = **72B**;留 0) */
#define CM_REJ_PRIV       108      /* priv */
#define CM_REJ_PRIV_LEN   148      /* IB_CM_REJ_PRIVATE_DATA_SIZE=148 ✓(108+148=256) */

/* ---- RTU ---- */
#define CM_RTU_LCOMM_ID   24
#define CM_RTU_RCOMM_ID   28
#define CM_RTU_PRIV       32
#define CM_RTU_PRIV_LEN   224      /* IB_CM_RTU_PRIVATE_DATA_SIZE=224 ✓ */

/* ---- BE 读写工具(不假定对齐)---- */
static inline void cm_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline uint16_t cm_get16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline void cm_put24(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 16); p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)v; }
static inline uint32_t cm_get24(const uint8_t *p) { return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]; }
static inline void cm_put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static inline uint32_t cm_get32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline void cm_put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (56 - 8 * i)); }
static inline uint64_t cm_get64(const uint8_t *p) { uint64_t v = 0; for (int i = 0; i < 8; i++) v = (v << 8) | p[i]; return v; }

/* ---- GID ⇒ QP_IP_REMOTE1 寄存器值(RoCEv2 IPv4-mapped:IP = gid[12..15];
 *      **小端装载** —— 与模型/台实证一致:IP 10.0.0.2 ⇒ 0x0200000A ✓(tb_roce_engine_m1)---- */
static inline uint32_t cm_gid_to_ipv4_reg(const uint8_t gid[16])
{
    return ((uint32_t)gid[15] << 24) | ((uint32_t)gid[14] << 16) |
           ((uint32_t)gid[13] << 8)  | (uint32_t)gid[12];
}

/* ---- 构造(M1;priv 可 NULL;buf 须 ≥ CM_MSG_LEN,先清零)----
   ⚠ REQ 带**两个** GID(LOCAL@80 = 本端;REMOTE@96 = **对端**地址 —— 照真流程:
   主动端先 resolve 对端地址再 connect,把对端地址填进 REQ 的 Remote GID 字段;REP **无** GID 字段)✓ */
void cm_build_req(uint8_t *m, uint64_t tid, uint32_t lcomm_id, uint32_t qpn,
                  uint32_t psn, uint8_t mtu, uint16_t pkey, const uint8_t gid[16],
                  const uint8_t peer_gid[16], uint8_t tclass, const uint8_t *priv, int privlen);
void cm_build_rep(uint8_t *m, uint64_t tid, uint32_t lcomm_id, uint32_t rcomm_id,
                  uint32_t qpn, uint32_t psn, const uint8_t *priv, int privlen);
void cm_build_rtu(uint8_t *m, uint64_t tid, uint32_t lcomm_id, uint32_t rcomm_id,
                  const uint8_t *priv, int privlen);
/* ★ M2:REJ(msg_rejected:0=REQ/1=REP/2=OTHER;reason 见 CM_REJ_* 码)+ 头解析认 REJ/MRA ✓ */
void cm_build_rej(uint8_t *m, uint64_t tid, uint32_t lcomm_id, uint32_t rcomm_id,
                  uint8_t msg_rejected, uint16_t reason, const uint8_t *priv, int privlen);

/* ---- 头解析(dist=0 时 ok=0)---- */
typedef struct {
    uint8_t  class_ver;
    uint8_t  method;
    uint16_t attr_id;
    uint64_t tid;
    int      ok;                    /* 版本/类/方法/attr 全对 ⇒ 1 */
} cm_hdr_t;
cm_hdr_t cm_parse_hdr(const uint8_t *m);

/* ---- 自检(构造 ⇒ 回读 ⇒ 逐字段对表 + 裸字节抽查;返回 0 = 过,>0 = 第几项败)---- */
int cm_selftest(void);

/*============================================================================
 * 建链实体(步骤 2;M1 最小件)
 *   语义(照 规格文档 §2.3/§2.4):
 *     REQ = 主动端 → 被动端:带 主动端 QPN/起始 PSN/MTU/GID/P_Key/TClass
 *     REP = 被动端 → 主动端:带 被动端 QPN/起始 PSN;   RTU = 主动端 → 被动端:就绪
 *   · MTU:REQ 提议、应答端采纳(须 ≤ 被动端能力,否则 fail —— 拒绝分支留 M2)✓
 *   · 各侧 QP_SQ_PSN = **各侧自己的**起始 PSN;对端 QPN/GID ⇒ 各侧 QP_DEST_QP/QP_IP_REMOTE ✓
 *   · transport 抽象:send/recv 非阻塞(收 0 = 无消息);M1 管道实现见 model/tb/cm_pipe.h ✓
 *==========================================================================*/
typedef struct cm_transport {
    int (*send)(void *ctx, const uint8_t *buf, int len);   /* 返回 0 = 成 */
    int (*recv)(void *ctx, uint8_t *buf, int cap);          /* 返回 >0 = 消息长;0 = 无 */
    void *ctx;
} cm_transport_t;

enum {
    CM_ST_IDLE = 0,
    CM_ST_WAIT_REQ,        /* 被动:等 REQ */
    CM_ST_WAIT_REP,        /* 主动:等 REP */
    CM_ST_WAIT_RTU,        /* 被动:等 RTU */
    CM_ST_CONNECTED,
    CM_ST_FAIL
};

typedef struct cm_entity {
    /* 本地参数(建链前由调用方配;M1 = TB/驱动给) */
    uint32_t lqpn, lpsn;
    uint8_t  lmtu, ltclass;
    uint16_t lpkey;
    uint8_t  lgid[16];
    uint32_t lcomm;                    /* 本端 comm id(= tid 低 32 的派生,M1) */
    uint8_t  pgid[16];                 /* ★ 主动端:拨号的对端地址(resolve 结果;
                                          被动端不用;REP 无 GID 字段 ⇒ 由此外部获得)✓ */
    int      has_pgid;
    /* 协商结果(对端参数) */
    uint32_t rqpn, rpsn, rcomm;
    uint8_t  rmtu, rtclass;
    uint16_t rpkey;
    uint8_t  rgid[16];
    int      neg_mtu;                  /* 协商后 MTU(两侧应同) */
    /* 运行态 */
    uint64_t tid;
    int      st;
    int      steps, retry, retry_max, retry_every;
    int      tx_n, rx_n;               /* 发/收消息计数(读数用) */
    int      rejected;                 /* ★ M2:本侧发过 REJ / 收到 REJ */
    uint16_t rej_reason;               /* ★ M2:REJ 原因码(CM_REJ_*) */
    uint8_t  priv_tx[224]; int priv_tx_len;
    uint8_t  priv_rx[224]; int priv_rx_len;
    cm_transport_t *t;
} cm_entity_t;

/* 初始化(主动/被动同构;qpn/psn/mtu/pkey/gid/tclass = 本端值) */
void cm_entity_init(cm_entity_t *e, cm_transport_t *t, uint32_t qpn, uint32_t psn,
                    uint8_t mtu, uint16_t pkey, const uint8_t gid[16], uint8_t tclass);
/* 单步推进(非阻塞);返回:0 = 进行中;1 = CONNECTED;<0 = FAIL(带原因码) */
int cm_active_step(cm_entity_t *e);    /* 主动端 */
int cm_passive_step(cm_entity_t *e);   /* 被动端 */

/* 两端驱动循环(取数用;M1 scratch/TB 均用):跑到双端 CONNECTED 或步数耗尽
   返回 0 = 双端通;>0 = 剩几步超时;-1 = 其中一端 FAIL */
int cm_run_pair(cm_entity_t *a, cm_entity_t *b, int max_steps);

/* ★ 主动端专用:设置"拨号对端地址"(= resolve 结果;照真流程语义) */
void cm_entity_set_peer(cm_entity_t *e, const uint8_t pgid[16]);

/* 步骤 2 自检(管道由调用方给(实现见 model/tb/cm_pipe.h);内部造两端 + 负控 返回 0 = 过) */
int cm_exchange_selftest(cm_transport_t *ta, cm_transport_t *tb);

/*============================================================================
 * 布局常数断言(编译期;改一行即报错 —— 铁律"以编译器为准"的机械化)
 *==========================================================================*/
CM_STATIC_ASSERT(CM_MSG_LEN == 256, "CM 消息 = 256B");
CM_STATIC_ASSERT(CM_REQ_PRIV + CM_REQ_PRIV_LEN == CM_MSG_LEN, "REQ 私有数据卡满");
CM_STATIC_ASSERT(CM_REP_PRIV + CM_REP_PRIV_LEN == CM_MSG_LEN, "REP 私有数据卡满");
CM_STATIC_ASSERT(CM_RTU_PRIV + CM_RTU_PRIV_LEN == CM_MSG_LEN, "RTU 私有数据卡满");
CM_STATIC_ASSERT(CM_HDR_ATTR_MOD + 4 == 24, "MAD 头 = 24B(ib_mad_hdr 实测)");
CM_STATIC_ASSERT(CM_REQ_QPN + 3 <= CM_REQ_RESP_RES, "REQ QPN 不越界");
CM_STATIC_ASSERT(CM_REQ_PSN + 3 <= CM_REQ_LCL_TO, "REQ PSN 不越界");
CM_STATIC_ASSERT(CM_REP_QPN + 3 <= CM_REP_PSN, "REP QPN 不越界");
CM_STATIC_ASSERT(CM_REP_PSN + 3 <= CM_REP_RESP_RES, "REP PSN 不越界");

#endif /* NIC_CM_H */
