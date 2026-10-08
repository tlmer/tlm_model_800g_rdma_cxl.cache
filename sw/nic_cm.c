/*============================================================================
 * nic_cm.c — CM 建链最小件 · M1 消息层实现(布局常数全在 nic_cm.h)
 * 口径:构造 = 按宏表逐字节填充;解析 = 头校验 + 字段回读;自检 = 构造⇒回读对表 ✓
 *==========================================================================*/
#include "nic_cm.h"
#include <string.h>

/* MAD 头公共填充(照 cm_format_mad_hdr,cm.c) */
static void cm_fill_hdr(uint8_t *m, uint16_t attr_id, uint64_t tid)
{
    m[CM_HDR_BASE_VER]   = CM_VAL_BASE_VER;
    m[CM_HDR_MGMT_CLASS] = CM_VAL_CLASS;
    m[CM_HDR_CLASS_VER]  = CM_VAL_CLASS_VER;
    m[CM_HDR_METHOD]     = CM_VAL_METHOD;
    cm_put16(&m[CM_HDR_STATUS], 0);
    cm_put16(&m[CM_HDR_CLASS_SPEC], 0);
    cm_put64(&m[CM_HDR_TID], tid);
    cm_put16(&m[CM_HDR_ATTR_ID], attr_id);
    cm_put16(&m[CM_HDR_RESV], 0);
    cm_put32(&m[CM_HDR_ATTR_MOD], 0);
}

static void cm_put_priv(uint8_t *m, int off, int cap, const uint8_t *priv, int privlen)
{
    if (priv && privlen > 0) {
        if (privlen > cap) privlen = cap;      /* 截断(调用方保证不超) */
        memcpy(&m[off], priv, (size_t)privlen);
    }
}

/* ---- REQ(字段序照 ibta_vol1_c12.h Table 106;偏移 +24)---- */
void cm_build_req(uint8_t *m, uint64_t tid, uint32_t lcomm_id, uint32_t qpn,
                  uint32_t psn, uint8_t mtu, uint16_t pkey, const uint8_t gid[16],
                  const uint8_t peer_gid[16], uint8_t tclass, const uint8_t *priv, int privlen)
{
    memset(m, 0, CM_MSG_LEN);
    cm_fill_hdr(m, CM_ATTR_ID_REQ, tid);
    cm_put32(&m[CM_REQ_LCOMM_ID], lcomm_id);
    /* Service ID(32-39)=0(M1 不协商服务类型) */
    /* CA GUID(40-47)=0(RoCE 不填 CA GUID 语义,留 0) */
    cm_put32(&m[CM_REQ_QKEY], 0x8001u);                    /* 常见默认 Q_Key */
    cm_put24(&m[CM_REQ_QPN], qpn & 0xFFFFFFu);             /* ★ */
    m[CM_REQ_RESP_RES] = 0;
    cm_put24(&m[CM_REQ_PSN], psn & 0xFFFFFFu);             /* ★ */
    m[CM_REQ_TO_TST]   = 0x00;                             /* [7:3]远端 TO=0;[2:1]TST=0;[0]E2E=0 */
    m[CM_REQ_LCL_TO]   = 0x00;                             /* Local TO / Retry = 0(M1) */
    cm_put16(&m[CM_REQ_PKEY], pkey);
    m[CM_REQ_MTU_RNR]  = (uint8_t)((mtu & 0x0Fu) << 4);    /* ★ MTU = 高 4b */
    m[CM_REQ_RETRY_SRQ] = 0x00;
    if (gid) memcpy(&m[CM_REQ_LOCAL_GID], gid, 16);        /* ★ 本端 */
    if (peer_gid) memcpy(&m[CM_REQ_REMOTE_GID], peer_gid, 16); /* ★ 对端(主动端拨号地址) */
    m[CM_REQ_TCLASS]   = tclass;                           /* ★ RoCEv2:DSCP<<2 由调用方算好 */
    m[CM_REQ_HOP_LIMIT] = 64;
    cm_put_priv(m, CM_REQ_PRIV, CM_REQ_PRIV_LEN, priv, privlen);
}

/* ---- REP ---- */
void cm_build_rep(uint8_t *m, uint64_t tid, uint32_t lcomm_id, uint32_t rcomm_id,
                  uint32_t qpn, uint32_t psn, const uint8_t *priv, int privlen)
{
    memset(m, 0, CM_MSG_LEN);
    cm_fill_hdr(m, CM_ATTR_ID_REP, tid);
    cm_put32(&m[CM_REP_LCOMM_ID], lcomm_id);
    cm_put32(&m[CM_REP_RCOMM_ID], rcomm_id);
    cm_put32(&m[CM_REP_QKEY], 0x8001u);
    cm_put24(&m[CM_REP_QPN], qpn & 0xFFFFFFu);             /* ★ */
    cm_put24(&m[CM_REP_PSN], psn & 0xFFFFFFu);             /* ★ */
    m[CM_REP_RESP_RES] = 0;
    m[CM_REP_INIT_DEPTH] = 0;
    m[CM_REP_ACK_DELAY] = 0;                               /* [7:3]Target ACK Delay = 0 */
    cm_put_priv(m, CM_REP_PRIV, CM_REP_PRIV_LEN, priv, privlen);
}

/* ---- RTU ---- */
void cm_build_rtu(uint8_t *m, uint64_t tid, uint32_t lcomm_id, uint32_t rcomm_id,
                  const uint8_t *priv, int privlen)
{
    memset(m, 0, CM_MSG_LEN);
    cm_fill_hdr(m, CM_ATTR_ID_RTU, tid);
    cm_put32(&m[CM_RTU_LCOMM_ID], lcomm_id);
    cm_put32(&m[CM_RTU_RCOMM_ID], rcomm_id);
    cm_put_priv(m, CM_RTU_PRIV, CM_RTU_PRIV_LEN, priv, privlen);
}

/* ---- ★ M2:REJ ---- */
void cm_build_rej(uint8_t *m, uint64_t tid, uint32_t lcomm_id, uint32_t rcomm_id,
                  uint8_t msg_rejected, uint16_t reason, const uint8_t *priv, int privlen)
{
    memset(m, 0, CM_MSG_LEN);
    cm_fill_hdr(m, CM_ATTR_ID_REJ, tid);
    cm_put32(&m[CM_REJ_LCOMM_ID], lcomm_id);
    cm_put32(&m[CM_REJ_RCOMM_ID], rcomm_id);
    m[CM_REJ_MSG_REJ]  = (uint8_t)((msg_rejected & 0x3u) << 6);   /* [7:6](MSB 起算)✓ */
    m[CM_REJ_INFO_LEN] = 0;
    cm_put16(&m[CM_REJ_REASON], reason);
    cm_put_priv(m, CM_REJ_PRIV, CM_REJ_PRIV_LEN, priv, privlen);
}

/* ---- 头解析 ---- */
cm_hdr_t cm_parse_hdr(const uint8_t *m)
{
    cm_hdr_t h;
    h.class_ver = m[CM_HDR_CLASS_VER];
    h.method    = m[CM_HDR_METHOD];
    h.attr_id   = cm_get16(&m[CM_HDR_ATTR_ID]);
    h.tid       = cm_get64(&m[CM_HDR_TID]);
    h.ok = (m[CM_HDR_BASE_VER]   == CM_VAL_BASE_VER &&
            m[CM_HDR_MGMT_CLASS] == CM_VAL_CLASS &&
            h.class_ver          == CM_VAL_CLASS_VER &&
            h.method             == CM_VAL_METHOD &&
            (h.attr_id == CM_ATTR_ID_REQ || h.attr_id == CM_ATTR_ID_REP ||
             h.attr_id == CM_ATTR_ID_RTU || h.attr_id == CM_ATTR_ID_REJ ||
             h.attr_id == CM_ATTR_ID_MRA));
    return h;
}

/*============================================================================
 * 自检:构造 ⇒ 回读对表(含裸字节抽查);返回 0 = 过
 *==========================================================================*/
int cm_selftest(void)
{
    static const uint8_t gid[16] = {0xfe,0x80,0,0,0,0,0,0,0,0,0,0,0x0a,0,0,0x01};
    uint8_t priv[224];
    uint8_t m[CM_MSG_LEN];
    cm_hdr_t h;
    int i;

    for (i = 0; i < 224; i++) priv[i] = (uint8_t)(0xC0 + (i & 0x1F));

    /* --- REQ --- */
    cm_build_req(m, 0x1122334455667788ull, 0xAABBCCDDu, 0x123456u, 0x654321u,
                 5, 0xFFFF, gid, gid, 0x2Cu, priv, CM_REQ_PRIV_LEN);
    h = cm_parse_hdr(m);
    if (!h.ok || h.attr_id != CM_ATTR_ID_REQ || h.tid != 0x1122334455667788ull) return 1;
    if (cm_get32(&m[CM_REQ_LCOMM_ID]) != 0xAABBCCDDu) return 2;
    if (cm_get24(&m[CM_REQ_QPN]) != 0x123456u) return 3;         /* ★ QPN */
    if (cm_get24(&m[CM_REQ_PSN]) != 0x654321u) return 4;         /* ★ PSN */
    if ((m[CM_REQ_MTU_RNR] >> 4) != 5) return 5;                 /* ★ MTU 高 4b */
    if (cm_get16(&m[CM_REQ_PKEY]) != 0xFFFF) return 6;
    if (memcmp(&m[CM_REQ_LOCAL_GID], gid, 16) != 0) return 7;    /* ★ GID */
    if (m[CM_REQ_TCLASS] != 0x2Cu) return 8;
    if (memcmp(&m[CM_REQ_PRIV], priv, CM_REQ_PRIV_LEN) != 0) return 9;
    /* 裸字节抽查(独立于 cm_get*,防"读写同错") */
    if (m[CM_REQ_QPN + 0] != 0x12 || m[CM_REQ_QPN + 1] != 0x34 || m[CM_REQ_QPN + 2] != 0x56) return 10;
    if (m[CM_REQ_PSN + 0] != 0x65 || m[CM_REQ_PSN + 1] != 0x43 || m[CM_REQ_PSN + 2] != 0x21) return 11;
    if (m[CM_HDR_MGMT_CLASS] != 0x07 || m[CM_HDR_CLASS_VER] != 0x02 || m[CM_HDR_METHOD] != 0x03) return 12;
    if (m[16] != 0x00 || m[17] != 0x10) return 13;               /* attr_id@16-17 BE */
    if (m[CM_MSG_LEN - 1] != priv[CM_REQ_PRIV_LEN - 1]) return 14; /* 私有数据卡满 256 */

    /* --- REP --- */
    cm_build_rep(m, 0xDEADBEEFCAFEF00Dull, 0x11223344u, 0xAABBCCDDu, 0xABCDEFu, 0x00FEDCu,
                 priv, CM_REP_PRIV_LEN);
    h = cm_parse_hdr(m);
    if (!h.ok || h.attr_id != CM_ATTR_ID_REP || h.tid != 0xDEADBEEFCAFEF00Dull) return 20;
    if (cm_get32(&m[CM_REP_LCOMM_ID]) != 0x11223344u) return 21;
    if (cm_get32(&m[CM_REP_RCOMM_ID]) != 0xAABBCCDDu) return 22;
    if (cm_get24(&m[CM_REP_QPN]) != 0xABCDEFu) return 23;        /* ★ */
    if (cm_get24(&m[CM_REP_PSN]) != 0x00FEDCu) return 24;        /* ★ */
    if (memcmp(&m[CM_REP_PRIV], priv, CM_REP_PRIV_LEN) != 0) return 25;
    if (m[16] != 0x00 || m[17] != 0x13) return 26;
    if (m[CM_MSG_LEN - 1] != priv[CM_REP_PRIV_LEN - 1]) return 27;

    /* --- RTU --- */
    cm_build_rtu(m, 0x0F1E2D3C4B5A6978ull, 0x11223344u, 0x55667788u, priv, CM_RTU_PRIV_LEN);
    h = cm_parse_hdr(m);
    if (!h.ok || h.attr_id != CM_ATTR_ID_RTU || h.tid != 0x0F1E2D3C4B5A6978ull) return 30;
    if (cm_get32(&m[CM_RTU_RCOMM_ID]) != 0x55667788u) return 31;
    if (memcmp(&m[CM_RTU_PRIV], priv, CM_RTU_PRIV_LEN) != 0) return 32;
    if (m[16] != 0x00 || m[17] != 0x14) return 33;
    if (m[CM_MSG_LEN - 1] != priv[CM_RTU_PRIV_LEN - 1]) return 34;

    /* --- 负控:截断包必须不被认(改类版本)--- */
    cm_build_req(m, 1, 2, 3, 4, 0, 0, gid, NULL, 0, NULL, 0);
    m[CM_HDR_CLASS_VER] = 0x03;
    h = cm_parse_hdr(m);
    if (h.ok) return 40;
    m[CM_HDR_CLASS_VER] = CM_VAL_CLASS_VER;
    m[CM_HDR_MGMT_CLASS] = 0x03;
    h = cm_parse_hdr(m);
    if (h.ok) return 41;

    return 0;
}

/*============================================================================
 * 建链实体(步骤 2;状态机见 nic_cm.h 注释块)
 *==========================================================================*/
#include <stdio.h>

void cm_entity_init(cm_entity_t *e, cm_transport_t *t, uint32_t qpn, uint32_t psn,
                    uint8_t mtu, uint16_t pkey, const uint8_t gid[16], uint8_t tclass)
{
    memset(e, 0, sizeof(*e));
    e->lqpn = qpn & 0xFFFFFFu;
    e->lpsn = psn & 0xFFFFFFu;
    e->lmtu = mtu & 0x0Fu;
    e->lpkey = pkey;
    e->ltclass = tclass;
    if (gid) memcpy(e->lgid, gid, 16);
    e->lcomm = 0xA5A50000u | (qpn & 0xFFFFu);        /* M1:comm id 由 qpn 派生(驱动可另配) */
    e->tid   = 0x1000000000000000ull | e->lcomm;     /* M1:事务 ID 派生 */
    e->neg_mtu = -1;
    e->st = CM_ST_IDLE;
    e->retry_max = 5; e->retry_every = 10;
    e->t = t;
}

/* 主动:IDLE → 发 REQ → WAIT_REP(超时重发,超限 FAIL)→ 收 REP → 发 RTU → CONNECTED */
int cm_active_step(cm_entity_t *e)
{
    uint8_t m[CM_MSG_LEN], r[CM_MSG_LEN];
    int n;

    if (e->st == CM_ST_CONNECTED) return 1;
    if (e->st == CM_ST_FAIL) return -1;

    if (e->st == CM_ST_IDLE) {
        cm_build_req(m, e->tid, e->lcomm, e->lqpn, e->lpsn, e->lmtu, e->lpkey,
                     e->lgid, e->has_pgid ? e->pgid : NULL, e->ltclass,
                     e->priv_tx_len ? e->priv_tx : NULL, e->priv_tx_len);
        if (e->t->send(e->t->ctx, m, CM_MSG_LEN) == 0) {
            if (e->has_pgid) memcpy(e->rgid, e->pgid, 16);  /* ★ 对端 GID = 拨号地址 */
            e->tx_n++; e->st = CM_ST_WAIT_REP; e->steps = 0;
        }
        return 0;
    }

    /* CM_ST_WAIT_REP */
    e->steps++;
    n = e->t->recv(e->t->ctx, r, CM_MSG_LEN);
    if (n == CM_MSG_LEN) {
        cm_hdr_t h = cm_parse_hdr(r);
        if (h.ok && h.attr_id == CM_ATTR_ID_REJ && h.tid == e->tid) {   /* ★ M2:被拒 */
            e->rejected = 1;
            e->rej_reason = cm_get16(&r[CM_REJ_REASON]);
            e->rx_n++;
            e->st = CM_ST_FAIL;
            return -5;
        }
        if (h.ok && h.attr_id == CM_ATTR_ID_REP && h.tid == e->tid) {
            memcpy(e->priv_rx, &r[CM_REP_PRIV], CM_REP_PRIV_LEN);       /* ★ M2:对端 app 块 */
            e->priv_rx_len = CM_REP_PRIV_LEN;
            e->rqpn  = cm_get24(&r[CM_REP_QPN]);
            e->rpsn  = cm_get24(&r[CM_REP_PSN]);
            e->rcomm = cm_get32(&r[CM_REP_LCOMM_ID]);   /* ★ 对端 id = REP 的 **LOCAL** comm id
                                                           (REP.RCOMM = 我方 id,回环用) */
            e->neg_mtu = e->lmtu;                       /* M1:MTU 由 REQ 提议、双方采纳 */
            e->rx_n++;
            cm_build_rtu(m, e->tid, e->lcomm, e->rcomm,
                         e->priv_tx_len ? e->priv_tx : NULL, e->priv_tx_len);
            if (e->t->send(e->t->ctx, m, CM_MSG_LEN) == 0) {
                e->tx_n++; e->st = CM_ST_CONNECTED;
                return 1;
            }
            return 0;                                   /* 发送暂塞 ⇒ 下步重试 */
        }
        /* 非本事务/坏包:忽略,继续等 */
    }
    if (e->steps % e->retry_every == 0) {               /* 超时重发 */
        if (e->retry >= e->retry_max) { e->st = CM_ST_FAIL; return -3; }
        e->retry++;
        cm_build_req(m, e->tid, e->lcomm, e->lqpn, e->lpsn, e->lmtu, e->lpkey,
                     e->lgid, e->has_pgid ? e->pgid : NULL, e->ltclass,
                     e->priv_tx_len ? e->priv_tx : NULL, e->priv_tx_len);
        if (e->t->send(e->t->ctx, m, CM_MSG_LEN) == 0) e->tx_n++;
    }
    return 0;
}

void cm_entity_set_peer(cm_entity_t *e, const uint8_t pgid[16])
{
    memcpy(e->pgid, pgid, 16);
    e->has_pgid = 1;
}

/* 被动:WAIT_REQ → 收 REQ(校验/采纳)⇒ 发 REP → WAIT_RTU → 收 RTU → CONNECTED */
int cm_passive_step(cm_entity_t *e)
{
    uint8_t m[CM_MSG_LEN], r[CM_MSG_LEN];
    int n;

    if (e->st == CM_ST_CONNECTED) return 1;
    if (e->st == CM_ST_FAIL) return -1;
    if (e->st == CM_ST_IDLE) { e->st = CM_ST_WAIT_REQ; return 0; }

    n = e->t->recv(e->t->ctx, r, CM_MSG_LEN);
    if (n != CM_MSG_LEN) return 0;                      /* 无消息 */

    if (e->st == CM_ST_WAIT_REQ) {
        cm_hdr_t h = cm_parse_hdr(r);
        if (!h.ok || h.attr_id != CM_ATTR_ID_REQ) return 0;      /* 坏包/非 REQ:忽略(负控) */
        uint32_t rqpn = cm_get24(&r[CM_REQ_QPN]);
        uint32_t rpsn = cm_get24(&r[CM_REQ_PSN]);
        uint8_t  pmtu = (uint8_t)(r[CM_REQ_MTU_RNR] >> 4);
        if (rqpn == 0 || rpsn == 0) return 0;                    /* 字段合法性(M1 最小) */
        if (pmtu > e->lmtu) {                                   /* ★ M2:拒绝分支 = 发 REJ(照 ib_cm.h 原因码) */
            uint8_t rj[CM_MSG_LEN];
            cm_build_rej(rj, h.tid, e->lcomm, cm_get32(&r[CM_REQ_LCOMM_ID]),
                         0 /*MESSAGE_REJECTED=REQ*/, CM_REJ_UNSUPPORTED,
                         e->priv_tx_len ? e->priv_tx : NULL, e->priv_tx_len);
            if (e->t->send(e->t->ctx, rj, CM_MSG_LEN) == 0) e->tx_n++;
            e->rejected = 1; e->rej_reason = CM_REJ_UNSUPPORTED;
            e->st = CM_ST_FAIL;
            return -4;
        }
        if (memcmp(&r[CM_REQ_REMOTE_GID], e->lgid, 16) != 0) return 0; /* REQ.Remote GID 应 = 本端(否则忽略) */
        memcpy(e->priv_rx, &r[CM_REQ_PRIV], CM_REQ_PRIV_LEN);   /* ★ M2:对端 app 块 */
        e->priv_rx_len = CM_REQ_PRIV_LEN;
        e->rqpn = rqpn; e->rpsn = rpsn;
        e->rcomm = cm_get32(&r[CM_REQ_LCOMM_ID]);
        e->rpkey = cm_get16(&r[CM_REQ_PKEY]);
        e->rtclass = r[CM_REQ_TCLASS];
        memcpy(e->rgid, &r[CM_REQ_LOCAL_GID], 16);
        e->neg_mtu = pmtu;
        e->rx_n++;
        e->tid = h.tid;                                          /* REP/RTU 沿同一事务 ID */
        cm_build_rep(m, e->tid, e->lcomm, e->rcomm, e->lqpn, e->lpsn,
                     e->priv_tx_len ? e->priv_tx : NULL, e->priv_tx_len);
        if (e->t->send(e->t->ctx, m, CM_MSG_LEN) == 0) {
            e->tx_n++; e->st = CM_ST_WAIT_RTU;
        }
        return 0;
    }

    /* CM_ST_WAIT_RTU */
    {
        cm_hdr_t h = cm_parse_hdr(r);
        if (h.ok && h.attr_id == CM_ATTR_ID_RTU && h.tid == e->tid) {
            e->rx_n++; e->st = CM_ST_CONNECTED;
            return 1;
        }
    }
    return 0;
}

int cm_run_pair(cm_entity_t *a, cm_entity_t *b, int max_steps)
{
    int i;
    for (i = 0; i < max_steps; i++) {
        int rb = cm_passive_step(b);
        int ra = cm_active_step(a);
        if (rb < 0 || ra < 0) return -1;
        if (rb == 1 && ra == 1) return 0;
    }
    return 1;                                           /* 超时 */
}

/* 步骤 2 自检:两端会合 + 协商对表 + 计数 + 两条负控;返回 0 = 过 */
int cm_exchange_selftest(cm_transport_t *ta, cm_transport_t *tb)
{
    static const uint8_t gidA[16] = {0xfe,0x80,0,0,0,0,0,0,0,0,0,0,0x0a,0,0,0x02};
    static const uint8_t gidB[16] = {0xfe,0x80,0,0,0,0,0,0,0,0,0,0,0x0a,0,0,0x03};
    cm_entity_t A, B, A2;
    int r;

    /* --- 正控:双端会合 --- */
    cm_entity_init(&A, ta, 0x000011u, 0xA00001u, 5, 0xFFFF, gidA, 0x2Cu);
    cm_entity_init(&B, tb, 0x000022u, 0xB00001u, 5, 0xFFFF, gidB, 0x2Cu);
    cm_entity_set_peer(&A, gidB);                       /* ★ 主动端"拨号地址" = B(照真流程) */
    r = cm_run_pair(&A, &B, 200);
    if (r != 0) return 1;
    /* 协商对表(逐字段) */
    if (A.rqpn != 0x000022u || B.rqpn != 0x000011u) return 2;      /* ★ 对端 QPN */
    if (A.rpsn != 0xB00001u || B.rpsn != 0xA00001u) return 3;      /* ★ 对端起始 PSN */
    if (A.rcomm != B.lcomm || B.rcomm != A.lcomm) return 4;        /* comm id 互认 */
    if (A.neg_mtu != 5 || B.neg_mtu != 5) return 5;                /* MTU 协商 */
    if (memcmp(A.rgid, gidB, 16) != 0 || memcmp(B.rgid, gidA, 16) != 0) return 6;  /* GID 互认 */
    /* ⚠ P_Key/TClass = **REQ 单向携带、应答端采纳**(REP 无此字段)⇒ 只查 B 侧(F 学习方向照真流程) */
    if (B.rpkey != 0xFFFF || B.rtclass != 0x2Cu) return 7;
    /* 消息计数(读数):A = REQ+RTU 发 2 / 收 REP 1;B = 发 REP 1 / 收 REQ+RTU 2 */
    if (A.tx_n != 2 || A.rx_n != 1 || B.tx_n != 1 || B.rx_n != 2) return 8;

    /* --- 负控 1:坏包(类版本错)⇒ 被动端不认、保持 WAIT_REQ ---
       (须在无应答负控【之前】做:坏包被弹出即清,不残留队列) */
    {
        cm_entity_t B2; uint8_t bad[CM_MSG_LEN];
        cm_entity_init(&B2, tb, 0x000022u, 0xB00001u, 5, 0xFFFF, gidB, 0x2Cu);
        cm_passive_step(&B2);                                       /* IDLE→WAIT_REQ */
        cm_build_req(bad, 0x1000000000000001ull, 0xA5A50011u, 0x11, 0xA00001u, 5,
                     0xFFFF, gidA, gidB, 0x2Cu, NULL, 0);
        bad[CM_HDR_CLASS_VER] = 0x09;                               /* 砸类版本 */
        if (ta->send(ta->ctx, bad, CM_MSG_LEN) != 0) return 9;
        for (int i = 0; i < 5; i++) cm_passive_step(&B2);
        if (B2.st != CM_ST_WAIT_REQ) return 10;
    }

    /* --- 负控 2:无应答端 ⇒ 超时 FAIL,且一包未收 --- */
    cm_entity_init(&A2, ta, 0x000011u, 0xA00001u, 5, 0xFFFF, gidA, 0x2Cu);
    for (int i = 0; i < 200; i++) { int rr = cm_active_step(&A2); if (rr < 0) break; (void)rr; }
    if (A2.st != CM_ST_FAIL) return 11;
    if (A2.rx_n != 0) return 12;
    return 0;
}
