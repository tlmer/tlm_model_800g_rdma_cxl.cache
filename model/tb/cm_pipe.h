/*============================================================================
 * cm_pipe.h — M1 transport 实现:进程内双向字节管道  [2026-10-07 建]
 *
 * 口径(规格文档 §2.1;规格 §4-4/§4-5):
 *   · **非真栈** —— 只提供"有序、可靠、保消息边界"的最小语义;TCP 的重传/窗口/拥塞**不在本件**;
 *     集成期换真载体(宿主 socket / 平台通道)时,**只换本件** —— CM 消息层一行不改 ✓
 *   · 消息边界:CM 三消息**定长 256B** ⇒ 真 TCP 字节流也可按 256B 分帧(无需长度前缀)✓
 *   · 单线程确定性:两端由调用方按步驱动(无锁、无调度);容量 16 条消息(超 ⇒ send 返回 -1)✓
 *==========================================================================*/
#ifndef CM_PIPE_H
#define CM_PIPE_H

#include <string.h>
#include "nic_cm.h"        /* cm_transport_t / CM_MSG_LEN */

#define CM_PIPE_NMSG 16
#define CM_PIPE_CAP  (CM_MSG_LEN * CM_PIPE_NMSG)

typedef struct cm_pipe_q {
    uint8_t buf[CM_PIPE_CAP];
    int     head, len;         /* len 恒为 256 的整数倍(保消息边界) */
} cm_pipe_q_t;

typedef struct cm_pipe {
    cm_pipe_q_t ab;            /* A→B 队列 */
    cm_pipe_q_t ba;            /* B→A 队列 */
} cm_pipe_t;

static inline int cm_pipe_push(cm_pipe_q_t *q, const uint8_t *b, int n)
{
    int i;
    if (n <= 0 || q->len + n > CM_PIPE_CAP) return -1;
    for (i = 0; i < n; i++)
        q->buf[(q->head + q->len + i) % CM_PIPE_CAP] = b[i];
    q->len += n;
    return 0;
}

static inline int cm_pipe_pop(cm_pipe_q_t *q, uint8_t *b, int cap)
{
    int i;
    if (q->len < CM_MSG_LEN || cap < CM_MSG_LEN) return 0;
    for (i = 0; i < CM_MSG_LEN; i++)
        b[i] = q->buf[(q->head + i) % CM_PIPE_CAP];
    q->head = (q->head + CM_MSG_LEN) % CM_PIPE_CAP;
    q->len -= CM_MSG_LEN;
    return CM_MSG_LEN;
}

/* A 端 transport:A 发 ⇒ ab;B 发 ⇒ ba */
static inline int cm_pipe_a_send(void *ctx, const uint8_t *b, int n)
{ return cm_pipe_push(&((cm_pipe_t *)ctx)->ab, b, n); }
static inline int cm_pipe_a_recv(void *ctx, uint8_t *b, int c)
{ return cm_pipe_pop(&((cm_pipe_t *)ctx)->ba, b, c); }
static inline int cm_pipe_b_send(void *ctx, const uint8_t *b, int n)
{ return cm_pipe_push(&((cm_pipe_t *)ctx)->ba, b, n); }
static inline int cm_pipe_b_recv(void *ctx, uint8_t *b, int c)
{ return cm_pipe_pop(&((cm_pipe_t *)ctx)->ab, b, c); }

/* 建管道并接好两端 transport */
static inline void cm_pipe_attach(cm_pipe_t *p, cm_transport_t *ta, cm_transport_t *tb)
{
    memset(p, 0, sizeof(*p));
    ta->send = cm_pipe_a_send; ta->recv = cm_pipe_a_recv; ta->ctx = p;
    tb->send = cm_pipe_b_send; tb->recv = cm_pipe_b_recv; tb->ctx = p;
}

#endif /* CM_PIPE_H */
