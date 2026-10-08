//============================================================================
// tb_cm_m3.cpp — ★ CM 建链 M3:验收场景合流(CM 建链 ⇒ perftest 量级工作负载)  [2026-10-07 建]
//
// 口径(施工册 = 规格文档;规格 §3 M3:「对接验收场景 ⇒ 与 B5 最小集合流」)
//   场景 = **B5 最小集**:CM 自动建链 ⇒ **写一笔/读一笔/SEND 一笔** × N 轮(perftest 量级),
//   每笔 CQE + 中断,数据面经真 MAC/线/对端替身(RocePeerTlm);判据与验收清单对齐 ✓
//   拓扑/CM 相位/三段落表照 M2(engine_m1 克隆;B 侧 = 被动实体 + 替身)✓
//
// 判据(预登记;读数取 `$finish` 前落定后):
//   ①②③ 会合 + priv 往返 + 三段(INIT→RTR→RTS)读回(照 M2)✓
//   ④ 工作负载(N=4 轮 × 写/读/SEND):**对端收 WRITE/READ/SEND 各 == N** · CQE 总数 == 3N ·
//      **CQE 序列 opcode 轮转(0x00/0x04/0x02)** · READ 响应落本地逐字节 · 中断抬起 ·
//      **QP_SQ_PSN == CM PSN + 3N**(每笔请求 +1)· 末帧 BTH PSN/DestQP == CM 值 ✓
//   ⑤ 清中断(真清路 = 写 CQ_INTR_STS1 掩码)⇒ INTR_STS[4] 落 ✓
//   ⑥ 计数收尾(线包 == 3N / 非预期 0 / 门铃丢弃 0 / goerr 0 / fcs 0)✓
//============================================================================
#include <systemc.h>
#include <cstring>
#include <amba_pv.h>
#include <models/amba_pv_ace_protocol_checker.h>
#include "stack_top.h"
#include "dummy_amba_master.h"
#include "mac_tlm.h"
#include "roce_peer_tlm.h"
#include "nic_regs.h"
#include "nic_hw.h"
#include "nic_cm.h"
#include "cm_pipe.h"
#include <cstdio>

//----------------------------------------------------------------------------
struct AceHostMem: sc_core::sc_module, amba_pv::amba_pv_ace_slave_base {
    amba_pv::amba_pv_ace_slave_socket<512> ace_s;
    unsigned char mem[65536 * cxlshim::LINE_BYTES];      // 4MB(堆分配,照双栈台踩坑)✓
    uint32_t rd_n = 0, wr_n = 0, bad = 0;
    AceHostMem(sc_core::sc_module_name nm): amba_pv_ace_slave_base("host_mem"), ace_s("ace_s") {
        ace_s(*this);
        for (unsigned i = 0; i < sizeof(mem); i++) mem[i] = 0;
    }
    void b_transport(int, amba_pv::amba_pv_transaction & tr, sc_core::sc_time & t) override {
        amba_pv::amba_pv_extension * ex = NULL; tr.get_extension(ex);
        size_t off = (size_t)((tr.get_address() >> 6) & 0xFFFF) * cxlshim::LINE_BYTES;
        unsigned char * d = tr.get_data_ptr();
        if (tr.is_read()) { rd_n++;  for (int i = 0; i < cxlshim::LINE_BYTES; i++) d[i] = mem[off + i]; }
        else if (tr.is_write()) { wr_n++; for (int i = 0; i < cxlshim::LINE_BYTES; i++) mem[off + i] = d[i]; }
        else bad++;
        if (ex != NULL) ex->set_resp(amba_pv::AMBA_PV_OKAY);
        t += sc_core::sc_time(4, sc_core::SC_NS);
    }
    unsigned char * line_ptr(sc_dt::uint64 byte_addr) {
        return &mem[(size_t)((byte_addr >> 6) & 0xFFFF) * cxlshim::LINE_BYTES];
    }
    unsigned int transport_dbg(int, amba_pv::amba_pv_transaction &) override { return 0; }
    bool get_direct_mem_ptr(int, amba_pv::amba_pv_transaction &, tlm::tlm_dmi &) override { return false; }
};

//----------------------------------------------------------------------------
struct CmM3Drv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    sc_in<bool> intr;
    sc_in<sc_uint<32>> o_cqe, o_wqe, o_dbd, o_goerr, o_lbig, o_ltx, o_lack, o_lun, o_dtx, o_mfcs;
    DeviceTlm  * dev = nullptr;
    AceHostMem * mem = nullptr;
    RocePeerTlm* peer = nullptr;
    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(CmM3Drv);
    CmM3Drv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char * name) {
        printf("   v %-56s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }
    void chk_eq(const char * name, uint32_t got, uint32_t exp) {
        printf("   v %-44s got=0x%08x exp=0x%08x\n", name, got, exp);
        if (got != exp) { fail_cnt++; printf("   ✗ %s 不符\n", name); } else pass_cnt++;
    }
    void tick() { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); }

    // ---- nic_io_t 适配(ctx = this;offset ⇄ 绝对地址 照 NIC_BASE)✓ ----
    static nic_u32 io_rd32(void *ctx, nic_u32 off) {
        CmM3Drv *d = (CmM3Drv *)ctx; sc_uint<32> v = 0;
        if (!d->dev->reg_read(NIC_BASE + off, v)) d->fail_cnt++;
        return v.to_uint();
    }
    static void io_wr32(void *ctx, nic_u32 off, nic_u32 v) {
        CmM3Drv *d = (CmM3Drv *)ctx;
        if (!d->dev->reg_write(NIC_BASE + off, sc_uint<32>(v))) d->fail_cnt++;
    }
    void rw(sc_dt::uint64 a, uint32_t v) {
        if (!dev->reg_write(a, sc_uint<32>(v))) { printf("   x 写 0x%llx 未实现\n", (unsigned long long)a); fail_cnt++; }
    }
    uint32_t rdr(sc_dt::uint64 a) {
        sc_uint<32> v = 0;
        if (!dev->reg_read(a, v)) { printf("   x 读 0x%llx 未实现\n", (unsigned long long)a); fail_cnt++; }
        return v.to_uint();
    }
    // 改已装 WQE 的 opcode 字节(装 WQE 后调;照 engine_m2 成法)✓
    static sc_dt::uint64 sq_slot(uint32_t pi) { return (pi == 8) ? 0 : (sc_dt::uint64)pi; }  /* ★ 槽 = pi,depth→0 ✓ */
    void mem_set_opc(sc_dt::uint64 SQ, uint32_t pi, uint8_t opc) {
        unsigned char * wq = mem->line_ptr(SQ + sq_slot(pi) * 64);
        wq[NIC_WQE_OPC_OFF] = opc;
    }
    void wqe(sc_dt::uint64 SQ, uint32_t pi, uint32_t wrid, uint32_t len,
             uint32_t loff, sc_dt::uint64 roff, uint8_t qpid, uint32_t rkey) {
        unsigned char * wq = mem->line_ptr(SQ + sq_slot(pi) * 64);
        for (int i = 0; i < 64; i++) wq[i] = 0;
        wq[NIC_WQE_WRID_OFF + 0] = (unsigned char)(wrid & 0xff);
        wq[NIC_WQE_WRID_OFF + 1] = (unsigned char)((wrid >> 8) & 0xff);
        *(uint32_t *)(wq + NIC_WQE_LEN_OFF)  = len;
        wq[NIC_WQE_OPC_OFF] = NIC_OPC_WRITE;
        *(uint32_t *)(wq + NIC_WQE_LOFF_OFF) = loff;
        *(uint32_t *)(wq + NIC_WQE_ROFF_OFF) = (uint32_t)roff;
        wq[NIC_WQE_QID_OFF] = qpid;                       // (诱饵:包 DestQP 真源 = 寄存器 0x48)✓
        *(uint32_t *)(wq + NIC_WQE_RKEY_OFF) = rkey;
    }

    void run() {
        rst_n.write(false);
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        tick();

        const sc_dt::uint64 SQ = 0x00020000, CQ = 0x00030000, SRC = 0x00040000, ROFF = 0x00090000;
        const sc_dt::uint64 DST_R = 0x00050000;   // ★ M3:READ 响应本地落点(照 engine_m2)✓
        const uint32_t WRID = 0x1234, RK = 0x98;
        const uint32_t PSN_A = 0x0BC001, PSN_B = 0x0BC101;    // ★ 各方**自己**的起始 PSN(CM 报出)✓
        const uint32_t QPN_A = 3, QPN_B = 5;                  // ★ 各方**自己**的 QPN(CM 报出)✓
        const uint16_t PKEY = 0x666F;
        static const uint8_t gidA[16] = {0xfe,0x80,0,0,0,0,0,0,0,0,0,0,0x0a,0,0,0x01};   // 内嵌 10.0.0.1
        static const uint8_t gidB[16] = {0xfe,0x80,0,0,0,0,0,0,0,0,0,0,0x0a,0,0,0x02};   // 内嵌 10.0.0.2

        // ================= 相位 ①:CM 建链(软件;管道会合)=================
        cm_pipe_t pipe; cm_transport_t ta, tb;
        cm_entity_t cma, cmb;
        cm_pipe_attach(&pipe, &ta, &tb);
        cm_entity_init(&cma, &ta, QPN_A, PSN_A, 5, PKEY, gidA, 0x2Cu);
        cm_entity_init(&cmb, &tb, QPN_B, PSN_B, 5, PKEY, gidB, 0x2Cu);
        cm_entity_set_peer(&cma, gidB);                       // 主动端"拨号地址" = B ✓
        static const uint8_t appA[16] = {0xA1,0xA2,0xA3,0xA4, 0x44,0x33,0x22,0x11, 0x00,0x00,0x40,0x00, 0,0,0,0}; /* rkey=0x11223344 VA=0x4000 替身 */
        static const uint8_t appB[16] = {0xB1,0xB2,0xB3,0xB4, 0x88,0x77,0x66,0x55, 0x00,0x00,0x80,0x00, 0,0,0,0}; /* rkey=0x55667788 VA=0x8000 */
        memcpy(cma.priv_tx, appA, 16); cma.priv_tx_len = 16;   /* ★ M2:app 块走 CM 私有数据 ✓ */
        memcpy(cmb.priv_tx, appB, 16); cmb.priv_tx_len = 16;

        // ---- ②' 负控:建链前 QP 未配置 ----
        chk(rdr(0x50180200 + 0x40) == 0 && rdr(0x50180200 + 0x48) == 0 && cma.st == CM_ST_IDLE,
            "②' 负控:建链前 QP 未配置(0x40/0x48==0)且未 CONNECTED");

        int rr = cm_run_pair(&cma, &cmb, 200);
        chk_eq("① 双端会合(cm_run_pair 返回)", (uint32_t)rr, 0);
        chk(cma.st == CM_ST_CONNECTED && cmb.st == CM_ST_CONNECTED, "① 两端 CONNECTED");
        chk(cma.tx_n == 2 && cma.rx_n == 1 && cmb.tx_n == 1 && cmb.rx_n == 2,
            "① 三消息计数(A:发2收1 / B:发1收2)");
        printf("   [协商] A 学到: 对端 QPN=%u PSN=0x%06x comm=0x%08x MTU=%d | B 学到: 对端 QPN=%u PSN=0x%06x\n",
               cma.rqpn, cma.rpsn, cma.rcomm, cma.neg_mtu, cmb.rqpn, cmb.rpsn);
        chk_eq("① 协商: A 学到对端 QPN", cma.rqpn, QPN_B);
        chk_eq("① 协商: A 学到对端 PSN", cma.rpsn, PSN_B);
        chk_eq("① 协商: B 学到对端 QPN", cmb.rqpn, QPN_A);
        chk_eq("① 协商: B 学到对端 PSN", cmb.rpsn, PSN_A);
        chk(memcmp(cmb.priv_rx, appA, 16) == 0, "② 私有数据往返:B 收到 A 的 16B app 块(逐字节)");
        chk(memcmp(cma.priv_rx, appB, 16) == 0, "② 私有数据往返:A 收到 B 的 16B app 块(逐字节)");

        // ================= 相位 ③:CM 协商值 ⇒ 寄存器落表(三段)=================
        // 节点身份(非 CM 协商面 ⇒ 直配;照发动机台配方)✓
        rw(0x50100000, 0x6851u << 8);                // UDP src port
        rw(0x50180200 + 0x50, 0x001c1eeau);          // MAC_REMOTE_LSB(DA 低 24: ea,1e,1c)
        rw(0x50180200 + 0x54, 0x0000edb8u);          // MAC_REMOTE_MSB(DA 高 24: 69,b8,ed)
        rw(0x50100010, 0xdc17762fu);                 // MAC_NIC_LSB(SA 低 32)
        rw(0x50100014, 0x00009a5eu);                 // MAC_NIC_MSB(SA 高 16)
        rw(0x50100070, 0x0a000001u);                 // IPV4_NIC_ADDR(源 IP;字节低先)
        rw(0x50100180, (1u << 4));                   // INTR_EN bit4(CQ)
        {
            DeviceTlm::LineId id;
            static const unsigned char SA[6] = {0x2f,0x76,0x17,0xdc,0x5e,0x9a};
            static const unsigned char DA[6] = {0xea,0x1e,0x1c,0x69,0xb8,0xed};
            for (int i = 0; i < 6; i++) { id.sa[i] = SA[i]; id.da[i] = DA[i]; }
            id.sip = 0x0a000001; id.dip = 0x0a000002; id.sport = 0x6851; id.dport = 0x12b7;
            dev->set_line_id(id);
            dev->set_line_pkt_mode(true);
        }
        //★ M2/M3:verbs 三段(INIT→RTR→RTS;逐段读回 = 状态推进的判据)✓
        nic_io_t io = { this, io_rd32, io_wr32 };
        nic_qp_to_init(&io, 2, (nic_u32)SQ, (nic_u32)CQ, 8, cma.lpsn);                 /* INIT */
        chk_eq("③ INIT 后:QP_SQ_PSN == CM 报出 PSN", rdr(0x50180200 + 0x40), PSN_A);
        chk_eq("③ INIT 后:DEST_QP 仍未配(==0)", rdr(0x50180200 + 0x48), 0);
        chk_eq("③ INIT 后:QP_CONF.EN == 0(未使能)", rdr(0x50180200 + 0x00) & 1u, 0);
        nic_qp_to_rtr(&io, 2, cma.rqpn, cm_gid_to_ipv4_reg(cma.rgid), cma.lpkey,
                       (nic_u32)cma.neg_mtu);                                            /* RTR */
        chk_eq("③ RTR 后:DEST_QP == CM 学到的对端 QPN", rdr(0x50180200 + 0x48), cma.rqpn);
        chk_eq("③ RTR 后:IP_REMOTE1 == GID⇒IP 换算(0x0200000a)", rdr(0x50180200 + 0x60), 0x0200000au);
        chk_eq("③ RTR 后:QP_CONF.EN 仍 == 0", rdr(0x50180200 + 0x00) & 1u, 0);
        nic_qp_to_rts(&io, 2);                                                           /* RTS */
        chk_eq("③ RTS 后:QP_CONF.EN == 1(RMW 置位)", rdr(0x50180200 + 0x00) & 1u, 1);
        chk_eq("③ RTS 后:QP_CONF[10:8] == 协商 MTU", (rdr(0x50180200 + 0x00) >> 8) & 7u, (uint32_t)cma.neg_mtu);
        chk_eq("③ ADV_CONF[31:16] == P_Key 寄存器形态", rdr(0x50180200 + 0x04) >> 16, 0x6f66u);
        chk_eq("③ 换算对表: cm_gid_to_ipv4_reg(对端 GID)", cm_gid_to_ipv4_reg(cma.rgid), 0x0200000a);

        // ================= 相位 ④:验收工作负载(B5 最小集;N 轮 × 写/读/SEND)=================
        {
            const int N = 4;
            unsigned char pread[64];
            for (int i = 0; i < 64; i++) pread[i] = (unsigned char)((i * 11 + 0x40) & 0xff);
            peer->set_pmem(pread, 64);                                // READ 响应载荷源 ✓
            unsigned char * src = mem->line_ptr(SRC);
            for (int i = 0; i < 64; i++) src[i] = (unsigned char)((i * 7 + 0x31) & 0xff);
            uint32_t pi = 1, wrid = 0xC000, cqk = 0;
            for (int r = 0; r < N; r++) {
                wqe(SQ, pi, ++wrid, 64, (uint32_t)SRC, ROFF, (uint8_t)9, RK);      /* 写一笔 */
                rw(0x50180200 + 0x38, pi);
                cqk++; for (int j = 0; j < 6000 && o_cqe.read().to_uint() < cqk; j++) tick();
                printf("   [逐笔] 轮%d 第1笔(WRITE) CQE=%u 对端 W=%u R=%u S=%u\n", r+1, o_cqe.read().to_uint(), peer->req_cnt, peer->rd_req_cnt, peer->send_cnt);
                pi = (pi == 8) ? 1 : pi + 1;   /* ★ 指针回绕 1..8(§9.2)✓ */
                wqe(SQ, pi, ++wrid, 64, (uint32_t)DST_R, ROFF, (uint8_t)9, RK);    /* 读一笔 */
                mem_set_opc(SQ, pi, NIC_OPC_READ);
                rw(0x50180200 + 0x38, pi);
                cqk++; for (int j = 0; j < 6000 && o_cqe.read().to_uint() < cqk; j++) tick();
                printf("   [逐笔] 轮%d 第2笔(READ)  CQE=%u 对端 W=%u R=%u S=%u\n", r+1, o_cqe.read().to_uint(), peer->req_cnt, peer->rd_req_cnt, peer->send_cnt);
                pi = (pi == 8) ? 1 : pi + 1;   /* ★ 指针回绕 1..8(§9.2)✓ */
                wqe(SQ, pi, ++wrid, 64, (uint32_t)SRC, 0, (uint8_t)9, RK);         /* SEND 一笔 */
                mem_set_opc(SQ, pi, NIC_OPC_SEND);
                rw(0x50180200 + 0x38, pi);
                cqk++; for (int j = 0; j < 6000 && o_cqe.read().to_uint() < cqk; j++) tick();
                printf("   [逐笔] 轮%d 第3笔(SEND)  CQE=%u 对端 W=%u R=%u S=%u\n", r+1, o_cqe.read().to_uint(), peer->req_cnt, peer->rd_req_cnt, peer->send_cnt);
                pi = (pi == 8) ? 1 : pi + 1;   /* ★ 指针回绕 1..8(§9.2)✓ */
            }
            for (int j = 0; j < 40; j++) tick();                       // 落定 ✓
            chk_eq("④ 对端收到 WRITE 数 == N", peer->req_cnt, (uint32_t)N);
            chk_eq("④ 对端收到 READ 数 == N", peer->rd_req_cnt, (uint32_t)N);
            chk_eq("④ 对端收到 SEND 数 == N", peer->send_cnt, (uint32_t)N);
            chk_eq("④ CQE 总数 == 3N", o_cqe.read().to_uint(), (uint32_t)(3 * N));
            {
                int bad = 0;                                  /* 环内末 8 笔(深度 8;前 4 笔已被回绕覆写,设计如此)✓ */
                static const uint8_t exp_opc[3] = {0x00, 0x04, 0x02};
                for (int k = 4; k < 12; k++) {                /* op k+1(1-based);类型 = k%3: W/R/S ✓ */
                    int hd = (k % 8) + 1;                     /* CQ hd 1..8 循环 ✓ */
                    int idx = (hd == 8) ? 0 : hd;             /* 照模型 cq_idx 公式 ✓ */
                    unsigned char * cq1 = mem->line_ptr(CQ + idx * 4) + (unsigned)((CQ + idx * 4) & 0x3C);
                    if (NIC_CQE_OPCODE(cq1) != exp_opc[k % 3]) bad++;
                }
                chk_eq("④ CQE 序列(环内末 8 笔)opcode 错项数", (uint32_t)bad, 0);
            }
            {
                int bad = 0;                                           /* READ 响应落本地逐字节 ✓ */
                unsigned char * d = mem->line_ptr(DST_R);
                for (int i = 0; i < 64; i++) if (d[i] != pread[i]) bad++;
                chk_eq("④ READ 响应落本地(错字节数)", (uint32_t)bad, 0);
            }
            chk_eq("④ 中断线抬起", intr.read() ? 1u : 0u, 1);
            chk_eq("④ QP_SQ_PSN == CM PSN + 3N(每笔请求 +1)", rdr(0x50180200 + 0x40), PSN_A + 3 * N);
            {
                const unsigned char * p = peer->rx_buf;                /* 末帧 = 最后一笔 SEND ✓ */
                chk_eq("④ 末帧 BTH DestQP == CM 学到的对端 QPN",
                       ((uint32_t)p[47] << 16) | ((uint32_t)p[48] << 8) | p[49], cma.rqpn);
                chk_eq("④ 末帧 BTH PSN == CM PSN + 3N - 1",
                       ((uint32_t)p[51] << 16) | ((uint32_t)p[52] << 8) | p[53], PSN_A + 3 * N - 1);
            }
        }

        // ================= 相位 ⑤:清中断(真清路;照 D14/R12 口径)=================
        {
            uint32_t sts = nic_intr_status(&io);
            chk((sts & NIC_INTR_WQE_CMPL) != 0, "⑤ 全局 INTR_STS[4] 已置(CQ 完成源)");
            nic_cq_intr_ack(&io, 1u << 2);                            // CQ_INTR_STS1 掩码(QP 索引 2)✓
            for (int j = 0; j < 8; j++) tick();
            uint32_t sts2 = nic_intr_status(&io);
            chk_eq("⑤ 清后 INTR_STS[4] 落(0)", (sts2 & NIC_INTR_WQE_CMPL) ? 1u : 0u, 0);
        }

        // ---- 计数收尾 ----
        chk_eq("⑥ line:已发请求包数 == 3N(=12)", o_ltx.read().to_uint(), 12);
        printf("   [计数] line:已匹配 ACK 数 = %u(信息项)\n", o_lack.read().to_uint());
        chk_eq("⑥ line:非预期帧 == 0", o_lun.read().to_uint(), 0);
        chk_eq("⑥ 门铃丢弃 / goerr == 0", o_dbd.read().to_uint() + o_goerr.read().to_uint(), 0);
        chk_eq("⑥ MAC 坏帧(fcs_err) == 0", o_mfcs.read().to_uint(), 0);

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    StackTop stack("stack");
    DummyAmbaMaster dum("dum");
    stack.dev.reg_s.bind(dum.m);
    stack.clk(clk); stack.rst_n(rst_n);

    MacTlm mac_b("mac_b");
    mac_b.clk(clk); mac_b.rst_n(rst_n);

    RocePeerTlm peer("peer");
    peer.clk(clk); peer.rst_n(rst_n);

    AceHostMem * mem = new AceHostMem("mem");
    amba_pv::amba_pv_ace_protocol_checker<512> chk("chk");
    stack.br.ace_m.bind(chk.amba_pv_s); chk.amba_pv_m.bind(mem->ace_s);

    sc_core::sc_signal<sc_biguint<512>> Wa0, Wa1, Wb0, Wb1;
    sc_core::sc_signal<sc_biguint<64>>  TCa0, TCa1, TCb0, TCb1;
    sc_core::sc_signal<bool> Tda0, Tda1, Tdb0, Tdb1;
    sc_core::sc_signal<bool> Eax0("Eax0", true), Eax1("Eax1", true), Ear0("Ear0", true), Ear1("Ear1", true);
    sc_core::sc_signal<bool> Ebx0("Ebx0", true), Ebx1("Ebx1", true), Ebr0("Ebr0", true), Ebr1("Ebr1", true);
    stack.mac.link_txd_0(Wa0);   stack.mac.link_txd_1(Wa1);
    stack.mac.link_txc_0(TCa0);  stack.mac.link_txc_1(TCa1);
    stack.mac.link_txdval_0(Tda0); stack.mac.link_txdval_1(Tda1);
    stack.mac.link_txclk_ena_0(Eax0); stack.mac.link_txclk_ena_1(Eax1);
    stack.mac.link_rxclk_ena_0(Ear0); stack.mac.link_rxclk_ena_1(Ear1);
    mac_b.link_txd_0(Wb0);   mac_b.link_txd_1(Wb1);
    mac_b.link_txc_0(TCb0);  mac_b.link_txc_1(TCb1);
    mac_b.link_txdval_0(Tdb0); mac_b.link_txdval_1(Tdb1);
    mac_b.link_txclk_ena_0(Ebx0); mac_b.link_txclk_ena_1(Ebx1);
    mac_b.link_rxclk_ena_0(Ebr0); mac_b.link_rxclk_ena_1(Ebr1);
    mac_b.link_rxd_0(Wa0); mac_b.link_rxd_1(Wa1);
    mac_b.link_rxc_0(TCa0); mac_b.link_rxc_1(TCa1);
    stack.mac.link_rxd_0(Wb0); stack.mac.link_rxd_1(Wb1);
    stack.mac.link_rxc_0(TCb0); stack.mac.link_rxc_1(TCb1);

    sc_core::sc_signal<bool> B_tv, B_tl, B_tr, B_rv, B_rl, B_tu;
    sc_core::sc_signal<sc_biguint<2048>> B_td, B_rd;
    sc_core::sc_signal<sc_biguint<256>>  B_tk, B_rk;
    mac_b.cmac_m_axis_tvalid(B_tv); mac_b.cmac_m_axis_tdata(B_td);
    mac_b.cmac_m_axis_tkeep(B_tk);  mac_b.cmac_m_axis_tlast(B_tl);
    mac_b.cmac_m_axis_tready(B_tr);
    mac_b.roce_cmac_s_axis_tvalid(B_rv); mac_b.roce_cmac_s_axis_tdata(B_rd);
    mac_b.roce_cmac_s_axis_tkeep(B_rk);  mac_b.roce_cmac_s_axis_tlast(B_rl);
    mac_b.roce_cmac_s_axis_tuser(B_tu);
    peer.cmac_m_axis_tvalid(B_rv); peer.cmac_m_axis_tdata(B_rd);
    peer.cmac_m_axis_tkeep(B_rk);  peer.cmac_m_axis_tlast(B_rl);
    peer.cmac_m_axis_tready(B_tr);
    peer.roce_cmac_s_axis_tvalid(B_tv); peer.roce_cmac_s_axis_tdata(B_td);
    peer.roce_cmac_s_axis_tkeep(B_tk);  peer.roce_cmac_s_axis_tlast(B_tl);
    peer.roce_cmac_s_axis_tuser(B_tu);
    sc_core::sc_signal<sc_uint<32>> p_req, p_ack, p_bad;
    sc_core::sc_signal<sc_uint<32>> b_fcs_lm;
    peer.o_req_cnt(p_req); peer.o_ack_sent(p_ack); peer.o_bad_cnt(p_bad);
    sc_core::sc_signal<sc_uint<32>> b_ftx, b_frx, b_idle, b_fcs;
    mac_b.o_frames_tx(b_ftx); mac_b.o_frames_rx(b_frx);
    mac_b.o_idle_beats(b_idle); mac_b.o_fcs_err(b_fcs);
    mac_b.o_lane_mismatch_cnt(b_fcs_lm);

    CmM3Drv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n); drv.intr(stack.sig_intr);
    drv.dev = &stack.dev; drv.mem = mem; drv.peer = &peer;
    drv.o_cqe(stack.sig_cqe); drv.o_wqe(stack.sig_wqe); drv.o_dbd(stack.sig_dbd);
    drv.o_goerr(stack.sig_goerr);
    drv.o_lbig(stack.sig_lbig); drv.o_ltx(stack.sig_ltx);
    drv.o_lack(stack.sig_lack); drv.o_lun(stack.sig_lun);
    drv.o_dtx(stack.sig_mtx); drv.o_mfcs(stack.sig_mfcs);

    printf("============================================================\n");
    printf("tb_cm_m3 - ★ CM 建链 M3:CM 建链 ⇒ B5 最小集工作负载(N 轮 写/读/SEND + 中断)\n");
    printf("============================================================\n");

    sc_core::sc_start(60, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_CM_M3 %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
