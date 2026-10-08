//============================================================================
// tb_cm_m2.cpp — ★ CM 建链 M2:参数面全量 + QP 状态推进(INIT→RTR→RTS)+ 错误分支  [2026-10-07 建]
//
// 口径(施工册 = 规格文档;规格 = 规格文档 §3 M2)
//   在 M1 之上补:① 私有数据往返(rkey/VA app 块,16B 定长替身)② verbs 三段逐段读回
//   (INIT/RTR 皆 EN=0;RTS = RMW 置 EN)③ 错误分支:**拒绝(REJ)** 与 **超时/重试(计数)**
//   拓扑与判据骨架照 M1(engine_m1 克隆;B 侧 = 被动实体 + RocePeerTlm)✓
//
// 判据(预登记;读数取 `$finish` 前落定后):
//   ① 会合 + 协商对表(照 M1)✓
//   ② **私有数据往返**:B 收 A 的 16B app 块 ∧ A 收 B 的(逐字节;rkey/VA 替身)✓
//   ③ **三段读回**:INIT(SQ_PSN ✓ / DEST_QP=0 / EN=0)· RTR(DEST_QP ✓ / EN=0)· RTS(EN=1)✓
//   ④ 一笔真 RDMA:包 DestQP/PSN == CM 值 + CQE + intr(照 M1)✓
//   ⑤ **拒绝分支**:被动能力 MTU=3 < 提议 5 ⇒ 发 REJ(reason=5 UNSUPPORTED)⇒ 主动 FAIL+原因 ✓ ∧ 支线零包 ✓
//   ⑥ **超时分支**:单跑无应答 ⇒ FAIL ∧ tx_n == 1+retry_max(=6) ∧ rx_n == 0 ✓
//   ⑦ 计数收尾(线包/ACK/异常/门铃/fcs)✓
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
struct CmM2Drv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    sc_in<bool> intr;
    sc_in<sc_uint<32>> o_cqe, o_wqe, o_dbd, o_goerr, o_lbig, o_ltx, o_lack, o_lun, o_dtx, o_mfcs;
    DeviceTlm  * dev = nullptr;
    AceHostMem * mem = nullptr;
    RocePeerTlm* peer = nullptr;
    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(CmM2Drv);
    CmM2Drv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

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
        CmM2Drv *d = (CmM2Drv *)ctx; sc_uint<32> v = 0;
        if (!d->dev->reg_read(NIC_BASE + off, v)) d->fail_cnt++;
        return v.to_uint();
    }
    static void io_wr32(void *ctx, nic_u32 off, nic_u32 v) {
        CmM2Drv *d = (CmM2Drv *)ctx;
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
    void wqe(sc_dt::uint64 SQ, uint32_t pi, uint32_t wrid, uint32_t len,
             uint32_t loff, sc_dt::uint64 roff, uint8_t qpid, uint32_t rkey) {
        unsigned char * wq = mem->line_ptr(SQ + (sc_dt::uint64)pi * 64);
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

        // ================= 相位 ④:一笔真 RDMA(包字段 = CM 值)=================
        wqe(SQ, 1, WRID, 64, (uint32_t)SRC, ROFF, (uint8_t)9, RK);   // byte17=9 诱饵 ✓
        unsigned char * src = mem->line_ptr(SRC);
        for (int i = 0; i < 64; i++) src[i] = (unsigned char)((i * 7 + 0x31) & 0xff);
        rw(0x50180200 + 0x38, 1);                                    // 门铃(pi=1)✓
        for (int i = 0; i < 4000 && o_cqe.read().to_uint() < 1; i++) tick();
        for (int i = 0; i < 20; i++) tick();

        chk_eq("④ 对端收到请求包数", peer->req_cnt, 1);
        chk_eq("④ 收到帧长", (uint32_t)peer->last_rx_len, 138);
        {
            const unsigned char * p = peer->rx_buf;
            chk_eq("④ BTH opcode(0x0a)", p[42], 0x0a);
            chk_eq("④ BTH DestQP == CM 学到的对端 QPN", ((uint32_t)p[47] << 16) | ((uint32_t)p[48] << 8) | p[49], cma.rqpn);
            chk_eq("④ BTH PSN == CM 报出 PSN", ((uint32_t)p[51] << 16) | ((uint32_t)p[52] << 8) | p[53], cma.lpsn);
            chk_eq("④ BTH AckReq", p[50], 1);
            {
                int bad = 0;
                for (int i = 0; i < 64; i++) if (p[70 + i] != (unsigned char)((i * 7 + 0x31) & 0xff)) bad++;
                chk_eq("④ 载荷 64B 错字节数", (uint32_t)bad, 0);
            }
            uint32_t c = 0xFFFFFFFFu;
            for (int k = 0; k < peer->last_rx_len; k++) c = RocePeerTlm::crc_icrc_step(c, p[k]);
            chk_eq("④ ICRC 残差 == 0xc704dd7b", c, 0xc704dd7b);
        }
        chk_eq("④ 设备 CQE 计数", o_cqe.read().to_uint(), 1);
        {
            unsigned char * cqe = mem->line_ptr(CQ + 4) + (unsigned)((CQ + 4) & 0x3C);
            chk_eq("④ CQE wrid", NIC_CQE_WRID(cqe), WRID);
            chk_eq("④ CQE opcode", NIC_CQE_OPCODE(cqe), 0x00);
        }
        chk_eq("④ 中断线抬起", intr.read() ? 1u : 0u, 1);
        chk_eq("④ QP_SQ_PSN 推进(+1)", rdr(0x50180200 + 0x40), PSN_A + 1);

        // ================= 相位 ⑤:错误分支① 拒绝(REJ)=================
        {
            uint32_t ltx0 = o_ltx.read().to_uint();
            cm_pipe_t p2; cm_transport_t ta2, tb2; cm_entity_t cma2, cmb2;
            cm_pipe_attach(&p2, &ta2, &tb2);                          // 新管道(独立支线)✓
            cm_entity_init(&cma2, &ta2, QPN_A, 0x0BD001, 5, PKEY, gidA, 0x2Cu);  /* 提议 MTU=5 */
            cm_entity_init(&cmb2, &tb2, QPN_B, 0x0BD101, 3, PKEY, gidB, 0x2Cu);  /* 能力 MTU=3 < 5 ⇒ 应拒 */
            cm_entity_set_peer(&cma2, gidB);
            (void)cm_run_pair(&cma2, &cmb2, 50);                      // 被动先放 REJ ⇒ 返回 -1
            for (int i = 0; i < 10 && cma2.st != CM_ST_FAIL; i++) (void)cm_active_step(&cma2);  // 主动吃 REJ
            chk(cma2.st == CM_ST_FAIL && cma2.rejected && cma2.rej_reason == CM_REJ_UNSUPPORTED,
                "⑤ 拒绝:主动 FAIL + 原因码 == UNSUPPORTED(5)");
            chk(cmb2.st == CM_ST_FAIL && cmb2.rejected, "⑤ 拒绝:被动端已发 REJ");
            chk_eq("⑤ 拒绝:被动只发 1 消息(REJ;未发 REP)", (uint32_t)cmb2.tx_n, 1);
            chk_eq("⑤ 拒绝:主动发 1 收 1(REQ + REJ)", (uint32_t)cma2.tx_n, 1);
            chk_eq("⑤ 拒绝:主动收 1", (uint32_t)cma2.rx_n, 1);
            chk_eq("⑤ 拒绝:支线零包上线(线计数不变)", o_ltx.read().to_uint(), ltx0);
        }

        // ================= 相位 ⑥:错误分支② 超时/重试(单跑无应答)=================
        {
            cm_pipe_t p3; cm_transport_t ta3, tb3; cm_entity_t cma3;
            cm_pipe_attach(&p3, &ta3, &tb3);
            cm_entity_init(&cma3, &ta3, QPN_A, 0x0BE001, 5, PKEY, gidA, 0x2Cu);
            cm_entity_set_peer(&cma3, gidB);
            for (int i = 0; i < 200; i++) { int r3 = cm_active_step(&cma3); if (r3 < 0) break; }
            chk(cma3.st == CM_ST_FAIL, "⑥ 超时:单跑无应答 ⇒ FAIL");
            chk_eq("⑥ 超时:发送计数 == 1 + retry_max(=6)", (uint32_t)cma3.tx_n, 6);
            chk_eq("⑥ 超时:一包未收", (uint32_t)cma3.rx_n, 0);
        }

        // ---- 计数收尾 ----
        chk_eq("⑦ line:已发请求包数 == 1", o_ltx.read().to_uint(), 1);
        chk_eq("⑦ line:已匹配 ACK 数 == 1", o_lack.read().to_uint(), 1);
        chk_eq("⑦ line:非预期帧 == 0", o_lun.read().to_uint(), 0);
        chk_eq("⑦ 门铃丢弃 / goerr == 0", o_dbd.read().to_uint() + o_goerr.read().to_uint(), 0);
        chk_eq("⑦ MAC 坏帧(fcs_err) == 0", o_mfcs.read().to_uint(), 0);

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

    CmM2Drv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n); drv.intr(stack.sig_intr);
    drv.dev = &stack.dev; drv.mem = mem; drv.peer = &peer;
    drv.o_cqe(stack.sig_cqe); drv.o_wqe(stack.sig_wqe); drv.o_dbd(stack.sig_dbd);
    drv.o_goerr(stack.sig_goerr);
    drv.o_lbig(stack.sig_lbig); drv.o_ltx(stack.sig_ltx);
    drv.o_lack(stack.sig_lack); drv.o_lun(stack.sig_lun);
    drv.o_dtx(stack.sig_mtx); drv.o_mfcs(stack.sig_mfcs);

    printf("============================================================\n");
    printf("tb_cm_m2 - ★ CM 建链 M2:priv 往返 + 三段(INIT→RTR→RTS)+ 拒绝/超时分支\n");
    printf("============================================================\n");

    sc_core::sc_start(60, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_CM_M2 %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
