//============================================================================
// tb_roce_engine_m3.cpp — ★★ **丢包 ⇒ 超时重发(同包/原 PSN)+ 2.1j WQE retried 回写
//   + RNR_RETRY 参数化(耗尽 ⇒ 异常终结)**  [2026-10-05 建]
//
// 判据(预登记):
//   A. 丢包(对端静默丢首个请求)⇒ 设备**超时重发**:
//     ① line tx == 2(同包发两遍)/ 对端 req_cnt == 1(丢的不计数)/ **重发包逐字节 == 原包**
//     ② **QP_SQ_PSN 只推进一次**(PSN0+1 —— 重发复用原 PSN)✓
//     ③ ★ **2.1j 回写**:WQE 行 **byte2 bit0 == 1**(retried;buf_id 位恒 0 明标占位)✓
//     ④ cqe == 1 / 中断抬起 / line_retries == 1 / 重试耗尽 = 0 / goerr == 0 ✓
//   B. **RNR_RETRY 参数化负控**(对端 RNR ×9;寄存器 RNR_RETRY = 2):
//     ⑤ 设备重发 2 次后**异常终结**:tx 增量 == 3、cqe **不增**、重试耗尽计数 +1、goerr +1 ✓
//
// ⚠ 机制照开卷:`QP_TIMEOUT(0x4C)` 字段 [4:0]TMO/[10:8]RETRY/[13:11]RNR_RETRY/[20:16]RNR_TVAL
//   (`sw/nic_regs.h:72` + RTL 逐位对上);**阈值单位 = 模型拍**(明标旋钮
//   `set_line_timeout`;RTL 计时器本版未开卷 —— 只有字段/组合 + `//Partial RD_REQ_RETRY` 半成品)✓
//============================================================================

#include <systemc.h>
#include <cstring>
#include <amba_pv.h>
#include <models/amba_pv_ace_protocol_checker.h>
#include "stack_top.h"
#include "dummy_amba_master.h"   // ★ 占位主端(reg_s 必绑)✓
#include "mac_tlm.h"
#include "roce_peer_tlm.h"
#include "nic_regs.h"
#include <cstdio>

//----------------------------------------------------------------------------
// 主机内存(ACE 从端;照 tb_dual_stack)✓
//----------------------------------------------------------------------------
struct AceHostMem: sc_core::sc_module, amba_pv::amba_pv_ace_slave_base {
    amba_pv::amba_pv_ace_slave_socket<512> ace_s;
    unsigned char mem[65536 * cxlshim::LINE_BYTES];      // 4MB ✓
    uint32_t rd_n = 0, wr_n = 0, bad = 0;

    AceHostMem(sc_core::sc_module_name nm): amba_pv_ace_slave_base("host_mem"), ace_s("ace_s") {
        ace_s(*this);
        for (unsigned i = 0; i < sizeof(mem); i++) mem[i] = 0;
    }
    void b_transport(int, amba_pv::amba_pv_transaction & tr, sc_core::sc_time & t) override {
        amba_pv::amba_pv_extension * ex = NULL; tr.get_extension(ex);
        size_t off = (size_t)((tr.get_address() >> 6) & 0xFFFF) * cxlshim::LINE_BYTES;
        unsigned char * d = tr.get_data_ptr();
        if (tr.is_read()) {
            rd_n++;
            for (int i = 0; i < cxlshim::LINE_BYTES; i++) d[i] = mem[off + i];
        } else if (tr.is_write()) {
            wr_n++;
            for (int i = 0; i < cxlshim::LINE_BYTES; i++) mem[off + i] = d[i];
        } else bad++;
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
struct EngM1Drv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    sc_in<bool> intr;
    sc_in<sc_uint<32>> o_cqe, o_wqe, o_dbd, o_goerr, o_lbig, o_ltx, o_lack, o_lun, o_dtx, o_mfcs;
    DeviceTlm  * dev = nullptr;
    AceHostMem * mem = nullptr;
    RocePeerTlm* peer = nullptr;
    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(EngM1Drv);
    EngM1Drv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char * name) {
        printf("   v %-54s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }
    void chk_eq(const char * name, uint32_t got, uint32_t exp) {
        printf("   v %-44s got=0x%08x exp=0x%08x\n", name, got, exp);
        if (got != exp) { fail_cnt++; printf("   ✗ %s 不符\n", name); } else pass_cnt++;
    }
    void tick() { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); }
    void rw(sc_dt::uint64 a, uint32_t v) {
        if (!dev->reg_write(a, sc_uint<32>(v))) { printf("   x 写 0x%llx 未实现\n", (unsigned long long)a); fail_cnt++; }
    }
    uint32_t rdr(sc_dt::uint64 a) {
        sc_uint<32> v = 0;
        if (!dev->reg_read(a, v)) { printf("   x 读 0x%llx 未实现\n", (unsigned long long)a); fail_cnt++; }
        return v.to_uint();
    }
    // 改已装 WQE 的 opcode 字节(装 WQE 后调;wqe() 默认写 WRITE)✓
    void mem_set_opc(AceHostMem * m, sc_dt::uint64 SQ, uint32_t pi, uint8_t opc) {
        unsigned char * wq = m->line_ptr(SQ + (sc_dt::uint64)pi * 64);
        wq[NIC_WQE_OPC_OFF] = opc;
    }
    // 装一个 WQE(SQ 第 pi 项;1-based)✓
    void wqe(AceHostMem * m, sc_dt::uint64 SQ, uint32_t pi, uint32_t wrid, uint32_t len,
             uint32_t loff, sc_dt::uint64 roff, uint8_t qpid, uint32_t rkey) {
        unsigned char * wq = m->line_ptr(SQ + (sc_dt::uint64)pi * 64);
        for (int i = 0; i < 64; i++) wq[i] = 0;
        wq[NIC_WQE_WRID_OFF + 0] = (unsigned char)(wrid & 0xff);
        wq[NIC_WQE_WRID_OFF + 1] = (unsigned char)((wrid >> 8) & 0xff);
        *(uint32_t *)(wq + NIC_WQE_LEN_OFF)  = len;
        wq[NIC_WQE_OPC_OFF] = NIC_OPC_WRITE;
        *(uint32_t *)(wq + NIC_WQE_LOFF_OFF) = loff;
        *(uint32_t *)(wq + NIC_WQE_ROFF_OFF) = (uint32_t)roff;
        wq[NIC_WQE_QID_OFF] = qpid;                       // ★ 进包 DestQP(byte17)✓
        *(uint32_t *)(wq + NIC_WQE_RKEY_OFF) = rkey;      // ★ 进包 RETH.rkey ✓
    }

    void run() {
        rst_n.write(false);
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        tick();

        const sc_dt::uint64 SQ = 0x00020000, CQ = 0x00030000;
        const sc_dt::uint64 SRC_S = 0x00040000, ROFF = 0x00090000;
        const uint32_t PSN0 = 0x0b77cf, RK = 0x98, DQP = 7;

        // ---- 配置:QP 面 + 中断 + QP_SQ_PSN + **QP_TIMEOUT(0x4C)** + 线侧身份/模式 ----
        rw(0x50180200 + 0x00, 0x00000001);                    // QP_CONF ✓
        rw(0x50180200 + 0x10, (uint32_t)SQ);
        rw(0x50180200 + 0x18, (uint32_t)CQ);
        rw(0x50180200 + 0x3C, 8);                             // Q_DEPTH ✓
        rw(0x50180200 + 0x40, PSN0);                          // QP_SQ_PSN ✓
        rw(0x50180200 + 0x4C, (0x02u << 16) | (2u << 11) | (3u << 8) | 0u);   // RNR_TVAL=2/RNR_RETRY=2/RETRY=3/TMO=0 ✓
        rw(0x50180200 + 0x48, (uint32_t)DQP);                  // ★ D16:DEST_QP_CONF(0x48)= 包 DestQP 真源 ✓
        rw(0x50180200 + 0x04, 0x6f66u << 16);        // ★ D16:QP_ADV_CONF[31:16] = P_Key(寄存器形态)✓
        rw(0x50100000, 0x6851u << 8);                // ★ D16:NIC_CONF[23:8] = UDP src port ✓
        rw(0x50180200 + 0x50, 0x001c1eeau);          // ★ D16:MAC_REMOTE_LSB(= DA 字节低 24:ea,1e,1c)✓
        rw(0x50180200 + 0x54, 0x0000edb8u);          // ★ D16:MAC_REMOTE_MSB(= DA 字节高 24:69,b8,ed)✓
        rw(0x50180200 + 0x60, 0x0200000au);          // ★ D16:IP_REMOTE1(字节低先:0a,00,00,02 ⇒ 值反序)✓
        rw(0x50100010, 0xdc17762fu);                 // ★ D16:MAC_NIC_LSB(= SA 字节低 32:2f,76,17,dc)✓
        rw(0x50100014, 0x00009a5eu);                 // ★ D16:MAC_NIC_MSB(= SA 字节高 16:5e,9a)✓
        rw(0x50100070, 0x0a000001u);                 // ★ D16:IPV4_NIC_ADDR(= 源 IP;字节低先)✓

        rw(0x50100180, (1u << 4));                            // INTR_EN bit4 ✓
        {
            DeviceTlm::LineId id;                             // 照 基准 配方 ✓
            static const unsigned char SA[6] = {0x2f,0x76,0x17,0xdc,0x5e,0x9a};
            static const unsigned char DA[6] = {0xea,0x1e,0x1c,0x69,0xb8,0xed};
            for (int i = 0; i < 6; i++) { id.sa[i] = SA[i]; id.da[i] = DA[i]; }
            id.sip = 0x0a000001; id.dip = 0x0a000002; id.sport = 0x6851; id.dport = 0x12b7;
            dev->set_line_id(id);
            dev->set_line_pkt_mode(true);
        }
        dev->set_line_timeout(200);                           // ★ M3:超时阈值 200 拍(明标旋钮)✓
        { unsigned char * s2 = mem->line_ptr(SRC_S);          // 源图案 ✓
          for (int i = 0; i < 64; i++) s2[i] = (unsigned char)((i * 7 + 0x31) & 0xff); }

        // ================= A. 丢包 ⇒ 超时重发 + 2.1j 回写 =================
        peer->drop_times = 1;                                 // 对端静默丢首个请求 ⇒ 设备必超时 ✗
        wqe(mem, SQ, 1, 0x3101, 64, (uint32_t)SRC_S, ROFF, (uint8_t)DQP, RK);
        rw(0x50180200 + 0x38, 1);                             // 门铃(pi=1)✓
        for (int i = 0; i < 6000 && o_cqe.read().to_uint() < 1; i++) tick();
        for (int i = 0; i < 40; i++) tick();                  // 落定(含回写槽)✓
        chk_eq("A① line tx(=2:同包两遍)", o_ltx.read().to_uint(), 2);
        chk_eq("A① 对端 req_cnt(=1:丢的不计数)", peer->req_cnt, 1);
        {
            bool eq = (peer->rnr_rx_len > 0) && (peer->rnr_rx_len == peer->last_rx_len);
            for (int i = 0; i < peer->rnr_rx_len && eq; i++)
                if (peer->rnr_rx_buf[i] != peer->rx_buf[i]) eq = false;
            chk(eq, "A① ★ 重发包逐字节 == 原包");
        }
        chk_eq("A② QP_SQ_PSN(只推进一次 ⇒ PSN0+1)", rdr(0x50180200 + 0x40), PSN0 + 1);
        {
            unsigned char * wq = mem->line_ptr(SQ + 64);      // ★ 2.1j:回写见证(直接读宿内存)✓
            chk_eq("A③ WQE byte2 bit0(retried)=1", (wq[2] >> 0) & 1, 1);
            chk_eq("A③ WQE byte2 其余位(buf_id)=0", (wq[2] >> 1) & 0x7F, 0);
        }
        chk_eq("A④ cqe_cnt", o_cqe.read().to_uint(), 1);
        chk_eq("A④ 中断线抬起", intr.read() ? 1u : 0u, 1);
        chk_eq("A④ line_retries(=1)", dev->line_retries, 1);
        chk_eq("A④ 重试耗尽 / goerr(均 0)", dev->line_rtry_exh + o_goerr.read().to_uint(), 0);

        // ================= B. RNR_RETRY 参数化负控(耗尽 ⇒ 异常终结)=================
        peer->rnr_times = 9;                                  // RNR ×9 > RNR_RETRY(2) ⇒ 必耗尽 ✗
        uint32_t tx0 = o_ltx.read().to_uint(), ex0 = dev->line_rtry_exh, ge0 = o_goerr.read().to_uint();
        wqe(mem, SQ, 2, 0x3102, 64, (uint32_t)SRC_S, 0x000A0000, (uint8_t)DQP, RK);
        rw(0x50180200 + 0x38, 2);                             // 门铃(pi=2)✓
        for (int i = 0; i < 4000 && dev->line_rtry_exh <= ex0; i++) tick();
        for (int i = 0; i < 40; i++) tick();
        chk_eq("B⑤ tx 增量(=3:首发 + RNR_RETRY 2 次)", o_ltx.read().to_uint() - tx0, 3);
        chk_eq("B⑤ cqe 不增(仍 1)", o_cqe.read().to_uint(), 1);
        chk_eq("B⑤ 重试耗尽计数 +1", dev->line_rtry_exh, ex0 + 1);
        chk_eq("B⑤ goerr +1(异常终结)", o_goerr.read().to_uint(), ge0 + 1);
        printf("   [内存] rd=%u wr=%u bad=%u\n", mem->rd_n, mem->wr_n, mem->bad);

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    StackTop stack("stack");                  // 设备+桥+MAC_A(内部接线全封装)✓
    DummyAmbaMaster dum("dum");               // ★ 占位主端(reg_s 必绑;E109 socket 版)✓
    stack.dev.reg_s.bind(dum.m);
    stack.clk(clk); stack.rst_n(rst_n);

    MacTlm mac_b("mac_b");                    // 对端侧 MAC(= 客户 MAC 的 TLM 替身)✓
    mac_b.clk(clk); mac_b.rst_n(rst_n);

    RocePeerTlm peer("peer");                 // 第三方 NIC 替身 ✓
    peer.clk(clk); peer.rst_n(rst_n);

    // ---- ACE 侧:桥 → 检查器 → 主机内存(⚠ 4MB 必须**堆分配**,照双栈 TB 踩坑)✓
    AceHostMem * mem = new AceHostMem("mem");
    amba_pv::amba_pv_ace_protocol_checker<512> chk("chk");
    stack.br.ace_m.bind(chk.amba_pv_s); chk.amba_pv_m.bind(mem->ace_s);

    // ---- 线侧:MAC_A ⇄ MAC_B(LINK 交叉 + 「PCS 角色」ena 恒 1)✓
    sc_core::sc_signal<sc_biguint<512>> Wa0, Wa1, Wb0, Wb1;
    sc_core::sc_signal<sc_biguint<64>>  TCa0, TCa1, TCb0, TCb1;
    sc_core::sc_signal<bool> Tda0, Tda1, Tdb0, Tdb1;          // dval(只到线)✓
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
    mac_b.link_rxd_0(Wa0); mac_b.link_rxd_1(Wa1);           // A 发 → B 收 ✓
    mac_b.link_rxc_0(TCa0); mac_b.link_rxc_1(TCa1);
    stack.mac.link_rxd_0(Wb0); stack.mac.link_rxd_1(Wb1);   // B 发 → A 收 ✓
    stack.mac.link_rxc_0(TCb0); stack.mac.link_rxc_1(TCb1);

    // ---- 对端 ⇄ MAC_B(NIC 侧 2048b AXIS;照 tb_roce_peer_m1 B 侧)✓
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
    sc_core::sc_signal<sc_uint<32>> p_req, p_ack, p_bad;      // ⚠ 观测口必绑(E109)✓
    sc_core::sc_signal<sc_uint<32>> b_fcs_lm;
    peer.o_req_cnt(p_req); peer.o_ack_sent(p_ack); peer.o_bad_cnt(p_bad);
    sc_core::sc_signal<sc_uint<32>> b_ftx, b_frx, b_idle, b_fcs;   // mac_b 观测口必绑(E109)✓
    mac_b.o_frames_tx(b_ftx); mac_b.o_frames_rx(b_frx);
    mac_b.o_idle_beats(b_idle); mac_b.o_fcs_err(b_fcs);
    mac_b.o_lane_mismatch_cnt(b_fcs_lm);

    // ---- 驱动 ----
    EngM1Drv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n); drv.intr(stack.sig_intr);
    drv.dev = &stack.dev; drv.mem = mem; drv.peer = &peer;
    drv.o_cqe(stack.sig_cqe); drv.o_wqe(stack.sig_wqe); drv.o_dbd(stack.sig_dbd);
    drv.o_goerr(stack.sig_goerr);
    drv.o_lbig(stack.sig_lbig); drv.o_ltx(stack.sig_ltx);
    drv.o_lack(stack.sig_lack); drv.o_lun(stack.sig_lun);
    drv.o_dtx(stack.sig_mtx); drv.o_mfcs(stack.sig_mfcs);

    printf("============================================================\n");
    printf("tb_roce_engine_m3 - ★★ 超时重发 + 2.1j 回写 + RNR_RETRY 参数化\n");
    printf("============================================================\n");

    sc_core::sc_start(60, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_ROCE_ENGINE_M3 %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
