//============================================================================
// tb_roce_peer_m1.cpp — ★ 最小 RoCEv2 对端(真包)自测  [2026-10-05 建]
//   拓扑(用现成装配 top):[TB] ⇄NIC⇄ [MAC_A] ══LINK 交叉══ [MAC_B] ⇄NIC⇄ [RocePeerTlm]
//
// 判据(预登记;M1 = 单笔 WRITE 请求 ⇒ 收 ACK):
//   ① 包库自检:请求**头部字段逐条 == 基准 配方常量**(opcode 0x0a/P_Key 0x666f/ver 0x45/
//      UDP dport 4791/IP total == len-14);**ICRC 残差 == 0xc704dd7b**(照 NIC 判据)✓
//   ② 请求经 MAC_A→线→MAC_B ⇒ **对端收到**(req_cnt=1;解析 PSN/QP == 发出值)⇒ 回 ACK
//   ③ ACK 回程 ⇒ TB 收到;**逐字节 == 期望**(库构造;DestQP/PSN 回抄)✓
//   ④ 计数:ack=1 · bad=0
//   ⑤ 负控:发一帧非 WRITE(opcode=0x00)⇒ req 不增、bad+1 ✓
//============================================================================
#include <systemc.h>
#include "dual_mac_top.h"
#include "roce_peer_tlm.h"
#include <vector>
#include <cstdio>

//----------------------------------------------------------------------------
struct M1Drv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    // A 侧 NIC(TB ⇄ mac_a)
    sc_out<bool> A_tv; sc_out<sc_biguint<2048>> A_td; sc_out<sc_biguint<256>> A_tk; sc_out<bool> A_tl;
    sc_in<bool>  A_tr;
    sc_in<bool>  A_rv; sc_in<sc_biguint<2048>> A_rd; sc_in<sc_biguint<256>> A_rk; sc_in<bool> A_rl;
    // 对端观测
    sc_in<sc_uint<32>> p_req, p_ack, p_bad;
    RocePeerTlm * peer = nullptr;

    int pass_cnt = 0, fail_cnt = 0; bool all_pass = false;
    SC_HAS_PROCESS(M1Drv);
    M1Drv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }
    void chk(bool ok, const char * name) {
        printf("   v %-52s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }
    void tick() { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); }

    // 发一帧(AXIS 单拍)✓
    void send(const unsigned char * f, int n) {
        sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
        for (int j = 0; j < n; j++) { d.range(j*8+7, j*8) = f[j]; k[j] = true; }
        A_td.write(d); A_tk.write(k); A_tv.write(true); A_tl.write(true);
        wait(clk.posedge_event()); A_tv.write(false); A_tl.write(false);
    }
    // 轮询收一帧(-1 超时)✓
    int recv(std::vector<unsigned char> & got, int budget) {
        got.clear();
        for (int c = 0; c < budget; c++) {
            wait(clk.posedge_event());
            if (A_rv.read()) {
                sc_biguint<2048> dd = A_rd.read(); sc_biguint<256> kk = A_rk.read();
                for (int j = 0; j < 256; j++)
                    if (kk[j].to_bool()) got.push_back((unsigned char)dd.range(j*8+7, j*8).to_uint());
                if (A_rl.read()) return (int)got.size();
            }
        }
        return -1;
    }

    void run() {
        // 身份值(照 基准 配方口径;TB 侧值,不绑具体客户)✓
        const unsigned char sa[6] = { 0x2f, 0x76, 0x17, 0xdc, 0x5e, 0x9a };   // 本机(设备侧)
        const unsigned char da[6] = { 0xea, 0x1e, 0x1c, 0x69, 0xb8, 0xed };   // 对端
        const uint32_t ip_src = 0x0a000001, ip_dst = 0x0a000002;
        const uint32_t qp = 3, psn = 0x0b77cf;
        const uint64_t va = 0xcad53074ULL; const uint32_t rkey = 0x98;
        unsigned char pay[64]; for (int i = 0; i < 64; i++) pay[i] = (unsigned char)(0xa0 + i);

        unsigned char req[RocePeerTlm::PKT_MAX];
        int rl = RocePeerTlm::build_write_request(req, da, sa, ip_src, ip_dst,
                                                  0x6851, 0x12b7, RocePeerTlm::OPC_WRITE_ONLY,
                                                  qp, psn, true, va, rkey, pay, 64);

        // ---- ① 包库自检(字段/ICRC 照 基准 配方口径)----
        chk(rl == 138, "① 请求总长 = 138(134 内容 + 4 ICRC)");
        chk(req[42] == 0x0a && req[43] == 0x30, "① BTH opcode=0x0a flags=0x30(配方)");
        chk(req[44] == 0x66 && req[45] == 0x6f && req[46] == 0x05, "① P_Key=0x666f reserved=0x05(配方)");
        chk(req[14] == 0x45 && req[23] == 0x11, "① IPv4 ver/IHL=0x45 proto=UDP(配方)");
        chk(req[36] == 0x12 && req[37] == 0xb7, "① UDP dport = 4791(配方)");
        {
            int ip_total = (req[16] << 8) | req[17];
            int udp_len  = (req[38] << 8) | req[39];
            chk(ip_total == rl - 14 && udp_len == rl - 34, "① IP/UDP 长度 = 总长-14/-34(配方)");
        }
        {
            uint32_t c = 0xFFFFFFFFu;                       // ★ 残差判据(照 NIC `o_crc_err`)
            for (int k = 0; k < rl; k++) c = RocePeerTlm::crc_icrc_step(c, req[k]);
            chk(c == 0xc704dd7b, "① ICRC 残差 == 0xc704dd7b(NIC 判据)");
        }

        // ---- ② 请求经 MAC_A→线→MAC_B ⇒ 对端收到 ⇒ 回 ACK ----
        rst_n.write(false);
        for (int i = 0; i < 4; i++) tick();
        rst_n.write(true);
        tick(); tick();

        send(req, rl);
        for (int i = 0; i < 800 && p_req.read().to_uint() < 1; i++) tick();
        chk(p_req.read().to_uint() == 1, "② 对端收到 WRITE 请求(req_cnt=1)");
        {                                                   // 对端解析出的 PSN/QP == 发出值 ✓
            unsigned char opc = 0; uint32_t dq = 0, dp = 0;
            bool ok = RocePeerTlm::parse_bth(req, rl, opc, dq, dp);
            chk(ok && dq == qp && dp == psn, "② 对端解析 DestQP/PSN == 发出值");
        }
        for (int i = 0; i < 800 && p_ack.read().to_uint() < 1; i++) tick();
        chk(p_ack.read().to_uint() == 1, "② 对端已回 ACK(ack_sent=1)");

        // ---- ③ ACK 回程:TB 收到 ⇒ 逐字节 == 期望(库构造)----
        std::vector<unsigned char> got;
        int gl = recv(got, 800);
        unsigned char exp_ack[64];
        int el = RocePeerTlm::build_ack(exp_ack, req, rl);
        bool eq = (gl == el);
        for (int i = 0; i < el && eq; i++) if (got[i] != exp_ack[i]) eq = false;
        chk(eq, "③ ACK 逐字节 == 期望(62B;DestQP/PSN 回抄)");
        if (!eq && gl > 0) {
            printf("   [对比] 收 %d 期望 %d;首 16:", gl, el);
            for (int i = 0; i < 16 && i < gl; i++) printf(" %02x", got[i]);
            printf("\n");
        }

        // ---- ④ 计数 ----
        tick(); tick();
        chk(p_ack.read().to_uint() == 1 && p_bad.read().to_uint() == 0, "④ 计数:ack=1 bad=0");

        // ---- ⑤ 负控:非 WRITE 包 ⇒ req 不增、bad+1 ----
        unsigned char junk[138];
        for (int i = 0; i < rl; i++) junk[i] = req[i];
        junk[42] = 0x00;                                     // opcode 改成非法值
        unsigned rq0 = p_req.read().to_uint(), bd0 = p_bad.read().to_uint();
        send(junk, rl);
        for (int i = 0; i < 800 && p_bad.read().to_uint() <= bd0; i++) tick();
        chk(p_req.read().to_uint() == rq0 && p_bad.read().to_uint() == bd0 + 1,
            "⑤ 负控:非 WRITE ⇒ req 不增、bad+1");

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    DualMacTop top("top");                       // ★ 复用双实例 top(MAC_A ⇄ 交叉线 ⇄ MAC_B)✓
    top.clk(clk); top.rst_n(rst_n);

    RocePeerTlm peer("peer");
    peer.clk(clk); peer.rst_n(rst_n);

    // ---- A 侧(TB ⇄ mac_a)----
    sc_core::sc_signal<bool> A_tv, A_tl, A_tr, A_rv, A_rl, A_tu;
    sc_core::sc_signal<sc_biguint<2048>> A_td, A_rd;
    sc_core::sc_signal<sc_biguint<256>>  A_tk, A_rk;
    top.mac_a.cmac_m_axis_tvalid(A_tv); top.mac_a.cmac_m_axis_tdata(A_td);
    top.mac_a.cmac_m_axis_tkeep(A_tk);  top.mac_a.cmac_m_axis_tlast(A_tl);
    top.mac_a.cmac_m_axis_tready(A_tr);
    top.mac_a.roce_cmac_s_axis_tvalid(A_rv); top.mac_a.roce_cmac_s_axis_tdata(A_rd);
    top.mac_a.roce_cmac_s_axis_tkeep(A_rk);  top.mac_a.roce_cmac_s_axis_tlast(A_rl);
    top.mac_a.roce_cmac_s_axis_tuser(A_tu);
    // ---- B 侧(对端 ⇄ mac_b;⚠ 交叉:mac_b 的 roce 出 → 对端收;对端出 → mac_b 的 cmac 入)----
    sc_core::sc_signal<bool> B_tv, B_tl, B_tr, B_rv, B_rl, B_tu;
    sc_core::sc_signal<sc_biguint<2048>> B_td, B_rd;
    sc_core::sc_signal<sc_biguint<256>>  B_tk, B_rk;
    top.mac_b.cmac_m_axis_tvalid(B_tv); top.mac_b.cmac_m_axis_tdata(B_td);
    top.mac_b.cmac_m_axis_tkeep(B_tk);  top.mac_b.cmac_m_axis_tlast(B_tl);
    top.mac_b.cmac_m_axis_tready(B_tr);
    top.mac_b.roce_cmac_s_axis_tvalid(B_rv); top.mac_b.roce_cmac_s_axis_tdata(B_rd);
    top.mac_b.roce_cmac_s_axis_tkeep(B_rk);  top.mac_b.roce_cmac_s_axis_tlast(B_rl);
    top.mac_b.roce_cmac_s_axis_tuser(B_tu);
    peer.cmac_m_axis_tvalid(B_rv); peer.cmac_m_axis_tdata(B_rd);
    peer.cmac_m_axis_tkeep(B_rk);  peer.cmac_m_axis_tlast(B_rl);
    peer.cmac_m_axis_tready(B_tr);
    peer.roce_cmac_s_axis_tvalid(B_tv); peer.roce_cmac_s_axis_tdata(B_td);
    peer.roce_cmac_s_axis_tkeep(B_tk);  peer.roce_cmac_s_axis_tlast(B_tl);
    peer.roce_cmac_s_axis_tuser(B_tu);   // ⚠ 单驱动:mac_b 出、对端收 ✓

    sc_core::sc_signal<sc_uint<32>> p_req, p_ack, p_bad;
    sc_core::sc_signal<sc_uint<32>> a_fcs_lm;
    sc_core::sc_signal<sc_uint<32>> b_fcs_lm;
    peer.o_req_cnt(p_req); peer.o_ack_sent(p_ack); peer.o_bad_cnt(p_bad);
    // 两实例观测口(必绑;E109 纪律)✓
    sc_core::sc_signal<sc_uint<32>> a_ftx, a_frx, a_idle, a_fcs, b_ftx, b_frx, b_idle, b_fcs;
    top.mac_a.o_frames_tx(a_ftx); top.mac_a.o_frames_rx(a_frx);
    top.mac_a.o_idle_beats(a_idle); top.mac_a.o_fcs_err(a_fcs);
    top.mac_a.o_lane_mismatch_cnt(a_fcs_lm);
    top.mac_b.o_frames_tx(b_ftx); top.mac_b.o_frames_rx(b_frx);
    top.mac_b.o_idle_beats(b_idle); top.mac_b.o_fcs_err(b_fcs);
    top.mac_b.o_lane_mismatch_cnt(b_fcs_lm);

    M1Drv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n);
    drv.A_tv(A_tv); drv.A_td(A_td); drv.A_tk(A_tk); drv.A_tl(A_tl); drv.A_tr(A_tr);
    drv.A_rv(A_rv); drv.A_rd(A_rd); drv.A_rk(A_rk); drv.A_rl(A_rl);
    drv.p_req(p_req); drv.p_ack(p_ack); drv.p_bad(p_bad);
    drv.peer = &peer;

    printf("============================================================\n");
    printf("tb_roce_peer_m1 - ★ 最小 RoCEv2 对端(WRITE 请求 -> ACK)\n");
    printf("============================================================\n");

    sc_core::sc_start(50, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_ROCE_PEER_M1 %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
