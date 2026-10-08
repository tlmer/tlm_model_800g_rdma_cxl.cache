//============================================================================
// tb_mac_pcs_lane.cpp — MAC(Lane 双档)⇄ PCS/FEC link 对接台  [2026-10-09 建 / rdma800 M3]
//
// 覆盖 = 验证方案 §2-L1「tb_mac_pcs_link(平移 + 800G 档)」+ §2-L4-2「速率档节流语义」✓
// 接线 = 同 tb_mac_pcs_link(**免 glue 直连**:mac ⇄ pcs 线侧输入输出恰好对偶)✓
//
// 判据(预登记):
//   ① 800G 档(LM_800G):帧 A(260B,跨拍)经 PCS link 环回**逐字节一致**;
//      且 **lane1 真承过载**(采样:T_1 出现过非 idle 字)✓
//   ② 400G 档(LM_400G,帧边界切换):同长帧 B 环回**逐字节一致**(仅 lane0);
//      负控:传帧期间 **T_1 恒 0 ∧ TC_1 恒 0 ∧ T_dval_1 恒 false** ✓
//   ③ 速率档(比值判据):同帧长下 **cyc800 < cyc400**(800G 128B/拍 > 400G 64B/拍)✓
//   ④ 误码负控(400G 档下):BER=1e-2 ⇒ 帧 C **不交付** ∧ `fcs_err` 增 ∧ `o_bit_errs>0` ✓
//   ⑤ 门面位:align=1 · lane_locked=0xFFFF ✓
//============================================================================
#include <systemc.h>
#include "mac_tlm.h"
#include "pcs_link_tlm.h"
#include <vector>
#include <cstdio>

struct LaneDrv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    // mac NIC 侧激励/回程(同 tb_mac_pcs_link)
    sc_out<bool> S_tvalid; sc_out<sc_biguint<2048>> S_tdata; sc_out<sc_biguint<256>> S_tkeep; sc_out<bool> S_tlast;
    sc_in<bool>  S_tready;
    sc_in<bool>  R_tvalid; sc_in<sc_biguint<2048>> R_tdata; sc_in<sc_biguint<256>> R_tkeep; sc_in<bool> R_tlast;
    // line 侧观测(lane1:数据/控制/dval;绑同一批信号)
    sc_in<sc_biguint<512>> T_1; sc_in<sc_biguint<64>> TC_1; sc_in<bool> T_dval_1;
    // 观测口
    sc_in<sc_uint<32>> m_frx, m_fcs;
    sc_in<sc_uint<32>> l_berr;
    sc_in<bool> l_align; sc_in<sc_uint<16>> l_locked;
    PcsLinkTlm* pcs = nullptr;
    MacTlm*     mac = nullptr;

    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;
    SC_HAS_PROCESS(LaneDrv);
    LaneDrv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char* name) {
        printf("   v %-52s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }
    static sc_biguint<512> all07() {                 // 全 0x07 字(idle 判据;位置置换下不变)✓
        sc_biguint<512> w = 0;
        for (int j = 0; j < 64; j++) w.range(j*8+7, j*8) = 0x07;
        return w;
    }
    bool lane1_seen_nonidle = false;                 // ① 见证(800G)
    bool neg_on = false, lane1_bad = false;          // ② 负控(400G 传帧期)

    // 送帧并等回收;期间逐拍采样 lane1。返回负载长(-1 超时);got/cyc 出参
    int go_frame(const unsigned char* f, int n, std::vector<unsigned char>& got, int* cycles_out) {
        got.clear();
        // ★ 多拍流式(2048b=256B/拍;每拍换新值,MAC 下一拍吃)✓
        int off = 0;
        while (off < n) {
            sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
            int nb = (n - off > 256) ? 256 : (n - off);
            for (int j = 0; j < nb; j++) { d.range(j*8+7, j*8) = f[off + j]; k[j] = true; }
            S_tdata.write(d); S_tkeep.write(k);
            S_tvalid.write(true); S_tlast.write(off + nb >= n);
            wait(clk.posedge_event());
            off += nb;
        }
        S_tvalid.write(false); S_tlast.write(false);
        for (int c = 0; c < 400; c++) {
            sc_biguint<512> w1 = T_1.read(); sc_biguint<64> c1 = TC_1.read(); bool dv1 = T_dval_1.read();
            if (dv1 && w1 != all07()) lane1_seen_nonidle = true;              // 800G 见证
            if (neg_on && (w1 != 0 || c1 != 0 || dv1)) lane1_bad = true;      // 400G 负控
            wait(clk.posedge_event());
            if (R_tvalid.read()) {
                sc_biguint<2048> rd = R_tdata.read(); sc_biguint<256> rk = R_tkeep.read();
                for (int j = 0; j < 256; j++)
                    if (rk[j].to_bool()) got.push_back((unsigned char)rd.range(j*8+7, j*8).to_uint());
                if (R_tlast.read()) { if (cycles_out) *cycles_out = c; return (int)got.size(); }
            }
        }
        if (cycles_out) *cycles_out = -1;
        return -1;
    }
    static bool eq(const std::vector<unsigned char>& a, const unsigned char* b, int n) {
        if ((int)a.size() != n) return false;
        for (int i = 0; i < n; i++) if (a[i] != b[i]) return false;
        return true;
    }

    void run() {
        rst_n.write(false);
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        wait(clk.posedge_event());
        printf("== TB_MAC_PCS_LANE(双档G/400G)==\n");

        const int N = 260;
        unsigned char A[N]; for (int i = 0; i < N; i++) A[i] = (unsigned char)((i*7+0x11) & 0xff);
        A[10] = 0xFB; A[11] = 0xFD; A[12] = 0x07; A[13] = 0x55; A[14] = 0xD5;  // 陷阱字节(数据位)✓
        std::vector<unsigned char> got; int cyc800 = -1, cyc400 = -1;

        // ① 800G 档(显式)
        mac->set_lane_mode(MacTlm::LM_800G);
        neg_on = false; lane1_seen_nonidle = false;
        int r1 = go_frame(A, N, got, &cyc800);
        chk(r1 == N && eq(got, A, N),  "① 800G 档B 经 PCS link 环回逐字节一致");
        chk(lane1_seen_nonidle,        "① 800G 档:lane1 真承过载(出现过非 idle 字)");

        // ② 400G 档(帧边界切换;仅 lane0)
        mac->set_lane_mode(MacTlm::LM_400G);
        lane1_bad = false; neg_on = true;
        int r2 = go_frame(A, N, got, &cyc400);
        neg_on = false;
        chk(r2 == N && eq(got, A, N),  "② 400G 档:同长帧环回逐字节一致(仅 lane0)");
        chk(!lane1_bad,                "② 负控G 档传帧期 T_1/TC_1/dval_1 恒 0");

        // ③ 速率档比值
        chk(cyc800 > 0 && cyc400 > 0 && cyc800 < cyc400,
            "③ 速率档:同帧长 cyc800 < cyc400(128B/拍 > 64B/拍)");

        // ④ 误码负控(400G 档下)
        pcs->set_ber(1e-2);
        unsigned int base_rx = m_frx.read().to_uint(), base_fcs = m_fcs.read().to_uint();
        int r3 = go_frame(A, N, got, 0);
        chk(l_berr.read().to_uint() > 0,           "④ link 已注入误码(o_bit_errs > 0)");
        chk(r3 == -1 || got.size() == 0,           "④ 坏帧未交付(负控能红)");
        chk(m_frx.read().to_uint() == base_rx,     "④ 坏帧 frames_rx 不增");
        chk(m_fcs.read().to_uint() > base_fcs,     "④ 坏帧被 FCS 拦(fcs_err 增)");
        pcs->set_ber(0.0);

        // ⑤ 门面位
        chk(l_align.read(),                        "⑤ align_status = 1");
        chk(l_locked.read().to_uint() == 0xFFFF,   "⑤ lane_locked = 0xFFFF");

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d(cyc800=%d cyc400=%d)\n", pass_cnt, fail_cnt, cyc800, cyc400);
        sc_core::sc_stop();
    }
};

int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;
    MacTlm     mac("mac");
    PcsLinkTlm pcs("pcs");

    sc_core::sc_signal<bool> S_tvalid, S_tlast, S_tready, R_tvalid, R_tlast, R_tuser;
    sc_core::sc_signal<sc_biguint<2048>> S_tdata, R_tdata;
    sc_core::sc_signal<sc_biguint<256>>  S_tkeep, R_tkeep;
    mac.clk(clk); mac.rst_n(rst_n); pcs.clk(clk); pcs.rst_n(rst_n);
    mac.cmac_m_axis_tvalid(S_tvalid); mac.cmac_m_axis_tdata(S_tdata);
    mac.cmac_m_axis_tkeep(S_tkeep);   mac.cmac_m_axis_tlast(S_tlast);
    mac.cmac_m_axis_tready(S_tready);
    mac.roce_cmac_s_axis_tvalid(R_tvalid); mac.roce_cmac_s_axis_tdata(R_tdata);
    mac.roce_cmac_s_axis_tkeep(R_tkeep);   mac.roce_cmac_s_axis_tlast(R_tlast);
    mac.roce_cmac_s_axis_tuser(R_tuser);

    // ★ mac ⇄ pcs 免 glue 直连(同 tb_mac_pcs_link)
    sc_core::sc_signal<sc_biguint<512>> T_0, T_1, R_0, R_1;
    sc_core::sc_signal<sc_biguint<64>>  TC_0, TC_1, RC_0, RC_1;
    sc_core::sc_signal<bool> T_dval_0, T_dval_1, E_tx_0, E_tx_1, E_rx_0, E_rx_1;
    mac.link_txd_0(T_0);   mac.link_txd_1(T_1);     pcs.link_txd_0(T_0);   pcs.link_txd_1(T_1);
    mac.link_txc_0(TC_0);  mac.link_txc_1(TC_1);    pcs.link_txc_0(TC_0);  pcs.link_txc_1(TC_1);
    mac.link_txdval_0(T_dval_0); mac.link_txdval_1(T_dval_1);
    pcs.link_txdval_0(T_dval_0); pcs.link_txdval_1(T_dval_1);
    mac.link_rxd_0(R_0);   mac.link_rxd_1(R_1);     pcs.link_rxd_0(R_0);   pcs.link_rxd_1(R_1);
    mac.link_rxc_0(RC_0);  mac.link_rxc_1(RC_1);    pcs.link_rxc_0(RC_0);  pcs.link_rxc_1(RC_1);
    mac.link_txclk_ena_0(E_tx_0); mac.link_txclk_ena_1(E_tx_1);
    pcs.link_txclk_ena_0(E_tx_0); pcs.link_txclk_ena_1(E_tx_1);
    mac.link_rxclk_ena_0(E_rx_0); mac.link_rxclk_ena_1(E_rx_1);
    pcs.link_rxclk_ena_0(E_rx_0); pcs.link_rxclk_ena_1(E_rx_1);

    sc_core::sc_signal<bool> l_align; sc_core::sc_signal<sc_uint<16>> l_locked;
    sc_core::sc_signal<sc_uint<32>> l_bin, l_bout, l_berr, m_fcs_lm;
    pcs.fec_rx_align_status(l_align); pcs.fec_rx_lane_locked(l_locked);
    pcs.o_beats_in(l_bin); pcs.o_beats_out(l_bout); pcs.o_bit_errs(l_berr);
    sc_core::sc_signal<sc_uint<32>> m_ftx, m_frx, m_idle, m_fcs;
    mac.o_frames_tx(m_ftx); mac.o_frames_rx(m_frx); mac.o_idle_beats(m_idle); mac.o_fcs_err(m_fcs);
    mac.o_lane_mismatch_cnt(m_fcs_lm);

    LaneDrv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n);
    drv.S_tvalid(S_tvalid); drv.S_tdata(S_tdata); drv.S_tkeep(S_tkeep); drv.S_tlast(S_tlast);
    drv.S_tready(S_tready);
    drv.R_tvalid(R_tvalid); drv.R_tdata(R_tdata); drv.R_tkeep(R_tkeep); drv.R_tlast(R_tlast);
    drv.T_1(T_1); drv.TC_1(TC_1); drv.T_dval_1(T_dval_1);
    drv.m_frx(m_frx); drv.m_fcs(m_fcs); drv.l_berr(l_berr);
    drv.l_align(l_align); drv.l_locked(l_locked);
    drv.pcs = &pcs; drv.mac = &mac;

    sc_core::sc_start(20, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_MAC_PCS_LANE %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
