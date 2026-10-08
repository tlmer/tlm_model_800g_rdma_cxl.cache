//============================================================================
// tb_mac_pcs_link.cpp — MAC TLM ⇄ PCS/FEC link TLM 对接(免 glue 直连)✓
// 规格 = 规格文档 §2(两 TLM 输入输出**恰好对偶** ⇒ 同信号直绑)✓
//
// 判据(预登记):
//   ① 干净环回:帧 A(120B)⇒ MAC → LINK → link(环回)→ MAC → roce ⇒ **逐字节一致**
//   ② ★ 误码负控:BER=1e-2 发帧 B ⇒ **不交付**(frames_rx 不增)且 **fcs_err ≥ 1**(注错必被拦)
//   ③ 回归卫士:BER 归零后 **陷阱字节帧 C**(0xFB/0xFD/0x07…)⇒ 逐字节一致
//   ④ 延迟旋钮:lat=3 后帧 D(=C 同长)仍逐字节一致,且到帧拍数 ≥ 帧 C 的 +2
//   ⑤ 门面位:align=1 · locked=0xFFFF;link o_beats_in/out ≥ 4
//============================================================================
#include <systemc.h>
#include "mac_tlm.h"
#include "pcs_link_tlm.h"
#include <vector>
#include <cstdio>

struct PcsDrv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    // mac NIC 侧激励/回程(同 M1 风格)
    sc_out<bool> S_tvalid; sc_out<sc_biguint<2048>> S_tdata; sc_out<sc_biguint<256>> S_tkeep; sc_out<bool> S_tlast;
    sc_in<bool>  S_tready;
    sc_in<bool> R_tvalid; sc_in<sc_biguint<2048>> R_tdata; sc_in<sc_biguint<256>> R_tkeep; sc_in<bool> R_tlast;
    // 观测
    sc_in<sc_uint<32>> m_frx, m_fcs, m_ftx;
    sc_in<sc_uint<32>> l_bin, l_bout, l_berr;
    sc_in<bool> l_align; sc_in<sc_uint<16>> l_locked;
    PcsLinkTlm* pcs = nullptr;

    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;
    SC_HAS_PROCESS(PcsDrv);
    PcsDrv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char* name) {
        printf("   v %-44s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }

    // 送一帧并等回收;cycles_out = 花掉的拍数;-1 超时。不比对内容(调用方判)
    int go_frame(const unsigned char* f, int n, std::vector<unsigned char>& got, int* cycles_out) {
        got.clear();
        sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
        for (int j = 0; j < n; j++) { d.range(j*8+7, j*8) = f[j]; k[j] = true; }
        S_tdata.write(d); S_tkeep.write(k); S_tvalid.write(true); S_tlast.write(true);
        wait(clk.posedge_event()); S_tvalid.write(false); S_tlast.write(false);
        int base = m_frx.read().to_uint();                        // ⚠ 信号读:上一拍值(判据前已 settle)✓
        for (int c = 0; c < 300; c++) {
            wait(clk.posedge_event());
            if (R_tvalid.read()) {
                sc_biguint<2048> rd = R_tdata.read(); sc_biguint<256> rk = R_tkeep.read();
                for (int j = 0; j < 256; j++)
                    if (rk[j].to_bool()) got.push_back((unsigned char)rd.range(j*8+7, j*8).to_uint());
                if (R_tlast.read()) { if (cycles_out) *cycles_out = c; return (int)got.size(); }
            }
            if (m_frx.read().to_uint() == base && m_fcs.read().to_uint() > 0 && c > 30) {
                // 供负控:等足够久后没收到 ⇒ 由调用方判
            }
        }
        if (cycles_out) *cycles_out = -1;
        return -1;
    }

    void run() {
        rst_n.write(false);
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        wait(clk.posedge_event()); wait(clk.posedge_event());

        chk(l_align.read(),            "⑤ link align_status = 1");
        chk(l_locked.read().to_uint() == 0xFFFF, "⑤ link lane_locked = 0xFFFF");

        // ---- ① 干净环回 ----
        unsigned char fa[120]; for (int i = 0; i < 120; i++) fa[i] = (unsigned char)((i*3+0x21) & 0xff);
        std::vector<unsigned char> got; int cycA = 0;
        int r1 = go_frame(fa, 120, got, &cycA);
        bool ok1 = (r1 == 120); for (int i = 0; i < 120 && ok1; i++) if (got[i] != fa[i]) ok1 = false;
        chk(ok1, "① 帧A: 经 PCS link 环回逐字节一致");

        // ---- ② 误码负控(BER 1e-2)----
        unsigned base_rx = m_frx.read().to_uint();
        unsigned base_fcs = m_fcs.read().to_uint();
        pcs->set_ber(1e-2);
        unsigned char fb[200]; for (int i = 0; i < 200; i++) fb[i] = (unsigned char)((i*5+0x11) & 0xff);
        int r2 = go_frame(fb, 200, got, nullptr);
        wait(clk.posedge_event()); wait(clk.posedge_event());      // settle
        chk(l_berr.read().to_uint() > 0, "② link 已注入误码(o_bit_errs > 0)");
        chk(r2 == -1 || (int)got.size() == 0, "② 坏帧未交付(负控:判据能红)");
        chk(m_frx.read().to_uint() == base_rx, "② 坏帧 frames_rx 不增");
        chk(m_fcs.read().to_uint() > base_fcs, "② 坏帧被 FCS 拦(fcs_err 增)");
        pcs->set_ber(0.0);

        // ---- ③ 陷阱字节帧(回归卫士)----
        unsigned char fc[200]; for (int i = 0; i < 200; i++) fc[i] = (unsigned char)((i*5+0x11) & 0xff);
        fc[7] = 0xFB; fc[80] = 0xFD; fc[3] = 0x07; fc[4] = 0x55; fc[5] = 0xD5; fc[98] = 0xFB;
        int cycC = 0;
        int r3 = go_frame(fc, 200, got, &cycC);
        bool ok3 = (r3 == 200); for (int i = 0; i < 200 && ok3; i++) if (got[i] != fc[i]) ok3 = false;
        chk(ok3, "③ 帧C(陷阱字节): 逐字节一致");

        // ---- ④ 延迟旋钮 ----
        pcs->set_latency(3);
        int cycD = 0;
        int r4 = go_frame(fc, 200, got, &cycD);
        bool ok4 = (r4 == 200); for (int i = 0; i < 200 && ok4; i++) if (got[i] != fc[i]) ok4 = false;
        chk(ok4, "④ lat=3: 帧D 仍逐字节一致");
        chk(cycD >= cycC + 2, "④ lat=3: 到帧拍数比 lat=0 增 ≥2");

        // ---- ⑤ link 列计数 ----
        chk(l_bin.read().to_uint() >= 4, "⑤ link o_beats_in ≥ 4");
        chk(l_bout.read().to_uint() >= 4, "⑤ link o_beats_out ≥ 4");

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    MacTlm     mac("mac");
    PcsLinkTlm pcs("pcs");

    // ---- mac NIC 侧(TB 激励)----
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

    // ---- ★ mac ⇄ pcs 线侧:**同信号直绑 = 免 glue**(两 TLM 输入输出恰好对偶)✓ ----
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
    // pcs 门面位输出(观测)
    sc_core::sc_signal<bool> l_align; sc_core::sc_signal<sc_uint<16>> l_locked;
    sc_core::sc_signal<sc_uint<32>> l_bin, l_bout, l_berr;
    sc_core::sc_signal<sc_uint<32>> m_fcs_lm;
    pcs.fec_rx_align_status(l_align); pcs.fec_rx_lane_locked(l_locked);
    pcs.o_beats_in(l_bin); pcs.o_beats_out(l_bout); pcs.o_bit_errs(l_berr);
    // mac 观测
    sc_core::sc_signal<sc_uint<32>> m_ftx, m_frx, m_idle, m_fcs;
    mac.o_frames_tx(m_ftx); mac.o_frames_rx(m_frx); mac.o_idle_beats(m_idle); mac.o_fcs_err(m_fcs);
    mac.o_lane_mismatch_cnt(m_fcs_lm);

    PcsDrv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n);
    drv.S_tvalid(S_tvalid); drv.S_tdata(S_tdata); drv.S_tkeep(S_tkeep); drv.S_tlast(S_tlast);
    drv.S_tready(S_tready);
    drv.R_tvalid(R_tvalid); drv.R_tdata(R_tdata); drv.R_tkeep(R_tkeep); drv.R_tlast(R_tlast);
    drv.m_frx(m_frx); drv.m_fcs(m_fcs); drv.m_ftx(m_ftx);
    drv.l_bin(l_bin); drv.l_bout(l_bout); drv.l_berr(l_berr);
    drv.l_align(l_align); drv.l_locked(l_locked);
    drv.pcs = &pcs;

    sc_core::sc_start(20, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_MAC_PCS_LINK %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
