//============================================================================
// tb_mac_loopback.cpp — MacTlm 自测(M1):线侧环回 + 形状判据 + FCS 正/负控 ✓
// 规格 = 规格文档 §7.5(判据**预登记**;⛔ 先写后跑)✓
// 判据:
//   ① 环回B 图案帧 ⇒ 线侧往返回环 ⇒ roce AXIS 逐字节一致 + 长度=100
//   ② 形状:首个非 idle 拍 lane0 字节(posn(0))= START 0xFB 且 ctrl=1;流中见 TERM 0xFD+ctrl;
//           mac o_idle_beats > 0(帧间有空闲)
//   ③ 正控(mac2):自造**好帧**直驱其 RX ⇒ frames_rx=1、fcs_err=0
//   ④ 负控(mac2):同帧**翻一载荷字节** ⇒ fcs_err=1 且 frames_rx **不增**(判据能红)✓
// ⚠ 字节序:REV 序(与 mac_tlm.cpp 默认同档;切档时两处必须同切)✗
//============================================================================
#include <systemc.h>
#include "mac_tlm.h"
#include <vector>
#include <cstdio>

namespace {
const unsigned char C_IDLE = 0x07, C_START = 0xFB, C_TERM = 0xFD, C_SFD = 0xD5, C_PRE = 0x55;
inline int posn(int i) { return (i/8)*8 + (7-(i%8)); }          // REV(vendor 原生;同 mac_tlm.cpp)
unsigned int eth_crc32(const unsigned char* d, int n) {
    unsigned int c = 0xFFFFFFFFu;
    for (int i = 0; i < n; i++) { c ^= (unsigned int)d[i]; for (int k = 0; k < 8; k++) c = (c>>1) ^ (0xEDB88320u & (0u - (c & 1u))); }
    return c ^ 0xFFFFFFFFu;
}
} // namespace

//----------------------------------------------------------------------------
// Drv — 激励/观测/判据(SC_THREAD;照现 TB 惯例:先驱动后 wait(posedge))
//----------------------------------------------------------------------------
struct Drv: sc_core::sc_module {
    sc_in<bool> clk;
    sc_out<bool> rst_n;
    // 到 mac(环回实例)的 AXIS 激励(Drv 驱动)/ 握手回读
    sc_out<bool> S_tvalid; sc_out<sc_biguint<2048>> S_tdata; sc_out<sc_biguint<256>> S_tkeep; sc_out<bool> S_tlast;
    sc_in<bool>  S_tready;
    // 自 mac 的 AXIS 回程(Drv 读)
    sc_in<bool> R_tvalid; sc_in<sc_biguint<2048>> R_tdata; sc_in<sc_biguint<256>> R_tkeep; sc_in<bool> R_tlast;
    // 环回线观测(Drv 读)
    sc_in<sc_biguint<512>> L_txd_0, L_txd_1; sc_in<sc_biguint<64>> L_txc_0, L_txc_1;
    // 环回使能(Drv 驱动;恒 1)
    sc_out<bool> L_ena_tx, L_ena_rx;
    // mac 观测口(Drv 读)
    sc_in<sc_uint<32>> m1_frames_tx, m1_frames_rx, m1_idle_beats;
    // mac2(直驱受害)RX 激励(Drv 驱动)+ 观测
    sc_out<sc_biguint<512>> M2_rxd_0, M2_rxd_1; sc_out<sc_biguint<64>> M2_rxc_0, M2_rxc_1;
    sc_out<bool> M2_ena;
    sc_in<sc_uint<32>> m2_frames_rx, m2_fcs_err;

    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(Drv);
    Drv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char* name) {
        printf("   v %-46s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }

    // 造 LINK 一拍 字节流 → 双 lane + ctrl(REV 映射)
    void put_beat(const unsigned char b[128], const bool c[128]) {
        sc_biguint<512> d0 = 0, d1 = 0; sc_biguint<64> c0 = 0, c1 = 0;
        for (int i = 0; i < 128; i++) {
            if (i < 64) { int p = posn(i);    d0.range(p*8+7, p*8) = b[i]; if (c[i]) c0[p] = true; }
            else        { int p = posn(i-64); d1.range(p*8+7, p*8) = b[i]; if (c[i]) c1[p] = true; }
        }
        M2_rxd_0.write(d0); M2_rxd_1.write(d1); M2_rxc_0.write(c0); M2_rxc_1.write(c1);
    }
    void idle_beat() { unsigned char b[128]; bool c[128]; for (int i = 0; i < 128; i++) { b[i] = C_IDLE; c[i] = true; } put_beat(b, c); }

    // 造整帧(START+6PRE+SFD+100B payload+FCS+TERM);bad ⇒ 先算 FCS 再翻载荷一位(必失配)
    int build_frame(unsigned char out[128], bool ctl[128], bool bad) {
        int n = 0; out[n] = C_START; ctl[n++] = true;
        for (int i = 0; i < 6; i++) { out[n] = C_PRE; ctl[n++] = false; }
        out[n] = C_SFD; ctl[n++] = false;
        unsigned char pay[100];
        for (int i = 0; i < 100; i++) pay[i] = (unsigned char)((i*7+3) & 0xff);
        unsigned int crc = eth_crc32(pay, 100);
        if (bad) pay[50] ^= 0x01;                                // 载荷翻一位(CRC 按原值算 ⇒ 必败)
        for (int i = 0; i < 100; i++) { out[n] = pay[i]; ctl[n++] = false; }
        out[n] = (unsigned char)(crc & 0xff);         ctl[n++] = false;
        out[n] = (unsigned char)((crc >> 8) & 0xff);  ctl[n++] = false;
        out[n] = (unsigned char)((crc >> 16) & 0xff); ctl[n++] = false;
        out[n] = (unsigned char)((crc >> 24) & 0xff); ctl[n++] = false;
        out[n] = C_TERM; ctl[n++] = true;
        return n;
    }

    void run() {
        // ---- 复位 ----
        rst_n.write(false); L_ena_tx.write(true); L_ena_rx.write(true); M2_ena.write(false);
        S_tvalid.write(false); S_tlast.write(false);
        idle_beat();
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        wait(clk.posedge_event());

        //================ 相位 ① 环回 + 相位 ② 形状观测 ================
        const int N = 100;
        unsigned char fr[N]; for (int i = 0; i < N; i++) fr[i] = (unsigned char)((i*5+0x11) & 0xff);
        {   sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
            for (int j = 0; j < N; j++) { d.range(j*8+7, j*8) = fr[j]; k[j] = true; }
            S_tdata.write(d); S_tkeep.write(k); S_tvalid.write(true); S_tlast.write(true);
        }
        wait(clk.posedge_event());                    // ★ 本拍被 mac 收下
        S_tvalid.write(false); S_tlast.write(false);  // ★ 主侧义务:发完即撤(⚠ 不撤=每拍重吃同一拍 ✗✓)
        std::vector<unsigned char> got;
        bool shape_start = false, shape_term = false, first_seen = false;
        for (int cyc = 0; cyc < 400; cyc++) {
            wait(clk.posedge_event());
            if (R_tvalid.read()) {                                   // AXIS 回程采样
                sc_biguint<2048> d = R_tdata.read(); sc_biguint<256> k = R_tkeep.read();
                for (int j = 0; j < 256; j++)
                    if (k[j].to_bool()) got.push_back((unsigned char)d.range(j*8+7, j*8).to_uint());
                if (R_tlast.read()) { S_tvalid.write(false); break; }
            }
            {   sc_biguint<512> d0 = L_txd_0.read(), d1 = L_txd_1.read();
                sc_biguint<64>  c0 = L_txc_0.read(), c1 = L_txc_1.read();
                bool all_idle = true;
                for (int i = 0; i < 128 && all_idle; i++) {
                    unsigned char b; bool c;
                    if (i < 64) { int p = posn(i);    b = (unsigned char)d0.range(p*8+7, p*8).to_uint(); c = c0[p].to_bool(); }
                    else        { int p = posn(i-64); b = (unsigned char)d1.range(p*8+7, p*8).to_uint(); c = c1[p].to_bool(); }
                    if (!(b == C_IDLE && c)) all_idle = false;
                }
                if (!all_idle) {
                    if (!first_seen) {
                        first_seen = true; int p = posn(0);
                        shape_start = ((unsigned char)d0.range(p*8+7, p*8).to_uint() == C_START) && c0[p].to_bool();
                    }
                    for (int i = 0; i < 128; i++) {
                        unsigned char b; bool c;
                        if (i < 64) { int p = posn(i);    b = (unsigned char)d0.range(p*8+7, p*8).to_uint(); c = c0[p].to_bool(); }
                        else        { int p = posn(i-64); b = (unsigned char)d1.range(p*8+7, p*8).to_uint(); c = c1[p].to_bool(); }
                        if (b == C_TERM && c) shape_term = true;
                    }
                }
            }
        }
        chk(got.size() == (size_t)N, "① 环回: 长度 = 100");
        bool byte_ok = (got.size() == (size_t)N);
        for (int i = 0; i < N && byte_ok; i++) if (got[i] != fr[i]) byte_ok = false;
        chk(byte_ok,               "① 环回: 逐字节一致");
        chk(shape_start,           "② 形状: 首拍 START 0xFB+ctrl");
        chk(shape_term,            "② 形状: 流中见 TERM 0xFD+ctrl");
        chk(m1_frames_tx.read().to_uint() == 1, "② mac: frames_tx = 1");
        chk(m1_frames_rx.read().to_uint() == 1, "② mac: frames_rx = 1");
        chk(m1_idle_beats.read().to_uint() > 0, "② mac: idle_beats > 0");

        //================ 相位 ③④ mac2:正控 + 负控 ================
        unsigned char b[128]; bool c[128];
        for (int i = 0; i < 128; i++) { b[i] = C_IDLE; c[i] = true; }
        // ③ 好帧
        build_frame(b, c, false);
        put_beat(b, c); M2_ena.write(true);
        for (int i = 0; i < 8; i++) { wait(clk.posedge_event()); idle_beat(); }
        chk(m2_frames_rx.read().to_uint() == 1, "③ 正控: mac2 好帧 ⇒ frames_rx = 1");
        chk(m2_fcs_err.read().to_uint() == 0,   "③ 正控: mac2 fcs_err = 0");
        // ④ 负控(同帧翻一位)
        for (int i = 0; i < 128; i++) { b[i] = C_IDLE; c[i] = true; }
        build_frame(b, c, true);
        put_beat(b, c);
        for (int i = 0; i < 8; i++) { wait(clk.posedge_event()); idle_beat(); }
        chk(m2_fcs_err.read().to_uint() == 1,   "④ 负控: mac2 坏帧 ⇒ fcs_err = 1");
        chk(m2_frames_rx.read().to_uint() == 1, "④ 负控: mac2 frames_rx 不增(=1)");
        M2_ena.write(false); idle_beat();

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);           // 500MHz(行为级)
    sc_core::sc_signal<bool> rst_n;

    MacTlm mac("mac"), mac2("mac2");

    // ---- mac(环回实例)绑定 ----
    sc_core::sc_signal<bool> S_tvalid, S_tlast, S_tready, R_tvalid, R_tlast, L_ena_tx, L_ena_rx;
    sc_core::sc_signal<sc_biguint<2048>> S_tdata, R_tdata;
    sc_core::sc_signal<sc_biguint<256>>  S_tkeep, R_tkeep;
    sc_core::sc_signal<sc_biguint<512>>  L_txd_0, L_txd_1;
    sc_core::sc_signal<sc_biguint<64>>   L_txc_0, L_txc_1;
    sc_core::sc_signal<sc_uint<32>> m1_ftx, m1_frx, m1_idle, m1_fcs;
    sc_core::sc_signal<sc_uint<32>> m1_fcs_lm;
    sc_core::sc_signal<sc_uint<32>> m2_fcs_lm;
    sc_core::sc_signal<bool> L_dval_0, L_dval_1, R_tuser;

    mac.clk(clk); mac.rst_n(rst_n);
    mac.cmac_m_axis_tvalid(S_tvalid); mac.cmac_m_axis_tdata(S_tdata);
    mac.cmac_m_axis_tkeep(S_tkeep);   mac.cmac_m_axis_tlast(S_tlast);
    mac.cmac_m_axis_tready(S_tready);
    mac.roce_cmac_s_axis_tvalid(R_tvalid); mac.roce_cmac_s_axis_tdata(R_tdata);
    mac.roce_cmac_s_axis_tkeep(R_tkeep);   mac.roce_cmac_s_axis_tlast(R_tlast);
    mac.roce_cmac_s_axis_tuser(R_tuser);
    // ★ 环回:TX 引脚直接回接 RX 引脚(同一 sc_signal;sc_signal 写=delta 后生效 ⇒ 恰 1 拍)✓
    mac.link_txdval_0(L_dval_0); mac.link_txdval_1(L_dval_1);   // 线侧 TX valid(环回侧只观测)
    mac.link_txd_0(L_txd_0); mac.link_txd_1(L_txd_1);
    mac.link_txc_0(L_txc_0); mac.link_txc_1(L_txc_1);
    mac.link_rxd_0(L_txd_0); mac.link_rxd_1(L_txd_1);
    mac.link_rxc_0(L_txc_0); mac.link_rxc_1(L_txc_1);
    mac.link_txclk_ena_0(L_ena_tx); mac.link_txclk_ena_1(L_ena_tx);
    mac.link_rxclk_ena_0(L_ena_rx); mac.link_rxclk_ena_1(L_ena_rx);
    mac.o_frames_tx(m1_ftx); mac.o_frames_rx(m1_frx); mac.o_idle_beats(m1_idle); mac.o_fcs_err(m1_fcs);
    mac.o_lane_mismatch_cnt(m1_fcs_lm);

    // ---- mac2(直驱受害)绑定:输入由 Drv 驱;输出绑定哑信号(⚠ 不绑空=write 崩)----
    sc_core::sc_signal<sc_biguint<512>> M2_rxd_0, M2_rxd_1;
    sc_core::sc_signal<sc_biguint<64>>  M2_rxc_0, M2_rxc_1;
    sc_core::sc_signal<bool> M2_ena;
    // ⚠ 每个 sc_out 一个独立哑信号(E115 实踩:多 out 绑同一 signal = 多驱动 ✗✓)
    sc_core::sc_signal<sc_uint<32>> m2_frx, m2_fcs, dm32_a, dm32_b;
    sc_core::sc_signal<bool> dmB_in, dmB_1, dmB_2, dmB_3, dmB_4, dmB_5, dmB_6;
    sc_core::sc_signal<sc_biguint<2048>> dm2048_in, dm2048_o;
    sc_core::sc_signal<sc_biguint<256>>  dm256_in, dm256_o;
    sc_core::sc_signal<sc_biguint<512>>  dm512_a, dm512_b;
    sc_core::sc_signal<sc_biguint<64>>   dm64_a, dm64_b;
    mac2.clk(clk); mac2.rst_n(rst_n);
    mac2.cmac_m_axis_tvalid(dmB_in); mac2.cmac_m_axis_tdata(dm2048_in);
    mac2.cmac_m_axis_tkeep(dm256_in); mac2.cmac_m_axis_tlast(dmB_in);
    mac2.cmac_m_axis_tready(dmB_1);
    mac2.roce_cmac_s_axis_tvalid(dmB_2); mac2.roce_cmac_s_axis_tdata(dm2048_o);
    mac2.roce_cmac_s_axis_tkeep(dm256_o); mac2.roce_cmac_s_axis_tlast(dmB_3); mac2.roce_cmac_s_axis_tuser(dmB_4);
    mac2.link_txd_0(dm512_a); mac2.link_txd_1(dm512_b);
    mac2.link_txc_0(dm64_a);  mac2.link_txc_1(dm64_b);
    mac2.link_txdval_0(dmB_5); mac2.link_txdval_1(dmB_6);
    mac2.link_rxd_0(M2_rxd_0); mac2.link_rxd_1(M2_rxd_1);
    mac2.link_rxc_0(M2_rxc_0); mac2.link_rxc_1(M2_rxc_1);
    mac2.link_txclk_ena_0(L_ena_tx); mac2.link_txclk_ena_1(L_ena_tx);
    mac2.link_rxclk_ena_0(M2_ena);   mac2.link_rxclk_ena_1(M2_ena);
    mac2.o_frames_tx(dm32_a); mac2.o_frames_rx(m2_frx); mac2.o_idle_beats(dm32_b); mac2.o_fcs_err(m2_fcs);
    mac2.o_lane_mismatch_cnt(m2_fcs_lm);

    // ---- Drv ----
    Drv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n);
    drv.S_tvalid(S_tvalid); drv.S_tdata(S_tdata); drv.S_tkeep(S_tkeep); drv.S_tlast(S_tlast);
    drv.S_tready(S_tready);
    drv.R_tvalid(R_tvalid); drv.R_tdata(R_tdata); drv.R_tkeep(R_tkeep); drv.R_tlast(R_tlast);
    drv.L_txd_0(L_txd_0); drv.L_txd_1(L_txd_1); drv.L_txc_0(L_txc_0); drv.L_txc_1(L_txc_1);
    drv.L_ena_tx(L_ena_tx); drv.L_ena_rx(L_ena_rx);
    drv.m1_frames_tx(m1_ftx); drv.m1_frames_rx(m1_frx); drv.m1_idle_beats(m1_idle);
    drv.M2_rxd_0(M2_rxd_0); drv.M2_rxd_1(M2_rxd_1); drv.M2_rxc_0(M2_rxc_0); drv.M2_rxc_1(M2_rxc_1);
    drv.M2_ena(M2_ena);
    drv.m2_frames_rx(m2_frx); drv.m2_fcs_err(m2_fcs);

    sc_core::sc_start(20, sc_core::SC_US);                        // Drv 完成即 sc_stop ✓
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_MAC_LOOPBACK %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
