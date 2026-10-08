//============================================================================
// tb_mac_lane800.cpp — MacTlm 线侧 lane 档位自测(rdma800 M2)✓
// 规格 = ../设计方案.md §2.1 + ../验证方案.md §2-L1;**判据预登记(⛔ 先写后跑)**✓
//
// 判据(预登记):
//   ① 800G 档环回B 图案帧 ⇒ 线侧往返回环 ⇒ roce AXIS **逐字节一致** + 长度 = 200
//   ② 800G 档双 lane 载荷:**START 落 lane0 首字节**(posn(0)=0xFB 且 ctrl=1);
//      发射期 **lane1 出现 ≥1 个非 idle 字节**(证明确用双 lane 拼接;200B 载荷 ⇒ 线帧 213B > 128B/拍)
//   ③ 800G 档契约(正 + 负控):发射全程 **dval_0 == dval_1 逐拍恒等**;
//      再令 `ena_0=1 ∧ ena_1=0` 驱动 1 拍(**负控**)⇒ `o_lane_mismatch_cnt ≥ 1`(计数不静默)
//   ④ 400G 档环回B 帧 ⇒ 逐字节一致(帧全程只走 lane0;线帧 133B ⇒ 3 拍)
//   ⑤ 400G 档 lane1 恒无效(负控两条):(a) 全程 dval_1 == 0 且 lane1 数据全 0;
//      (b) 在 **lane1 注入一整帧** ⇒ frames_rx 不增 ∧ fcs_err 不增(完全不入译码)
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
// 线帧:START + 6×PRE + SFD + payload(N) + FCS(4) + TERM(与 tb_mac_loopback 同族)
int build_frame(const unsigned char* pay, int N, unsigned char* out, bool* ctl) {
    int n = 0; out[n] = C_START; ctl[n++] = true;
    for (int i = 0; i < 6; i++) { out[n] = C_PRE; ctl[n++] = false; }
    out[n] = C_SFD; ctl[n++] = false;
    unsigned int crc = eth_crc32(pay, N);
    for (int i = 0; i < N; i++) { out[n] = pay[i]; ctl[n++] = false; }
    out[n] = (unsigned char)(crc & 0xff);         ctl[n++] = false;
    out[n] = (unsigned char)((crc >> 8) & 0xff);  ctl[n++] = false;
    out[n] = (unsigned char)((crc >> 16) & 0xff); ctl[n++] = false;
    out[n] = (unsigned char)((crc >> 24) & 0xff); ctl[n++] = false;
    out[n] = C_TERM; ctl[n++] = true;
    return n;
}
// 线拍字节读数(REV 映射;lane 选择)
inline unsigned char beat_byte(const sc_biguint<512>& d0, const sc_biguint<512>& d1, int i) {
    if (i < 64) { int p = posn(i);    return (unsigned char)d0.range(p*8+7, p*8).to_uint(); }
    else        { int p = posn(i-64); return (unsigned char)d1.range(p*8+7, p*8).to_uint(); }
}
inline bool beat_ctl(const sc_biguint<64>& c0, const sc_biguint<64>& c1, int i) {
    if (i < 64) { int p = posn(i);    return c0[p].to_bool(); }
    else        { int p = posn(i-64); return c1[p].to_bool(); }
}
} // namespace

//----------------------------------------------------------------------------
struct Drv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    // m800(800G 环回):AXIS 激励 + 回程 + 线侧观测 + 分 lane ena + 观测口
    sc_out<bool> S8_tvalid; sc_out<sc_biguint<2048>> S8_tdata; sc_out<sc_biguint<256>> S8_tkeep; sc_out<bool> S8_tlast;
    sc_in<bool>  S8_tready;
    sc_in<bool>  R8_tvalid; sc_in<sc_biguint<2048>> R8_tdata; sc_in<sc_biguint<256>> R8_tkeep; sc_in<bool> R8_tlast;
    sc_in<sc_biguint<512>> L8_txd_0, L8_txd_1; sc_in<sc_biguint<64>> L8_txc_0, L8_txc_1;
    sc_in<bool> L8_dval_0, L8_dval_1;
    sc_out<bool> L8_ena_tx0, L8_ena_tx1, L8_ena_rx;
    sc_in<sc_uint<32>> m8_ftx, m8_frx, m8_lanemm;
    // m400(400G 档):AXIS 激励 + 回程;lane1 注入口(Drv 驱)+ 观测
    sc_out<bool> S4_tvalid; sc_out<sc_biguint<2048>> S4_tdata; sc_out<sc_biguint<256>> S4_tkeep; sc_out<bool> S4_tlast;
    sc_in<bool>  S4_tready;
    sc_in<bool>  R4_tvalid; sc_in<sc_biguint<2048>> R4_tdata; sc_in<sc_biguint<256>> R4_tkeep; sc_in<bool> R4_tlast;
    sc_out<sc_biguint<512>> M4_inj_d1; sc_out<sc_biguint<64>> M4_inj_c1;    // ★ lane1 注入(400G 档应被忽略)
    sc_in<sc_biguint<512>> L4_txd_0, L4_txd_1; sc_in<sc_biguint<64>> L4_txc_0, L4_txc_1;
    sc_in<bool> L4_dval_0, L4_dval_1;
    sc_out<bool> L4_ena_tx, L4_ena_rx;
    sc_in<sc_uint<32>> m4_frx, m4_fcs;

    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;
    SC_HAS_PROCESS(Drv);
    Drv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char* name) {
        printf("   v %-48s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }
    void idle_inj() { M4_inj_d1.write(0); M4_inj_c1.write(0); }          // lane1 注入口置闲

    void run() {
        // ---- 复位(全 ena=1;lane1 注入口置闲)----
        rst_n.write(false);
        L8_ena_tx0.write(true); L8_ena_tx1.write(true); L8_ena_rx.write(true);
        L4_ena_tx.write(true);  L4_ena_rx.write(true);
        S8_tvalid.write(false); S8_tlast.write(false); S4_tvalid.write(false); S4_tlast.write(false);
        idle_inj();
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        wait(clk.posedge_event());

        //================ 相位 ①②③ m800G 环回 + 双 lane + 契约 ================
        const int N8 = 200;
        unsigned char pay8[N8]; for (int i = 0; i < N8; i++) pay8[i] = (unsigned char)((i*7+0x23) & 0xff);
        {
            sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
            for (int j = 0; j < N8; j++) { d.range(j*8+7, j*8) = pay8[j]; k[j] = true; }
            S8_tdata.write(d); S8_tkeep.write(k); S8_tvalid.write(true); S8_tlast.write(true);
        }
        wait(clk.posedge_event());
        S8_tvalid.write(false); S8_tlast.write(false);       // ★ 主侧义务:发完即撤 ✓

        std::vector<unsigned char> got8;
        bool start_on_lane0 = false, first_seen = false, dval_mismatch = false;
        unsigned int lane1_busy_bytes = 0;
        for (int cyc = 0; cyc < 800; cyc++) {
            wait(clk.posedge_event());
            // ③ 契约采样:全程 dval_0 == dval_1(逐拍)
            if (L8_dval_0.read() != L8_dval_1.read()) dval_mismatch = true;
            // ② 线侧观测:首非 idle 拍 ⇒ START 落 lane0 首字节;统计 lane1 非 idle 字节
            {
                sc_biguint<512> d0 = L8_txd_0.read(), d1 = L8_txd_1.read();
                sc_biguint<64>  c0 = L8_txc_0.read(), c1 = L8_txc_1.read();
                bool all_idle = true;
                for (int i = 0; i < 128 && all_idle; i++)
                    if (!(beat_byte(d0, d1, i) == C_IDLE && beat_ctl(c0, c1, i))) all_idle = false;
                if (!all_idle) {
                    if (!first_seen) {
                        first_seen = true;
                        start_on_lane0 = (beat_byte(d0, d1, 0) == C_START) && beat_ctl(c0, c1, 0);
                    }
                    for (int i = 64; i < 128; i++)
                        if (!(beat_byte(d0, d1, i) == C_IDLE && beat_ctl(c0, c1, i))) lane1_busy_bytes++;
                }
            }
            if (R8_tvalid.read()) {
                sc_biguint<2048> d = R8_tdata.read(); sc_biguint<256> k = R8_tkeep.read();
                for (int j = 0; j < 256; j++)
                    if (k[j].to_bool()) got8.push_back((unsigned char)d.range(j*8+7, j*8).to_uint());
                if (R8_tlast.read()) break;
            }
        }
        chk(got8.size() == (size_t)N8, "① 800G 环回: 长度 = 200");
        { bool ok = (got8.size() == (size_t)N8);
          for (int i = 0; i < N8 && ok; i++) if (got8[i] != pay8[i]) ok = false;
          chk(ok, "① 800G 环回: 逐字节一致"); }
        chk(start_on_lane0,             "② START 落 lane0 首字节(posn(0))");
        chk(lane1_busy_bytes > 0,       "② lane1 出现非 idle 字节(双 lane 真用)");
        chk(!dval_mismatch,             "③ 契约: dval_0 == dval_1 逐拍恒等");
        chk(m8_ftx.read().to_uint() == 1 && m8_frx.read().to_uint() == 1, "③ mac: frames_tx = frames_rx = 1");

        // ③ 负控:ena_1 单独掉 1 拍 ⇒ 契约违例计数
        unsigned int mm0 = m8_lanemm.read().to_uint();
        L8_ena_tx1.write(false);
        for (int i = 0; i < 3; i++) wait(clk.posedge_event());
        L8_ena_tx1.write(true);
        chk(m8_lanemm.read().to_uint() > mm0, "③ 负控: ena_1 不一致 ⇒ lane_mismatch_cnt 增");

        //================ 相位 ④⑤ m400G 档 ================
        const int N4 = 120;
        unsigned char pay4[N4]; for (int i = 0; i < N4; i++) pay4[i] = (unsigned char)((i*5+0x41) & 0xff);
        {
            sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
            for (int j = 0; j < N4; j++) { d.range(j*8+7, j*8) = pay4[j]; k[j] = true; }
            S4_tdata.write(d); S4_tkeep.write(k); S4_tvalid.write(true); S4_tlast.write(true);
        }
        wait(clk.posedge_event());
        S4_tvalid.write(false); S4_tlast.write(false);

        std::vector<unsigned char> got4;
        bool l4_dval1_any = false, l4_data1_any = false;
        for (int cyc = 0; cyc < 800; cyc++) {
            wait(clk.posedge_event());
            if (L4_dval_1.read()) l4_dval1_any = true;                   // ⑤a:dval_1 应恒 0
            if (L4_txd_1.read().to_uint64() != 0) l4_data1_any = true;   // ⑤a:lane1 数据应恒 0
            if (R4_tvalid.read()) {
                sc_biguint<2048> d = R4_tdata.read(); sc_biguint<256> k = R4_tkeep.read();
                for (int j = 0; j < 256; j++)
                    if (k[j].to_bool()) got4.push_back((unsigned char)d.range(j*8+7, j*8).to_uint());
                if (R4_tlast.read()) break;
            }
        }
        chk(got4.size() == (size_t)N4, "④ 400G 环回: 长度 = 120");
        { bool ok = (got4.size() == (size_t)N4);
          for (int i = 0; i < N4 && ok; i++) if (got4[i] != pay4[i]) ok = false;
          chk(ok, "④ 400G 环回: 逐字节一致(仅 lane0 通道)"); }
        chk(!l4_dval1_any,  "⑤a 400G: dval_1 全程恒 0");
        chk(!l4_data1_any,  "⑤a 400G: lane1 数据全程恒 0");
        chk(m4_frx.read().to_uint() == 1, "④ mac: frames_rx = 1");

        // ⑤b 负控:lane1 注入**一整帧** ⇒ 完全不入译码(帧数不增、fcs_err 不增)
        {
            unsigned char fr[256]; bool fc[256];
            int n = build_frame(pay4, N4, fr, fc);                       // 133B ⇒ 3 拍
            for (int b = 0; b < (n + 63) / 64; b++) {
                sc_biguint<512> d1 = 0; sc_biguint<64> c1 = 0;
                for (int i = 0; i < 64; i++) {
                    int idx = b * 64 + i; if (idx >= n) break;
                    int p = posn(i);
                    d1.range(p*8+7, p*8) = fr[idx];
                    if (fc[idx]) c1[p] = true;
                }
                M4_inj_d1.write(d1); M4_inj_c1.write(c1);
                wait(clk.posedge_event());
            }
            idle_inj();
            for (int i = 0; i < 8; i++) wait(clk.posedge_event());
        }
        chk(m4_frx.read().to_uint() == 1, "⑤b 负控: lane1 注入帧 ⇒ frames_rx 不增(=1)");
        chk(m4_fcs.read().to_uint() == 0, "⑤b 负控: lane1 注入帧 ⇒ fcs_err 不增(=0)");

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    MacTlm m8("m8"), m4("m4");
    m4.set_lane_mode(MacTlm::LM_400G);                                   // ★ 400G 档(仅 lane0)✓

    // ---- m8(800G 环回)----
    sc_core::sc_signal<bool> S8_tvalid, S8_tlast, S8_tready, R8_tvalid, R8_tlast;
    sc_core::sc_signal<sc_biguint<2048>> S8_tdata, R8_tdata;
    sc_core::sc_signal<sc_biguint<256>>  S8_tkeep, R8_tkeep;
    sc_core::sc_signal<sc_biguint<512>>  L8_txd_0, L8_txd_1;
    sc_core::sc_signal<sc_biguint<64>>   L8_txc_0, L8_txc_1;
    sc_core::sc_signal<bool> L8_dval_0, L8_dval_1, L8_ena_tx0, L8_ena_tx1, L8_ena_rx, R8_tuser;
    sc_core::sc_signal<sc_uint<32>> m8_ftx, m8_frx, m8_idle, m8_fcs, m8_lanemm;
    m8.clk(clk); m8.rst_n(rst_n);
    m8.cmac_m_axis_tvalid(S8_tvalid); m8.cmac_m_axis_tdata(S8_tdata);
    m8.cmac_m_axis_tkeep(S8_tkeep);   m8.cmac_m_axis_tlast(S8_tlast);
    m8.cmac_m_axis_tready(S8_tready);
    m8.roce_cmac_s_axis_tvalid(R8_tvalid); m8.roce_cmac_s_axis_tdata(R8_tdata);
    m8.roce_cmac_s_axis_tkeep(R8_tkeep);   m8.roce_cmac_s_axis_tlast(R8_tlast);
    m8.roce_cmac_s_axis_tuser(R8_tuser);
    m8.link_txdval_0(L8_dval_0); m8.link_txdval_1(L8_dval_1);
    m8.link_txd_0(L8_txd_0); m8.link_txd_1(L8_txd_1);
    m8.link_txc_0(L8_txc_0); m8.link_txc_1(L8_txc_1);
    m8.link_rxd_0(L8_txd_0); m8.link_rxd_1(L8_txd_1);                  // 双 lane 环回 ✓
    m8.link_rxc_0(L8_txc_0); m8.link_rxc_1(L8_txc_1);
    m8.link_txclk_ena_0(L8_ena_tx0); m8.link_txclk_ena_1(L8_ena_tx1);  // ★ 分 lane(负控用)✓
    m8.link_rxclk_ena_0(L8_ena_rx);  m8.link_rxclk_ena_1(L8_ena_rx);
    m8.o_frames_tx(m8_ftx); m8.o_frames_rx(m8_frx); m8.o_idle_beats(m8_idle);
    m8.o_fcs_err(m8_fcs); m8.o_lane_mismatch_cnt(m8_lanemm);

    // ---- m4(400G 档):lane0 环回;lane1 注入(Drv 驱)----
    sc_core::sc_signal<bool> S4_tvalid, S4_tlast, S4_tready, R4_tvalid, R4_tlast;
    sc_core::sc_signal<sc_biguint<2048>> S4_tdata, R4_tdata;
    sc_core::sc_signal<sc_biguint<256>>  S4_tkeep, R4_tkeep;
    sc_core::sc_signal<sc_biguint<512>>  L4_txd_0, L4_txd_1, M4_inj_d1;
    sc_core::sc_signal<sc_biguint<64>>   L4_txc_0, L4_txc_1, M4_inj_c1;
    sc_core::sc_signal<bool> L4_dval_0, L4_dval_1, L4_ena_tx, L4_ena_rx, R4_tuser;
    sc_core::sc_signal<sc_uint<32>> m4_ftx, m4_frx, m4_idle, m4_fcs, m4_lanemm;
    m4.clk(clk); m4.rst_n(rst_n);
    m4.cmac_m_axis_tvalid(S4_tvalid); m4.cmac_m_axis_tdata(S4_tdata);
    m4.cmac_m_axis_tkeep(S4_tkeep);   m4.cmac_m_axis_tlast(S4_tlast);
    m4.cmac_m_axis_tready(S4_tready);
    m4.roce_cmac_s_axis_tvalid(R4_tvalid); m4.roce_cmac_s_axis_tdata(R4_tdata);
    m4.roce_cmac_s_axis_tkeep(R4_tkeep);   m4.roce_cmac_s_axis_tlast(R4_tlast);
    m4.roce_cmac_s_axis_tuser(R4_tuser);
    m4.link_txdval_0(L4_dval_0); m4.link_txdval_1(L4_dval_1);
    m4.link_txd_0(L4_txd_0); m4.link_txd_1(L4_txd_1);
    m4.link_txc_0(L4_txc_0); m4.link_txc_1(L4_txc_1);
    m4.link_rxd_0(L4_txd_0); m4.link_rxc_0(L4_txc_0);                  // lane0 环回 ✓
    m4.link_rxd_1(M4_inj_d1); m4.link_rxc_1(M4_inj_c1);                // ★ lane1 = 注入(负控)✓
    m4.link_txclk_ena_0(L4_ena_tx); m4.link_txclk_ena_1(L4_ena_tx);
    m4.link_rxclk_ena_0(L4_ena_rx); m4.link_rxclk_ena_1(L4_ena_rx);
    m4.o_frames_tx(m4_ftx); m4.o_frames_rx(m4_frx); m4.o_idle_beats(m4_idle);
    m4.o_fcs_err(m4_fcs); m4.o_lane_mismatch_cnt(m4_lanemm);

    // ---- Drv ----
    Drv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n);
    drv.S8_tvalid(S8_tvalid); drv.S8_tdata(S8_tdata); drv.S8_tkeep(S8_tkeep); drv.S8_tlast(S8_tlast);
    drv.S8_tready(S8_tready);
    drv.R8_tvalid(R8_tvalid); drv.R8_tdata(R8_tdata); drv.R8_tkeep(R8_tkeep); drv.R8_tlast(R8_tlast);
    drv.L8_txd_0(L8_txd_0); drv.L8_txd_1(L8_txd_1); drv.L8_txc_0(L8_txc_0); drv.L8_txc_1(L8_txc_1);
    drv.L8_dval_0(L8_dval_0); drv.L8_dval_1(L8_dval_1);
    drv.L8_ena_tx0(L8_ena_tx0); drv.L8_ena_tx1(L8_ena_tx1); drv.L8_ena_rx(L8_ena_rx);
    drv.m8_ftx(m8_ftx); drv.m8_frx(m8_frx); drv.m8_lanemm(m8_lanemm);
    drv.S4_tvalid(S4_tvalid); drv.S4_tdata(S4_tdata); drv.S4_tkeep(S4_tkeep); drv.S4_tlast(S4_tlast);
    drv.S4_tready(S4_tready);
    drv.R4_tvalid(R4_tvalid); drv.R4_tdata(R4_tdata); drv.R4_tkeep(R4_tkeep); drv.R4_tlast(R4_tlast);
    drv.M4_inj_d1(M4_inj_d1); drv.M4_inj_c1(M4_inj_c1);
    drv.L4_txd_0(L4_txd_0); drv.L4_txd_1(L4_txd_1); drv.L4_txc_0(L4_txc_0); drv.L4_txc_1(L4_txc_1);
    drv.L4_dval_0(L4_dval_0); drv.L4_dval_1(L4_dval_1);
    drv.L4_ena_tx(L4_ena_tx); drv.L4_ena_rx(L4_ena_rx);
    drv.m4_frx(m4_frx); drv.m4_fcs(m4_fcs);

    sc_core::sc_start(30, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("TB_MAC_LANE800 %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
