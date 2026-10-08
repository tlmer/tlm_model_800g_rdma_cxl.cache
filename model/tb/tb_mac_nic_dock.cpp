//============================================================================
// tb_mac_nic_dock.cpp — NIC(设备 TLM)↔ MAC TLM 对接(帧级透传)✓
// 规格 = 规格文档 §7.4(⚠ M2 边界:设备侧**不做组帧**,帧内容 = TB 注入什么发什么)✗
//
// 链路:TB ──钩子── [DeviceTlm] ──cmac_m_axis(2048b AXIS)── [MacTlm]
//         [MacTlm] ──roce_cmac_s_axis── [DeviceTlm] ──钩子── TB(取回比对)
//         [MacTlm 线侧 LINK] ──TB 环回 glue(照 §3:txd/txc→rxd/rxc;ena 恒 1)── 回接
//
// 判据(预登记):
//   ① 帧 A(120B)⇒ **逐字节一致** + 长度 = 120
//   ② 帧 B(**陷阱字节帧**:含 0xFB/0xFD/0x07/0x55/0xD5)⇒ 逐字节一致
//      (★ 回归卫士:M1 的"控制位按位置"坑若回潮,本帧必红)✓
//   ③ 计数一致性:dev mac_tx_frames=2 · dev mac_rx_frames=2 · mac o_frames_tx=2 · o_frames_rx=2 ·
//      o_fcs_err=0 · 丢弃计数全 0 ✓
//============================================================================
#include <systemc.h>
#include "device_tlm.h"
#include "dummy_amba_master.h"   // ★ 占位主端(reg_s 必绑)✓
#include "mac_tlm.h"
#include <cstdio>

//----------------------------------------------------------------------------
struct DockDrv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    sc_in<sc_uint<32>> d_mtx, d_mrx, m_ftx, m_frx, m_fcs;
    sc_out<bool> L_ena_tx, L_ena_rx;   // 线侧 ena(照 §3 glue 角色;恒 1)
    DeviceTlm* d = nullptr;                       // 钩子宿主(构造后设置)
    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(DockDrv);
    DockDrv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char* name) {
        printf("   v %-44s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }

    // 走一帧:注入 ⇒ 轮询回收 ⇒ 比对。返回长度(-1 = 超时)
    int round_trip(const unsigned char* f, int n, const char* tag) {
        unsigned int base = d->mac_rx_frames;        // ★ 等**增量**(勿判 >=1:上一帧后恒真 ⇒ 空取 ✗✓)
        d->mac_push_tx_frame(f, n);
        unsigned char got[4096]; int rn = -1;
        for (int cyc = 0; cyc < 300; cyc++) {
            wait(clk.posedge_event());
            if (d->mac_rx_frames >= base + 1) { rn = d->mac_pop_rx_frame(got, 4096); break; }
        }
        if (rn < 0) { printf("   ✗ %s: 回收超时\n", tag); return -1; }
        if (rn != n) { printf("   ✗ %s: 长度 %d ≠ %d\n", tag, rn, n); return rn; }
        for (int i = 0; i < n; i++)
            if (got[i] != f[i]) { printf("   ✗ %s: 逐字节分叉 @%d(got=%02x exp=%02x)\n", tag, i, got[i], f[i]); return -2; }
        return n;
    }

    void run() {
        L_ena_tx.write(true); L_ena_rx.write(true);
        rst_n.write(false);
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        wait(clk.posedge_event()); wait(clk.posedge_event());

        // ---- ① 帧 A(120B)—— 走链并比对 ----
        unsigned char fa[120]; for (int i = 0; i < 120; i++) fa[i] = (unsigned char)((i*3+0x21) & 0xff);
        int r1 = round_trip(fa, 120, "① 帧A");
        chk(r1 == 120, "① 帧A: 长度 = 120 且逐字节一致");

        // ---- ② 帧 B(200B;陷阱字节)----
        unsigned char fb[200]; for (int i = 0; i < 200; i++) fb[i] = (unsigned char)((i*5+0x11) & 0xff);
        fb[7] = 0xFB; fb[80] = 0xFD; fb[3] = 0x07; fb[4] = 0x55; fb[5] = 0xD5; fb[98] = 0xFB;
        int r2 = round_trip(fb, 200, "② 帧B(陷阱字节)");
        chk(r2 == 200, "② 帧B: 长度 = 200 且逐字节一致(回归卫士)");

        // ---- ③ 计数一致性 ----
        // ⚠ 观测口是**信号**(跨 delta 生效)⇒ 判据前多等一拍再读(同 delta 读会拿到旧值 ✗✓)
        wait(clk.posedge_event());
        chk(d_mtx.read().to_uint() == 2, "③ dev mac_tx_frames = 2");
        chk(d_mrx.read().to_uint() == 2, "③ dev mac_rx_frames = 2");
        chk(m_ftx.read().to_uint() == 2, "③ mac o_frames_tx   = 2");
        chk(m_frx.read().to_uint() == 2, "③ mac o_frames_rx   = 2");
        chk(m_fcs.read().to_uint() == 0, "③ mac o_fcs_err     = 0");
        chk(d->mac_tx_drop_cnt == 0 && d->mac_rx_drop_cnt == 0, "③ 丢弃计数全 0");

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    DeviceTlm dev("dev");
    DummyAmbaMaster dum("dum");                 // ★ 占位主端(reg_s 必绑;E109 的 socket 版)✓
    dev.reg_s.bind(dum.m);
    MacTlm    mac("mac");

    // ---- dev ⇄ mac:NIC 侧 AXIS(dev 出 = mac 入;同一信号)----
    sc_core::sc_signal<bool>              D_tvalid, D_tlast, D_tready;
    sc_core::sc_signal<sc_biguint<2048>> D_tdata;
    sc_core::sc_signal<sc_biguint<256>>   D_tkeep;
    dev.cmac_m_axis_tvalid(D_tvalid); mac.cmac_m_axis_tvalid(D_tvalid);
    dev.cmac_m_axis_tdata(D_tdata);   mac.cmac_m_axis_tdata(D_tdata);
    dev.cmac_m_axis_tkeep(D_tkeep);   mac.cmac_m_axis_tkeep(D_tkeep);
    dev.cmac_m_axis_tlast(D_tlast);   mac.cmac_m_axis_tlast(D_tlast);
    dev.cmac_m_axis_tready(D_tready); mac.cmac_m_axis_tready(D_tready);
    // ---- mac → dev:roce RX AXIS(无 ready)----
    sc_core::sc_signal<bool>              RD_tvalid, RD_tlast, RD_tuser;
    sc_core::sc_signal<sc_biguint<2048>>  RD_tdata;
    sc_core::sc_signal<sc_biguint<256>>   RD_tkeep;
    mac.roce_cmac_s_axis_tvalid(RD_tvalid); dev.roce_cmac_s_axis_tvalid(RD_tvalid);
    mac.roce_cmac_s_axis_tdata(RD_tdata);   dev.roce_cmac_s_axis_tdata(RD_tdata);
    mac.roce_cmac_s_axis_tkeep(RD_tkeep);   dev.roce_cmac_s_axis_tkeep(RD_tkeep);
    mac.roce_cmac_s_axis_tlast(RD_tlast);   dev.roce_cmac_s_axis_tlast(RD_tlast);
    mac.roce_cmac_s_axis_tuser(RD_tuser);   dev.roce_cmac_s_axis_tuser(RD_tuser);
    // ---- mac 线侧:TB 环回 glue(照 §3;ena 恒 1)----
    sc_core::sc_signal<sc_biguint<512>> L_txd_0, L_txd_1;
    sc_core::sc_signal<sc_biguint<64>>  L_txc_0, L_txc_1;
    sc_core::sc_signal<bool> L_ena_tx, L_ena_rx, L_dval_0, L_dval_1;
    mac.link_txdval_0(L_dval_0); mac.link_txdval_1(L_dval_1);
    mac.link_txd_0(L_txd_0); mac.link_txd_1(L_txd_1);
    mac.link_txc_0(L_txc_0); mac.link_txc_1(L_txc_1);
    mac.link_rxd_0(L_txd_0); mac.link_rxd_1(L_txd_1);
    mac.link_rxc_0(L_txc_0); mac.link_rxc_1(L_txc_1);
    mac.link_txclk_ena_0(L_ena_tx); mac.link_txclk_ena_1(L_ena_tx);
    mac.link_rxclk_ena_0(L_ena_rx); mac.link_rxclk_ena_1(L_ena_rx);

    // ---- mac 复位/时钟 + 观测 ----
    sc_core::sc_signal<sc_uint<32>> m_ftx, m_frx, m_idle, m_fcs;
    sc_core::sc_signal<sc_uint<32>> m_fcs_lm;
    mac.clk(clk); mac.rst_n(rst_n);
    mac.o_frames_tx(m_ftx); mac.o_frames_rx(m_frx); mac.o_idle_beats(m_idle); mac.o_fcs_err(m_fcs);
    mac.o_lane_mismatch_cnt(m_fcs_lm);

    // ---- dev:时钟/复位 + 观测 + CXL 侧哑绑定(⛔ 每个 out 独立哑信号,E115 纪律)----
    // ⚠ 哑信号纪律(E115 实踩):**每个 dev 出脚一个独立信号**;入脚可共享(仅入脚用的小集合)✓
    sc_core::sc_signal<sc_uint<32>> du32_sq;
    sc_core::sc_signal<sc_uint<32>> d_mtx, d_mrx,
        du32_a, du32_b, du32_c, du32_d, du32_e, du32_f,
        du32_g, du32_h, du32_i, du32_j, du32_k, du32_l;
    sc_core::sc_signal<bool> dB_in,
        dB_o1, dB_o2, dB_o3, dB_o4, dB_o5, dB_o6, dB_o7, dB_o8, dB_o9;
    sc_core::sc_signal<sc_uint<5>>  du5_in, du5_o1, du5_o2, du5_o3;
    sc_core::sc_signal<sc_uint<3>>  du3_in;
    sc_core::sc_signal<sc_uint<4>>  du4_in;
    sc_core::sc_signal<sc_uint<12>> du12_in, du12_o1, du12_o2, du12_o3, du12_o4;
    sc_core::sc_signal<sc_uint<46>> du46_in, du46_o1, du46_o2;
    sc_core::sc_signal<sc_biguint<512>> du512_in, du512_o1;
    dev.clk(clk); dev.rst_n(rst_n);
    // 输入(各自只作输入;共享同一哑源 ✓)
    dev.d2h_req0_ready(dB_in); dev.d2h_req1_ready(dB_in); dev.d2h_data_ready(dB_in); dev.d2h_rsp_ready(dB_in);
    dev.h2d_req_valid(dB_in);  dev.h2d_req_opcode(du3_in); dev.h2d_req_addr(du46_in); dev.h2d_req_uqid(du12_in);
    dev.h2d_rsp_valid(dB_in);  dev.h2d_rsp_opcode(du4_in); dev.h2d_rsp_rsp_data(du12_in); dev.h2d_rsp_cqid(du12_in);
    dev.h2d_data_valid(dB_in); dev.h2d_data_cqid(du12_in); dev.h2d_data_chunk_valid(dB_in); dev.h2d_data_payload(du512_in);
    // 输出(★ 每个独立)
    dev.d2h_req0_valid(dB_o1); dev.d2h_req0_opcode(du5_o1); dev.d2h_req0_cqid(du12_o1); dev.d2h_req0_addr(du46_o1);
    dev.d2h_req1_valid(dB_o2); dev.d2h_req1_opcode(du5_o2); dev.d2h_req1_cqid(du12_o2); dev.d2h_req1_addr(du46_o2);
    dev.d2h_data_valid(dB_o3); dev.d2h_data_uqid(du12_o3); dev.d2h_data_chunk_valid(dB_o4); dev.d2h_data_payload(du512_o1);
    dev.d2h_rsp_valid(dB_o5);  dev.d2h_rsp_opcode(du5_o3); dev.d2h_rsp_uqid(du12_o4);
    dev.h2d_req_ready(dB_o6);  dev.h2d_rsp_ready(dB_o7);   dev.h2d_data_ready(dB_o8);
    dev.nic_intr(dB_o9);
    dev.o_db_sq_cnt(du32_sq); dev.o_db_rq_cnt(du32_a); dev.o_ro_wr_cnt(du32_b); dev.o_wqe_cnt(du32_c); dev.o_cqe_cnt(du32_d);
    dev.o_db_drop_cnt(du32_e); dev.o_max_rd_ostd(du32_f); dev.o_max_act_per_qp(du32_g);
    dev.o_goerr_cnt(du32_h); dev.o_rnr_cnt(du32_i); dev.o_opc_simpl_cnt(du32_j);
    dev.o_snp_held_cnt(du32_k); dev.o_qp_fatal_cnt(du32_l);
    dev.o_mac_tx_frames(d_mtx); dev.o_mac_rx_frames(d_mrx);
    sc_core::sc_signal<sc_uint<32>> d_ltx, d_lack, d_lun, d_lbig;   // ★ v1.0 线侧口 ✓
    dev.o_line_tx_pkts(d_ltx); dev.o_line_rx_acks(d_lack);
    dev.o_line_unexp_cnt(d_lun); dev.o_line_big_cnt(d_lbig);

    // ---- 驱动 ----
    DockDrv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n);
    drv.d_mtx(d_mtx); drv.d_mrx(d_mrx); drv.m_ftx(m_ftx); drv.m_frx(m_frx); drv.m_fcs(m_fcs);
    drv.L_ena_tx(L_ena_tx); drv.L_ena_rx(L_ena_rx);
    drv.d = &dev;

    sc_core::sc_start(20, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_MAC_NIC_DOCK %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
