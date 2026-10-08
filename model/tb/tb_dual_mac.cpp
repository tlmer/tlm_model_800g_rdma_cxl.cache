//============================================================================
// tb_dual_mac.cpp — ★ 双实例 top 互测(2× MacTlm 背靠背;用户令 2026-10-05)✓
// 被测 = `top/dual_mac_top.h`(内含 **2 个 MacTlm 实例** + LINK 交叉线 + 「PCS 角色」ena glue)✓
//
// 判据(预登记):
//   ① A→B:帧 A(120B)⇒ A 的 NIC 进、B 的 NIC 出 ⇒ 逐字节一致
//   ② B→A:帧 B(200B,**陷阱字节**)⇒ 反向逐字节一致(控制位按位置的回归卫士)✓
//   ③ ★ **双向同刻**:A 发帧 C(64B)/ B 发帧 D(180B)(两拍内相继)⇒ 各自到达对方,**不串**
//      (合并轮询:每拍同时看两边接收 ⇒ 不漏单拍 valid)✓
//   ④ 计数交叉见证:a.tx=2 ∧ b.rx=2;b.tx=2 ∧ a.rx=2;两侧 FCS 错 = 0;idle 在涨 ✓
//   ⑤ 静默负控:静置 200 拍 ⇒ 两侧 rx 计数不涨(无鬼帧);idle 继续涨 ✓
//============================================================================
#include <systemc.h>
#include "dual_mac_top.h"
#include <vector>
#include <cstdio>

//----------------------------------------------------------------------------
struct DualDrv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    // A 侧 NIC(发送:TB → mac_a.cmac)
    sc_out<bool> A_tv; sc_out<sc_biguint<2048>> A_td; sc_out<sc_biguint<256>> A_tk; sc_out<bool> A_tl;
    sc_in<bool>  A_tr;
    // A 侧 NIC(回收:mac_a.roce)
    sc_in<bool>  A_rv; sc_in<sc_biguint<2048>> A_rd; sc_in<sc_biguint<256>> A_rk; sc_in<bool> A_rl;
    // B 侧 NIC(发送)
    sc_out<bool> B_tv; sc_out<sc_biguint<2048>> B_td; sc_out<sc_biguint<256>> B_tk; sc_out<bool> B_tl;
    sc_in<bool>  B_tr;
    // B 侧 NIC(回收)
    sc_in<bool>  B_rv; sc_in<sc_biguint<2048>> B_rd; sc_in<sc_biguint<256>> B_rk; sc_in<bool> B_rl;
    // 观测
    sc_in<sc_uint<32>> a_ftx, a_frx, a_idle, a_fcs;
    sc_in<sc_uint<32>> b_ftx, b_frx, b_idle, b_fcs;

    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;
    SC_HAS_PROCESS(DualDrv);
    DualDrv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char* name) {
        printf("   v %-44s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }

    // 发一帧(AXIS 单拍;n ≤ 256B);★ 握手后立即撤 valid(防重吃)✓
    void send(sc_core::sc_out<bool>& tv, sc_core::sc_out<sc_biguint<2048>>& td,
              sc_core::sc_out<sc_biguint<256>>& tk, sc_core::sc_out<bool>& tl,
              const unsigned char* f, int n) {
        sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
        for (int j = 0; j < n; j++) { d.range(j*8+7, j*8) = f[j]; k[j] = true; }
        td.write(d); tk.write(k); tv.write(true); tl.write(true);
        wait(clk.posedge_event()); tv.write(false); tl.write(false);
    }
    // 收一帧(单侧轮询;-1 = 超时)
    int poll(sc_core::sc_in<bool>& rv, sc_core::sc_in<sc_biguint<2048>>& rd,
             sc_core::sc_in<sc_biguint<256>>& rk, sc_core::sc_in<bool>& rl,
             std::vector<unsigned char>& got) {
        got.clear();
        for (int c = 0; c < 800; c++) {
            wait(clk.posedge_event());
            if (rv.read()) {
                sc_biguint<2048> dd = rd.read(); sc_biguint<256> kk = rk.read();
                for (int j = 0; j < 256; j++)
                    if (kk[j].to_bool()) got.push_back((unsigned char)dd.range(j*8+7, j*8).to_uint());
                if (rl.read()) return (int)got.size();
            }
        }
        return -1;
    }
    bool eq(const std::vector<unsigned char>& got, const unsigned char* f, int n) {
        if ((int)got.size() != n) return false;
        for (int i = 0; i < n; i++) if (got[i] != f[i]) return false;
        return true;
    }

    void run() {
        rst_n.write(false);
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        wait(clk.posedge_event()); wait(clk.posedge_event());

        std::vector<unsigned char> got;
        // ---- ① A→B(120B)----
        unsigned char fa[120];
        for (int i = 0; i < 120; i++) fa[i] = (unsigned char)((i * 3 + 0x21) & 0xff);
        send(A_tv, A_td, A_tk, A_tl, fa, 120);
        int r1 = poll(B_rv, B_rd, B_rk, B_rl, got);
        chk(r1 == 120 && eq(got, fa, 120), "① A→B: 120B 逐字节一致");

        // ---- ② B→A(200B;陷阱字节)----
        unsigned char fb[200];
        for (int i = 0; i < 200; i++) fb[i] = (unsigned char)((i * 5 + 0x11) & 0xff);
        fb[7] = 0xFB; fb[80] = 0xFD; fb[3] = 0x07; fb[4] = 0x55; fb[5] = 0xD5; fb[98] = 0xFB;
        send(B_tv, B_td, B_tk, B_tl, fb, 200);
        int r2 = poll(A_rv, A_rd, A_rk, A_rl, got);
        chk(r2 == 200 && eq(got, fb, 200), "② B→A(陷阱字节): 200B 逐字节一致");

        // ---- ③ ★ 双向同刻(两拍内相继发;合并轮询不漏单拍 valid)✓ ----
        unsigned char fc[64], fd[180];
        for (int i = 0; i < 64; i++)  fc[i] = (unsigned char)((i * 7 + 0x0B) & 0xff);
        for (int i = 0; i < 180; i++) fd[i] = (unsigned char)((i * 11 + 0x42) & 0xff);
        fd[88] = 0xFD; fd[5] = 0x07;                       // 陷阱字节 ✓
        send(A_tv, A_td, A_tk, A_tl, fc, 64);              // A 发 C(⇒ B 收)
        send(B_tv, B_td, B_tk, B_tl, fd, 180);             // B 发 D(⇒ A 收)
        std::vector<unsigned char> ga, gb; bool da = false, db = false;
        for (int c = 0; c < 800 && !(da && db); c++) {
            wait(clk.posedge_event());
            if (!db && B_rv.read()) {                      // B 收(C)
                sc_biguint<2048> dd = B_rd.read(); sc_biguint<256> kk = B_rk.read();
                for (int j = 0; j < 256; j++)
                    if (kk[j].to_bool()) gb.push_back((unsigned char)dd.range(j*8+7, j*8).to_uint());
                if (B_rl.read()) db = true;
            }
            if (!da && A_rv.read()) {                      // A 收(D)
                sc_biguint<2048> dd = A_rd.read(); sc_biguint<256> kk = A_rk.read();
                for (int j = 0; j < 256; j++)
                    if (kk[j].to_bool()) ga.push_back((unsigned char)dd.range(j*8+7, j*8).to_uint());
                if (A_rl.read()) da = true;
            }
        }
        chk(da && db && eq(gb, fc, 64) && eq(ga, fd, 180), "③ 双向同刻: 互达且不串");

        // ---- ④ 计数交叉见证 ----
        wait(clk.posedge_event()); wait(clk.posedge_event());   // settle(观测口跨 delta)✓
        chk(a_ftx.read().to_uint() == 2 && b_frx.read().to_uint() == 2, "④ 交叉计数: a.tx=2 ∧ b.rx=2");
        chk(b_ftx.read().to_uint() == 2 && a_frx.read().to_uint() == 2, "④ 交叉计数: b.tx=2 ∧ a.rx=2");
        chk(a_fcs.read().to_uint() == 0 && b_fcs.read().to_uint() == 0, "④ 两侧 FCS 错 = 0");
        chk(a_idle.read().to_uint() > 0 && b_idle.read().to_uint() > 0, "④ 两侧 idle 拍在涨(链路活)");

        // ---- ⑤ 静默负控(无鬼帧)----
        unsigned ea = a_frx.read().to_uint(), eb = b_frx.read().to_uint();
        unsigned ia = a_idle.read().to_uint();
        for (int i = 0; i < 200; i++) wait(clk.posedge_event());
        chk(a_frx.read().to_uint() == ea && b_frx.read().to_uint() == eb, "⑤ 负控: 静置 200 拍无鬼帧");
        chk(a_idle.read().to_uint() > ia, "⑤ idle 继续涨");

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    DualMacTop top("top");                       // ★ 2 个 MacTlm 实例在 top 内
    top.clk(clk); top.rst_n(rst_n);

    // ---- A 侧 AXIS 线(NIC 面由 TB 直绑)✓ ----
    sc_core::sc_signal<bool> A_tv, A_tl, A_tr, A_rv, A_rl, A_tu;
    sc_core::sc_signal<sc_biguint<2048>> A_td, A_rd;
    sc_core::sc_signal<sc_biguint<256>>  A_tk, A_rk;
    top.mac_a.cmac_m_axis_tvalid(A_tv); top.mac_a.cmac_m_axis_tdata(A_td);
    top.mac_a.cmac_m_axis_tkeep(A_tk);  top.mac_a.cmac_m_axis_tlast(A_tl);
    top.mac_a.cmac_m_axis_tready(A_tr);
    top.mac_a.roce_cmac_s_axis_tvalid(A_rv); top.mac_a.roce_cmac_s_axis_tdata(A_rd);
    top.mac_a.roce_cmac_s_axis_tkeep(A_rk);  top.mac_a.roce_cmac_s_axis_tlast(A_rl);
    top.mac_a.roce_cmac_s_axis_tuser(A_tu);                   // ⚠ 独立哑(E115)✓
    // ---- B 侧 AXIS 线 ----
    sc_core::sc_signal<bool> B_tv, B_tl, B_tr, B_rv, B_rl, B_tu;
    sc_core::sc_signal<sc_biguint<2048>> B_td, B_rd;
    sc_core::sc_signal<sc_biguint<256>>  B_tk, B_rk;
    top.mac_b.cmac_m_axis_tvalid(B_tv); top.mac_b.cmac_m_axis_tdata(B_td);
    top.mac_b.cmac_m_axis_tkeep(B_tk);  top.mac_b.cmac_m_axis_tlast(B_tl);
    top.mac_b.cmac_m_axis_tready(B_tr);
    top.mac_b.roce_cmac_s_axis_tvalid(B_rv); top.mac_b.roce_cmac_s_axis_tdata(B_rd);
    top.mac_b.roce_cmac_s_axis_tkeep(B_rk);  top.mac_b.roce_cmac_s_axis_tlast(B_rl);
    top.mac_b.roce_cmac_s_axis_tuser(B_tu);
    // ---- 观测 ----
    sc_core::sc_signal<sc_uint<32>> a_ftx, a_frx, a_idle, a_fcs, b_ftx, b_frx, b_idle, b_fcs;
    sc_core::sc_signal<sc_uint<32>> a_fcs_lm;
    sc_core::sc_signal<sc_uint<32>> b_fcs_lm;
    top.mac_a.o_frames_tx(a_ftx); top.mac_a.o_frames_rx(a_frx);
    top.mac_a.o_idle_beats(a_idle); top.mac_a.o_fcs_err(a_fcs);
    top.mac_a.o_lane_mismatch_cnt(a_fcs_lm);
    top.mac_b.o_frames_tx(b_ftx); top.mac_b.o_frames_rx(b_frx);
    top.mac_b.o_idle_beats(b_idle); top.mac_b.o_fcs_err(b_fcs);
    top.mac_b.o_lane_mismatch_cnt(b_fcs_lm);

    DualDrv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n);
    drv.A_tv(A_tv); drv.A_td(A_td); drv.A_tk(A_tk); drv.A_tl(A_tl); drv.A_tr(A_tr);
    drv.A_rv(A_rv); drv.A_rd(A_rd); drv.A_rk(A_rk); drv.A_rl(A_rl);
    drv.B_tv(B_tv); drv.B_td(B_td); drv.B_tk(B_tk); drv.B_tl(B_tl); drv.B_tr(B_tr);
    drv.B_rv(B_rv); drv.B_rd(B_rd); drv.B_rk(B_rk); drv.B_rl(B_rl);
    drv.a_ftx(a_ftx); drv.a_frx(a_frx); drv.a_idle(a_idle); drv.a_fcs(a_fcs);
    drv.b_ftx(b_ftx); drv.b_frx(b_frx); drv.b_idle(b_idle); drv.b_fcs(b_fcs);

    printf("============================================================\n");
    printf("tb_dual_mac - ★ 双实例 top:2× MacTlm 背靠背(LINK 交叉 + ena glue)\n");
    printf("============================================================\n");

    sc_core::sc_start(20, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_DUAL_MAC %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
