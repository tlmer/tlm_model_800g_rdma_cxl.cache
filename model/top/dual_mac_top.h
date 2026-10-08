//============================================================================
// dual_mac_top.h — ★ **双实例 top**:一个 top 内例化 **2 个 MacTlm 实例**,
//   LINK 交叉直连 ⇒ 「客户 MAC ↔ 本 MAC」场景的**互测装配**(用户令 2026-10-05)✓
//
// 拓扑(★ 两个 instance,各自完整):
//   [mac_a(角色 A:替身「客户 MAC」)] ──LINK 交叉线──> [mac_b(角色 B:本 MAC TLM)]
//   [mac_b] ──LINK 交叉线──> [mac_a]
//   · 两侧 NIC 面(2048b AXIS)**留白** ⇒ 由调用方(TB/客户)直绑 `top.mac_a` / `top.mac_b` 的端口 ✓
//   · ⛔ 内部 LINK 已在本 top 接好(调用方**不要**再动)✗
//
// ★ 「PCS 角色」glue(规格 `规格文档` §3/§7.2 预告的样例):LINK 的 clk/ena
//   **是 MAC 输入**(原生由 PCS 侧给)⇒ 两裸 MAC 直连时由本 top 补两件:
//     ① **交叉线**:txd/txc/txdval → 对端 rxd/rxc(dval 不进对端契约;照 `RTL 源` 头注)✓
//     ② **ena 恒 1**(干净链路 = 「本拍可收列」)⇒ 每拍上摆(§节气纪律:hold-until-confirmed)✓
//   ⚠ 「真链路」(serdes 位级/延迟/误码)⇒ 用 `PcsLinkTlm` 插在中间(样例 = `tb/tb_mac_pcs_link.cpp`);
//     本 top 是**直连**版(零延迟、无误码 ⇒ 判据更硬:交叉/隔离类)✓
//
// 用法(照 `tb/tb_dual_mac.cpp`):`DualMacTop top("top"); top.clk(clk); top.rst_n(rst);
//   top.mac_a.cmac_m_axis_tvalid(sig); …`(NIC 面直绑即可)✓
//============================================================================
#ifndef DUAL_MAC_TOP_H
#define DUAL_MAC_TOP_H

#include <systemc.h>
#include "mac_tlm.h"

struct DualMacTop: sc_core::sc_module {
    // ---------------- 时钟/复位(两实例共用)----------------
    sc_in<bool> clk, rst_n;

    // ---------------- ★ 两个实例(公开:NIC 面由调用方直绑)----------------
    MacTlm mac_a, mac_b;

    // ---------------- 内部 LINK 交叉线(a.tx → b.rx;b.tx → a.rx)----------------
    sc_core::sc_signal<sc_biguint<512>> W_ab_0, W_ab_1, W_ba_0, W_ba_1;
    sc_core::sc_signal<sc_biguint<64>>  WC_ab_0, WC_ab_1, WC_ba_0, WC_ba_1;
    sc_core::sc_signal<bool> Wd_ab_0, Wd_ab_1, Wd_ba_0, Wd_ba_1;
    // ---------------- 「PCS 角色」ena(初值 1;glue 每拍上摆)----------------
    sc_core::sc_signal<bool> E_ax0, E_ax1, E_ar0, E_ar1;    // a 的 tx/rx ena
    sc_core::sc_signal<bool> E_bx0, E_bx1, E_br0, E_br1;    // b 的 tx/rx ena

    SC_HAS_PROCESS(DualMacTop);
    DualMacTop(sc_core::sc_module_name nm)
    : sc_module(nm), mac_a("mac_a"), mac_b("mac_b"),
      W_ab_0("W_ab_0"), W_ab_1("W_ab_1"), W_ba_0("W_ba_0"), W_ba_1("W_ba_1"),
      WC_ab_0("WC_ab_0"), WC_ab_1("WC_ab_1"), WC_ba_0("WC_ba_0"), WC_ba_1("WC_ba_1"),
      Wd_ab_0("Wd_ab_0"), Wd_ab_1("Wd_ab_1"), Wd_ba_0("Wd_ba_0"), Wd_ba_1("Wd_ba_1"),
      E_ax0("E_ax0", true), E_ax1("E_ax1", true), E_ar0("E_ar0", true), E_ar1("E_ar1", true),
      E_bx0("E_bx0", true), E_bx1("E_bx1", true), E_br0("E_br0", true), E_br1("E_br1", true)
    {
        mac_a.clk(clk);  mac_a.rst_n(rst_n);
        mac_b.clk(clk);  mac_b.rst_n(rst_n);
        // ---- a.tx → b.rx(交叉;dval 只到线,不进对端契约)----
        mac_a.link_txdval_0(Wd_ab_0); mac_a.link_txdval_1(Wd_ab_1);
        mac_a.link_txd_0(W_ab_0);     mac_a.link_txd_1(W_ab_1);
        mac_a.link_txc_0(WC_ab_0);    mac_a.link_txc_1(WC_ab_1);
        mac_b.link_rxd_0(W_ab_0);     mac_b.link_rxd_1(W_ab_1);
        mac_b.link_rxc_0(WC_ab_0);    mac_b.link_rxc_1(WC_ab_1);
        // ---- b.tx → a.rx ----
        mac_b.link_txdval_0(Wd_ba_0); mac_b.link_txdval_1(Wd_ba_1);
        mac_b.link_txd_0(W_ba_0);     mac_b.link_txd_1(W_ba_1);
        mac_b.link_txc_0(WC_ba_0);    mac_b.link_txc_1(WC_ba_1);
        mac_a.link_rxd_0(W_ba_0);     mac_a.link_rxd_1(W_ba_1);
        mac_a.link_rxc_0(WC_ba_0);    mac_a.link_rxc_1(WC_ba_1);
        // ---- ena(「PCS 角色」)----
        mac_a.link_txclk_ena_0(E_ax0); mac_a.link_txclk_ena_1(E_ax1);
        mac_a.link_rxclk_ena_0(E_ar0); mac_a.link_rxclk_ena_1(E_ar1);
        mac_b.link_txclk_ena_0(E_bx0); mac_b.link_txclk_ena_1(E_bx1);
        mac_b.link_rxclk_ena_0(E_br0); mac_b.link_rxclk_ena_1(E_br1);

        SC_METHOD(ena_glue);
        sensitive << clk.pos();
        dont_initialize();
    }
    // 「本拍可收列」恒真(干净链路)⇒ 每拍上摆 ✓
    void ena_glue() {
        E_ax0.write(true); E_ax1.write(true); E_ar0.write(true); E_ar1.write(true);
        E_bx0.write(true); E_bx1.write(true); E_br0.write(true); E_br1.write(true);
    }
};

#endif // DUAL_MAC_TOP_H
