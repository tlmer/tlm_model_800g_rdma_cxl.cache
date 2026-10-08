//============================================================================
// stack_top.h — ★ **完整端点栈 top**:一套 = [DeviceTlm(COH+NIC)] + [Cxl2AceBridge]
//   + [MacTlm],内部接线全封装(用户令 2026-10-05:「双实例 mac 测试要加上 COH 和 cxl2ace
//   全链路交互测试」)✓
//
// 一个 StackTop = **一个 RDMA 端点**(照系统拓扑:CPU/ACE ←→ 桥 ←→ 设备 ←→ MAC ←→ 线)✓
//   · ACE 侧:**不封装** —— `stack.br.ace_m` 直接外露,由调用方绑主机内存/CCI(经协议检查器)✓
//   · 线侧:**不封装** —— `stack.mac` 的 LINK 口外露,由调用方接对端(交叉线 + ena glue;
//     样例 = `tb/tb_dual_stack.cpp`;⚠ clk/ena 是 MAC 输入 ⇒ 直连必须补「PCS 角色」glue)✓
//   · 三模型**公开成员**(TB 直用:hooks/reg 直连方法/观测)✓
//
// 内部接线(自 `tb_device_e2e.cpp` 顶层**移植**,判据面零改动)①:
//   设备 ↔ 桥:pin 级 CXL.cache 逐条显式信号(e2e 原样);设备 ↔ MAC:NIC 侧 2048b AXIS ✓
//   ⚠ 未用观测口各自独立哑信号(E115 纪律:每个 out 独立)✓
//
// ★ 用法:见 `tb/tb_dual_stack.cpp`(2× StackTop + 交叉线 + 2× ACE 内存 + 互测判据)✓
//   ⚠ 版本口径:本 top 是**装配样例**(非四交付库之一);三模型接口版本各自独立 ✓
//============================================================================
#ifndef STACK_TOP_H
#define STACK_TOP_H

#include <systemc.h>
#include <amba_pv.h>
#include "device_tlm.h"
#include "cxl2ace_bridge.h"
#include "mac_tlm.h"

struct StackTop: sc_core::sc_module {
    // ---------------- 时钟/复位 ----------------
    sc_in<bool> clk, rst_n;

    // ---------------- ★ 三模型(公开:TB 直绑)----------------
    DeviceTlm    dev;
    Cxl2AceBridge br;
    MacTlm       mac;

    // ---------------- 公开观测(判据用;⛔ 非接口面)----------------
    sc_core::sc_signal<bool>        sig_intr;
    sc_core::sc_signal<sc_uint<32>> sig_sq, sig_rq, sig_ro, sig_wqe, sig_cqe, sig_dbd;
    sc_core::sc_signal<sc_uint<32>> sig_mfcs_lm;
    sc_core::sc_signal<sc_uint<32>> sig_goerr, sig_rd, sig_wr, sig_err, sig_snp;
    sc_core::sc_signal<sc_uint<32>> sig_mtx, sig_mrx;                 // dev 侧 MAC 帧计数 ✓
    sc_core::sc_signal<sc_uint<32>> sig_ltx, sig_lack, sig_lun, sig_lbig;   // ★ v1.0 线侧真包口 ✓
    sc_core::sc_signal<sc_uint<32>> sig_mftx, sig_mfrx, sig_midle, sig_mfcs;  // mac 侧 ✓

    // ---------------- 内部接线(私有)----------------
    // 设备 ↔ 桥(pin 级 CXL.cache;e2e 原样)✓
    sc_core::sc_signal<bool> d_r0v, d_r0r, d_r1v, d_r1r, d_dv, d_dr, d_rv_, d_rr_;
    sc_core::sc_signal<bool> h_qrdy, h_srdy, h_drdy, h_rv, h_rr_, h_dv_;
    sc_core::sc_signal<sc_uint<5>>  d_r0op, d_r1op, d_rop;
    sc_core::sc_signal<sc_uint<4>>  h_rop;
    sc_core::sc_signal<sc_uint<3>>  h_rop3;
    sc_core::sc_signal<sc_uint<12>> d_r0cq, d_r1cq, d_duq, d_ruq, h_rrq, h_dcq;
    sc_core::sc_signal<sc_uint<12>> h_rspd, h_dcq2;      // ⚠ 桥两输出别共用一条线(E115)✓
    sc_core::sc_signal<sc_uint<46>> d_r0ad, d_r1ad, h_rad;
    sc_core::sc_signal<bool> d_dcv, h_dcv;
    sc_core::sc_signal<sc_biguint<512>> d_dpl, h_dpl;
    // 设备 ↔ MAC(NIC 侧 AXIS;dev 出 = mac 入,同信号)✓
    sc_core::sc_signal<bool>             D_tvalid, D_tlast, D_tready;
    sc_core::sc_signal<sc_biguint<2048>> D_tdata;
    sc_core::sc_signal<sc_biguint<256>>  D_tkeep;
    sc_core::sc_signal<bool>             RD_tvalid, RD_tlast, RD_tuser;
    sc_core::sc_signal<sc_biguint<2048>> RD_tdata;
    sc_core::sc_signal<sc_biguint<256>>  RD_tkeep;
    // 未外露的观测口 ⇒ 各自独立哑(E115)✓
    sc_core::sc_signal<sc_uint<32>> du_ostd, du_actqp, du_rnr, du_osimpl, du_snpheld, du_qpf;
    sc_core::sc_signal<sc_uint<32>> du_br_ostd;
    sc_core::sc_signal<sc_uint<12>> du_frd, du_fwr, du_rmax, du_wmin;   // ★ 照桥口宽(12b;E109 类型须配)✓

    SC_HAS_PROCESS(StackTop);
    StackTop(sc_core::sc_module_name nm)
    : sc_module(nm), dev("dev"), br("br"), mac("mac"),
      sig_intr("sig_intr"),
      sig_sq("sig_sq"), sig_rq("sig_rq"), sig_ro("sig_ro"), sig_wqe("sig_wqe"),
      sig_cqe("sig_cqe"), sig_dbd("sig_dbd"), sig_goerr("sig_goerr"),
      sig_rd("sig_rd"), sig_wr("sig_wr"), sig_err("sig_err"), sig_snp("sig_snp"),
      sig_mtx("sig_mtx"), sig_mrx("sig_mrx"),
      sig_ltx("sig_ltx"), sig_lack("sig_lack"), sig_lun("sig_lun"), sig_lbig("sig_lbig"),
      sig_mftx("sig_mftx"), sig_mfrx("sig_mfrx"), sig_midle("sig_midle"), sig_mfcs("sig_mfcs"),
      d_r0v("d_r0v"), d_r0r("d_r0r"), d_r1v("d_r1v"), d_r1r("d_r1r"),
      d_dv("d_dv"), d_dr("d_dr"), d_rv_("d_rv_"), d_rr_("d_rr_"),
      h_qrdy("h_qrdy"), h_srdy("h_srdy"), h_drdy("h_drdy"),
      h_rv("h_rv"), h_rr_("h_rr_"), h_dv_("h_dv_"),
      d_r0op("d_r0op"), d_r1op("d_r1op"), d_rop("d_rop"),
      h_rop("h_rop"), h_rop3("h_rop3"),
      d_r0cq("d_r0cq"), d_r1cq("d_r1cq"), d_duq("d_duq"), d_ruq("d_ruq"),
      h_rrq("h_rrq"), h_dcq("h_dcq"), h_rspd("h_rspd"), h_dcq2("h_dcq2"),
      d_r0ad("d_r0ad"), d_r1ad("d_r1ad"), h_rad("h_rad"),
      d_dcv("d_dcv"), h_dcv("h_dcv"), d_dpl("d_dpl"), h_dpl("h_dpl"),
      D_tvalid("D_tvalid"), D_tlast("D_tlast"), D_tready("D_tready"),
      D_tdata("D_tdata"), D_tkeep("D_tkeep"),
      RD_tvalid("RD_tvalid"), RD_tlast("RD_tlast"), RD_tuser("RD_tuser"),
      RD_tdata("RD_tdata"), RD_tkeep("RD_tkeep"),
      du_ostd("du_ostd"), du_actqp("du_actqp"), du_rnr("du_rnr"), du_osimpl("du_osimpl"),
      du_snpheld("du_snpheld"), du_qpf("du_qpf"),
      du_br_ostd("du_br_ostd"), du_frd("du_frd"), du_fwr("du_fwr"),
      du_rmax("du_rmax"), du_wmin("du_wmin")
    {
        dev.clk(clk); dev.rst_n(rst_n);
        br.clk(clk);  br.rst_n(rst_n);
        mac.clk(clk); mac.rst_n(rst_n);

        // ---- 设备 ↔ 桥:pin 级 CXL.cache 直连(逐条显式;e2e 原样移植)✓ ----
        dev.d2h_req0_valid(d_r0v); dev.d2h_req0_opcode(d_r0op); dev.d2h_req0_cqid(d_r0cq);
        dev.d2h_req0_addr(d_r0ad); dev.d2h_req0_ready(d_r0r);
        br.d2h_req0_valid(d_r0v);  br.d2h_req0_opcode(d_r0op);  br.d2h_req0_cqid(d_r0cq);
        br.d2h_req0_addr(d_r0ad);  br.d2h_req0_ready(d_r0r);
        dev.d2h_req1_valid(d_r1v); dev.d2h_req1_opcode(d_r1op); dev.d2h_req1_cqid(d_r1cq);
        dev.d2h_req1_addr(d_r1ad); dev.d2h_req1_ready(d_r1r);
        br.d2h_req1_valid(d_r1v);  br.d2h_req1_opcode(d_r1op);  br.d2h_req1_cqid(d_r1cq);
        br.d2h_req1_addr(d_r1ad);  br.d2h_req1_ready(d_r1r);
        dev.d2h_data_valid(d_dv); dev.d2h_data_uqid(d_duq); dev.d2h_data_chunk_valid(d_dcv);
        dev.d2h_data_payload(d_dpl); dev.d2h_data_ready(d_dr);
        br.d2h_data_valid(d_dv);  br.d2h_data_uqid(d_duq);  br.d2h_data_chunk_valid(d_dcv);
        br.d2h_data_payload(d_dpl); br.d2h_data_ready(d_dr);
        dev.d2h_rsp_valid(d_rv_); dev.d2h_rsp_opcode(d_rop); dev.d2h_rsp_uqid(d_ruq); dev.d2h_rsp_ready(d_rr_);
        br.d2h_rsp_valid(d_rv_);  br.d2h_rsp_opcode(d_rop);  br.d2h_rsp_uqid(d_ruq);  br.d2h_rsp_ready(d_rr_);
        br.h2d_req_valid(h_rv); dev.h2d_req_valid(h_rv);
        br.h2d_req_opcode(h_rop3); dev.h2d_req_opcode(h_rop3);
        br.h2d_req_addr(h_rad); dev.h2d_req_addr(h_rad);
        br.h2d_req_uqid(h_rrq); dev.h2d_req_uqid(h_rrq);
        br.h2d_req_ready(h_qrdy); dev.h2d_req_ready(h_qrdy);
        br.h2d_rsp_valid(h_rr_); dev.h2d_rsp_valid(h_rr_);
        br.h2d_rsp_opcode(h_rop); dev.h2d_rsp_opcode(h_rop);
        br.h2d_rsp_rsp_data(h_rspd); dev.h2d_rsp_rsp_data(h_rspd);
        br.h2d_rsp_cqid(h_dcq); dev.h2d_rsp_cqid(h_dcq);
        br.h2d_rsp_ready(h_srdy); dev.h2d_rsp_ready(h_srdy);
        br.h2d_data_valid(h_dv_); dev.h2d_data_valid(h_dv_);
        br.h2d_data_cqid(h_dcq2); dev.h2d_data_cqid(h_dcq2);
        br.h2d_data_chunk_valid(h_dcv); dev.h2d_data_chunk_valid(h_dcv);
        br.h2d_data_payload(h_dpl); dev.h2d_data_payload(h_dpl);
        br.h2d_data_ready(h_drdy); dev.h2d_data_ready(h_drdy);

        // ---- 设备 ⇄ MAC:NIC 侧 2048b AXIS(dev 出 = mac 入)✓ ----
        dev.cmac_m_axis_tvalid(D_tvalid); mac.cmac_m_axis_tvalid(D_tvalid);
        dev.cmac_m_axis_tdata(D_tdata);   mac.cmac_m_axis_tdata(D_tdata);
        dev.cmac_m_axis_tkeep(D_tkeep);   mac.cmac_m_axis_tkeep(D_tkeep);
        dev.cmac_m_axis_tlast(D_tlast);   mac.cmac_m_axis_tlast(D_tlast);
        dev.cmac_m_axis_tready(D_tready); mac.cmac_m_axis_tready(D_tready);
        mac.roce_cmac_s_axis_tvalid(RD_tvalid); dev.roce_cmac_s_axis_tvalid(RD_tvalid);
        mac.roce_cmac_s_axis_tdata(RD_tdata);   dev.roce_cmac_s_axis_tdata(RD_tdata);
        mac.roce_cmac_s_axis_tkeep(RD_tkeep);   dev.roce_cmac_s_axis_tkeep(RD_tkeep);
        mac.roce_cmac_s_axis_tlast(RD_tlast);   dev.roce_cmac_s_axis_tlast(RD_tlast);
        mac.roce_cmac_s_axis_tuser(RD_tuser);   dev.roce_cmac_s_axis_tuser(RD_tuser);

        // ---- 观测 ----
        dev.nic_intr(sig_intr);
        dev.o_db_sq_cnt(sig_sq); dev.o_db_rq_cnt(sig_rq); dev.o_ro_wr_cnt(sig_ro);
        dev.o_wqe_cnt(sig_wqe);  dev.o_cqe_cnt(sig_cqe);  dev.o_db_drop_cnt(sig_dbd);
        dev.o_goerr_cnt(sig_goerr);
        dev.o_max_rd_ostd(du_ostd); dev.o_max_act_per_qp(du_actqp);
        dev.o_rnr_cnt(du_rnr); dev.o_opc_simpl_cnt(du_osimpl);
        dev.o_snp_held_cnt(du_snpheld); dev.o_qp_fatal_cnt(du_qpf);
        dev.o_mac_tx_frames(sig_mtx); dev.o_mac_rx_frames(sig_mrx);
        dev.o_line_tx_pkts(sig_ltx); dev.o_line_rx_acks(sig_lack);
        dev.o_line_unexp_cnt(sig_lun); dev.o_line_big_cnt(sig_lbig);
        br.o_rd_done(sig_rd); br.o_wr_done(sig_wr); br.o_err(sig_err); br.o_snoop_cnt(sig_snp);
        br.o_rd_ostd_max(du_br_ostd); br.o_fst_rd_cqid(du_frd); br.o_fst_wr_cqid(du_fwr);
        br.o_rd_cqid_max(du_rmax); br.o_wr_cqid_min(du_wmin);
        mac.o_frames_tx(sig_mftx); mac.o_frames_rx(sig_mfrx);
        mac.o_idle_beats(sig_midle); mac.o_fcs_err(sig_mfcs);
    mac.o_lane_mismatch_cnt(sig_mfcs_lm);
        // ⚠ mac 的 LINK 口**故意不绑** ⇒ 由调用方接线侧(交叉/对端/PCS link)✓
        // ⚠ br.ace_m **故意不绑** ⇒ 由调用方绑主机内存/CCI(经协议检查器)✓
    }
};

#endif // STACK_TOP_H
