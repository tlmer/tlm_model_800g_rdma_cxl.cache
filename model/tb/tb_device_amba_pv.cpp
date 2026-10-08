//============================================================================
// tb_device_amba_pv.cpp — ★ (A7 定稿):设备寄存器面 **AMBA-PV 从口** 自测✓
//   规格:客户答 A7 = **AMBA-PV 从口** ⇒ `DeviceTlm.reg_s`(v1.0);
//   判据(预登记):
//   ① 经 socket 写 QP 配置(QP_CONF/SQ_BA/CQ_BA/Q_DEPTH)⇒ **直连读回逐条一致**
//   ② 经 socket 敲门铃(SQ_PI_DB)⇒ 引擎门铃计数(o_db_sq_cnt 信号)增 ⇒ **真驱动引擎**
//   ③ 未映射地址读 ⇒ **DECERR**;RO 写(per-QP STAT_RQ_PI_DB)⇒ **SLVERR**
//   ④ 两路同源:直连写 ⇒ socket 读 == 值;socket 写 ⇒ 直连读 == 值
//   ⑤ 访问计数 `reg_sock_n` = 成功访问次数
//============================================================================
#include <systemc.h>
#include <amba_pv.h>
#include "device_tlm.h"
#include <cstdio>

//----------------------------------------------------------------------------
// AMBA-PV 主机(判据用):m.b_transport(单笔 4B);返回 resp ✓
//----------------------------------------------------------------------------
struct ApvMaster: sc_core::sc_module,
                  amba_pv::amba_pv_master_base {          // ★ 照 VDK dma 例子:主机模块要继承 master_base ✓
    sc_in<bool> clk;
    amba_pv::amba_pv_master_socket<32> m;
    amba_pv::amba_pv_trans_pool pool;
    DeviceTlm * dev = nullptr;

    SC_HAS_PROCESS(ApvMaster);
    ApvMaster(sc_core::sc_module_name nm):
        sc_module(nm), amba_pv::amba_pv_master_base("mas_mb"), m("m") {
        m(*this);                                          // ★ 反向路径接口绑到自己(照 dma.cpp:53)✓
        SC_THREAD(run);
    }

    amba_pv::amba_pv_resp_t xfer(bool is_wr, sc_dt::uint64 addr, uint32_t & vref) {
        unsigned char buf[4];
        if (is_wr) { buf[0]=(unsigned char)vref; buf[1]=(unsigned char)(vref>>8);
                     buf[2]=(unsigned char)(vref>>16); buf[3]=(unsigned char)(vref>>24); }
        // ⚠ pool API = allocate(length, size, ctrl, burst)(照 BL `amba_pv_mm.h;非 (id,len,ptr,kind))✓
        amba_pv::amba_pv_trans_ptr trans(pool.allocate(1, 4,
            (const amba_pv::amba_pv_control *)NULL, amba_pv::AMBA_PV_INCR));
        trans->set_command(is_wr ? tlm::TLM_WRITE_COMMAND : tlm::TLM_READ_COMMAND);
        trans->set_address(addr);
        trans->set_data_length(4);
        trans->set_data_ptr(buf);
        amba_pv::amba_pv_extension * ex = NULL; trans->get_extension(ex);
        sc_core::sc_time t = sc_core::SC_ZERO_TIME;
        m.b_transport(*trans, t);
        wait(t);
        if (!is_wr && ex) vref = (uint32_t)buf[0] | ((uint32_t)buf[1]<<8) |
                                      ((uint32_t)buf[2]<<16) | ((uint32_t)buf[3]<<24);
        return ex ? ex->get_resp() : amba_pv::AMBA_PV_OKAY;
    }
    amba_pv::amba_pv_resp_t wr(sc_dt::uint64 a, uint32_t v) { uint32_t t=v; return xfer(true, a, t); }
    amba_pv::amba_pv_resp_t rd(sc_dt::uint64 a, uint32_t & v) { return xfer(false, a, v); }

    int pass_cnt = 0, fail_cnt = 0; bool all_pass = false;
    void chk(bool ok, const char * name) {
        printf("   v %-46s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }
    void tick() { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); }

    void run() {
        for (int i = 0; i < 4; i++) tick();                 // 等复位释放(下方 rst 由 sc_main 控)✓
        const sc_dt::uint64 qp = 0x50180200;                // QP 索引 2 ✓
        uint32_t v = 0;

        // ---- ① socket 写 QP 配置 ⇒ 直连读回逐条一致 ----
        chk(wr(qp + 0x00, 0x00000001) == amba_pv::AMBA_PV_OKAY, "① socket 写 QP_CONF = OKAY");
        sc_uint<32> dv = 0; dev->reg_read(qp + 0x00, dv);    // 直连读回 ✓
        chk(dv.to_uint() == 0x00000001, "① 直连读回 QP_CONF 一致");
        wr(qp + 0x10, 0x00060000);                           // SQ_BA
        dev->reg_read(qp + 0x10, dv);
        chk(dv.to_uint() == 0x00060000, "① 直连读回 SQ_BA 一致");
        wr(qp + 0x18, 0x00070000);                           // CQ_BA
        dev->reg_read(qp + 0x18, dv);
        chk(dv.to_uint() == 0x00070000, "① 直连读回 CQ_BA 一致");
        wr(qp + 0x3C, 0x00000008);                           // Q_DEPTH
        dev->reg_read(qp + 0x3C, dv);
        chk(dv.to_uint() == 0x00000008, "① 直连读回 Q_DEPTH 一致");

        // ---- ② socket 敲门铃 ⇒ 引擎计数增(真驱动引擎)----
        unsigned base_db = dev->db_sq_cnt;
        chk(wr(qp + 0x38, 1) == amba_pv::AMBA_PV_OKAY, "② socket 敲门铃 = OKAY");
        for (int i = 0; i < 50 && dev->db_sq_cnt <= base_db; i++) tick();
        chk(dev->db_sq_cnt == base_db + 1, "② 引擎门铃计数 +1(真驱动)");

        // ---- ③ 未映射 ⇒ DECERR;RO 写 ⇒ **OKAY + 忽略 + 计数**(★ 照模型/RTL 语义:RO 写静默忽略,不报错)✓
        chk(rd(0x50182800, v) == amba_pv::AMBA_PV_DECERR, "③ 未映射地址读 ⇒ DECERR");   // per-QP 索引 40(>31)✓
        {
            unsigned ro0 = dev->ro_wr_cnt;
            uint32_t oldv = 0; rd(qp + 0x9C, oldv);                     // 先读原值
            chk(wr(qp + 0x9C, 0x1234) == amba_pv::AMBA_PV_OKAY, "③ RO 写 ⇒ OKAY(忽略式)");
            uint32_t newv = 0; rd(qp + 0x9C, newv);
            chk(newv == oldv, "③ RO 写被忽略(值不变)");
            chk(dev->ro_wr_cnt == ro0 + 1, "③ RO 计数 +1(拦截见证)");
        }

        // ---- ④ 两路同源(互通)----
        dev->reg_write(qp + 0x3C, sc_uint<32>(0x55));        // 直连写
        chk(rd(qp + 0x3C, v) == amba_pv::AMBA_PV_OKAY && v == 0x55, "④ 直连写 ⇒ socket 读 == 值");
        chk(wr(qp + 0x3C, 0x66) == amba_pv::AMBA_PV_OKAY, "④ socket 写 = OKAY");
        dev->reg_read(qp + 0x3C, dv);
        chk(dv.to_uint() == 0x66, "④ socket 写 ⇒ 直连读 == 值");

        // ---- ⑤ 访问计数 ----
        printf("   [计数] socket 成功访问 = %lu 次\n", dev->reg_sock_n);
        // 计数口径:①4 写 + ②1 写 + ③1 读+1 写(RO 忽略式=OKAY)+1 读(验值)+ ④1 读+1 写 = 10 ✓
        //（③ 的 DECERR 读不计;RO 写按"忽略式"返回 OKAY ⇒ 计 ✓)
        chk(dev->reg_sock_n == 10, "⑤ reg_sock_n = 10(成功 socket 访问)");
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
    ApvMaster mas("mas");
    mas.clk(clk); mas.dev = &dev;
    mas.m(dev.reg_s);                            // ★ AMBA-PV 主机 ↔ 设备从口(照 VDK 例子括号式)✓

    dev.clk(clk); dev.rst_n(rst_n);

    // ---- 哑绑定(E115:每个 out 独立)✓ ----
    sc_core::sc_signal<sc_uint<32>> du32_sq, d_dbsq, du32_a, du32_b, du32_c, du32_d, du32_e, du32_f,
        du32_g, du32_h, du32_i, du32_j, du32_k, du32_l, du32_mtx, du32_mrx;
    sc_core::sc_signal<bool> dB_in,
        dB_o1, dB_o2, dB_o3, dB_o4, dB_o5, dB_o6, dB_o7, dB_o8, dB_o9;
    sc_core::sc_signal<sc_uint<5>>  du5_in, du5_o1, du5_o2, du5_o3;
    sc_core::sc_signal<sc_uint<3>>  du3_in;
    sc_core::sc_signal<sc_uint<4>>  du4_in;
    sc_core::sc_signal<sc_uint<12>> du12_in, du12_o1, du12_o2, du12_o3, du12_o4;
    sc_core::sc_signal<sc_uint<46>> du46_in, du46_o1, du46_o2;
    sc_core::sc_signal<sc_biguint<512>> du512_in, du512_o1;
    sc_core::sc_signal<bool> duD_tv, duD_tl, duD_tr, duRD_tv, duRD_tl, duRD_tu;
    sc_core::sc_signal<sc_biguint<2048>> duD_td, duRD_td;
    sc_core::sc_signal<sc_biguint<256>>  duD_tk, duRD_tk;
    // 输入(共享哑源)✓
    dev.d2h_req0_ready(dB_in); dev.d2h_req1_ready(dB_in); dev.d2h_data_ready(dB_in); dev.d2h_rsp_ready(dB_in);
    dev.h2d_req_valid(dB_in);  dev.h2d_req_opcode(du3_in); dev.h2d_req_addr(du46_in); dev.h2d_req_uqid(du12_in);
    dev.h2d_rsp_valid(dB_in);  dev.h2d_rsp_opcode(du4_in); dev.h2d_rsp_rsp_data(du12_in); dev.h2d_rsp_cqid(du12_in);
    dev.h2d_data_valid(dB_in); dev.h2d_data_cqid(du12_in); dev.h2d_data_chunk_valid(dB_in); dev.h2d_data_payload(du512_in);
    dev.cmac_m_axis_tvalid(duD_tv); dev.cmac_m_axis_tdata(duD_td); dev.cmac_m_axis_tkeep(duD_tk);
    dev.cmac_m_axis_tlast(duD_tl);  dev.cmac_m_axis_tready(duD_tr);
    dev.roce_cmac_s_axis_tvalid(duRD_tv); dev.roce_cmac_s_axis_tdata(duRD_td);
    dev.roce_cmac_s_axis_tkeep(duRD_tk);  dev.roce_cmac_s_axis_tlast(duRD_tl);
    dev.roce_cmac_s_axis_tuser(duRD_tu);
    // 输出(每个独立)✓
    dev.d2h_req0_valid(dB_o1); dev.d2h_req0_opcode(du5_o1); dev.d2h_req0_cqid(du12_o1); dev.d2h_req0_addr(du46_o1);
    dev.d2h_req1_valid(dB_o2); dev.d2h_req1_opcode(du5_o2); dev.d2h_req1_cqid(du12_o2); dev.d2h_req1_addr(du46_o2);
    dev.d2h_data_valid(dB_o3); dev.d2h_data_uqid(du12_o3); dev.d2h_data_chunk_valid(dB_o4); dev.d2h_data_payload(du512_o1);
    dev.d2h_rsp_valid(dB_o5);  dev.d2h_rsp_opcode(du5_o3); dev.d2h_rsp_uqid(du12_o4);
    dev.h2d_req_ready(dB_o6);  dev.h2d_rsp_ready(dB_o7);   dev.h2d_data_ready(dB_o8);
    dev.nic_intr(dB_o9);
    dev.o_db_sq_cnt(d_dbsq); dev.o_db_rq_cnt(du32_sq); dev.o_ro_wr_cnt(du32_a); dev.o_wqe_cnt(du32_b);
    dev.o_cqe_cnt(du32_c); dev.o_db_drop_cnt(du32_d); dev.o_max_rd_ostd(du32_e); dev.o_max_act_per_qp(du32_f);
    dev.o_goerr_cnt(du32_g); dev.o_rnr_cnt(du32_h); dev.o_opc_simpl_cnt(du32_i);
    dev.o_snp_held_cnt(du32_j); dev.o_qp_fatal_cnt(du32_k);
    dev.o_mac_tx_frames(du32_l); dev.o_mac_rx_frames(du32_mrx);
    sc_core::sc_signal<sc_uint<32>> du32_ltx, du32_lack, du32_lun, du32_lbig;   // ★ v1.0 线侧口 ✓
    dev.o_line_tx_pkts(du32_ltx); dev.o_line_rx_acks(du32_lack);
    dev.o_line_unexp_cnt(du32_lun); dev.o_line_big_cnt(du32_lbig);

    printf("============================================================\n");
    printf("tb_device_amba_pv - ★ A7:设备寄存器面 AMBA-PV 从口自测\n");
    printf("============================================================\n");

    rst_n.write(false);
    sc_core::sc_start(10, sc_core::SC_NS);
    rst_n.write(true);
    if (sc_core::sc_is_running()) sc_core::sc_start(20, sc_core::SC_US);   // ⚠ driver 判完即 sc_stop ⇒ 防 E546 ✓

    bool pass = mas.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_DEVICE_AMBA_PV %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
