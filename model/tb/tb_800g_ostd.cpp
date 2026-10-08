//============================================================================
// tb_800g_ostd.cpp — 800G 专项 ★ L4-4:**多笔在飞 ×800G(★ 桥侧口径 + 槽号)**  [2026-10-09 建]
//
// 依据 = `md/L4_背压与多笔在飞_TB设计.md` §1-L4-4 行;★ 验证册口径:**读桥侧,不看设备自证** ✓
// 拓扑 = `tb_driver_flow.cpp` 同源(设备 ── pin CXL.cache ── 桥 ── ACE 检查器 ── 主机内存)✓
//
// 用例(判据 = 验证册 §2-L4-4 行细化,预登记):
//   ⓪ **下界对照**:QP2 **单笔**(无并发)⇒ 桥 `o_rd_ostd_max == 1`(**证明读数对并发敏感,非恒真**)✓
//   ① **并发**:QP2+QP3 **各 1 笔同拍投** ⇒ 桥 `o_rd_ostd_max > 1`(★ 判据**只认桥侧**)✓
//   ② **槽号口径**(桥侧四口):`o_fst_rd_cqid==0` ∧ `o_fst_wr_cqid==64` ∧ `o_rd_cqid_max<=63` ∧
//      `o_wr_cqid_min>=64`(RTL 槽号:读 0..63 / 写 64..127)∧ `o_err==0` ✓
//   ③ 抽检:3 笔 CQE wrid 逐笔 + 写数据 64B 逐字节(双 QP 各自 CQ)✓
//   ⚠ 并发来源 = **双 QP**(实据:同 QP 保序"在飞 ≤ 1 job" ⇒ 单 QP 恒测不出 >1,见 L4-3 §7)✓
//============================================================================
#include <systemc.h>
#include <cstring>
#include <amba_pv.h>
#include <models/amba_pv_ace_protocol_checker.h>
#include "device_tlm.h"
#include "dummy_amba_master.h"   // ★ 占位主端(reg_s 必绑)✓
#include "cxl2ace_bridge.h"
#include "nic_regs.h"
#include "nic_hw.h"

// ---- 内存布局(QP2/QP3 独立缓冲;4MB 内互不撞)✓ ----
static const sc_dt::uint64 SQ_BA  = 0x00200000, CQ_BA  = 0x00210000;
static const sc_dt::uint64 SRC_BA = 0x00220000, DST_BA = 0x00230000;
static const sc_dt::uint64 SQ2_BA = 0x00260000, CQ2_BA = 0x00270000;
static const sc_dt::uint64 SRC2_BA = 0x00280000, DST2_BA = 0x00290000;
static const uint32_t QPIX = 2, QPIX2 = 3, DEPTH_SQ = 4;

static uint64_t pat_word(int job, int i) { return 0x9B0B0000ULL + ((uint64_t)job << 16) + (uint64_t)(i * 0x0133); }

//----------------------------------------------------------------------------
// 主机内存(ACE 从端;与 driver_flow 同款最小版)✓
//----------------------------------------------------------------------------
struct HostMem: sc_core::sc_module, amba_pv::amba_pv_ace_slave_base {
    amba_pv::amba_pv_ace_slave_socket<512> ace_s;
    unsigned char mem[65536 * cxlshim::LINE_BYTES];
    uint32_t rd_n = 0, wr_n = 0, bad = 0;

    SC_HAS_PROCESS(HostMem);
    HostMem(sc_core::sc_module_name nm): amba_pv_ace_slave_base("host_mem"), ace_s("ace_s") {
        ace_s(*this);
        for (unsigned i = 0; i < sizeof(mem); i++) mem[i] = 0;
    }
    void b_transport(int, amba_pv::amba_pv_transaction & tr, sc_core::sc_time & t) override {
        amba_pv::amba_pv_extension * ex = NULL; tr.get_extension(ex);
        size_t off = (size_t)((tr.get_address() >> 6) & 0xFFFF) * cxlshim::LINE_BYTES;
        unsigned char * d = tr.get_data_ptr();
        if (tr.is_read())       { rd_n++; for (int i = 0; i < cxlshim::LINE_BYTES; i++) d[i] = mem[off + i]; }
        else if (tr.is_write()) { wr_n++; for (int i = 0; i < cxlshim::LINE_BYTES; i++) mem[off + i] = d[i]; }
        else bad++;
        if (ex != NULL) ex->set_resp(amba_pv::AMBA_PV_OKAY);
        t += sc_core::sc_time(4, sc_core::SC_NS);
    }
    unsigned char * line_ptr(sc_dt::uint64 a) {
        return &mem[(size_t)((a >> 6) & 0xFFFF) * cxlshim::LINE_BYTES];
    }
    unsigned int transport_dbg(int, amba_pv::amba_pv_transaction &) override { return 0; }
    bool get_direct_mem_ptr(int, amba_pv::amba_pv_transaction &, tlm::tlm_dmi &) override { return false; }
};

//----------------------------------------------------------------------------
// 驱动流程(L4-4 版:单笔下界 → 双 QP 并发)✓
//----------------------------------------------------------------------------
struct Software: sc_core::sc_module {
    sc_in<bool> clk, rst_n;
    DeviceTlm * p_dev; HostMem * p_mem;
    sc_in<sc_uint<32>> i_cqe_cnt;                 // 设备完成计数(等完成用)✓
    sc_in<sc_uint<32>> i_br_ostd;                 // ★ 桥侧在飞高水位(判据只认它)✓
    nic_io_t io;
    int nfail = 0;
    bool done = false;
    uint32_t ostd_a = 0, ostd_b = 0;              // ⓪/① 两读数(打印用)✓

    SC_HAS_PROCESS(Software);
    Software(sc_core::sc_module_name nm): sc_core::sc_module(nm) { SC_THREAD(run); }

    static nic_u32 md_rd32(void *ctx, nic_u32 off) {
        DeviceTlm * d = (DeviceTlm *)ctx; sc_uint<32> v = 0;
        (void)d->reg_read((sc_dt::uint64)NIC_BASE + off, v);
        return v.to_uint();
    }
    static void md_wr32(void *ctx, nic_u32 off, nic_u32 val) {
        DeviceTlm * d = (DeviceTlm *)ctx;
        (void)d->reg_write((sc_dt::uint64)NIC_BASE + off, sc_uint<32>(val));
    }

    void tick() { sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME); }
    void chk(const char * n, uint32_t got, uint32_t exp) {
        if (got != exp) { printf("   x %s:got 0x%08x exp 0x%08x\n", n, got, exp); nfail++; }
        else printf("   v %s = 0x%08x\n", n, got);
    }
    void wait_cqe(uint32_t n) {
        int spins = 0;
        while (i_cqe_cnt.read().to_uint() < n && spins < 20000) { tick(); spins++; }
        if (spins >= 20000) { nfail++; printf("   x 等完成(cqe>=%u)超时\n", n); }
    }
    // 装一行源图案 ✓
    void mk_src(sc_dt::uint64 ba, int job) {
        unsigned char * sl = p_mem->line_ptr(ba);
        for (int i = 0; i < 8; i++) {
            uint64_t w = pat_word(job, i);
            for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
        }
    }
    void mk_wqe(sc_dt::uint64 sq_ba, uint32_t pi, nic_u32 wrid, sc_dt::uint64 src, sc_dt::uint64 dst) {
        unsigned char * wq = p_mem->line_ptr(sq_ba) +
                             (size_t)nic_ring_idx(pi, DEPTH_SQ) * NIC_WQE_BYTES;
        memset(wq, 0, NIC_WQE_BYTES);
        nic_wqe_build(wq, wrid, NIC_OPC_WRITE, src, dst, 64);
    }

    void run() {
        io.ctx = p_dev; io.rd32 = md_rd32; io.wr32 = md_wr32;
        tick();
        while (!rst_n.read()) tick();

        // 初始化:中断 + 双 QP 配置 ✓
        nic_intr_enable(&io, NIC_INTR_WQE_CMPL | NIC_INTR_RNR_NAK);
        nic_qp_cfg_t c1;
        c1.qp_idx = QPIX; c1.sq_ba = (uint32_t)SQ_BA; c1.cq_ba = (uint32_t)CQ_BA;
        c1.rq_ba = (uint32_t)SRC_BA; c1.sq_depth = DEPTH_SQ; c1.rq_depth = DEPTH_SQ;
        c1.rq_buf_sz = 1; c1.pmtu = 1;
        nic_qp_init(&io, &c1);
        nic_qp_cfg_t c2;
        c2.qp_idx = QPIX2; c2.sq_ba = (uint32_t)SQ2_BA; c2.cq_ba = (uint32_t)CQ2_BA;
        c2.rq_ba = (uint32_t)SRC2_BA; c2.sq_depth = DEPTH_SQ; c2.rq_depth = DEPTH_SQ;
        c2.rq_buf_sz = 1; c2.pmtu = 1;
        nic_qp_init(&io, &c2);

        // ⓪ 下界对照:QP2 **单笔** ⇒ 桥水位应 == 1 ✓
        mk_src(SRC_BA, 0);
        mk_wqe(SQ_BA, 1, 0x8C0, SRC_BA, DST_BA);
        nic_doorbell_sq(&io, QPIX, 1);
        wait_cqe(1);
        ostd_a = i_br_ostd.read().to_uint();
        chk("⓪下界对照:单笔 ⇒ 桥 o_rd_ostd_max == 1", ostd_a, 1);

        // ① 并发:QP2+QP3 **各 1 笔同拍投** ⇒ 桥水位应 > 1 ✓
        mk_src(SRC_BA + 64, 1);
        mk_wqe(SQ_BA, 2, 0x8C1, SRC_BA + 64, DST_BA + 64);
        mk_src(SRC2_BA, 2);
        mk_wqe(SQ2_BA, 1, 0x8D0, SRC2_BA, DST2_BA);
        nic_doorbell_sq(&io, QPIX, 2);
        nic_doorbell_sq(&io, QPIX2, 1);
        wait_cqe(3);
        ostd_b = i_br_ostd.read().to_uint();
        chk("①并发:双 QP ⇒ 桥 o_rd_ostd_max > 1", (ostd_b > 1) ? 1u : 0u, 1u);
        printf("   [读数] 桥 o_rd_ostd_max:单笔=%u / 并发=%u\n", ostd_a, ostd_b);

        // ③ 抽检:3 笔 CQE wrid + 写数据逐字节(双 QP 各自 CQ)✓
        int cqe_bad = 0, dat_bad = 0;
        for (int j = 0; j < 2; j++) {
            const nic_u8 * cqe = nic_cqe_at_ptr(p_mem->line_ptr(CQ_BA), DEPTH_SQ, (uint32_t)(j + 1));
            if (NIC_CQE_WRID(cqe) != (nic_u32)(0x8C0 + j)) cqe_bad++;
            unsigned char * s = p_mem->line_ptr(SRC_BA + (sc_dt::uint64)j * 64);
            unsigned char * d = p_mem->line_ptr(DST_BA + (sc_dt::uint64)j * 64);
            for (int i = 0; i < 64; i++) if (d[i] != s[i]) dat_bad++;
        }
        const nic_u8 * cqe3 = nic_cqe_at_ptr(p_mem->line_ptr(CQ2_BA), DEPTH_SQ, 1);
        if (NIC_CQE_WRID(cqe3) != (nic_u32)0x8D0) cqe_bad++;
        {
            unsigned char * s = p_mem->line_ptr(SRC2_BA);
            unsigned char * d = p_mem->line_ptr(DST2_BA);
            for (int i = 0; i < 64; i++) if (d[i] != s[i]) dat_bad++;
        }
        chk("③抽检:3 笔 CQE wrid 逐笔", (uint32_t)cqe_bad, 0);
        chk("③抽检:3 笔写数据 64B 逐字节", (uint32_t)dat_bad, 0);
        done = true;
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n("rst_n");

    DeviceTlm  dev("dev");
    DummyAmbaMaster dum("dum");
    dev.reg_s.bind(dum.m);

    // 设备 MAC 面向 + 观测哑绑 ✓
    sc_core::sc_signal<bool> v7B1, v7B2, v7B3, v7B4, v7Bin;
    sc_core::sc_signal<sc_biguint<2048>> v721_o, v721_in;
    sc_core::sc_signal<sc_biguint<256>>  v7256_o, v7256_in;
    sc_core::sc_signal<sc_uint<32>> v7U1, v7U2;
    dev.cmac_m_axis_tvalid(v7B1); dev.cmac_m_axis_tdata(v721_o); dev.cmac_m_axis_tkeep(v7256_o);
    dev.cmac_m_axis_tlast(v7B2);  dev.cmac_m_axis_tready(v7Bin);
    dev.roce_cmac_s_axis_tvalid(v7Bin); dev.roce_cmac_s_axis_tdata(v721_in);
    dev.roce_cmac_s_axis_tkeep(v7256_in); dev.roce_cmac_s_axis_tlast(v7Bin); dev.roce_cmac_s_axis_tuser(v7Bin);
    dev.o_mac_tx_frames(v7U1); dev.o_mac_rx_frames(v7U2);
    sc_core::sc_signal<sc_uint<32>> v7L1, v7L2, v7L3, v7L4;
    dev.o_line_tx_pkts(v7L1); dev.o_line_rx_acks(v7L2);
    dev.o_line_unexp_cnt(v7L3); dev.o_line_big_cnt(v7L4);

    Cxl2AceBridge br("br");
    HostMem    mem("mem");
    Software   sw("sw");

    dev.clk(clk); dev.rst_n(rst_n);
    br.clk(clk);  br.rst_n(rst_n);
    sw.clk(clk);  sw.rst_n(rst_n);
    sw.p_dev = &dev; sw.p_mem = &mem;

    // 设备 ↔ 桥:逐条显式信号(同 driver_flow)✓
    sc_core::sc_signal<bool> d_r0v, d_r0r, d_r1v, d_r1r, d_dv, d_dr, d_rv_, d_rr_, h_rv, h_rr_, h_dv_;
    sc_core::sc_signal<bool> h_qrdy, h_srdy, h_drdy;
    sc_core::sc_signal<sc_uint<5>>  d_r0op, d_r1op, d_rop;
    sc_core::sc_signal<sc_uint<4>>  h_rop;
    sc_core::sc_signal<sc_uint<3>>  h_rop3;
    sc_core::sc_signal<sc_uint<12>> d_r0cq, d_r1cq, d_duq, d_ruq, h_rrq, h_rspd, h_dcq, h_dcq2;
    sc_core::sc_signal<sc_uint<46>> d_r0ad, d_r1ad, h_rad;
    sc_core::sc_signal<bool> d_dcv, h_dcv;
    sc_core::sc_signal<sc_biguint<512>> d_dpl, h_dpl;
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

    // 桥 → 检查器 → 主机内存 ✓
    amba_pv::amba_pv_ace_protocol_checker<512> chk("chk");
    br.ace_m.bind(chk.amba_pv_s);
    chk.amba_pv_m.bind(mem.ace_s);

    // 观测:设备 + ★ 桥侧四槽号口/水位/err(★ 判据只认桥侧)✓
    sc_core::sc_signal<sc_uint<32>> o_db_sq, o_db_rq, o_ro, o_wqe, o_cqe, o_drop, o_ostd, o_act, o_goerr;
    sc_core::sc_signal<sc_uint<32>> o_rnr, o_simpl, o_snp, o_qpf;
    sc_core::sc_signal<sc_uint<32>> br_rd, br_wr, br_err, br_snp, br_ostd;
    sc_core::sc_signal<sc_uint<12>> b1, b2, b3, b4;
    dev.o_db_sq_cnt(o_db_sq); dev.o_db_rq_cnt(o_db_rq); dev.o_ro_wr_cnt(o_ro);
    dev.o_wqe_cnt(o_wqe); dev.o_cqe_cnt(o_cqe); dev.o_db_drop_cnt(o_drop);
    dev.o_max_rd_ostd(o_ostd); dev.o_max_act_per_qp(o_act); dev.o_goerr_cnt(o_goerr);
    dev.o_rnr_cnt(o_rnr); dev.o_opc_simpl_cnt(o_simpl); dev.o_snp_held_cnt(o_snp);
    dev.o_qp_fatal_cnt(o_qpf);
    sc_core::sc_signal<bool> s_intr; dev.nic_intr(s_intr);
    br.o_rd_done(br_rd); br.o_wr_done(br_wr); br.o_err(br_err); br.o_snoop_cnt(br_snp);
    br.o_rd_ostd_max(br_ostd); br.o_fst_rd_cqid(b1); br.o_fst_wr_cqid(b2);
    br.o_rd_cqid_max(b3); br.o_wr_cqid_min(b4);
    sw.i_cqe_cnt(o_cqe); sw.i_br_ostd(br_ostd);

    printf("============================================================\n");
    printf("tb_800g_ostd — ★ L4-4 多笔在飞(桥侧口径)+ 槽号(单笔下界对照)\n");
    printf("============================================================\n");

    rst_n.write(false);
    sc_core::sc_start(10, sc_core::SC_NS);
    rst_n.write(true);
    sc_core::sc_start(200, sc_core::SC_US);

    // ② ★ 判据只认桥侧:槽号四口 + err **全读桥侧** ✓
    printf("   [桥槽号] 首读 cqid=%u(应=0)/ 首写 cqid=%u(应=64)/ 读 max=%u(应<=63)/ 写 min=%u(应>=64)\n",
           b1.read().to_uint(), b2.read().to_uint(), b3.read().to_uint(), b4.read().to_uint());
    printf("   [桥读数] rd_done=%u wr_done=%u err=%u | [内存] rd=%u wr=%u bad=%u\n",
           br_rd.read().to_uint(), br_wr.read().to_uint(), br_err.read().to_uint(),
           mem.rd_n, mem.wr_n, mem.bad);
    bool slot_ok = (b1.read().to_uint() == 0) && (b2.read().to_uint() == 64) &&
                   (b3.read().to_uint() <= 63) && (b4.read().to_uint() >= 64) &&
                   (br_err.read() == 0);
    if (!slot_ok) printf("   x [判据②] 桥侧槽号/err 不符\n");
    int nf = sw.nfail + (slot_ok ? 0 : 1);
    bool pass = (nf == 0) && sw.done;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_800G_OSTD %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
