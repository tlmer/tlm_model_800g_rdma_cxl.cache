//============================================================================
// tb_800g_bpwm.cpp — 800G 专项 ★ L4-3:**背压 / 水位 / 丢弃(计数不静默)**  [2026-10-09 建]
//
// 依据 = `md/L4_背压与多笔在飞_TB设计.md`(纸面,判据**预登记**;读数出后入 当前状态.md)✓
// 拓扑 = `tb_driver_flow.cpp` **同源**(设备 ── pin CXL.cache ── 桥 ── ACE 检查器 ── 主机内存)✓
//
// 用例(判据 = 验证册 §2-L4-3 行细化):
//   ① 正控(高负载水位):**双 QP(QP2/QP3)各连投 2 笔**(4 笔在飞,不等完成)⇒ 判据:
//      `o_max_rd_ostd > 1`(水位)∧ `o_db_drop_cnt == 0`(不丢)∧ `o_cqe_cnt == 4` ∧
//      4 笔 CQE wrid 逐笔 + 写数据逐字节(双 QP 各自 CQ)+ 桥 `o_rd_ostd_max > 1`(读数入册)✓
//      ★ 首跑实据(2026-10-09):**单 QP 保序 ⇒ 水位恒 1**(设备"同 QP 在飞 ≤ 1 job")
//        ⇒ 按设计册 §5 风险②预案改**双 QP 并发**(设备许可"别的 QP 先跑";桥侧计"队列+在飞")✓
//   ② 负控(人为压满 ⇒ 丢弃可见):**零时间连敲 SQ 门铃 DBDEPTH+3 = 11 次** ⇒
//      `o_db_drop_cnt` 增量 == **3**(★ 容量精确 DBDEPTH=8;**不静默**)✓
//   ⚠ 负控在正控判据**捕获之后**才做(8 条入队门铃会重放 stale WQE 的副作用,不污染已捕获值)✓
//   ⚠ 速率档:本线默认 `LM_800G`(mac_tlm v1.0 默认;判据只看计数/水位,不看绝对时序)✓
//============================================================================
#include <systemc.h>
#include <cstring>
#include <amba_pv.h>
#include <models/amba_pv_ace_protocol_checker.h>
#include "device_tlm.h"
#include "dummy_amba_master.h"   // ★ 占位主端(reg_s 必绑)✓
#include "cxl2ace_bridge.h"
#include "nic_regs.h"      // ★ sw/ 共同基准(地址/位域/WQE·CQE 布局)✓
#include "nic_hw.h"        // ★ 序列层(与内核驱动**同一套**)✓

// ---- 内存布局(本 TB 的"主机 DDR";4MB 内、64B 行互不撞)✓ ----
static const sc_dt::uint64 SQ_BA  = 0x00200000, CQ_BA  = 0x00210000;
static const sc_dt::uint64 SRC_BA = 0x00220000, DST_BA = 0x00230000;
static const uint32_t QPIX = 2, QPIX2 = 3, DEPTH_SQ = 4;   // ★ 双 QP:并发 = 水位来源 ✓
// ★ 首跑实据(2026-10-09):**单 QP 保序 ⇒ 水位恒 1**(设备同 QP 在飞 ≤ 1 job)⇒ 按设计册 §5 风险②
//   预案改**双 QP 并发**(设备许可"别的 QP 先跑";桥侧 o_rd_ostd_max 计"队列+在飞")✓
static const sc_dt::uint64 SQ2_BA = 0x00260000, CQ2_BA = 0x00270000;
static const sc_dt::uint64 SRC2_BA = 0x00280000, DST2_BA = 0x00290000;

static uint64_t pat_word(int job, int i) { return 0x8A0A0000ULL + ((uint64_t)job << 16) + (uint64_t)(i * 0x0111); }

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
// 驱动流程(L4-3 版:满环连投 + 门铃压满负控)✓
//----------------------------------------------------------------------------
struct Software: sc_core::sc_module {
    sc_in<bool> clk, rst_n;
    DeviceTlm * p_dev; HostMem * p_mem;
    sc_in<sc_uint<32>> i_cqe_cnt, i_db_drop, i_max_rd_ostd;   // 设备观测口(读数用)✓
    nic_io_t io;
    int nfail = 0;
    bool done = false;
    // 正控捕获值(负控副作用不污染)✓
    uint32_t cap_ostd = 0, cap_drop = 0, cap_cqe = 0;

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

    void run() {
        io.ctx = p_dev; io.rd32 = md_rd32; io.wr32 = md_wr32;
        tick();
        while (!rst_n.read()) tick();

        // ① probe 式:开中断;② QP 配置(序列同 driver_flow)✓
        nic_intr_enable(&io, NIC_INTR_WQE_CMPL | NIC_INTR_RNR_NAK);
        nic_qp_cfg_t cfg;
        cfg.qp_idx = QPIX; cfg.sq_ba = (uint32_t)SQ_BA; cfg.cq_ba = (uint32_t)CQ_BA;
        cfg.rq_ba = (uint32_t)SRC_BA; cfg.sq_depth = DEPTH_SQ; cfg.rq_depth = DEPTH_SQ;
        cfg.rq_buf_sz = 1; cfg.pmtu = 1;
        nic_qp_init(&io, &cfg);
        nic_qp_cfg_t cfg2;
        cfg2.qp_idx = QPIX2; cfg2.sq_ba = (uint32_t)SQ2_BA; cfg2.cq_ba = (uint32_t)CQ2_BA;
        cfg2.rq_ba = (uint32_t)SRC2_BA; cfg2.sq_depth = DEPTH_SQ; cfg2.rq_depth = DEPTH_SQ;
        cfg2.rq_buf_sz = 1; cfg2.pmtu = 1;
        nic_qp_init(&io, &cfg2);

        // ③ ★ 双 QP × 2 笔 WRITE(非均匀图案;各 QP 独立缓冲、行互不撞)✓
        for (int j = 0; j < 2; j++) {
            unsigned char * sl = p_mem->line_ptr(SRC_BA + (sc_dt::uint64)j * 64);
            for (int i = 0; i < 8; i++) {
                uint64_t w = pat_word(j, i);
                for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
            }
            unsigned char * wq = p_mem->line_ptr(SQ_BA) +
                                 (size_t)nic_ring_idx((uint32_t)(j + 1), DEPTH_SQ) * NIC_WQE_BYTES;
            memset(wq, 0, NIC_WQE_BYTES);
            nic_wqe_build(wq, (nic_u32)(0x8A0 + j), NIC_OPC_WRITE,
                           SRC_BA + (sc_dt::uint64)j * 64, DST_BA + (sc_dt::uint64)j * 64, 64);
            unsigned char * sl2 = p_mem->line_ptr(SRC2_BA + (sc_dt::uint64)j * 64);
            for (int i = 0; i < 8; i++) {
                uint64_t w = pat_word(j + 4, i);
                for (int k = 0; k < 8; k++) sl2[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
            }
            unsigned char * wq2 = p_mem->line_ptr(SQ2_BA) +
                                  (size_t)nic_ring_idx((uint32_t)(j + 1), DEPTH_SQ) * NIC_WQE_BYTES;
            memset(wq2, 0, NIC_WQE_BYTES);
            nic_wqe_build(wq2, (nic_u32)(0x8B0 + j), NIC_OPC_WRITE,
                           SRC2_BA + (sc_dt::uint64)j * 64, DST2_BA + (sc_dt::uint64)j * 64, 64);
        }
        // ④ **双 QP 连投门铃**(不等完成;零时间入队 ⇒ 双 job 并发)✓
        for (uint32_t pi = 1; pi <= 2; pi++) nic_doorbell_sq(&io, QPIX, pi);
        for (uint32_t pi = 1; pi <= 2; pi++) nic_doorbell_sq(&io, QPIX2, pi);

        // ⑤ 等 4 笔全完(轮询设备观测口;有界)✓
        int spins = 0;
        while (i_cqe_cnt.read().to_uint() < 4 && spins < 20000) { tick(); spins++; }
        if (spins >= 20000) { nfail++; printf("   x 等 4 笔完成超时(cqe=%u)\n", i_cqe_cnt.read().to_uint()); }

        // ⑥ 正控判据(★ 先捕获读数,再做负控)✓
        cap_ostd = i_max_rd_ostd.read().to_uint();
        cap_drop = i_db_drop.read().to_uint();
        cap_cqe  = i_cqe_cnt.read().to_uint();
        chk("①正控:o_cqe_cnt == 4(双 QP 全完成)", cap_cqe, 4u);
        chk("①正控:o_max_rd_ostd > 1(水位;双 QP 并发)", (cap_ostd > 1) ? 1u : 0u, 1u);
        chk("①正控:o_db_drop_cnt == 0(高负载不丢)", cap_drop, 0);
        printf("   [读数] o_max_rd_ostd=%u / o_db_drop_cnt=%u / o_cqe_cnt=%u\n",
               cap_ostd, cap_drop, cap_cqe);
        // 逐笔 CQE wrid + 写数据逐字节(双 QP 各自 CQ)✓
        int cqe_bad = 0, dat_bad = 0;
        for (int j = 0; j < 2; j++) {
            const nic_u8 * cqe = nic_cqe_at_ptr(p_mem->line_ptr(CQ_BA), DEPTH_SQ, (uint32_t)(j + 1));
            if (NIC_CQE_WRID(cqe) != (nic_u32)(0x8A0 + j)) cqe_bad++;
            unsigned char * s = p_mem->line_ptr(SRC_BA + (sc_dt::uint64)j * 64);
            unsigned char * d = p_mem->line_ptr(DST_BA + (sc_dt::uint64)j * 64);
            for (int i = 0; i < 64; i++) if (d[i] != s[i]) dat_bad++;
            const nic_u8 * cqe2 = nic_cqe_at_ptr(p_mem->line_ptr(CQ2_BA), DEPTH_SQ, (uint32_t)(j + 1));
            if (NIC_CQE_WRID(cqe2) != (nic_u32)(0x8B0 + j)) cqe_bad++;
            unsigned char * s2 = p_mem->line_ptr(SRC2_BA + (sc_dt::uint64)j * 64);
            unsigned char * d2 = p_mem->line_ptr(DST2_BA + (sc_dt::uint64)j * 64);
            for (int i = 0; i < 64; i++) if (d2[i] != s2[i]) dat_bad++;
        }
        chk("①正控:4 笔 CQE wrid 逐笔(双 QP)", (uint32_t)cqe_bad, 0);
        chk("①正控:4 笔写数据 64B 逐字节(双 QP)", (uint32_t)dat_bad, 0);

        // ⑦ 负控:零时间**连敲 11 次** SQ 门铃(pi=4,已消费 ⇒ 数据面无新增)⇒ 丢弃 == 3 ✓
        for (int k = 0; k < 11; k++) nic_doorbell_sq(&io, QPIX, DEPTH_SQ);
        for (int k = 0; k < 8; k++) tick();
        uint32_t d_after = i_db_drop.read().to_uint();
        chk("②负控:连敲 11 次 ⇒ 丢弃 == 3(容量精确 DBDEPTH=8;不静默)", d_after - cap_drop, 3);
        printf("   [读数] 负控后 o_db_drop_cnt=%u(基线 %u)\n", d_after, cap_drop);
        done = true;
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n("rst_n");

    DeviceTlm  dev("dev");
    DummyAmbaMaster dum("dum");                 // ★ 占位主端(reg_s 必绑)✓
    dev.reg_s.bind(dum.m);

    // 设备 MAC 面向 + 观测哑绑(⚠ E115:每个 out 独立)✓
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

    // 观测:设备口 → 命名信号(sw 读数 + 终判打印)✓
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
    sw.i_cqe_cnt(o_cqe); sw.i_db_drop(o_drop); sw.i_max_rd_ostd(o_ostd);

    printf("============================================================\n");
    printf("tb_800g_bpwm — ★ L4-3 背压/水位/丢弃(计数不静默;正控+负控)\n");
    printf("============================================================\n");

    rst_n.write(false);
    sc_core::sc_start(10, sc_core::SC_NS);
    rst_n.write(true);
    sc_core::sc_start(200, sc_core::SC_US);

    printf("   [桥读数]rd_done=%u wr_done=%u err=%u rd_ostd_max=%u | [内存]rd=%u wr=%u bad=%u\n",
           br_rd.read().to_uint(), br_wr.read().to_uint(), br_err.read().to_uint(),
           br_ostd.read().to_uint(), mem.rd_n, mem.wr_n, mem.bad);
    bool br_ok = (br_rd.read() >= DEPTH_SQ) && (br_err.read() == 0) && (br_ostd.read().to_uint() > 1);
    if (!br_ok) printf("   x [判据] 桥侧读数不符(期望 rd>=%u ∧ err==0 ∧ rd_ostd_max>1)\n", DEPTH_SQ);
    int nf = sw.nfail + (br_ok ? 0 : 1);
    bool pass = (nf == 0) && sw.done;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_800G_BPWM %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
