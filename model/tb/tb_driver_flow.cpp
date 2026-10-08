//============================================================================
// tb_driver_flow.cpp — 【驱动流程自测:拿模型当"硬件"】  [2026-10-04 建]
//
// ★ 目的(P4.3 的"模型侧一半"):**驱动要跑的寄存器/门铃/中断序列表**,在 Linux 还不存在时
//   就能验 —— 序列代码 = `sw/nic_hw.h` + `sw/nic_regs.h`(**与内核驱动骨架同一套**),
//   只把两个原语换掉:
//     · 内核:  readl/writel(ioremap 的 2MB 窗)       ⇒ 见 `sw/nic_drv.c`(已用本机内核树编译过)
//     · 本 TB: DeviceTlm::reg_read / reg_write(**直连方法**)
//   ⚠ **寄存器传输面 = A7 未定**(直连是占位):本 TB 验的是**序列语义**(写什么/读什么/什么次序),
//     不是通路形态 —— 通路定了只要换原语,序列不动 ✓
//
// 拓扑(与 tb_device_e2e 同源,取最小):设备 ── pin CXL.cache ── 桥 ── ACE 检查器 ── 主机内存
// 用例(driver-like 顺序):
//   ① probe 式:开中断(INTR_EN)② `nic_qp_init()` 配 QP ③ `nic_wqe_build()` 装 WQE
//   ④ `nic_doorbell_sq()` 敲门铃 ⑤ **中断驱动完成**:轮询 INTR_STS → **CQ_INTR_STS1 掩码应答**
//      (★ /D14 对齐:写全局 INTR_STS[4] **不清** CQ 中断 —— 照 RTL 死路)⇒ 抓线落 ✓
//   ⑥ CQE 轮询(内存)⑦ WRITE 数据比对 ⑧ 再来一笔 SEND ⇒ `nic_stat_rq_pi()` 回读 + RQ 项数据 ✓
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
static const sc_dt::uint64 RQ_BA  = 0x00240000, SSRC_BA = 0x00250000;
static const uint32_t QPIX = 2, DEPTH_SQ = 4, DEPTH_RQ = 4;   // ★ depth=4 ⇒ 测**指针回绕**
//   (SQ:pi=4 → WQE@索引0;CQ:hd=4 → 索引0;★ **CQ 深度 = SQ 深度** 照 RTL)✓

static uint64_t patW_word(int i) { return 0x51510000ULL + (uint64_t)(i * 0x1111); }
static uint64_t patS_word(int i) { return 0x52520000ULL + (uint64_t)(i * 0x2222); }

//----------------------------------------------------------------------------
// 主机内存(ACE 从端;与 e2e 同款最小版)✓
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
// ★ 驱动流程:全部经 `nic_hw.h`(内核驱动同款序列)✓
//----------------------------------------------------------------------------
struct Software: sc_core::sc_module {
    sc_in<bool> clk, rst_n;
    DeviceTlm * p_dev; HostMem * p_mem;
    sc_in<bool> s_intr;                       // 中断线(观测"线落")
    nic_io_t io;                             // ★ 原语槽:内核=readl/writel;这里=模型直连
    int nfail = 0, nwr_fail = 0;
    bool done = false;

    SC_HAS_PROCESS(Software);
    Software(sc_core::sc_module_name nm): sc_core::sc_module(nm) { SC_THREAD(run); }

    // ---- 原语(模型直连;**只有这两行**是"平台相关",其余序列与内核驱动完全同码)✓ ----
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
    int cmp_line(sc_dt::uint64 mem_ba, sc_dt::uint64 src_ba) {
        unsigned char * d = p_mem->line_ptr(mem_ba), * s2 = p_mem->line_ptr(src_ba);
        int bad = 0;
        for (int i = 0; i < 64; i++) if (d[i] != s2[i]) bad++;
        return bad;
    }
    void chk(const char * n, uint32_t got, uint32_t exp) {
        if (got != exp) { printf("   x %s:got 0x%08x exp 0x%08x\n", n, got, exp); nfail++; }
        else printf("   v %s = 0x%08x\n", n, got);
    }
    // ★ 中断驱动完成(★ /D14 对齐后):非 CQ 位 = W1C;★ CQ 位(bit4)= 写 `CQ_INTR_STS1` 掩码清;
    //   ⚠ 写全局 INTR_STS[4] **不清**(照 RTL 死路)—— 本函数顺带做该语义判据 ✓
    uint32_t wait_intr(uint32_t bit, uint32_t cq_mask) {
        for (int i = 0; i < 4000; i++) {
            tick();
            nic_u32 sts = nic_intr_status(&io);
            if (sts & bit) {
                nic_intr_ack(&io, sts & ~NIC_INTR_WQE_CMPL);        // 非 CQ 位:W1C ✓
                if (sts & NIC_INTR_WQE_CMPL) {
                    nic_intr_ack(&io, NIC_INTR_WQE_CMPL);          // ★ 先试全局 W1C(应**无效**)
                    for (int k = 0; k < 2; k++) tick();
                    uint32_t still = nic_intr_status(&io);
                    chk("D14: 写全局 INTR_STS[4] 不清 CQ 中断", (still & NIC_INTR_WQE_CMPL) ? 1u : 0u, 1u);
                    nic_cq_intr_ack(&io, cq_mask);                     // ★ 真清路:CQ_INTR_STS1 掩码 ✓
                }
                for (int k = 0; k < 4; k++) tick();      // 让线/状态落定 ✓
                return sts;
            }
        }
        nfail++; printf("   x 等中断 bit0x%x 超时\n", bit);
        return 0;
    }

    void run() {
        io.ctx = p_dev; io.rd32 = md_rd32; io.wr32 = md_wr32;     // ★ 原语注入(内核换 readl/writel)
        tick();
        while (!rst_n.read()) tick();                              // 等复位释放 ✓

        // ① probe 式:开中断(§10.3 复位全屏蔽)✓
        nic_intr_enable(&io, NIC_INTR_WQE_CMPL | NIC_INTR_RNR_NAK);
        // ② QP 配置(序列 = nic_qp_init;字段全用寄存器原生布局)✓
        nic_qp_cfg_t cfg;
        cfg.qp_idx = QPIX; cfg.sq_ba = (uint32_t)SQ_BA; cfg.cq_ba = (uint32_t)CQ_BA;
        cfg.rq_ba = (uint32_t)RQ_BA; cfg.sq_depth = DEPTH_SQ; cfg.rq_depth = DEPTH_RQ;
        cfg.rq_buf_sz = 1;                     // 1 × 256B ✓
        cfg.pmtu = 1;
        nic_qp_init(&io, &cfg);
        nic_doorbell_rq(&io, QPIX, 1);        // RQ_CI_DB 初始化:空环 ci == wrptr ✓
        chk("QP 配置回读(Q_DEPTH)", nic_rd(&io, NIC_QP(QPIX) + NIC_QP_Q_DEPTH), (DEPTH_RQ << 16) | DEPTH_SQ);
        chk("QP 配置回读(QP_EN 置位)", nic_rd(&io, NIC_QP(QPIX) + NIC_QP_CONF) & NIC_QPC_QP_EN, 1);

        // ③④ WRITE 一笔:装 WQE(§9.1 布局宏)⇒ 敲门铃(§8.5)✓
        for (int i = 0; i < 8; i++) {          // 源行图案(非均匀)✓
            unsigned char * sl = p_mem->line_ptr(SRC_BA);
            uint64_t w = patW_word(i);
            for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
        }
        unsigned char * wq = p_mem->line_ptr(SQ_BA + 64);           // idx = pi = 1 ✓
        memset(wq, 0, NIC_WQE_BYTES);
        nic_wqe_build(wq, 0x77AA, NIC_OPC_WRITE, SRC_BA, DST_BA, 64);
        nic_doorbell_sq(&io, QPIX, 1);
        // ⑤ 中断驱动完成:等 CQ 完成位 ⇒ W1C 应答 ⇒ 抓线落 ✓
        uint32_t sts1 = wait_intr(NIC_INTR_WQE_CMPL, 1u << QPIX);
        chk("①中断:CQ 完成位已置", (sts1 & NIC_INTR_WQE_CMPL) ? 1 : 0, 1);
        chk("①中断:CQ_INTR_STS1 应答后线落下", s_intr.read() ? 1 : 0, 0);
        chk("①中断:STS 该位已清", nic_intr_status(&io) & NIC_INTR_WQE_CMPL, 0);
        chk("D14: CQ_INTR_STS1 回读已清", nic_rd(&io, NIC_GLB_CQ_INTR_STS(1)), 0);
        // ⑥⑦ CQE + 数据(驱动看自己的内存)✓
        const nic_u8 * cqe = nic_cqe_at(p_mem->line_ptr(CQ_BA), 1);   // hdptr 1-based ⇒ idx 1 ✓
        chk("①CQE wrid", NIC_CQE_WRID(cqe), 0x77AA);
        chk("①CQE opcode(WRITE 原样)", NIC_CQE_OPCODE(cqe), NIC_OPC_WRITE);
        chk("①CQE fatal", NIC_CQE_FATAL(cqe), 0);
        int bad = 0;
        for (int i = 0; i < 64; i++) if (p_mem->line_ptr(DST_BA)[i] != p_mem->line_ptr(SRC_BA)[i]) bad++;
        chk("①WRITE 数据 64B 逐字节", (uint32_t)bad, 0);

        // ⑧ SEND 一笔(0x02;目标 = 本 QP 的 RQ 环;§4.4 口径)✓
        for (int i = 0; i < 8; i++) {
            unsigned char * sl = p_mem->line_ptr(SSRC_BA);
            uint64_t w = patS_word(i);
            for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
        }
        // ★ 指针口径(照 §9.2):WQE 位置 = SQ_BA + **idx(pi, depth)*64**;pi=4 ⇒ idx 0(回绕!)✓
        unsigned char * wq2 = p_mem->line_ptr(SQ_BA) +
                              (size_t)nic_ring_idx(2, DEPTH_SQ) * NIC_WQE_BYTES;
        memset(wq2, 0, NIC_WQE_BYTES);
        nic_wqe_build(wq2, 0x77BB, NIC_OPC_SEND, SSRC_BA, 0, 64);
        nic_sq_post(&io, QPIX, 2);
        uint32_t sts2 = wait_intr(NIC_INTR_WQE_CMPL, 1u << QPIX);
        chk("②中断:CQ 完成位(第二笔)", (sts2 & NIC_INTR_WQE_CMPL) ? 1 : 0, 1);
        const nic_u8 * cqe2 = nic_cqe_at(p_mem->line_ptr(CQ_BA), 2);
        chk("②CQE wrid", NIC_CQE_WRID(cqe2), 0x77BB);
        chk("②CQE opcode(SEND 原样)", NIC_CQE_OPCODE(cqe2), NIC_OPC_SEND);
        chk("②RQ 生产指针回读(STAT_RQ_PI_DB)", nic_stat_rq_pi(&io, QPIX), 2);
        // RQ 项地址照 §4.4:wrptr=1 ⇒ RQ_BUF_BA + 256 ✓
        int bad2 = 0;
        for (int i = 0; i < 64; i++)
            if (p_mem->line_ptr(RQ_BA + 256)[i] != p_mem->line_ptr(SSRC_BA)[i]) bad2++;
        chk("②SEND 数据落 RQ 项1(RQ_BA+256)逐字节", (uint32_t)bad2, 0);
        // ⑨ ★ 回绕相位:pi=3(WRITE)+ pi=4(SEND;**WQE 在 SQ_BA+0 = 索引 0**)⇒ 共 4 条 CQE ✓
        for (int i = 0; i < 8; i++) {
            unsigned char * sl = p_mem->line_ptr(SSRC_BA + 64);
            uint64_t w = 0x53530000ULL + (uint64_t)i;
            for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
        }
        unsigned char * wq3 = p_mem->line_ptr(SQ_BA) +
                              (size_t)nic_ring_idx(3, DEPTH_SQ) * NIC_WQE_BYTES;
        memset(wq3, 0, NIC_WQE_BYTES);
        nic_wqe_build(wq3, 0x77CC, NIC_OPC_SEND, SSRC_BA + 64, 0, 64);
        unsigned char * wq4 = p_mem->line_ptr(SQ_BA) +
                              (size_t)nic_ring_idx(4, DEPTH_SQ) * NIC_WQE_BYTES;   // ★ = SQ_BA + 0
        memset(wq4, 0, NIC_WQE_BYTES);
        nic_wqe_build(wq4, 0x77DD, NIC_OPC_SEND, SSRC_BA + 128, 0, 64);
        nic_sq_post(&io, QPIX, 3);
        wait_intr(NIC_INTR_WQE_CMPL, 1u << QPIX);
        nic_sq_post(&io, QPIX, 4);
        wait_intr(NIC_INTR_WQE_CMPL, 1u << QPIX);
        // 4 条 CQE:CQE#1..3 在 idx 1..3(=cq_ba+4/8/12);**CQE#4 回索引 0(=cq_ba+0)** ✓
        const nic_u8 * c0 = nic_cqe_at_ptr(p_mem->line_ptr(CQ_BA), DEPTH_SQ, 1);
        const nic_u8 * c4 = nic_cqe_at_ptr(p_mem->line_ptr(CQ_BA), DEPTH_SQ, 4);   // hd=4 → idx 0 ✓
        const nic_u8 * c3 = nic_cqe_at_ptr(p_mem->line_ptr(CQ_BA), DEPTH_SQ, 3);   // hd=3 → idx 3 ✓
        chk("③CQE1(索引1)wrid", NIC_CQE_WRID(c0), 0x77AA);
        chk("③CQE3(索引3)wrid", NIC_CQE_WRID(c3), 0x77CC);
        chk("③CQE4(**回绕索引0**)wrid", NIC_CQE_WRID(c4), 0x77DD);
        chk("③RQ 生产指针(3 笔 SEND 后)", nic_stat_rq_pi(&io, QPIX), 4);   // 1→2→3→4 ✓
        // 三笔 SEND 的落项(照 §4.4:wrptr=1/2/3 ⇒ RQ_BA+256 / +512 / +768)✓
        chk("③RQ 项2(RQ_BA+2*256)=第2笔源", (uint32_t)cmp_line(RQ_BA + 2 * 256, SSRC_BA + 64), 0);
        chk("③RQ 项3(RQ_BA+3*256)=第3笔源", (uint32_t)cmp_line(RQ_BA + 3 * 256, SSRC_BA + 128), 0);
        printf("   内存 rd=%u wr=%u bad=%u | 写失败=%d\n", p_mem->rd_n, p_mem->wr_n, p_mem->bad, nwr_fail);
        done = true;
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n("rst_n");

    DeviceTlm  dev("dev");
    DummyAmbaMaster dum("dum");                 // ★ 占位主端(reg_s 必绑;E109 的 socket 版)✓
    dev.reg_s.bind(dum.m);

    // ★ v1.0/v1.0 新口(/#27):MAC 面向 + 观测 —— 哑绑定(⚠ E115:每个 out 独立;inputs 共享)✓
    sc_core::sc_signal<bool> v7B1, v7B2, v7B3, v7B4, v7Bin;
    sc_core::sc_signal<sc_biguint<2048>> v721_o, v721_in;
    sc_core::sc_signal<sc_biguint<256>>  v7256_o, v7256_in;
    sc_core::sc_signal<sc_uint<32>> v7U1, v7U2;
    dev.cmac_m_axis_tvalid(v7B1); dev.cmac_m_axis_tdata(v721_o); dev.cmac_m_axis_tkeep(v7256_o);
    dev.cmac_m_axis_tlast(v7B2);  dev.cmac_m_axis_tready(v7Bin);
    dev.roce_cmac_s_axis_tvalid(v7Bin); dev.roce_cmac_s_axis_tdata(v721_in);
    dev.roce_cmac_s_axis_tkeep(v7256_in); dev.roce_cmac_s_axis_tlast(v7Bin); dev.roce_cmac_s_axis_tuser(v7Bin);
    dev.o_mac_tx_frames(v7U1); dev.o_mac_rx_frames(v7U2);
    sc_core::sc_signal<sc_uint<32>> v7L1, v7L2, v7L3, v7L4;   // ★ v1.0 线侧口 ✓
    dev.o_line_tx_pkts(v7L1); dev.o_line_rx_acks(v7L2);
    dev.o_line_unexp_cnt(v7L3); dev.o_line_big_cnt(v7L4);

    Cxl2AceBridge br("br");
    HostMem    mem("mem");
    Software   sw("sw");

    dev.clk(clk); dev.rst_n(rst_n);
    br.clk(clk);  br.rst_n(rst_n);
    sw.clk(clk);  sw.rst_n(rst_n);
    sw.p_dev = &dev; sw.p_mem = &mem;

    // 设备 ↔ 桥:逐条显式信号 ✓
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
    br.h2d_rsp_rsp_data(h_rspd); dev.h2d_rsp_rsp_data(h_rspd);   // ⚠ 别与 req_uqid 共用一条线(E115)✓
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

    // 观测:中断线 + 设备必须绑的观测口(未用的也显式绑;E109 教训)✓
    sc_core::sc_signal<bool> s_intr;
    sc_core::sc_signal<sc_uint<32>> o1, o2, o3, o4, o5, o6, o7, o8, o9;
    sc_core::sc_signal<sc_uint<12>> b1, b2, b3, b4, b5;
    dev.nic_intr(s_intr); sw.s_intr(s_intr);
    dev.o_db_sq_cnt(o1); dev.o_db_rq_cnt(o2); dev.o_ro_wr_cnt(o3);
    dev.o_wqe_cnt(o4); dev.o_cqe_cnt(o5); dev.o_db_drop_cnt(o6);
    dev.o_max_rd_ostd(o7); dev.o_max_act_per_qp(o8); dev.o_goerr_cnt(o9);
    sc_core::sc_signal<sc_uint<32>> o10, o11, o12;
    dev.o_rnr_cnt(o10); dev.o_opc_simpl_cnt(o11); dev.o_snp_held_cnt(o12);
    sc_core::sc_signal<sc_uint<32>> o13;                     // ★ v1.0 QP fatal 计数 ✓
    dev.o_qp_fatal_cnt(o13);
    sc_core::sc_signal<sc_uint<32>> br1, br2, br3, br4, br5;
    br.o_rd_done(br1); br.o_wr_done(br2); br.o_err(br3); br.o_snoop_cnt(br4); br.o_rd_ostd_max(br5);
    br.o_fst_rd_cqid(b1); br.o_fst_wr_cqid(b2); br.o_rd_cqid_max(b3); br.o_wr_cqid_min(b4);

    printf("============================================================\n");
    printf("tb_driver_flow — 驱动流程自测(序列 = sw/nic_hw.h;原语 = 模型直连)✓\n");
    printf("============================================================\n");

    rst_n.write(false);
    sc_core::sc_start(10, sc_core::SC_NS);
    rst_n.write(true);
    sc_core::sc_start(20, sc_core::SC_US);

    bool pass = (sw.nfail == 0) && sw.done;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_DRIVER_FLOW %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
