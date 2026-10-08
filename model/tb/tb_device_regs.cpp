//============================================================================
// tb_device_regs.cpp — 设备侧 TLM v1.0 寄存器面自测  [2026-10-04 建]
//
// 判据(逐条照 `规格文档` §8 的实测属性)✓:
//   ① RW:写读一致(per-QP QP_CONF / PD 段)
//   ② RO:先注入再读(**强判据**:若解码错会读成 0/别的值)、写它读回不变 + `o_ro_wr_cnt` 计数 ✓
//   ③ 门铃:RQ_CI_DB(0x0234)/SQ_PI_DB(0x0238)写 ⇒ 存储 + 计数 ✓
//   ④ 中断:INTR_EN & INTR_STS(W1C)—— 含**两条正控**(清别的位不清它;使能为 0 时不抬线)
//      ★ /D14:bit4 例外 —— 写全局 INTR_STS[4] **不清** CQ 中断(照 RTL 死路);
//      真清路 = 写 **CQ_INTR_STS1** 掩码(per-QP;本 TB 连 per-QP 掩码语义一并判)✓
//   ⑤ 段解码正控:PD 段写入不带坏邻字段、不串到 per-QP ✓
// ⚠ 仪器自检:每次比较都打印期望/实测(失败必现形)✓
//============================================================================
#include <systemc.h>
#include "device_tlm.h"
#include "dummy_amba_master.h"   // ★ 占位主端(reg_s 必绑)✓
#include "nic_regs.h"      // ★ sw/ 共同基准(带出处)—— 本 TB 用它 ⇒ **头文件与模型对拍** ✓

// 地址(软件视角;**全部由 `nic_regs.h` 推导**,不再写死)✓
// ★ 口径自检(编译期;任何一条不符 ⇒ 编不过):§8.1 的索引/别名/编码规则 ✓
static_assert(NIC_QP(2) == 0x50180200u, "per-QP:块 = 0x100 / 索引 = addr[15:8](§8.1)不符");
static_assert(NIC_QP_QPN(2) == 3u,      "厂商 QPn = 地址索引 + 1(§8.1)不符");
static_assert(NIC_GLB(NIC_INTR_EN) == 0x50100180u, "全局段基址/别名口径不符");
static_assert(NIC_PD(0) == 0x50000000u, "PD 段基址不符");
static const sc_dt::uint64 A_QP_CONF   = NIC_QP(2) + NIC_QP_CONF;
static const sc_dt::uint64 A_RQ_CI_DB  = NIC_QP(2) + NIC_QP_RQ_CI_DB;
static const sc_dt::uint64 A_SQ_PI_DB  = NIC_QP(2) + NIC_QP_SQ_PI_DB;
static const sc_dt::uint64 A_STAT_RQPI = NIC_QP(2) + NIC_QP_STAT_RQ_PI;
static const sc_dt::uint64 A_INTR_EN   = NIC_GLB(NIC_INTR_EN);
static const sc_dt::uint64 A_INTR_STS  = NIC_GLB(NIC_INTR_STS);
static const sc_dt::uint64 A_PD0_VA0   = NIC_PD(0) + NIC_PD_VIRT_ADDR0;
static const sc_dt::uint64 A_PD0_VA1   = NIC_PD(0) + NIC_PD_VIRT_ADDR1;

//----------------------------------------------------------------------------
// 寄存器主机:⚠ **直连方法**(寄存器访问**路径待定**,见 device_tlm.h 头注释/规格 §2.3)✗
//   —— 本轮验的是**语义**(RW/RO/W1C/门铃/中断),不是通路形态 ✓
//----------------------------------------------------------------------------
struct RegMaster: sc_core::sc_module {
    sc_in<bool> clk;
    DeviceTlm * p_dev;                       // 直连 + 白盒钩子(注入中断/STAT)⚠

    int ntests = 0, nfail = 0;
    bool done = false;
    sc_in<bool> s_intr;                              // 中断线
    sc_in<sc_uint<32>> s_db_sq, s_db_rq, s_ro_wr;    // 32b 计数(⚠ 别写成 bool)✓

    SC_HAS_PROCESS(RegMaster);
    RegMaster(sc_core::sc_module_name nm): sc_core::sc_module(nm) {
        SC_THREAD(run);
    }

    uint32_t rd(sc_dt::uint64 a) {
        sc_uint<32> v = 0;
        if (!p_dev->reg_read(a, v)) { printf("   ✗ 读 0x%llx 未实现\n", (unsigned long long)a); nfail++; }
        return v.to_uint();
    }

    bool rd_fail(sc_dt::uint64 a) {                  // 期望**未实现/非法**(§8.1 段/索引边界)✓
        sc_uint<32> v = 0;
        return !p_dev->reg_read(a, v);
    }

    void wr(sc_dt::uint64 a, uint32_t v) {
        if (!p_dev->reg_write(a, sc_uint<32>(v))) { printf("   ✗ 写 0x%llx 未实现\n", (unsigned long long)a); nfail++; }
    }

    void chk(const char * name, uint32_t got, uint32_t exp) {
        ntests++;
        if (got != exp) {
            printf("   ✗ [判据] %s:读到 0x%08x 期望 0x%08x\n", name, got, exp);
            nfail++;
        } else {
            printf("   ✓ %s = 0x%08x\n", name, got);
        }
    }

    void run() {
        sc_core::wait(sc_core::sc_time(20, sc_core::SC_NS));      // 复位落定 ✓

        // ---- ① RW:per-QP QP_CONF(含 PMTU 等位域,值非均匀以防"恒 0 假过")✓ ----
        wr(A_QP_CONF, 0xABC10305);           // ★ 各域**全非零**(QM_EN/RQ_BUF_SZ/MAX_RD_OS/PMTU)防"恒 0 假过"✓
        chk("①RW QP3 QP_CONF", rd(A_QP_CONF), 0xABC10305);

        // ---- ② RO:STAT_RQ_PI_DB(先注入 ⇒ 强判据)----
        p_dev->set_rq_pi(2, 0x1234);                              // 白盒钩子(真源=引擎,下轮)⚠
        chk("②RO STAT_RQ_PI_DB(注入后)", rd(A_STAT_RQPI), 0x00001234);
        wr(A_STAT_RQPI, 0xFFFF);                                  // 写 RO ⇒ 应被忽略 ✓
        chk("②RO 写后不变", rd(A_STAT_RQPI), 0x00001234);
        sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME);   // ⚠ 另等一 delta:让该拍写入走完 update 相(§同 slot 纪律)✓
        chk("②RO 写被计数", (uint32_t)s_ro_wr.read().to_uint(), 1);

        // ---- ③ 门铃:RQ_CI_DB / SQ_PI_DB(写=门铃:存储 + 计数)✓ ----
        wr(A_RQ_CI_DB, 0x0005);
        chk("③门铃 RQ_CI_DB 读回", rd(A_RQ_CI_DB), 0x0005);
        wr(A_SQ_PI_DB, 0x0007);
        chk("③门铃 SQ_PI_DB 读回", rd(A_SQ_PI_DB), 0x0007);
        sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME);   // ⚠ 另等一 delta:让该拍写入走完 update 相(§同 slot 纪律)✓
        chk("③RQ 门铃计数", (uint32_t)s_db_rq.read().to_uint(), 1);
        chk("③SQ 门铃计数", (uint32_t)s_db_sq.read().to_uint(), 1);

        // ---- ④ 中断:INTR_EN 掩蔽 + W1C 清位(**两条正控**)✓ ----
        chk("④INTR_EN 复位值(全屏蔽)", rd(A_INTR_EN), 0);
        p_dev->set_intr_status(4, true);                          // CQ 完成源(白盒注入)⚠
        sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME);   // ⚠ 另等一 delta:让该拍写入走完 update 相(§同 slot 纪律)✓
        chk("④未使能 ⇒ 线不抬(正控)", s_intr.read() ? 1 : 0, 0);
        wr(A_INTR_EN, NIC_INTR_WQE_CMPL);                     // 使能 bit4(宏 ⇒ 与头文件对拍)✓
        sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME);   // ⚠ 另等一 delta:让该拍写入走完 update 相(§同 slot 纪律)✓
        chk("④使能后 ⇒ 线抬起", s_intr.read() ? 1 : 0, 1);
        chk("④INTR_STS 读回 bit4", (rd(A_INTR_STS) >> 4) & 1, 1);
        wr(A_INTR_STS, NIC_INTR_BYPASS);                      // 清别的位(bit2)⇒ 不该清它 ✓
        sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME);   // ⚠ 另等一 delta:让该拍写入走完 update 相(§同 slot 纪律)✓
        chk("④清别的位不清它(正控)", s_intr.read() ? 1 : 0, 1);
        // ★ D14():写全局 INTR_STS[4] **不清** CQ 中断(照 RTL 死路)⇒ 真清路 = 写 CQ_INTR_STS1 掩码 ✓
        wr(A_INTR_STS, NIC_INTR_WQE_CMPL);
        sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME);   // ⚠ 另等一 delta:让该拍写入走完 update 相(§同 slot 纪律)✓
        chk("④D14: 写全局 INTR_STS[4] 不清 ⇒ 线仍高", s_intr.read() ? 1 : 0, 1);
        chk("④D14: INTR_STS 读回 bit4 仍为 1", (rd(A_INTR_STS) >> 4) & 1, 1);
        wr(NIC_GLB_CQ_INTR_STS(1), ~(1u << 5));               // 掩码**不给** QP5 ⇒ 该 QP 清不掉(per-QP 见证)
        sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME);   // ⚠ 另等一 delta:让该拍写入走完 update 相(§同 slot 纪律)✓
        chk("④CQ_INTR_STS 掩码不全 ⇒ 线仍高(per-QP 语义)", s_intr.read() ? 1 : 0, 1);
        wr(NIC_GLB_CQ_INTR_STS(1), 1u << 5);                  // 补上 QP5(白盒注入置的是全 QP ⇒ 两步清完)✓
        sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME);   // ⚠ 另等一 delta:让该拍写入走完 update 相(§同 slot 纪律)✓
        chk("④真清路: CQ_INTR_STS1 掩码清后 ⇒ 线落下", s_intr.read() ? 1 : 0, 0);
        chk("④真清路: INTR_STS bit4 已清", (rd(A_INTR_STS) >> 4) & 1, 0);

        // ---- ⑤ 段解码正控:PD 段写入不串段、不带坏邻字段 ✓ ----
        wr(A_PD0_VA0, 0xABCD1234);
        chk("⑤PD 段 RW", rd(A_PD0_VA0), 0xABCD1234);
        chk("⑤邻字段未被带写", rd(A_PD0_VA1), 0);
        chk("⑤per-QP 未被串写", rd(A_QP_CONF), 0xABC10305);

        // ---- ⑥ ★ sw/ 共同基准 × 模型 交叉判据(头文件位域/编码拆模型实值)✓ ----
        chk("⑥nic_regs.h:QP_CONF QP_EN 位", rd(A_QP_CONF) & NIC_QPC_QP_EN, 1);
        chk("⑥nic_regs.h:QP_CONF RQ_BUF_SZ 域", (rd(A_QP_CONF) & NIC_QPC_RQBUFSZ_MASK) >> 24, 0xAB);
        chk("⑥nic_regs.h:QP_CONF MAX_RD_OS 域", (rd(A_QP_CONF) & NIC_QPC_MAXRD_MASK) >> 16, 0xC1);
        chk("⑥nic_regs.h:QP_CONF PMTU 域", (rd(A_QP_CONF) & NIC_QPC_PMTU_MASK) >> 8, 0x03);
        chk("⑥nic_regs.h:INTR_STS bit4 已清(经 CQ_INTR_STS1)", rd(A_INTR_STS) & NIC_INTR_WQE_CMPL, 0);
        printf("   [头文件] 窗口 0x%08x+0x%x | per-QP 索引2 ⇒ 地址 0x%llx(厂商 QP%u)✓\n",
               NIC_BASE, NIC_SIZE, (unsigned long long)NIC_QP(2), NIC_QP_QPN(2));

        // ---- ⑦ ★ 别名 / 段边界 / 索引边界 / RO 群扫(2026-10-04 开卷口再落判据)✓ ----
        // ① 全局段 **0x800 别名**(读译码 `case (rd_addr_c1[10:2])` 实证 ⇒ [18:11] 被忽略)✓
        wr(NIC_GLB(0x088), 0x5A5A1234);                     // IN_ERRSTS_Q_BA_REG(RW)✓
        chk("⑦别名:全局段 0x088 写后读回", rd(NIC_GLB(0x088)), 0x5A5A1234);
        chk("⑦别名:GLB(0x888)= GLB(0x088)(0x800 别名)", rd(NIC_GLB(0x888)), 0x5A5A1234);
        chk("⑦别名:GLB(0x1088)= 同值(0x1000 别名)", rd(NIC_GLB(0x1088)), 0x5A5A1234);
        // ② 段间不串:上面的全局写不得落到 PD 段/同偏移(PD0 off 0x08 = 之前写过的 0xABCD1234 邻域)✓
        chk("⑦段界:全局写未串到 PD 段", rd(A_PD0_VA1), 0);
        // ③ per-QP 索引边界(NQP = **32**;索引 = addr[15:8])✓
        wr(NIC_QP(31) + NIC_QP_CQ_BA, 0x00AA0000);      // 索引 31 = 最大有效 ✓
        chk("⑦索引31(最大有效)写读一致", rd(NIC_QP(31) + NIC_QP_CQ_BA), 0x00AA0000);
        chk("⑦索引32(越界)⇒ 读被拒", rd_fail(NIC_QP(32) + NIC_QP_CQ_BA) ? 1 : 0, 1);
        // ④ RO 群扫(三个不同段/组:全局计数类 / CNP 状态区 / per-QP STAT)(写被忽略 + 计数)✓
        uint32_t ro0 = rd(A_STAT_RQPI);                         // per-QP RO(0x9C)已注入 0x1234 ✓
        wr(A_STAT_RQPI, 0xFFFF);                                // 上面 ② 已测过 per-QP RO;此处扫全局两处
        wr(NIC_GLB(0x49C), 0xDEADBEEF);                     // RD_REQ_CNT(RO)✓
        wr(NIC_GLB(0x390), 0xDEADBEEF);                     // CNP_SCHD_STS1(RO 区头)✓
        chk("⑦RO 群扫:写入后读回不变(0x49C)", rd(NIC_GLB(0x49C)), 0);
        chk("⑦RO 群扫:写入后读回不变(0x390)", rd(NIC_GLB(0x390)), 0);
        chk("⑦RO 群扫:per-QP RO 未被改(0x9C)", rd(A_STAT_RQPI), ro0);

        done = true;
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n("rst_n");

    DeviceTlm dev("dev");
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

    RegMaster mst("mst");
    mst.p_dev = &dev;
    dev.clk(clk); dev.rst_n(rst_n);
    mst.clk(clk);

    // ⚠ 寄存器访问 = 直连方法(路径待定;见 device_tlm.h 头注释)✗

    // 设备 CXL.cache 引脚(本轮不驱动 ⇒ 绑空信号;下轮接桥 ✓)
    sc_core::sc_signal<bool> d_r0v, d_r0r, d_r1v, d_r1r, d_dv, d_dr, d_rv_, d_rr_, h_rv, h_rr_, h_dv_;
    sc_core::sc_signal<bool> h_qrdy, h_srdy, h_drdy;   // ⚠ 三个 ready 输出各一条(同信号会 E115 双驱动)✓
    sc_core::sc_signal<sc_uint<5>>  d_r0op, d_r1op, d_rop;
    sc_core::sc_signal<sc_uint<4>>  h_rop;
    sc_core::sc_signal<sc_uint<3>>  h_rop3;
    sc_core::sc_signal<sc_uint<12>> d_r0cq, d_r1cq, d_duq, d_ruq, h_rrq, h_dcq;
    sc_core::sc_signal<sc_uint<46>> d_r0ad, d_r1ad, h_rad;
    sc_core::sc_signal<bool> d_dcv, h_dcv;
    sc_core::sc_signal<sc_biguint<512>> d_dpl, h_dpl;
    dev.d2h_req0_valid(d_r0v); dev.d2h_req0_opcode(d_r0op); dev.d2h_req0_cqid(d_r0cq);
    dev.d2h_req0_addr(d_r0ad); dev.d2h_req0_ready(d_r0r);
    dev.d2h_req1_valid(d_r1v); dev.d2h_req1_opcode(d_r1op); dev.d2h_req1_cqid(d_r1cq);
    dev.d2h_req1_addr(d_r1ad); dev.d2h_req1_ready(d_r1r);
    dev.d2h_data_valid(d_dv); dev.d2h_data_uqid(d_duq); dev.d2h_data_chunk_valid(d_dcv);
    dev.d2h_data_payload(d_dpl); dev.d2h_data_ready(d_dr);
    dev.d2h_rsp_valid(d_rv_); dev.d2h_rsp_opcode(d_rop); dev.d2h_rsp_uqid(d_ruq); dev.d2h_rsp_ready(d_rr_);
    dev.h2d_req_valid(h_rv); dev.h2d_req_opcode(h_rop3); dev.h2d_req_addr(h_rad);
    dev.h2d_req_uqid(h_rrq); dev.h2d_req_ready(h_qrdy);
    dev.h2d_rsp_valid(h_rr_); dev.h2d_rsp_opcode(h_rop); dev.h2d_rsp_rsp_data(h_rrq);
    dev.h2d_rsp_cqid(h_dcq); dev.h2d_rsp_ready(h_srdy);
    dev.h2d_data_valid(h_dv_); dev.h2d_data_cqid(h_dcq); dev.h2d_data_chunk_valid(h_dcv);
    dev.h2d_data_payload(h_dpl); dev.h2d_data_ready(h_drdy);

    // 观测/中断
    sc_core::sc_signal<bool>         s_intr;
    sc_core::sc_signal<sc_uint<32>>  o_sq, o_rq, o_ro;
    dev.nic_intr(s_intr);
    dev.o_db_sq_cnt(o_sq); dev.o_db_rq_cnt(o_rq); dev.o_ro_wr_cnt(o_ro);
    sc_core::sc_signal<sc_uint<32>> o_wqe, o_cqe, o_dbd;
    dev.o_wqe_cnt(o_wqe); dev.o_cqe_cnt(o_cqe); dev.o_db_drop_cnt(o_dbd);
    // ★ v1.0 新观测口(多笔在飞/保序/GO-Err)也须显式绑(观测口也算端口,E109 教训)✓
    sc_core::sc_signal<sc_uint<32>> o_ostd, o_actqp, o_goerr;
    dev.o_max_rd_ostd(o_ostd); dev.o_max_act_per_qp(o_actqp); dev.o_goerr_cnt(o_goerr);
    sc_core::sc_signal<sc_uint<32>> o_rnr, o_osimpl;          // ★ v1.0 RQ/简化 opcode 计数 ✓
    dev.o_rnr_cnt(o_rnr); dev.o_opc_simpl_cnt(o_osimpl);
    sc_core::sc_signal<sc_uint<32>> o_snpheld;                // ★ v1.0 ✓
    dev.o_snp_held_cnt(o_snpheld);
    sc_core::sc_signal<sc_uint<32>> o_qpf;                    // ★ v1.0 QP fatal 计数 ✓
    dev.o_qp_fatal_cnt(o_qpf);
    mst.s_intr(s_intr); mst.s_db_sq(o_sq); mst.s_db_rq(o_rq); mst.s_ro_wr(o_ro);

    printf("============================================================\n");
    printf("tb_device_regs — 设备 TLM v1.0 寄存器面(§8 属性:RW/RO/门铃/W1C)✓\n");
    printf("============================================================\n");

    rst_n.write(false);
    sc_core::sc_start(10, sc_core::SC_NS);
    rst_n.write(true);
    sc_core::sc_start(5, sc_core::SC_US);

    printf("   读数:用例 %d,失败 %d | 门铃计数 SQ=%u RQ=%u | RO 写拦截=%u\n",
           mst.ntests, mst.nfail, o_sq.read().to_uint(), o_rq.read().to_uint(), o_ro.read().to_uint());
    bool pass = (mst.nfail == 0) && (mst.ntests >= 12) && mst.done;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_DEVICE_REGS %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
