//============================================================================
// tb_dual_stack.cpp — ★ 双实例**完整栈**互测  [2026-10-05 建]
//   用户令:「双实例 mac 这个测试要加上 **COH 和 cxl2ace 全链路**交互测试」✓
// 被测 = `top/stack_top.h`(每侧 = DeviceTlm(COH+NIC)+ Cxl2AceBridge + MacTlm,内部接线全封装)✓
//
// 拓扑(★ 两个完整端点,线侧背靠背):
//   [Stack A: ACE内存A ←→ 桥A ←→ 设备A ←→ MAC_A] ══LINK 交叉 + ena glue══
//   [Stack B: ACE内存B ←→ 桥B ←→ 设备B ←→ MAC_B]
//
// 判据(预登记):
//   ① A 栈 RDMA WRITE 全流程:目标区逐字节 == 源区(256B);CQE wrid/opcode 对;**intr 抬起**
//   ② B 栈同样(**不同 wrid/图案** ⇒ 串栈必红)
//   ③ ★ 双栈**同刻**(两门铃 → 两完成):两侧计数 1/1;丢弃/err 全 0;CQE 各落**各自**内存
//   ④ 帧路 A→B:经 A.dev→A.mac→线→B.mac→B.dev ⇒ 逐字节一致(120B)
//   ⑤ 帧路 B→AB(**陷阱字节**)⇒ 逐字节一致
//   ⑥ 帧路计数**交叉见证**:dev 侧 a.tx=1∧b.rx=1 ∧ b.tx=1∧a.rx=1(同 mac 侧);fcs_err=0
//   ⑦ ★ **互不干扰**:帧路跑完两侧 wqe/cqe 与桥 rd/wr 计数**不变**
//============================================================================
#include <systemc.h>
#include <cstring>
#include <amba_pv.h>
#include <models/amba_pv_ace_protocol_checker.h>
#include "stack_top.h"
#include "dummy_amba_master.h"   // ★ 占位主端(reg_s 必绑)✓
#include "nic_regs.h"      // ★ sw/ 共同基准(WQE/CQE 布局宏)✓
#include <cstdio>

//----------------------------------------------------------------------------
// 主机内存(ACE 从端;TB 每栈一个)✓
//----------------------------------------------------------------------------
struct AceHostMem: sc_core::sc_module, amba_pv::amba_pv_ace_slave_base {
    amba_pv::amba_pv_ace_slave_socket<512> ace_s;
    unsigned char mem[65536 * cxlshim::LINE_BYTES];      // 4MB ✓
    uint32_t rd_n = 0, wr_n = 0, bad = 0;

    AceHostMem(sc_core::sc_module_name nm): amba_pv_ace_slave_base("host_mem"), ace_s("ace_s") {
        ace_s(*this);
        for (unsigned i = 0; i < sizeof(mem); i++) mem[i] = 0;
    }
    void b_transport(int, amba_pv::amba_pv_transaction & tr, sc_core::sc_time & t) override {
        amba_pv::amba_pv_extension * ex = NULL; tr.get_extension(ex);
        size_t off = (size_t)((tr.get_address() >> 6) & 0xFFFF) * cxlshim::LINE_BYTES;
        unsigned char * d = tr.get_data_ptr();
        if (tr.is_read()) {
            rd_n++;
            for (int i = 0; i < cxlshim::LINE_BYTES; i++) d[i] = mem[off + i];
        } else if (tr.is_write()) {
            wr_n++;
            for (int i = 0; i < cxlshim::LINE_BYTES; i++) mem[off + i] = d[i];
        } else bad++;
        if (ex != NULL) ex->set_resp(amba_pv::AMBA_PV_OKAY);
        t += sc_core::sc_time(4, sc_core::SC_NS);
    }
    unsigned char * line_ptr(sc_dt::uint64 byte_addr) {
        return &mem[(size_t)((byte_addr >> 6) & 0xFFFF) * cxlshim::LINE_BYTES];
    }
    unsigned int transport_dbg(int, amba_pv::amba_pv_transaction &) override { return 0; }
    bool get_direct_mem_ptr(int, amba_pv::amba_pv_transaction &, tlm::tlm_dmi &) override { return false; }
};

//----------------------------------------------------------------------------
struct DualStackDrv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    // A/B 观测
    sc_in<bool> a_intr, b_intr;
    sc_in<sc_uint<32>> a_wqe, a_cqe, a_dbd, a_rd, a_wr, a_err, a_mtx, a_mrx, a_mftx, a_mfrx;
    sc_in<sc_uint<32>> a_mfcs, a_midle;
    sc_in<sc_uint<32>> b_wqe, b_cqe, b_dbd, b_rd, b_wr, b_err, b_mtx, b_mrx, b_mftx, b_mfrx;
    sc_in<sc_uint<32>> b_mfcs, b_midle;
    DeviceTlm   *p_a = nullptr, *p_b = nullptr;
    AceHostMem  *m_a = nullptr, *m_b = nullptr;
    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(DualStackDrv);
    DualStackDrv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char * name) {
        printf("   v %-46s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }
    void chk_eq(const char * name, uint32_t got, uint32_t exp) {
        printf("   v %-40s got=0x%08x exp=0x%08x\n", name, got, exp);
        if (got != exp) { fail_cnt++; printf("   ✗ %s 不符\n", name); } else pass_cnt++;
    }
    void tick() { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); }
    void rw(DeviceTlm * d, sc_dt::uint64 a, uint32_t v) {
        if (!d->reg_write(a, sc_uint<32>(v))) { printf("   x 写 0x%llx 未实现\n", (unsigned long long)a); fail_cnt++; }
    }
    uint32_t rdr(DeviceTlm * d, sc_dt::uint64 a) {
        sc_uint<32> v = 0;
        if (!d->reg_read(a, v)) { printf("   x 读 0x%llx 未实现\n", (unsigned long long)a); fail_cnt++; }
        return v.to_uint();
    }
    // 配 QP + 装 WQE/源数据(每栈一份;tag 选图案)✓
    void setup(DeviceTlm * d, AceHostMem * m, sc_dt::uint64 SQ, sc_dt::uint64 CQ,
               sc_dt::uint64 SRC, sc_dt::uint64 DST, uint32_t wrid, int tag) {
        rw(d, 0x50180200 + 0x00, 0x00000001);                   // QP_CONF:QP_EN ✓
        rw(d, 0x50180200 + 0x10, (uint32_t)SQ);
        rw(d, 0x50180200 + 0x18, (uint32_t)CQ);
        rw(d, 0x50180200 + 0x3C, 8);                            // Q_DEPTH ✓
        unsigned char * wq = m->line_ptr(SQ + 64);              // WQE 行(pi=1)✓
        for (int i = 0; i < 64; i++) wq[i] = 0;
        wq[NIC_WQE_WRID_OFF + 0] = (unsigned char)(wrid & 0xff);
        wq[NIC_WQE_WRID_OFF + 1] = (unsigned char)((wrid >> 8) & 0xff);
        *(uint32_t *)(wq + NIC_WQE_LEN_OFF)  = 256u;        // 4 行 ✓
        wq[NIC_WQE_OPC_OFF] = NIC_OPC_WRITE;
        *(uint32_t *)(wq + NIC_WQE_LOFF_OFF) = (uint32_t)SRC;
        *(uint32_t *)(wq + NIC_WQE_ROFF_OFF) = (uint32_t)DST;
        for (int ln = 0; ln < 4; ln++) {                        // 源行:每栈图案不同 ✓
            unsigned char * sl = m->line_ptr(SRC + ln * 64);
            for (int j = 0; j < 64; j++)
                sl[j] = (unsigned char)((tag == 0) ? ((j * 3 + ln * 17 + 0x21) & 0xff)
                                                   : ((j * 5 + ln * 29 + 0x11) & 0xff));
        }
    }
    int cmp_line(AceHostMem * m, sc_dt::uint64 dst, sc_dt::uint64 src) {   // 4×64B 逐字节 ✓
        int bad = 0;
        for (int ln = 0; ln < 4; ln++) {
            unsigned char * dd = m->line_ptr(dst + ln * 64);
            unsigned char * ss = m->line_ptr(src + ln * 64);
            for (int j = 0; j < 64; j++) if (dd[j] != ss[j]) bad++;
        }
        return bad;
    }

    void run() {
        rst_n.write(false);
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        tick();

        const sc_dt::uint64 SQ = 0x00020000, CQ = 0x00030000, SRC = 0x00040000, DST = 0x00050000;
        const uint32_t WA = 0xBEEF, WB = 0xCAFE;

        // ---- ① 双栈同刻:两侧先后配置(各约 10 次直连写),再**两门铃** ----
        rw(p_a, 0x50100180, (1u << 4));                         // INTR_EN bit4(CQ)✓
        rw(p_b, 0x50100180, (1u << 4));
        setup(p_a, m_a, SQ, CQ, SRC, DST, WA, 0);
        setup(p_b, m_b, SQ, CQ, SRC, DST, WB, 1);
        rw(p_a, 0x50180200 + 0x38, 1);                          // 门铃 A ✓
        rw(p_b, 0x50180200 + 0x38, 1);                          // 门铃 B(同刻)✓
        for (int i = 0; i < 6000 && !(a_cqe.read().to_uint() >= 1 && b_cqe.read().to_uint() >= 1); i++) tick();
        for (int i = 0; i < 20; i++) tick();                    // 落定 ✓

        // ---- 判据 ①②③ ----
        chk_eq("① A:目标区错字节数", (uint32_t)cmp_line(m_a, DST, SRC), 0);
        {
            unsigned char * cqe = m_a->line_ptr(CQ + 4) + (unsigned)((CQ + 4) & 0x3C);
            chk_eq("① A:CQE wrid", NIC_CQE_WRID(cqe), WA);
            chk_eq("① A:CQE opcode", NIC_CQE_OPCODE(cqe), 0x00);
        }
        chk_eq("① A:intr 线抬起", a_intr.read() ? 1u : 0u, 1);
        chk_eq("② B:目标区错字节数", (uint32_t)cmp_line(m_b, DST, SRC), 0);
        {
            unsigned char * cqe = m_b->line_ptr(CQ + 4) + (unsigned)((CQ + 4) & 0x3C);
            chk_eq("② B:CQE wrid", NIC_CQE_WRID(cqe), WB);
            chk_eq("② B:CQE opcode", NIC_CQE_OPCODE(cqe), 0x00);
        }
        chk_eq("② B:intr 线抬起", b_intr.read() ? 1u : 0u, 1);
        chk_eq("③ 计数: a.wqe", a_wqe.read().to_uint(), 1);
        chk_eq("③ 计数: a.cqe", a_cqe.read().to_uint(), 1);
        chk_eq("③ 计数: b.wqe", b_wqe.read().to_uint(), 1);
        chk_eq("③ 计数: b.cqe", b_cqe.read().to_uint(), 1);
        chk_eq("③ 丢弃(两侧均 0)", a_dbd.read().to_uint() + b_dbd.read().to_uint(), 0);
        chk_eq("③ 桥 err(两侧均 0)", a_err.read().to_uint() + b_err.read().to_uint(), 0);
        printf("   [内存] A rd=%u wr=%u bad=%u | B rd=%u wr=%u bad=%u\n",
               m_a->rd_n, m_a->wr_n, m_a->bad, m_b->rd_n, m_b->wr_n, m_b->bad);

        // ---- ④⑤ 帧路:A.dev→A.mac→线→B.mac→B.dev(反向同)----
        uint32_t w0a = a_wqe.read().to_uint(), c0a = a_cqe.read().to_uint();
        uint32_t w0b = b_wqe.read().to_uint(), c0b = b_cqe.read().to_uint();
        uint32_t r0a = a_rd.read().to_uint(), r0b = b_rd.read().to_uint();
        uint32_t x0a = a_wr.read().to_uint(), x0b = b_wr.read().to_uint();
        unsigned char fa[120], fb[200], got[4096];
        for (int i = 0; i < 120; i++) fa[i] = (unsigned char)((i * 3 + 0x21) & 0xff);
        for (int i = 0; i < 200; i++) fb[i] = (unsigned char)((i * 5 + 0x11) & 0xff);
        fb[7] = 0xFB; fb[80] = 0xFD; fb[3] = 0x07; fb[4] = 0x55; fb[5] = 0xD5; fb[98] = 0xFB;
        // A → B
        p_a->mac_push_tx_frame(fa, 120);
        for (int c = 0; c < 600 && p_b->mac_rx_frames < 1; c++) tick();
        int r1 = p_b->mac_pop_rx_frame(got, 4096);
        bool ok1 = (r1 == 120);
        for (int i = 0; i < 120 && ok1; i++) if (got[i] != fa[i]) ok1 = false;
        chk(ok1, "④ 帧路 A→B: 120B 逐字节一致");
        // B → A(陷阱字节)
        p_b->mac_push_tx_frame(fb, 200);
        for (int c = 0; c < 600 && p_a->mac_rx_frames < 1; c++) tick();
        int r2 = p_a->mac_pop_rx_frame(got, 4096);
        bool ok2 = (r2 == 200);
        for (int i = 0; i < 200 && ok2; i++) if (got[i] != fb[i]) ok2 = false;
        chk(ok2, "⑤ 帧路 B→A(陷阱字节): 200B 逐字节一致");
        for (int i = 0; i < 4; i++) tick();                     // settle ✓

        // ---- ⑥ 帧路计数交叉见证 ----
        chk_eq("⑥ dev 侧 a.tx", a_mtx.read().to_uint(), 1);
        chk_eq("⑥ dev 侧 b.rx", b_mrx.read().to_uint(), 1);
        chk_eq("⑥ dev 侧 b.tx", b_mtx.read().to_uint(), 1);
        chk_eq("⑥ dev 侧 a.rx", a_mrx.read().to_uint(), 1);
        chk_eq("⑥ mac 侧 a.tx", a_mftx.read().to_uint(), 1);
        chk_eq("⑥ mac 侧 b.rx", b_mfrx.read().to_uint(), 1);
        chk_eq("⑥ mac 侧 b.tx", b_mftx.read().to_uint(), 1);
        chk_eq("⑥ mac 侧 a.rx", a_mfrx.read().to_uint(), 1);
        chk_eq("⑥ fcs_err(两侧均 0)", a_mfcs.read().to_uint() + b_mfcs.read().to_uint(), 0);
        chk(a_midle.read().to_uint() > 0 && b_midle.read().to_uint() > 0, "⑥ 两侧 idle 拍在涨(链路活)");

        // ---- ⑦ 互不干扰(帧路不碰寄存器面/内存面)----
        chk((a_wqe.read().to_uint() == w0a && a_cqe.read().to_uint() == c0a &&
             b_wqe.read().to_uint() == w0b && b_cqe.read().to_uint() == c0b),
            "⑦ 互不干扰: 两侧 wqe/cqe 不变");
        chk((a_rd.read().to_uint() == r0a && a_wr.read().to_uint() == x0a &&
             b_rd.read().to_uint() == r0b && b_wr.read().to_uint() == x0b),
            "⑦ 互不干扰: 两侧桥 rd/wr 不变");

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    StackTop sa("sa"), sb("sb");
    DummyAmbaMaster dum_a("dum_a"), dum_b("dum_b");   // ★ 两栈 reg_s 必绑(E109 socket 版)✓
    sa.dev.reg_s.bind(dum_a.m); sb.dev.reg_s.bind(dum_b.m);
    sa.clk(clk); sa.rst_n(rst_n);
    sb.clk(clk); sb.rst_n(rst_n);

    // ---- ACE 侧:每栈 桥 → 检查器 → 主机内存 ✓
    //   ⚠ 两块 4MB 内存**必须堆分配** —— 栈上内联 ⇒ sc_main 帧 >8MB 默认栈 ⇒ **构造期段错误**
    //     (与 e2e 单块 4.3MB 超 --max-stackframe 同族;此处两块直接爆栈)✗✓
    AceHostMem *m_a = new AceHostMem("m_a");
    AceHostMem *m_b = new AceHostMem("m_b");
    amba_pv::amba_pv_ace_protocol_checker<512> chk_a("chk_a"), chk_b("chk_b");
    sa.br.ace_m.bind(chk_a.amba_pv_s); chk_a.amba_pv_m.bind(m_a->ace_s);
    sb.br.ace_m.bind(chk_b.amba_pv_s); chk_b.amba_pv_m.bind(m_b->ace_s);

    // ---- ★ 线侧:两个 MAC 背靠背(LINK 交叉 + 「PCS 角色」ena glue)----
    //   ⚠ clk/ena 是 MAC **输入**(PCS 侧给)⇒ 直连必须补 glue(照 mac_tlm_spec §3/§7.2)✓
    sc_core::sc_signal<sc_biguint<512>> Ta0, Ta1, Tb0, Tb1;    // MAC_A 发(→B 收)/ MAC_B 发(→A 收)
    sc_core::sc_signal<sc_biguint<64>>  TCa0, TCa1, TCb0, TCb1;
    sc_core::sc_signal<bool> Tda0, Tda1, Tdb0, Tdb1;           // dval(只到线,不进对端契约)✓
    sc_core::sc_signal<bool> Eax0("Eax0", true), Eax1("Eax1", true), Ear0("Ear0", true), Ear1("Ear1", true);  // ena 恒 1 ✓
    sc_core::sc_signal<bool> Ebx0("Ebx0", true), Ebx1("Ebx1", true), Ebr0("Ebr0", true), Ebr1("Ebr1", true);
    sa.mac.link_txd_0(Ta0);   sa.mac.link_txd_1(Ta1);
    sa.mac.link_txc_0(TCa0);  sa.mac.link_txc_1(TCa1);
    sa.mac.link_txdval_0(Tda0); sa.mac.link_txdval_1(Tda1);
    sa.mac.link_txclk_ena_0(Eax0); sa.mac.link_txclk_ena_1(Eax1);
    sa.mac.link_rxclk_ena_0(Ear0); sa.mac.link_rxclk_ena_1(Ear1);
    sb.mac.link_txd_0(Tb0);   sb.mac.link_txd_1(Tb1);
    sb.mac.link_txc_0(TCb0);  sb.mac.link_txc_1(TCb1);
    sb.mac.link_txdval_0(Tdb0); sb.mac.link_txdval_1(Tdb1);
    sb.mac.link_txclk_ena_0(Ebx0); sb.mac.link_txclk_ena_1(Ebx1);
    sb.mac.link_rxclk_ena_0(Ebr0); sb.mac.link_rxclk_ena_1(Ebr1);
    // 交叉:A 发 → B 收;B 发 → A 收 ✓
    sb.mac.link_rxd_0(Ta0); sb.mac.link_rxd_1(Ta1);
    sb.mac.link_rxc_0(TCa0); sb.mac.link_rxc_1(TCa1);
    sa.mac.link_rxd_0(Tb0); sa.mac.link_rxd_1(Tb1);
    sa.mac.link_rxc_0(TCb0); sa.mac.link_rxc_1(TCb1);

    // ---- 驱动(单线程;两栈判据合一)✓ ----
    DualStackDrv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n);
    drv.p_a = &sa.dev; drv.p_b = &sb.dev; drv.m_a = m_a; drv.m_b = m_b;
    drv.a_intr(sa.sig_intr); drv.b_intr(sb.sig_intr);
    drv.a_wqe(sa.sig_wqe); drv.a_cqe(sa.sig_cqe); drv.a_dbd(sa.sig_dbd);
    drv.a_rd(sa.sig_rd); drv.a_wr(sa.sig_wr); drv.a_err(sa.sig_err);
    drv.a_mtx(sa.sig_mtx); drv.a_mrx(sa.sig_mrx);
    drv.a_mftx(sa.sig_mftx); drv.a_mfrx(sa.sig_mfrx); drv.a_mfcs(sa.sig_mfcs); drv.a_midle(sa.sig_midle);
    drv.b_wqe(sb.sig_wqe); drv.b_cqe(sb.sig_cqe); drv.b_dbd(sb.sig_dbd);
    drv.b_rd(sb.sig_rd); drv.b_wr(sb.sig_wr); drv.b_err(sb.sig_err);
    drv.b_mtx(sb.sig_mtx); drv.b_mrx(sb.sig_mrx);
    drv.b_mftx(sb.sig_mftx); drv.b_mfrx(sb.sig_mfrx); drv.b_mfcs(sb.sig_mfcs); drv.b_midle(sb.sig_midle);

    printf("============================================================\n");
    printf("tb_dual_stack - ★ 双实例完整栈:2× (COH/设备+桥+MAC) 背靠背互测\n");
    printf("============================================================\n");

    sc_core::sc_start(60, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_DUAL_STACK %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
