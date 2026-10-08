//============================================================================
// tb_cache_m2.cpp — ★★ **写命中⇒M(脏行)+ snoop 状态化应答 + 脏数据转发**  [2026-10-05 建]
//   判据(预登记;照 `规格文档` §1.2;读数 = 落定后):
//   A. 铺底:① WQE1 读 SRC(经缓存)⇒ 4 行填 **S**;② WQE2 写**目标 = SRC**(源 = SRC2)
//      ⇒ 写命中 4 行 ⇒ **M(3)** 且 `cch_wr_hit=4`;★ **宿内存 SRC 仍旧值**(脏行在设备,见证)✓
//   B. snoop A(**ReadUnique → SNP_INV**,SRC+0,脏行):设备 ⇒ **I_FWD_M + 行数据**;桥 ⇒
//      `data_transfer=1`;**数据逐字节 == 新写入值**;行态 ⇒ **I** ✓
//   C. snoop B(**ReadShared → SNP_DATA**,SRC+64):⇒ **S_FWD_M + 行数据**;行态 ⇒ I ✓
//   D. snoop C(未缓存地址):⇒ **I_HIT_I** 且 **无数据**(负控)✓
//   E. 收尾:cch_snp_hit=2 / 桥 err 不增 / goerr=0 / 中断 ✓
//   ⚠ snoop 注入法 = 照 `tb_device_e2e` 的 `snoop_thread`(宿侧 ACE 从口 `b_snoop`,
//     阻塞到桥+设备服务完;回读扩展位 + 数据指针)✓
//============================================================================
//
// 拓扑(= 端点栈;线侧静默):
//   [ACE内存] ←checker← [桥] ←pin级→ [设备(缓存行表 M1')] ↔ [MAC(空转,LINK 哑绑)]
//
// 判据(预登记;语义照 `规格文档` §1.2;读数 = 落定后):
//   A. WQE1(WRITE 256B,SRC→DST1;缓存**开**):① 目标区逐字节 == 源 ② **cch_fills=4**(填 4 行)
//      且 **cch_state(SRC) == S(1)**(填充态 = GO 带回态,"读回程默认 S" ✓)
//   B. WQE2(**同 SRC**→DST2):③ 数据仍逐字节正确(经缓存供给)④ **cch_hits=4**(4 条源读全命中)
//      ⑤ ★ **桥读增量 == 2**(仅 WQE 行 + CQE 行;4 条源读**没落内存**)✓
//   C. **默认关负控**:`set_cache_mode(false)` + WQE3(同 SRC→DST3)⇒ ⑥ 桥读增量 == 6
//      (4 源 + 2 行)且 hits 不增 ⇒ A/B 的效果确由缓存所致 ✓
//   D. ⑦ 收尾:cue=3 / 中断抬起 / goerr=0 / 门铃丢弃=0 / fcs_err=0 ✓
//============================================================================
#include <systemc.h>
#include <cstring>
#include <amba_pv.h>
#include <models/amba_pv_ace_protocol_checker.h>
#include "stack_top.h"
#include "dummy_amba_master.h"   // ★ 占位主端(reg_s 必绑)✓
#include "nic_regs.h"
#include <cstdio>

//----------------------------------------------------------------------------
struct AceHostMem: sc_core::sc_module, amba_pv::amba_pv_ace_slave_base {
    amba_pv::amba_pv_ace_slave_socket<512> ace_s;
    unsigned char mem[65536 * cxlshim::LINE_BYTES];      // 4MB(⚠ 堆分配)✓
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
struct CchM1Drv: sc_core::sc_module {
    sc_in<bool> clk; sc_out<bool> rst_n;
    sc_in<bool> intr;
    sc_in<sc_uint<32>> o_cqe, o_wqe, o_dbd, o_goerr, o_rd, o_fcs, o_rerr;
    DeviceTlm  * dev = nullptr;
    AceHostMem * mem = nullptr;
    bool all_pass = false; int pass_cnt = 0, fail_cnt = 0;
    // ★ M2':snoop 注入设施(照 tb_device_e2e snoop_thread:阻塞到桥+设备服务完)✓
    amba_pv::amba_pv_trans_pool snp_pool;
    unsigned char snp_data[64];
    int snp_bits = 0;
    int snoop(sc_dt::uint64 byte_addr, amba_pv::amba_pv_snoop_t type, unsigned char * out) {
        amba_pv::amba_pv_trans_ptr trans(snp_pool.allocate(1, cxlshim::LINE_BYTES, NULL, amba_pv::AMBA_PV_INCR));
        amba_pv::amba_pv_extension * ex = NULL; trans->get_extension(ex);
        trans->set_address(byte_addr); trans->set_data_length(cxlshim::LINE_BYTES);
        if (ex) ex->set_snoop(type);
        sc_core::sc_time t = sc_core::SC_ZERO_TIME;
        mem->ace_s.b_snoop(*trans, t);                        // ★ 阻塞:桥+设备服务完才返回 ✓
        wait(t);
        int bits = 0;
        if (ex) {
            if (ex->is_snoop_data_transfer()) bits |= 1;
            if (ex->is_snoop_error())         bits |= 2;
            if (ex->is_pass_dirty())          bits |= 4;
            if (ex->is_shared())              bits |= 8;
            if (ex->is_snoop_was_unique())    bits |= 16;
        }
        if (bits & 1) {                                       // 带数据 ⇒ 拷回(socket 数据指针)✓
            unsigned char * dp = (unsigned char *)trans->get_data_ptr();
            if (dp) for (int k = 0; k < 64; k++) out[k] = dp[k];
        }
        return bits;
    }

    SC_HAS_PROCESS(CchM1Drv);
    CchM1Drv(sc_core::sc_module_name nm): sc_module(nm) { SC_THREAD(run); }

    void chk(bool ok, const char * name) {
        printf("   v %-52s = 0x%08x\n", name, ok ? 1u : 0u);
        if (ok) pass_cnt++; else { fail_cnt++; printf("   ✗ %s 不符\n", name); }
    }
    void chk_eq(const char * name, uint32_t got, uint32_t exp) {
        printf("   v %-44s got=0x%08x exp=0x%08x\n", name, got, exp);
        if (got != exp) { fail_cnt++; printf("   ✗ %s 不符\n", name); } else pass_cnt++;
    }
    void tick() { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); }
    void rw(sc_dt::uint64 a, uint32_t v) {
        if (!dev->reg_write(a, sc_uint<32>(v))) { printf("   x 写 0x%llx 未实现\n", (unsigned long long)a); fail_cnt++; }
    }
    void wqe(sc_dt::uint64 SQ, uint32_t pi, uint32_t wrid, uint32_t len, uint32_t loff, uint32_t roff) {
        unsigned char * wq = mem->line_ptr(SQ + (sc_dt::uint64)pi * 64);
        for (int i = 0; i < 64; i++) wq[i] = 0;
        wq[NIC_WQE_WRID_OFF + 0] = (unsigned char)(wrid & 0xff);
        wq[NIC_WQE_WRID_OFF + 1] = (unsigned char)((wrid >> 8) & 0xff);
        *(uint32_t *)(wq + NIC_WQE_LEN_OFF)  = len;
        wq[NIC_WQE_OPC_OFF] = NIC_OPC_WRITE;
        *(uint32_t *)(wq + NIC_WQE_LOFF_OFF) = loff;
        *(uint32_t *)(wq + NIC_WQE_ROFF_OFF) = roff;
    }
    int cmp(const unsigned char * a, const unsigned char * b, int n) {
        int bad = 0; for (int i = 0; i < n; i++) if (a[i] != b[i]) bad++; return bad;
    }

    void run() {
        rst_n.write(false);
        for (int i = 0; i < 4; i++) wait(clk.posedge_event());
        rst_n.write(true);
        tick();

        const sc_dt::uint64 SQ = 0x00020000, CQ = 0x00030000, SRC = 0x00040000;
        const sc_dt::uint64 DST1 = 0x00050000, SRC2 = 0x00080000;

        // ---- 配置:QP 面 + 中断使能 + 源图案 + **缓存开** ----
        rw(0x50180200 + 0x00, 0x00000001);                    // QP_EN ✓
        rw(0x50180200 + 0x10, (uint32_t)SQ);
        rw(0x50180200 + 0x18, (uint32_t)CQ);
        rw(0x50180200 + 0x3C, 8);                             // Q_DEPTH ✓
        rw(0x50100180, (1u << 4));                            // INTR_EN bit4 ✓
        unsigned char src[256];
        for (int i = 0; i < 256; i++) src[i] = (unsigned char)((i * 9 + 0x21) & 0xff);
        { unsigned char * s = mem->line_ptr(SRC); for (int i = 0; i < 256; i++) s[i] = src[i]; }
        dev->set_cache_mode(true, 16);                        // ★ M1':缓存开(16 行)✓

        // ================= A. 铺底:① 读填 S;② 写命中 ⇒ M =================
        unsigned char p1[256], p2[256];
        for (int i = 0; i < 256; i++) { p1[i] = (unsigned char)((i * 9 + 0x21) & 0xff);
                                        p2[i] = (unsigned char)((i * 13 + 0x57) & 0xff); }
        { unsigned char * a = mem->line_ptr(SRC2); for (int i = 0; i < 256; i++) a[i] = p2[i]; }
        { unsigned char * b = mem->line_ptr(SRC);  for (int i = 0; i < 256; i++) b[i] = p1[i]; }

        wqe(SQ, 1, 0x2101, 256, (uint32_t)SRC, (uint32_t)DST1);       // 读 SRC(填 S)⇒ 写 DST1 ✓
        rw(0x50180200 + 0x38, 1);
        for (int i = 0; i < 6000 && o_cqe.read().to_uint() < 1; i++) tick();
        for (int i = 0; i < 20; i++) tick();
        chk_eq("A① cqe_cnt", o_cqe.read().to_uint(), 1);
        chk_eq("A① cch_fills(4)", dev->cch_fills, 4);
        chk_eq("A① cch_state(SRC) == S(1)", (uint32_t)dev->cch_state(SRC), 1);

        wqe(SQ, 2, 0x2102, 256, (uint32_t)SRC2, (uint32_t)SRC);       // 写进已缓存行 ⇒ 命中 ⇒ M ✓
        rw(0x50180200 + 0x38, 2);
        for (int i = 0; i < 6000 && o_cqe.read().to_uint() < 2; i++) tick();
        for (int i = 0; i < 20; i++) tick();
        chk_eq("A② cqe_cnt", o_cqe.read().to_uint(), 2);
        chk_eq("A② cch_wr_hit(写命中 4 行)", dev->cch_wr_hit, 4);
        chk_eq("A② cch_state(SRC) == M(3)", (uint32_t)dev->cch_state(SRC), 3);
        chk_eq("A② ★ 宿内存 SRC 仍旧值(脏行在设备)", (uint32_t)cmp(mem->line_ptr(SRC), p1, 256), 0);
        uint32_t err0 = o_rerr.read().to_uint();

        // ================= B. snoop A:ReadUnique(→SNP_INV)脏行 ⇒ I_FWD_M + 数据 =================
        snp_bits = snoop(SRC, amba_pv::AMBA_PV_READ_UNIQUE, snp_data);
        chk_eq("B③ snoop A:data_transfer 位", (snp_bits >> 0) & 1, 1);
        chk_eq("B③ snoop A:数据 == 新写入值(脏行转发)", (uint32_t)cmp(snp_data, p2, 64), 0);
        chk_eq("B③ snoop A:行态 ⇒ I(0)", (uint32_t)dev->cch_state(SRC), 0);

        // ================= C. snoop B:ReadShared(→SNP_DATA)⇒ S_FWD_M + 数据 =================
        snp_bits = snoop(SRC + 64, amba_pv::AMBA_PV_READ_SHARED, snp_data);
        chk_eq("C④ snoop B:data_transfer 位", (snp_bits >> 0) & 1, 1);
        chk_eq("C④ snoop B:数据 == 新写入值第 2 行", (uint32_t)cmp(snp_data, p2 + 64, 64), 0);
        chk_eq("C④ snoop B:行态 ⇒ I", (uint32_t)dev->cch_state(SRC + 64), 0);

        // ================= D. snoop C:未缓存地址 ⇒ I_HIT_I(无数据)负控 =================
        snp_bits = snoop(0x000D0000, amba_pv::AMBA_PV_READ_UNIQUE, snp_data);
        chk_eq("D⑤ snoop C:data_transfer=0(无数据)", (snp_bits >> 0) & 1, 0);
        chk_eq("D⑤ snoop C:err 位 = 0", (snp_bits >> 1) & 1, 0);

        // ================= E. 收尾 =================
        chk_eq("E⑥ cch_snp_hit(=2 命中两次)", dev->cch_snp_hit, 2);
        chk_eq("E⑥ 桥 err 不增", o_rerr.read().to_uint(), err0);
        chk_eq("E⑥ goerr / 门铃丢弃(均 0)", o_goerr.read().to_uint() + o_dbd.read().to_uint(), 0);
        chk_eq("E⑥ 中断线抬起", intr.read() ? 1u : 0u, 1);
        printf("   [内存] rd=%u wr=%u bad=%u\n", mem->rd_n, mem->wr_n, mem->bad);

        all_pass = (fail_cnt == 0);
        printf("   [合计] pass=%d fail=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    StackTop stack("stack");                  // 设备+桥+MAC(内部接线全封装)✓
    stack.clk(clk); stack.rst_n(rst_n);
    DummyAmbaMaster dum("dum");               // ★ reg_s 必绑(E109 socket 版)✓
    stack.dev.reg_s.bind(dum.m);

    AceHostMem * mem = new AceHostMem("mem"); // ⚠ 4MB 堆分配 ✓
    amba_pv::amba_pv_ace_protocol_checker<512> chk("chk");
    stack.br.ace_m.bind(chk.amba_pv_s); chk.amba_pv_m.bind(mem->ace_s);

    // ---- MAC 线侧哑绑(本 TB 不用线;E109 纪律:每个口都绑)✓
    sc_core::sc_signal<sc_biguint<512>> md0, md1, mr0, mr1;
    sc_core::sc_signal<sc_biguint<64>>  mc0, mc1, mc2, mc3;
    sc_core::sc_signal<bool> mv0, mv1, men0("men0", true), men1("men1", true);
    sc_core::sc_signal<bool> men2("men2", true), men3("men3", true);
    stack.mac.link_txd_0(md0);   stack.mac.link_txd_1(md1);
    stack.mac.link_txc_0(mc0);   stack.mac.link_txc_1(mc1);
    stack.mac.link_txdval_0(mv0); stack.mac.link_txdval_1(mv1);
    stack.mac.link_rxd_0(mr0);   stack.mac.link_rxd_1(mr1);   // 哑:恒静默(无人驱动 = 0)✓
    stack.mac.link_rxc_0(mc2);   stack.mac.link_rxc_1(mc3);
    stack.mac.link_txclk_ena_0(men0); stack.mac.link_txclk_ena_1(men1);
    stack.mac.link_rxclk_ena_0(men2); stack.mac.link_rxclk_ena_1(men3);

    CchM1Drv drv("drv");
    drv.clk(clk); drv.rst_n(rst_n); drv.intr(stack.sig_intr);
    drv.dev = &stack.dev; drv.mem = mem;
    drv.o_cqe(stack.sig_cqe); drv.o_wqe(stack.sig_wqe); drv.o_dbd(stack.sig_dbd);
    drv.o_goerr(stack.sig_goerr); drv.o_rd(stack.sig_rd); drv.o_fcs(stack.sig_mfcs);
    drv.o_rerr(stack.sig_err);

    printf("============================================================\n");
    printf("tb_cache_m2 - ★★ 写命中⇒M + snoop 状态化应答 + 脏数据转发\n");
    printf("============================================================\n");

    sc_core::sc_start(60, sc_core::SC_US);
    bool pass = drv.all_pass;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_CACHE_M2 %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
