//============================================================================
// tb_full_chain.cpp — ★ **四模型整机串联**:e2e 全判据 + 线侧帧路(dev+桥+ACE 内存+MAC+PCS link)  [2026-10-05 建]
// ⚠ 本 TB 的 e2e 判据 = **自 `tb_device_e2e` 全量复制**(撤 MAC 哑绑定、接真 MacTlm+PcsLinkTlm);
//   分工:e2e = 引擎/寄存器面;本 TB = **四模型同场装配证明**;e2e 判据若更新 ⇒ 两边同步 ✓
//
// 拓扑(★ 四条链全接上):① 主机链:设备(DeviceTlm)── pin 级 CXL.cache ── [Cxl2AceBridge]
//   ── AMBA-PV ACE ──> [协议检查器] ──> 主机内存(AceHostMem)✓
//   ② 线侧链:设备 ── cmac AXIS(2048b)── [MacTlm] ── LINK(2×512b)── [PcsLinkTlm(环回)]⇒ 回接 ✓
//   (③ 寄存器面 = 直连方法占位(A7);④ 中断 = nic_intr 线)✓
//
// 用例(驱动 = TB 的 Driver 线程,经**直连方法**写寄存器 = 占位路径,见 A7)✓:
//   ① 配 QP3/QP5/QP7:QP_EN/SQ_BA/CQ_BA/Q_DEPTH/INTR_EN(bit4)② 主机内存放 WQE + 源数据(非均匀)
//   ③ 敲门铃(★ 含 **QP3 连敲两笔** ⇒ 同 QP 保序用例)④ 轮询到跑完 ⑤ 判据:
//      **目标行逐字** == 源行(数据真搬了)+ CQE 4B {wrid,opcode} 正确 + `nic_intr` 抬起 +
//      计数(dev wqe/cqe;桥 rd/wr;内存 rd/wr)+ 无 err ✓
//   ★ v1.0 多笔在飞判据(独立双见证;>1 = 真多笔):
//      · 设备侧 `o_max_rd_ostd`(读槽在飞高水位)/ 桥侧 `o_rd_ostd_max`(已收读 − 已回)✓
//      · **槽号照 RTL**:桥侧 `o_fst_rd_cqid`==0 / `o_fst_wr_cqid`==64 / 读 cqid ≤ 63 / 写 cqid ≥ 64 ✓
//      · 同 QP 保序:`o_max_act_per_qp` == **1**(同 QP 在飞 job ≤ 1)✓
//      · ⚠ 数据正确性本身即 cqid 匹配的判据:源行**逐行图案不同** ⇒ 若应答串槽,256B 比对必崩 ✓
//   ===== ★ 新增判据(帧路;预登记)=====
//   ⑥ 帧 A(120B)经 dev→mac→PCS link→环回→mac→dev ⇒ **逐字节一致**
//   ⑦ 帧 B(200B,**陷阱字节**)⇒ 逐字节一致(控制位「按位置」的回归卫士)✓
//   ⑧ 计数一致:dev mac_tx/rx_frames=2 · mac o_frames_tx/rx=2 · o_fcs_err=0 · pcs o_bit_errs=0
//   ⑨ 门面位:pcs align=1 · lane_locked=0xFFFF · 列进出 ≥2
//   ⑩ **互不干扰**:帧路跑完 dev wqe/cqe 与桥 rd/wr 计数**不变**;丢弃计数全 0 ✓
// ⚠ 模型简化(明写,见 P2 规格 §4):R_key/VA 不译(offset 直用为行地址);
//   CQE 4B 写用整行 RMW 替代字节使能 ✓
//============================================================================
#include <systemc.h>
#include <cstring>
#include <amba_pv.h>
#include <models/amba_pv_ace_protocol_checker.h>
#include "device_tlm.h"
#include "dummy_amba_master.h"   // ★ 占位主端(reg_s 必绑)✓
#include "cxl2ace_bridge.h"
#include "mac_tlm.h"
#include "pcs_link_tlm.h"
#include "nic_regs.h"      // ★ sw/ 共同基准(WQE/CQE 布局宏 ⇒ 与模型解析**实测对拍**)✓

static const sc_dt::uint64 SQ_BA  = 0x00020000;      // 字节地址(寄存器值,64B 对齐)✓
static const sc_dt::uint64 CQ_BA  = 0x00030000;
static const sc_dt::uint64 SRC_BA = 0x00040000;      // 源数据行(WQE.local_offset)
static const sc_dt::uint64 DST_BA = 0x00050000;      // 目标行(WQE.remote_offset)
// ★ v1.0 多 QP(QP5 = 索引 4):各自 SQ/CQ/源/目标 + 各自 wrid/图案 ⇒ 验"不串"✓
static const sc_dt::uint64 SQ2_BA = 0x00060000, CQ2_BA = 0x00070000;
static const sc_dt::uint64 SRC2_BA = 0x00080000, DST2_BA = 0x00090000;
static const uint32_t      WRID2 = 0xC0DE;
// ★ v1.0:QP7(索引 6)① RDMA READ(0x04,角色翻转)② 非法 opcode 负控 ✓
static const sc_dt::uint64 SQ3_BA = 0x000A0000, CQ3_BA = 0x000B0000;
static const sc_dt::uint64 RMT_BA = 0x000C0000, LOC_BA = 0x000D0000;
static const uint32_t      WRID3 = 0x0DD0;
// ★ v1.0:QP3 第二笔(pi=2;同 QP 保序用例;WQE 行 = SQ_BA + 128)✓
static const sc_dt::uint64 SRC3_BA = 0x000E0000, DST3_BA = 0x000F0000;
static const uint32_t      WRID4 = 0xBEE2;
static uint64_t patE_word(int ln, int i) { return 0x7E7E0000ULL + (uint64_t)(ln * 0x1000) + (uint64_t)i; }
static uint64_t patD_word(int ln, int i) { return 0x6D6D0000ULL + (uint64_t)(ln * 0x100) + (uint64_t)i; }
// ★ v1.0 SEND + RQ 环(QP 索引 5):RQ 环 depth=2、RQ_BUF_SZ=1(256B)
//   项地址(照 RTL 映射 `(wrptr==depth)?0:wrptr`):wrptr=1 → BA+256=0x100100;wrptr=2 → BA=0x100000 ✓
static const sc_dt::uint64 RQBA    = 0x00100000;     // RQ_BUF_BA(256B 粒度 [31:8])✓
static const sc_dt::uint64 SQ4_BA  = 0x00120000, CQ4_BA = 0x00130000;
static const sc_dt::uint64 SRC4_BA = 0x00140000;
static const uint32_t      WRID_S1 = 0x5E01, WRID_S2 = 0x5E02, WRID_S3 = 0x5E03;
static uint64_t patS_word(int m, int i) { return 0x7A7A0000ULL + (uint64_t)(m * 0x100) + (uint64_t)i; }
// ★ v1.0 snoop × 在飞地址顺序用例(QP 索引 7):源行读**加 400ns 延迟**造在飞窗口 ✓
static const sc_dt::uint64 SQ5_BA = 0x001A0000, CQ5_BA = 0x001B0000;
static const sc_dt::uint64 SRCT_BA = 0x001C0000, DSTT_BA = 0x001D0000;
static const uint32_t      WRID_T = 0x7A11;
static uint64_t patT_word(int i) { return 0x6B6B0000ULL + (uint64_t)(i * 0x0333); }
static const uint32_t      WRID   = 0xBEEF;
static const uint32_t      DEPTH  = 8;

static uint64_t patB_word(int i) { return 0x5B5B0000ULL + (uint64_t)(i * 0x1111); }
static uint64_t patC_word(int i) { return 0x3C3C0000ULL + (uint64_t)(i * 0x2222); }   // QP5 图案 ✓

//----------------------------------------------------------------------------
// 主机内存(ACE 从端):b_transport 服务整行读/写 + **直连**预装/检查口 ✓
//----------------------------------------------------------------------------
struct AceHostMem: sc_core::sc_module, amba_pv::amba_pv_ace_slave_base {
    amba_pv::amba_pv_ace_slave_socket<512> ace_s;
    unsigned char mem[65536 * cxlshim::LINE_BYTES];      // ★ 4MB(原 1MB:mask 会把新 SEND 地址折回旧区)✓
    uint32_t rd_n = 0, wr_n = 0, bad = 0;

    SC_HAS_PROCESS(AceHostMem);                       // 自定义 ctor + SC_THREAD ⇒ 必须 ✓
    AceHostMem(sc_core::sc_module_name nm): amba_pv_ace_slave_base("host_mem"), ace_s("ace_s") {
        ace_s(*this);
        for (unsigned i = 0; i < sizeof(mem); i++) mem[i] = 0;
        SC_THREAD(snoop_thread);
    }
    void b_transport(int, amba_pv::amba_pv_transaction & tr, sc_core::sc_time & t) override {
        amba_pv::amba_pv_extension * ex = NULL; tr.get_extension(ex);
        size_t off = (size_t)((tr.get_address() >> 6) & 0xFFFF) * cxlshim::LINE_BYTES;   // 4MB(多 QP/SEND 不撞行)✓
        unsigned char * d = tr.get_data_ptr();
        if (tr.is_read()) {
            rd_n++;
            for (int i = 0; i < cxlshim::LINE_BYTES; i++) d[i] = mem[off + i];
            if (stall_en && ((tr.get_address() >> 6) == stall_line)) {      // ★ 造"在飞"窗口 ✓
                t += sc_core::sc_time(stall_ns, sc_core::SC_NS);
                stall_seen = true;
            }
        } else if (tr.is_write()) {
            wr_n++;
            for (int i = 0; i < cxlshim::LINE_BYTES; i++) mem[off + i] = d[i];
        } else bad++;
        if (ex != NULL) ex->set_resp(amba_pv::AMBA_PV_OKAY);
        t += sc_core::sc_time(4, sc_core::SC_NS);
    }
    // 直连口(TB 预装/检查)✓
    unsigned char * line_ptr(sc_dt::uint64 byte_addr) {
        return &mem[(size_t)((byte_addr >> 6) & 0xFFFF) * cxlshim::LINE_BYTES];
    }

    // ================= ★ v1.0:snoop × 在飞地址顺序的**激励设施** =================
    // ① 读延迟注入:对指定行的**读**加 N ns ⇒ 该读长时间在飞(造窗口)✓
    sc_dt::uint64 stall_line = ~0ULL; uint32_t stall_ns = 0; bool stall_en = false;
    bool stall_seen = false;
    void set_stall(sc_dt::uint64 line, uint32_t ns) { stall_line = line; stall_ns = ns; stall_en = true; stall_seen = false; }
    void stall_off() { stall_en = false; }
    // ② 主动发 snoop(照 tb_bridge_loopback 同款:从端 socket `b_snoop`,阻塞到桥服务完)✓
    amba_pv::amba_pv_trans_pool m_pool;
    sc_core::sc_event ev_snp;
    int  snp_req = 0; sc_dt::uint64 snp_line = 0;      // 驱动填 req/line,notify;agent 线程跑 ✓
    bool snp_done = false; int snp_bits = -1; double snp_ns = 0.0;
    void snd_snoop(sc_dt::uint64 line) { snp_line = line; snp_done = false; snp_req = 1; ev_snp.notify(); }

    void snoop_thread() {
        while (true) {
            wait(ev_snp);
            snp_req = 0;
            amba_pv::amba_pv_trans_ptr trans(m_pool.allocate(1, cxlshim::LINE_BYTES, NULL, amba_pv::AMBA_PV_INCR));
            amba_pv::amba_pv_extension * ex = NULL;
            trans->get_extension(ex);
            trans->set_address((sc_dt::uint64)snp_line << 6);           // 行 → 字节 ✓
            trans->set_data_length(cxlshim::LINE_BYTES);
            if (ex != NULL) ex->set_snoop((amba_pv::amba_pv_snoop_t)amba_pv::AMBA_PV_READ_UNIQUE);
            sc_core::sc_time t = sc_core::SC_ZERO_TIME;
            sc_core::sc_time t0 = sc_core::sc_time_stamp();
            ace_s.b_snoop(*trans, t);                                   // ★ 阻塞:桥+设备服务完才返回 ✓
            wait(t);
            snp_ns = (sc_core::sc_time_stamp() - t0).to_double() / 1000.0;   // ps → ns ✓
            snp_bits = 0;
            if (ex != NULL) {
                if (ex->is_snoop_data_transfer()) snp_bits |= 1;
                if (ex->is_snoop_error())         snp_bits |= 2;
                if (ex->is_pass_dirty())          snp_bits |= 4;
                if (ex->is_shared())              snp_bits |= 8;
                if (ex->is_snoop_was_unique())    snp_bits |= 16;
            }
            snp_done = true;
        }
    }
    unsigned int transport_dbg(int, amba_pv::amba_pv_transaction &) override { return 0; }
    bool get_direct_mem_ptr(int, amba_pv::amba_pv_transaction &, tlm::tlm_dmi &) override { return false; }
};

//----------------------------------------------------------------------------
// 驱动线程:配寄存器 → 装 WQE/源数据 → 敲门铃 → 等 → 判据 ✓
//----------------------------------------------------------------------------
struct Driver: sc_core::sc_module {
    sc_in<bool> clk, rst_n;
    DeviceTlm * p_dev; AceHostMem * p_mem; Cxl2AceBridge * p_br;
    sc_in<bool> s_intr;
    sc_in<sc_uint<32>> s_wqe, s_cqe, s_br_rd, s_br_wr, s_br_err, s_br_snp, s_dbd;
    // ★ v1.0 多笔在飞/槽号仲裁观(设备侧 + 桥侧**独立双见证**)✓
    sc_in<sc_uint<32>> s_ostd, s_actqp, s_goerr, s_br_ostd;
    sc_in<sc_uint<32>> s_rnr, s_osimpl;              // ★ v1.0 RQ/简化 opcode 计数 ✓
    sc_in<sc_uint<32>> s_snpheld;                    // ★ v1.0 snoop 被压计数 ✓
    sc_in<sc_uint<32>> s_qpf;                        // ★ v1.0 QP fatal 计数 ✓
    sc_in<sc_uint<32>> s_mtx, s_mrx, s_mftx, s_mfrx, s_mfcs;   // ★ MAC 观测 ✓
    sc_in<sc_uint<32>> s_lbin, s_lbout, s_lberr;               // ★ PCS link 观测 ✓
    sc_in<bool> s_lalign; sc_in<sc_uint<16>> s_llocked;
    sc_in<sc_uint<12>> s_frd, s_fwr, s_rmax, s_wmin;
    int nfail = 0; bool done = false;

    SC_HAS_PROCESS(Driver);
    Driver(sc_core::sc_module_name nm): sc_core::sc_module(nm) { SC_THREAD(run); }

    void rw(sc_dt::uint64 a, uint32_t v) {                       // 寄存器写(直连方法,占位路径)⚠
        if (!p_dev->reg_write(a, sc_uint<32>(v))) { printf("   x 写 0x%llx 未实现\n", (unsigned long long)a); nfail++; }
    }
    void chk(const char * n, uint32_t got, uint32_t exp) {
        if (got != exp) { printf("   x %s:got 0x%08x exp 0x%08x\n", n, got, exp); nfail++; }
        else printf("   v %s = 0x%08x\n", n, got);
    }
    void tick() { sc_core::wait(clk.posedge_event()); sc_core::wait(sc_core::SC_ZERO_TIME); }
    uint32_t rdr(sc_dt::uint64 a) {                  // 寄存器读(直连方法,占位路径)⚠
        sc_uint<32> v = 0;
        if (!p_dev->reg_read(a, v)) { printf("   x 读 0x%llx 未实现\n", (unsigned long long)a); nfail++; }
        return v.to_uint();
    }
    int cmp_line(sc_dt::uint64 mem_ba, sc_dt::uint64 src_ba) {   // 64B 逐字节比对 ✓
        unsigned char * d = p_mem->line_ptr(mem_ba), * s2 = p_mem->line_ptr(src_ba);
        int bad = 0;
        for (int i = 0; i < 64; i++) if (d[i] != s2[i]) bad++;
        return bad;
    }
    // ★ 走一帧(dev→mac→pcs link→环回→mac→dev);返回长度(-1 = 超时)——
    //   ⚠ 等**增量**(勿判 >=1:上一帧后恒真 ⇒ 空取,已踩)✓
    int round_trip(const unsigned char * f, int n, const char * tag) {
        unsigned int base = p_dev->mac_rx_frames;
        p_dev->mac_push_tx_frame(f, n);
        unsigned char got[4096]; int rn = -1;
        for (int cyc = 0; cyc < 600; cyc++) {
            tick();
            if (p_dev->mac_rx_frames >= base + 1) { rn = p_dev->mac_pop_rx_frame(got, 4096); break; }
        }
        if (rn < 0) { printf("   x %s: 回收超时\n", tag); return -1; }
        if (rn != n) { printf("   x %s: 长度 %d != %d\n", tag, rn, n); return rn; }
        for (int i = 0; i < n; i++)
            if (got[i] != f[i]) { printf("   x %s: 逐字节分叉 @%d(got=%02x exp=%02x)\n", tag, i, got[i], f[i]); return -2; }
        return n;
    }

    void run() {
        sc_dt::uint64 qp3 = 0x50180200;                          // QP3 基址(索引 2)✓
        tick();
        while (!rst_n.read()) tick();                            // ★ 先等复位释放(否则门铃被复位分支清掉 ✗ 已踩)✓
        // ① 配 QP3 + 中断使能
        rw(qp3 + 0x00, 0x00000001);                              // QP_CONF:[0] QP_EN ✓
        rw(qp3 + 0x10, (uint32_t)SQ_BA);                         // SQ_BA ✓
        rw(qp3 + 0x18, (uint32_t)CQ_BA);                         // CQ_BA ✓
        rw(qp3 + 0x3C, DEPTH);                                   // Q_DEPTH ✓
        rw(0x50100180, (1u << 4) | (1u << 3));                   // INTR_EN bit4(CQ)+bit3(RNR-NAK)✓
        // ② 主机内存:WQE(行 = SQ_BA>>6 + 1,照 §9.2 指针 1-based)+ 源数据行(非均匀)✓
        unsigned char * wq = p_mem->line_ptr(SQ_BA + 64);        // idx = pi = 1 ✓
        for (int i = 0; i < 64; i++) wq[i] = 0;
        wq[NIC_WQE_WRID_OFF + 0] = (unsigned char)(WRID & 0xff);   // ★ 偏移走 sw/ 宏 ✓
        wq[NIC_WQE_WRID_OFF + 1] = (unsigned char)((WRID >> 8) & 0xff);
        *(uint32_t *)(wq + NIC_WQE_LEN_OFF) = 256u;                // dma_len = 256(4 行)✓
        wq[NIC_WQE_OPC_OFF] = NIC_OPC_WRITE;                   // wqe_opcode = 0x00 ✓
        *(uint32_t *)(wq + NIC_WQE_LOFF_OFF) = (uint32_t)SRC_BA;   // local_offset ✓
        *(uint32_t *)(wq + NIC_WQE_ROFF_OFF) = (uint32_t)DST_BA;   // remote_offset ✓
        unsigned char * src = p_mem->line_ptr(SRC_BA);
        for (int ln = 0; ln < 4; ln++) {                          // ★ 4 行,每行图案不同(防"恒等假过")✓
            unsigned char * sl = p_mem->line_ptr(SRC_BA + ln * 64);
            for (int i = 0; i < 8; i++) {
                uint64_t w = patB_word(i) + (uint64_t)ln;
                for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
            }
        }
        // ②a2 ★ QP3 第二笔(pi=2,**同 QP 连投** ⇒ 保序用例):WQE 行 = SQ_BA + 128 ✓
        unsigned char * wq5 = p_mem->line_ptr(SQ_BA + 128);
        for (int i = 0; i < 64; i++) wq5[i] = 0;
        wq5[NIC_WQE_WRID_OFF + 0] = (unsigned char)(WRID4 & 0xff);
        wq5[NIC_WQE_WRID_OFF + 1] = (unsigned char)((WRID4 >> 8) & 0xff);
        *(uint32_t *)(wq5 + NIC_WQE_LEN_OFF) = 128u;               // 128(2 行)✓
        wq5[NIC_WQE_OPC_OFF] = NIC_OPC_WRITE;
        *(uint32_t *)(wq5 + NIC_WQE_LOFF_OFF)  = (uint32_t)SRC3_BA;
        *(uint32_t *)(wq5 + NIC_WQE_ROFF_OFF) = (uint32_t)DST3_BA;
        for (int ln = 0; ln < 2; ln++) {                           // 自己的图案(与 QP3 第一笔不同)✓
            unsigned char * sl = p_mem->line_ptr(SRC3_BA + ln * 64);
            for (int i = 0; i < 8; i++) {
                uint64_t w = patE_word(ln, i);
                for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
            }
        }
        // ②b ★ QP5(索引 4):配寄存器 + 装 WQE + 自己的源行(不同图案)✓
        sc_dt::uint64 qp5 = 0x50180400;
        rw(qp5 + 0x00, 0x00000001);
        rw(qp5 + 0x10, (uint32_t)SQ2_BA);
        rw(qp5 + 0x18, (uint32_t)CQ2_BA);
        rw(qp5 + 0x3C, DEPTH);
        unsigned char * wq2 = p_mem->line_ptr(SQ2_BA + 64);
        for (int i = 0; i < 64; i++) wq2[i] = 0;
        wq2[0] = (unsigned char)(WRID2 & 0xff); wq2[1] = (unsigned char)((WRID2 >> 8) & 0xff);
        wq2[12] = 64; wq2[16] = 0x01;                              // ★ 0x01 = WRITE+IMMDT(IMMDT 侧带不入模型)✓
        *(uint32_t *)(wq2 + 4)  = (uint32_t)SRC2_BA;
        *(uint32_t *)(wq2 + 20) = (uint32_t)DST2_BA;
        unsigned char * src2 = p_mem->line_ptr(SRC2_BA);
        for (int i = 0; i < 8; i++) {
            uint64_t w = patC_word(i);
            for (int k = 0; k < 8; k++) src2[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
        }
        // ②c ★ QP7(索引 6):① RDMA READ 128B(2 行)② 非法 opcode 负控 ✓
        sc_dt::uint64 qp7 = 0x50180600;
        rw(qp7 + 0x00, 0x00000001);
        rw(qp7 + 0x10, (uint32_t)SQ3_BA);
        rw(qp7 + 0x18, (uint32_t)CQ3_BA);
        rw(qp7 + 0x3C, DEPTH);
        unsigned char * wq3 = p_mem->line_ptr(SQ3_BA + 64);        // idx = pi = 1 ✓
        for (int i = 0; i < 64; i++) wq3[i] = 0;
        wq3[0] = (unsigned char)(WRID3 & 0xff); wq3[1] = (unsigned char)((WRID3 >> 8) & 0xff);
        wq3[12] = 128; wq3[13] = 0;                                // dma_len = 128(2 行)✓
        wq3[16] = 0x04;                                            // ★ opcode = RDMA READ ✓
        *(uint32_t *)(wq3 + 4)  = (uint32_t)LOC_BA;                // local = **目标**(角色翻转)✓
        *(uint32_t *)(wq3 + 20) = (uint32_t)RMT_BA;                // remote = **源** ✓
        for (int ln = 0; ln < 2; ln++) {
            unsigned char * rl = p_mem->line_ptr(RMT_BA + ln * 64);
            for (int i = 0; i < 8; i++) {
                uint64_t w = patD_word(ln, i);
                for (int k = 0; k < 8; k++) rl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
            }
        }
        unsigned char * wq4 = p_mem->line_ptr(SQ3_BA + 128);       // 第二笔:idx = 2(pi=2)✓
        for (int i = 0; i < 64; i++) wq4[i] = 0;
        *(uint32_t *)(wq4 + NIC_WQE_LEN_OFF) = 64u;
        wq4[NIC_WQE_OPC_OFF] = 0x7F;                           // 集合外 opcode ⇒ 负控 ✓
        *(uint32_t *)(wq4 + 4)  = (uint32_t)LOC_BA;
        *(uint32_t *)(wq4 + 20) = (uint32_t)RMT_BA;
        // ②d ★ v1.0 SEND + RQ 环(QP 索引 5):① RQ depth=2 / RQ_BUF_SZ=1(256B)
        //   ② 三笔 SEND(0x02/0x03/0x02);③ 第 2/3 笔碰撞"环满 ⇒ RNR"+ 消费后补送 ✓
        sc_dt::uint64 qpS = 0x50180500;                   // 索引 5(照 §8.1:块 = 0x100,索引 = addr[15:8];厂商 QPn = 索引+1)✓
        rw(qpS + 0x00, 0x01000001);                    // QP_EN=1 | RQ_BUF_SZ[31:24]=1(256B)✓
        rw(qpS + 0x08, (uint32_t)RQBA);                // RQ_BUF_BA([31:8] 256B 粒度)✓
        rw(qpS + 0x10, (uint32_t)SQ4_BA);
        rw(qpS + 0x18, (uint32_t)CQ4_BA);
        rw(qpS + 0x3C, (2u << 16) | 8u);               // Q_DEPTH:[15:0]SQ=8 / [31:16]RQ=2 ✓
        rw(qpS + 0x34, 1);                             // RQ_CI_DB 初始化 = 1(空环:ci == wrptr)✓
        for (int m = 0; m < 3; m++) {                  // 三笔 SEND 的 WQE(pi=1,2,3)+ 各自源行 ✓
            unsigned char * ws = p_mem->line_ptr(SQ4_BA + 64 * (m + 1));
            for (int i = 0; i < 64; i++) ws[i] = 0;
            uint32_t wr = (m == 0) ? WRID_S1 : (m == 1) ? WRID_S2 : WRID_S3;
            ws[NIC_WQE_WRID_OFF + 0] = (unsigned char)(wr & 0xff);
            ws[NIC_WQE_WRID_OFF + 1] = (unsigned char)((wr >> 8) & 0xff);
            *(uint32_t *)(ws + NIC_WQE_LEN_OFF) = 64u;                  // 64(1 行)✓
            ws[NIC_WQE_OPC_OFF] = (m == 1) ? NIC_OPC_SEND_IMMDT : NIC_OPC_SEND;  // ★+
            *(uint32_t *)(ws + NIC_WQE_LOFF_OFF) = (uint32_t)(SRC4_BA + m * 64);
            // ⚠ remote_offset 对 SEND **无效**(目标 = 本 QP 的 RQ 项)⇒ 故意填 0 ✓
            unsigned char * sl = p_mem->line_ptr(SRC4_BA + m * 64);
            for (int i = 0; i < 8; i++) {
                uint64_t w = patS_word(m, i);
                for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
            }
        }
        // ③ 门铃:各 QP **连敲**(门铃队列 ⇒ 相继处理)✓;★ QP3/QP7 各两笔(pi=1,2)✓
        //   ⚠ SEND QP 先只敲第 1 笔(第 2/3 笔在下面**按 RNR 时序**逐步敲)✓
        rw(qp3 + 0x38, 1);
        rw(qp3 + 0x38, 2);                             // ★ 同 QP 第二笔(保序:job1 完成才起 job2)✓
        rw(qp5 + 0x38, 1);
        rw(qp7 + 0x38, 1);
        rw(qp7 + 0x38, 2);
        rw(qpS + 0x38, 1);                             // ★ SEND 第 1 笔 ✓
        // ④ 轮询到跑完(多笔在飞 ⇒ 完成时刻不固定,判据取**稳定后**读数;上限兜底)✓
        int spins = 0;
        for (; spins < 4000; spins++) {
            tick();
            if (s_wqe.read().to_uint() == 6 && s_cqe.read().to_uint() == 5 &&
                s_br_rd.read().to_uint() == 21 && s_br_wr.read().to_uint() == 15) break;
        }
        for (int i = 0; i < 20; i++) tick();           // 让尾笔/计数器落定 ✓
        // ⑤ 判据
        chk("dev.wqe_cnt(6 笔:含非法 + SEND 首笔)", s_wqe.read().to_uint(), 6);
        chk("dev.cqe_cnt(5 笔:SEND 首笔已出 CQE)", s_cqe.read().to_uint(), 5);
        chk("门铃丢弃数(须为 0)", s_dbd.read().to_uint(), 0);
        chk("dev GO-Err/无主回程(须为 0)", s_goerr.read().to_uint(), 0);
        chk("intr 线抬起", s_intr.read() ? 1 : 0, 1);
        // 读 = 每笔 WQE 行 + 源行 + **CQE 行 RMW 读**:6+4+3+4+1 = 18,再加 SEND 首笔 3 = 21 ✓
        chk("桥 rd_done(18 + SEND1 3 笔)", s_br_rd.read().to_uint(), 21);
        chk("桥 wr_done(13 + SEND1 2 笔)", s_br_wr.read().to_uint(), 15);
        chk("桥 err", s_br_err.read().to_uint(), 0);
        // ★ v1.0 多笔在飞 + 槽号照 RTL + 同 QP 保序(**独立双见证**:设备侧 + 桥侧)✓
        printf("   [多笔在飞] 设备在读槽高水位=%u | 桥侧读在飞高水位=%u\n",
               s_ostd.read().to_uint(), s_br_ostd.read().to_uint());
        chk("多笔在飞(设备侧读槽高水位 ≥ 3)", (uint32_t)(s_ostd.read().to_uint() >= 3), 1);
        chk("多笔在飞(桥侧读在飞高水位 ≥ 3)", (uint32_t)(s_br_ostd.read().to_uint() >= 3), 1);
        chk("同 QP 在飞 job 高水位(须 = 1 = 保序)", s_actqp.read().to_uint(), 1);
        printf("   [槽号] 首读 cqid=%u(期望 0)/ 首写 cqid=%u(期望 64)| 读 cqid 上界=%u(≤63)/ 写 cqid 下界=%u(≥64)\n",
               s_frd.read().to_uint(), s_fwr.read().to_uint(),
               s_rmax.read().to_uint(), s_wmin.read().to_uint());
        chk("槽号照 RTL:首读 cqid = 0", s_frd.read().to_uint(), 0);
        chk("槽号照 RTL:首写 cqid = 64", s_fwr.read().to_uint(), 64);
        chk("槽号照 RTL:读 cqid ≤ 63", (uint32_t)(s_rmax.read().to_uint() <= 63), 1);
        chk("槽号照 RTL:写 cqid ≥ 64", (uint32_t)(s_wmin.read().to_uint() >= 64), 1);
        // 目标行逐字 == 源行(数据真搬了)✓
        int bad = 0;
        unsigned char * dst = p_mem->line_ptr(DST_BA);
        for (int i = 0; i < 256; i++) if (dst[i] != src[i]) bad++;      // ★ 256B(4 行)✓
        chk("目标区逐字节一致(256B 错字节数)", (uint32_t)bad, 0);
        int bad5 = 0;                                                   // 正控:第 5 行不该被碰 ✓
        unsigned char * d5 = p_mem->line_ptr(DST_BA + 256);
        for (int i = 0; i < 64; i++) if (d5[i] != 0) bad5++;
        chk("未越界(第 5 行未被写)", (uint32_t)bad5, 0);
        // CQE 4B {wrid,opcode}(照 §9.4;位置 = CQ_BA + cq_idx*4,cq_idx = hd = 1)✓
        unsigned char * cqline = p_mem->line_ptr(CQ_BA + 4);   // 该行
        unsigned char * cqe = cqline + (unsigned)((CQ_BA + 4) & 0x3C);   // ★ 行内偏移 4(首版按行首读错 ✗)✓
        chk("CQE wrid", NIC_CQE_WRID(cqe), WRID);
        chk("CQE opcode", NIC_CQE_OPCODE(cqe), 0x00);
        chk("CQE fatal", NIC_CQE_FATAL(cqe), 0x00);
        // ★ QP3 第二笔(同 QP 连投):数据 + **CQE 序在第 2 槽**(cq_idx=2 ⇒ 行内偏移 8)✓
        int bad4 = 0;
        for (int i = 0; i < 128; i++)
            if (p_mem->line_ptr(DST3_BA)[i] != p_mem->line_ptr(SRC3_BA)[i]) bad4++;
        chk("QP3 第二笔目标区一致(128B 错字节数)", (uint32_t)bad4, 0);
        int bad4b = 0;                                                 // 正控:第 3 行不该被碰 ✓
        for (int i = 0; i < 64; i++) if (p_mem->line_ptr(DST3_BA + 128)[i] != 0) bad4b++;
        chk("QP3 第二笔未越界(第 3 行未被写)", (uint32_t)bad4b, 0);
        unsigned char * cqe4 = p_mem->line_ptr(CQ_BA + 8) + (unsigned)((CQ_BA + 8) & 0x3C);
        chk("QP3 第二笔 CQE wrid(自己在第 2 槽)", NIC_CQE_WRID(cqe4), WRID4);
        chk("QP3 第二笔 CQE opcode", NIC_CQE_OPCODE(cqe4), 0x00);
        // ★ QP5:目标行 == 自己的源行;且两 QP 数据**不串** ✓
        unsigned char * dst2 = p_mem->line_ptr(DST2_BA);
        int bad2 = 0;
        for (int i = 0; i < 64; i++) if (dst2[i] != src2[i]) bad2++;
        chk("QP5 目标行逐字节一致(错字节数)", (uint32_t)bad2, 0);
        chk("两 QP 数据不串(dst1 != src2)", (uint32_t)(memcmp(p_mem->line_ptr(DST_BA), src2, 64) != 0), 1);
        unsigned char * cqe2 = p_mem->line_ptr(CQ2_BA + 4) + (unsigned)((CQ2_BA + 4) & 0x3C);
        chk("QP5 CQE wrid(自己的)", NIC_CQE_WRID(cqe2), WRID2);
        chk("QP5 CQE opcode(=WRITE_IMMDT 原样)", NIC_CQE_OPCODE(cqe2), NIC_OPC_WRITE_IMMDT);
        // ★ QP7 RDMA READ:本地目标区 == 远端源区(角色翻转得证)✓
        int badr = 0;
        for (int i = 0; i < 128; i++) if (p_mem->line_ptr(LOC_BA)[i] != p_mem->line_ptr(RMT_BA)[i]) badr++;
        chk("RDMA READ 本地区 == 远端区(128B)", (uint32_t)badr, 0);
        unsigned char * cqe3 = p_mem->line_ptr(CQ3_BA + 4) + (unsigned)((CQ3_BA + 4) & 0x3C);
        chk("QP7 CQE wrid", NIC_CQE_WRID(cqe3), WRID3);
        chk("QP7 CQE opcode(=READ)", NIC_CQE_OPCODE(cqe3), NIC_OPC_READ);
        // ★ 负控:非法 opcode ⇒ 不出 CQE(第 2 个 CQE 槽应仍为 0)+ 本地未被再写 ✓
        unsigned char * cqe3b = p_mem->line_ptr(CQ3_BA + 8) + (unsigned)((CQ3_BA + 8) & 0x3C);
        chk("负控:非法 opcode 无 CQE", NIC_CQE_WRID(cqe3b), 0);
        // ================= ★ v1.0 SEND + RQ 环时序(逐步对账)=================
        // 首笔已送达;项地址照 RTL 映射:wrptr=1 → BA+256 ✓(不是 (ptr-1) 口径)
        chk("SEND1:STAT_RQ_PI_DB(送达后 =2)", rdr(qpS + 0x9C), 2);
        chk("SEND1:数据落项1(BA+256)错字节数", (uint32_t)cmp_line(RQBA + 256, SRC4_BA + 0), 0);
        rw(qpS + 0x38, 2);                             // 第 2 笔:环满(容量 = depth-1 = 1)⇒ 应 RNR ✓
        for (int i = 0; i < 500 && s_rnr.read().to_uint() < 1; i++) tick();
        chk("SEND2:环满 ⇒ RNR 计数 = 1", s_rnr.read().to_uint(), 1);
        chk("SEND2:被挡时 wrptr 未动(=2)", rdr(qpS + 0x9C), 2);
        chk("SEND2:被挡时未出 CQE(=5)", s_cqe.read().to_uint(), 5);
        rw(qpS + 0x34, 2);                             // SW 消费一项 ⇒ 引擎**等到不满再送**(代重传)✓
        for (int i = 0; i < 800 && s_cqe.read().to_uint() < 6; i++) tick();
        chk("SEND2:补送后 wrptr 回绕 = 1", rdr(qpS + 0x9C), 1);
        chk("SEND2:数据落项2(BA,wrptr==depth→idx0)错字节数", (uint32_t)cmp_line(RQBA, SRC4_BA + 64), 0);
        rw(qpS + 0x38, 3);                             // 第 3 笔:又满 ⇒ 再 RNR ✓
        for (int i = 0; i < 500 && s_rnr.read().to_uint() < 2; i++) tick();
        chk("SEND3:再满 ⇒ RNR 计数 = 2", s_rnr.read().to_uint(), 2);
        rw(qpS + 0x34, 1);                             // 再消费(ci 2→1 回绕)✓
        for (int i = 0; i < 800 && s_cqe.read().to_uint() < 7; i++) tick();
        chk("SEND3:补送后 wrptr = 2", rdr(qpS + 0x9C), 2);
        chk("SEND3:数据落项1(复写)错字节数", (uint32_t)cmp_line(RQBA + 256, SRC4_BA + 128), 0);
        chk("INTR_STS bit3(RNR-NAK)已置", (rdr(0x50100184) >> 3) & 1, 1);
        for (int m = 0; m < 3; m++) {                  // SEND 三条 CQE:wrid 各自 + opcode 原样 ✓
            unsigned char * cqs = p_mem->line_ptr(CQ4_BA + 4 * (m + 1)) + (unsigned)((CQ4_BA + 4 * (m + 1)) & 0x3C);
            uint32_t wr = (m == 0) ? WRID_S1 : (m == 1) ? WRID_S2 : WRID_S3;
            chk("SEND CQE wrid", NIC_CQE_WRID(cqs), wr);
            chk("SEND CQE opcode(原样)", NIC_CQE_OPCODE(cqs),
                (m == 1) ? NIC_OPC_SEND_IMMDT : NIC_OPC_SEND);
        }
        chk("IMMDT 简化计数(0x01 + 0x03)", s_osimpl.read().to_uint(), 2);
        // ================= ★ v1.0 snoop × 在飞地址顺序(照 RTL"对在飞地址先等完成")=================
        sc_dt::uint64 qpT = 0x50180700;                // QP 索引 7 ✓
        rw(qpT + 0x00, 0x00000001);
        rw(qpT + 0x10, (uint32_t)SQ5_BA);
        rw(qpT + 0x18, (uint32_t)CQ5_BA);
        rw(qpT + 0x3C, DEPTH);
        unsigned char * wqt = p_mem->line_ptr(SQ5_BA + 64);
        for (int i = 0; i < 64; i++) wqt[i] = 0;
        wqt[0] = (unsigned char)(WRID_T & 0xff); wqt[1] = (unsigned char)((WRID_T >> 8) & 0xff);
        wqt[12] = 64; wqt[16] = 0x00;                  // WRITE 64B ✓
        *(uint32_t *)(wqt + 4)  = (uint32_t)SRCT_BA;
        *(uint32_t *)(wqt + 20) = (uint32_t)DSTT_BA;
        for (int i = 0; i < 8; i++) {                  // 源行图案(非均匀)✓
            unsigned char * sl = p_mem->line_ptr(SRCT_BA);
            uint64_t w = patT_word(i);
            for (int k = 0; k < 8; k++) sl[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xff);
        }
        p_mem->set_stall(SRCT_BA >> 6, 400);           // 源行读加 400ns ⇒ 长在飞窗口 ✓
        rw(qpT + 0x38, 1);                             // 门铃 ✓
        for (int i = 0; i < 4000 && !p_mem->stall_seen; i++) tick();       // 等该读**确已在飞** ✓
        p_mem->snd_snoop(SRCT_BA >> 6);                // ★ 正向:snoop 打**在飞地址** ✓
        for (int i = 0; i < 8000 && !p_mem->snp_done; i++) tick();
        printf("   [snoop] 正向(在飞地址):耗时 %.0f ns | 设备压住计数=%u\n",
               p_mem->snp_ns, s_snpheld.read().to_uint());
        chk("snoop×在飞:设备压住计数 ≥1", (uint32_t)(s_snpheld.read().to_uint() >= 1), 1);
        chk("snoop×在飞:耗时 ≥300ns(等到该读完成)", (uint32_t)(p_mem->snp_ns >= 300.0), 1);
        chk("snoop×在飞:CRRESP 全 0(I_HIT_I)", (uint32_t)p_mem->snp_bits, 0);
        for (int i = 0; i < 800 && s_cqe.read().to_uint() < 8; i++) tick();   // 该 job 随后正常完成 ✓
        chk("snoop 后 job 正常完成(数据一致)", (uint32_t)cmp_line(DSTT_BA, SRCT_BA), 0);
        // 负控:snoop 打**安静地址**(该行已无在飞)⇒ 应立刻回 ✓
        p_mem->stall_off();
        uint32_t held0 = s_snpheld.read().to_uint();
        p_mem->snd_snoop(SRCT_BA >> 6);
        for (int i = 0; i < 3000 && !p_mem->snp_done; i++) tick();
        printf("   [snoop] 负控(安静地址):耗时 %.0f ns | 设备压住计数=%u\n",
               p_mem->snp_ns, s_snpheld.read().to_uint());
        chk("snoop 负控:不被压(耗时 <100ns)", (uint32_t)(p_mem->snp_ns < 100.0), 1);
        chk("snoop 负控:压住计数不变", s_snpheld.read().to_uint(), held0);
        // ================= ★ v1.0 QP fatal(D7-① 开卷):非法 opcode ⇒ 该 QP fatal ✓ =================
        // QP7 已因 pi=2(非法 opcode)判 fatal ⇒ ① 中断 bit7 ② **再敲一笔记数被丢弃、WQE 不取** ✓
        chk("QP fatal:INTR_STS bit7(fatal_err)已置", (uint32_t)((rdr(0x50100184) >> 7) & 1), 1);
        {
            uint32_t wqe0 = s_wqe.read().to_uint();
            rw(qp7 + 0x38, 3);                             // ★ 同 QP 第三笔(合法 WQE)⇒ fatal ⇒ 应被丢 ✓
            for (int i = 0; i < 200; i++) tick();
            chk("QP fatal:同 QP 门铃被丢弃(计数 ≥2)", (uint32_t)(s_qpf.read().to_uint() >= 2), 1);
            chk("QP fatal:其后 WQE 未被取(wqe_cnt 不变)", s_wqe.read().to_uint(), wqe0);
        }
        // ⚠ 末段轮询:桥 `wr_done++` 落在 issuer 的 ACE 事务**之后** ⇒ 等它计完再取终值 ✓
        for (int i = 0; i < 600 && (s_br_rd.read().to_uint() < 30 || s_br_wr.read().to_uint() < 21); i++) tick();
        chk("dev.wqe_cnt(终:9 笔)", s_wqe.read().to_uint(), 9);
        chk("dev.cqe_cnt(终:8 笔)", s_cqe.read().to_uint(), 8);
        chk("桥 rd_done(终:27+3=30)", s_br_rd.read().to_uint(), 30);
        chk("桥 wr_done(终:19+2=21)", s_br_wr.read().to_uint(), 21);
        printf("   内存 rd=%u wr=%u bad=%u | 桥 snoop=%u\n", p_mem->rd_n, p_mem->wr_n, p_mem->bad,
               s_br_snp.read().to_uint());
        // ================= ★ 帧路:四模型同场(dev→mac→pcs link→环回→mac→dev)=================
        // ⚠ 此处 e2e 全判据已过、引擎空闲 ⇒ 帧路与寄存器路不争;仍验「互不干扰」(计数不变)✓
        uint32_t wqe_b = s_wqe.read().to_uint(), cqe_b = s_cqe.read().to_uint();
        uint32_t brd_b = s_br_rd.read().to_uint(), bwr_b = s_br_wr.read().to_uint();
        unsigned char fa[120];
        for (int i = 0; i < 120; i++) fa[i] = (unsigned char)((i * 3 + 0x21) & 0xff);
        chk("帧A: 经 PCS link 环回逐字节一致", (uint32_t)(round_trip(fa, 120, "帧A") == 120), 1);
        unsigned char fb[200];
        for (int i = 0; i < 200; i++) fb[i] = (unsigned char)((i * 5 + 0x11) & 0xff);
        fb[7] = 0xFB; fb[80] = 0xFD; fb[3] = 0x07; fb[4] = 0x55; fb[5] = 0xD5; fb[98] = 0xFB;
        chk("帧B(陷阱字节): 逐字节一致", (uint32_t)(round_trip(fb, 200, "帧B") == 200), 1);
        for (int i = 0; i < 4; i++) tick();                    // settle(观测口跨 delta)✓
        chk("dev mac_tx_frames = 2", s_mtx.read().to_uint(), 2);
        chk("dev mac_rx_frames = 2", s_mrx.read().to_uint(), 2);
        chk("mac o_frames_tx = 2", s_mftx.read().to_uint(), 2);
        chk("mac o_frames_rx = 2", s_mfrx.read().to_uint(), 2);
        chk("mac o_fcs_err = 0", s_mfcs.read().to_uint(), 0);
        chk("pcs o_bit_errs = 0", s_lberr.read().to_uint(), 0);
        chk("pcs 列进出均 ≥2", (uint32_t)(s_lbin.read().to_uint() >= 2 && s_lbout.read().to_uint() >= 2), 1);
        chk("pcs align=1 && lane_locked=0xFFFF",
            (uint32_t)(s_lalign.read() && s_llocked.read().to_uint() == 0xFFFF), 1);
        chk("互不干扰:dev wqe/cqe 不变",
            (uint32_t)(s_wqe.read().to_uint() == wqe_b && s_cqe.read().to_uint() == cqe_b), 1);
        chk("互不干扰:桥 rd/wr 不变(帧路不碰内存)",
            (uint32_t)(s_br_rd.read().to_uint() == brd_b && s_br_wr.read().to_uint() == bwr_b), 1);
        chk("丢弃计数全 0", (uint32_t)(p_dev->mac_tx_drop_cnt == 0 && p_dev->mac_rx_drop_cnt == 0), 1);
        done = true;
        sc_core::sc_stop();                                    // 判完即停(免跑满预算)✓
    }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n("rst_n");

    DeviceTlm   dev("dev");
    DummyAmbaMaster dum("dum");                 // ★ 占位主端(reg_s 必绑;E109 的 socket 版)✓
    dev.reg_s.bind(dum.m);

    // ★ 线侧两模型 —— **撤 MAC 哑绑定、接真 MacTlm + PcsLinkTlm**(绑线见下)✓
    MacTlm      mac("mac");
    PcsLinkTlm  pcs("pcs");

    Cxl2AceBridge br("br");
    AceHostMem  mem("mem");
    Driver      drv("drv");

    dev.clk(clk); dev.rst_n(rst_n);
    br.clk(clk);  br.rst_n(rst_n);
    drv.clk(clk); drv.rst_n(rst_n);
    drv.p_dev = &dev; drv.p_mem = &mem; drv.p_br = &br;
    mac.clk(clk); mac.rst_n(rst_n); pcs.clk(clk); pcs.rst_n(rst_n);

    // ---- 设备 ↔ 桥:CXL.cache 引脚直连(逐条显式信号)✓ ----
    sc_core::sc_signal<bool> d_r0v, d_r0r, d_r1v, d_r1r, d_dv, d_dr, d_rv_, d_rr_;
    sc_core::sc_signal<bool> s_sqv, s_sqr, s_drv_, s_drr, h_qrdy, h_srdy, h_drdy, h_rv, h_rr_, h_dv_;
    sc_core::sc_signal<sc_uint<5>>  d_r0op, d_r1op, d_rop;
    sc_core::sc_signal<sc_uint<4>>  h_rop;
    sc_core::sc_signal<sc_uint<3>>  h_rop3;
    sc_core::sc_signal<sc_uint<12>> d_r0cq, d_r1cq, d_duq, d_ruq, s_squq, h_rrq, h_dcq;
    sc_core::sc_signal<sc_uint<12>> h_rspd, h_dcq2;   // ⚠ 桥的两个输出别共用一条线(E115 双驱动)✓
    sc_core::sc_signal<sc_uint<46>> d_r0ad, d_r1ad, s_sqad, h_rad;
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

    // ================= ★ 设备 ⇄ MAC ⇄ PCS link(线侧链)=================
    // dev ⇄ mac:NIC 侧 AXIS(dev 出 = mac 入;同一信号)✓
    sc_core::sc_signal<bool>              D_tvalid, D_tlast, D_tready;
    sc_core::sc_signal<sc_biguint<2048>>  D_tdata;
    sc_core::sc_signal<sc_biguint<256>>   D_tkeep;
    dev.cmac_m_axis_tvalid(D_tvalid); mac.cmac_m_axis_tvalid(D_tvalid);
    dev.cmac_m_axis_tdata(D_tdata);   mac.cmac_m_axis_tdata(D_tdata);
    dev.cmac_m_axis_tkeep(D_tkeep);   mac.cmac_m_axis_tkeep(D_tkeep);
    dev.cmac_m_axis_tlast(D_tlast);   mac.cmac_m_axis_tlast(D_tlast);
    dev.cmac_m_axis_tready(D_tready); mac.cmac_m_axis_tready(D_tready);
    // mac → dev:roce RX AXIS(无 ready;tuser 同绑)✓
    sc_core::sc_signal<bool>              RD_tvalid, RD_tlast, RD_tuser;
    sc_core::sc_signal<sc_biguint<2048>>  RD_tdata;
    sc_core::sc_signal<sc_biguint<256>>   RD_tkeep;
    mac.roce_cmac_s_axis_tvalid(RD_tvalid); dev.roce_cmac_s_axis_tvalid(RD_tvalid);
    mac.roce_cmac_s_axis_tdata(RD_tdata);   dev.roce_cmac_s_axis_tdata(RD_tdata);
    mac.roce_cmac_s_axis_tkeep(RD_tkeep);   dev.roce_cmac_s_axis_tkeep(RD_tkeep);
    mac.roce_cmac_s_axis_tlast(RD_tlast);   dev.roce_cmac_s_axis_tlast(RD_tlast);
    mac.roce_cmac_s_axis_tuser(RD_tuser);   dev.roce_cmac_s_axis_tuser(RD_tuser);
    // mac 线侧 ⇄ pcs link(★ 免 glue 对偶直连;照 规格文档 §2)✓
    sc_core::sc_signal<sc_biguint<512>> T_0, T_1, R_0, R_1;
    sc_core::sc_signal<sc_biguint<64>>  TC_0, TC_1, RC_0, RC_1;
    sc_core::sc_signal<bool> T_dval_0, T_dval_1, E_tx_0, E_tx_1, E_rx_0, E_rx_1;
    mac.link_txd_0(T_0);   mac.link_txd_1(T_1);   pcs.link_txd_0(T_0);   pcs.link_txd_1(T_1);
    mac.link_txc_0(TC_0);  mac.link_txc_1(TC_1);  pcs.link_txc_0(TC_0);  pcs.link_txc_1(TC_1);
    mac.link_txdval_0(T_dval_0); mac.link_txdval_1(T_dval_1);
    pcs.link_txdval_0(T_dval_0); pcs.link_txdval_1(T_dval_1);
    mac.link_rxd_0(R_0);   mac.link_rxd_1(R_1);   pcs.link_rxd_0(R_0);   pcs.link_rxd_1(R_1);
    mac.link_rxc_0(RC_0);  mac.link_rxc_1(RC_1);  pcs.link_rxc_0(RC_0);  pcs.link_rxc_1(RC_1);
    mac.link_txclk_ena_0(E_tx_0); mac.link_txclk_ena_1(E_tx_1);
    pcs.link_txclk_ena_0(E_tx_0); pcs.link_txclk_ena_1(E_tx_1);
    mac.link_rxclk_ena_0(E_rx_0); mac.link_rxclk_ena_1(E_rx_1);
    pcs.link_rxclk_ena_0(E_rx_0); pcs.link_rxclk_ena_1(E_rx_1);
    // 线侧观测(dev/mac/pcs)✓
    sc_core::sc_signal<sc_uint<32>> o_mtx, o_mrx, o_mftx, o_mfrx, o_mfcs, o_midle;
    sc_core::sc_signal<sc_uint<32>> o_mfcs_lm;
    sc_core::sc_signal<sc_uint<32>> o_lbin, o_lbout, o_lberr;
    sc_core::sc_signal<bool> o_align; sc_core::sc_signal<sc_uint<16>> o_locked;
    dev.o_mac_tx_frames(o_mtx); dev.o_mac_rx_frames(o_mrx);
    sc_core::sc_signal<sc_uint<32>> o_ltx, o_lack, o_lun, o_lbig;   // ★ v1.0 线侧口 ✓
    dev.o_line_tx_pkts(o_ltx); dev.o_line_rx_acks(o_lack);
    dev.o_line_unexp_cnt(o_lun); dev.o_line_big_cnt(o_lbig);
    mac.o_frames_tx(o_mftx); mac.o_frames_rx(o_mfrx); mac.o_idle_beats(o_midle); mac.o_fcs_err(o_mfcs);
    mac.o_lane_mismatch_cnt(o_mfcs_lm);
    pcs.o_beats_in(o_lbin); pcs.o_beats_out(o_lbout); pcs.o_bit_errs(o_lberr);
    pcs.fec_rx_align_status(o_align); pcs.fec_rx_lane_locked(o_locked);

    // ---- 桥 ↔ 主机内存(经检查器)✓
    amba_pv::amba_pv_ace_protocol_checker<512> chk("chk");       // ★ ACE 侧 oracle 装回 ✓
    br.ace_m.bind(chk.amba_pv_s);
    chk.amba_pv_m.bind(mem.ace_s);

    // ---- 观测 ----
    sc_core::sc_signal<bool> s_intr;
    sc_core::sc_signal<sc_uint<32>> o_sq, o_rq, o_ro, o_wqe, o_cqe, o_dbd, br_rd, br_wr, br_err, br_snp;
    dev.nic_intr(s_intr);
    dev.o_db_sq_cnt(o_sq); dev.o_db_rq_cnt(o_rq); dev.o_ro_wr_cnt(o_ro);
    dev.o_wqe_cnt(o_wqe); dev.o_cqe_cnt(o_cqe); dev.o_db_drop_cnt(o_dbd);
    br.o_rd_done(br_rd); br.o_wr_done(br_wr); br.o_err(br_err); br.o_snoop_cnt(br_snp);
    // ★ v1.0 新观测口(观测口也算端口,必须显式绑 ⇒ E109 教训)✓
    sc_core::sc_signal<sc_uint<32>> o_ostd, o_actqp, o_goerr, o_br_ostd;
    dev.o_max_rd_ostd(o_ostd); dev.o_max_act_per_qp(o_actqp); dev.o_goerr_cnt(o_goerr);
    sc_core::sc_signal<sc_uint<32>> o_rnr, o_osimpl;                 // ★ v1.0 ✓
    dev.o_rnr_cnt(o_rnr); dev.o_opc_simpl_cnt(o_osimpl);
    sc_core::sc_signal<sc_uint<32>> o_snpheld;                       // ★ v1.0 snoop 被压计数 ✓
    dev.o_snp_held_cnt(o_snpheld);
    sc_core::sc_signal<sc_uint<32>> o_qpf;                           // ★ v1.0 QP fatal 计数 ✓
    dev.o_qp_fatal_cnt(o_qpf);
    drv.s_qpf(o_qpf);
    sc_core::sc_signal<sc_uint<12>> o_frd, o_fwr, o_rmax, o_wmin;
    br.o_rd_ostd_max(o_br_ostd); br.o_fst_rd_cqid(o_frd); br.o_fst_wr_cqid(o_fwr);
    br.o_rd_cqid_max(o_rmax); br.o_wr_cqid_min(o_wmin);
    drv.s_intr(s_intr); drv.s_wqe(o_wqe); drv.s_cqe(o_cqe);
    drv.s_br_rd(br_rd); drv.s_br_wr(br_wr); drv.s_br_err(br_err); drv.s_br_snp(br_snp);
    drv.s_dbd(o_dbd);
    drv.s_ostd(o_ostd); drv.s_actqp(o_actqp); drv.s_goerr(o_goerr); drv.s_br_ostd(o_br_ostd);
    drv.s_frd(o_frd); drv.s_fwr(o_fwr); drv.s_rmax(o_rmax); drv.s_wmin(o_wmin);
    drv.s_rnr(o_rnr); drv.s_osimpl(o_osimpl); drv.s_snpheld(o_snpheld);
    drv.s_mtx(o_mtx); drv.s_mrx(o_mrx); drv.s_mftx(o_mftx); drv.s_mfrx(o_mfrx); drv.s_mfcs(o_mfcs);
    drv.s_lbin(o_lbin); drv.s_lbout(o_lbout); drv.s_lberr(o_lberr);
    drv.s_lalign(o_align); drv.s_llocked(o_locked);

    printf("============================================================\n");
    printf("tb_full_chain - ★ 四模型整机串联:dev+br+ACE mem + MAC + PCS link\n");
    printf("============================================================\n");

    rst_n.write(false);
    sc_core::sc_start(10, sc_core::SC_NS);
    rst_n.write(true);
    sc_core::sc_start(60, sc_core::SC_US);   // ⚠ 预算放宽(e2e 段 + 帧路段;Driver 判完即 sc_stop)✓

    bool pass = (drv.nfail == 0) && drv.done;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_FULL_CHAIN %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
