//============================================================================
// tb_bridge_loopback.cpp — cxl2ace 桥 v1.0 回环自测  [2026-10-04]
//
// 拓扑(**与 v1.0 相反**:桥本身就是 ACE master ⇒ TB 从设备侧驱动)✓
//
//   设备侧伙伴(DevPartner,pin 级 syn_*)──[Cxl2AceBridge]── AMBA-PV ACE ──>
//      ├─ ACE 协议检查器(amba_pv_ace_protocol_checker,★ 库自带 = ACE 侧 oracle)✓
//      └─ ACE 从内存(AceSlaveMem,512b,64B 行)✓
//
// 用例(正控齐):① 设备写一行 A(**非均匀图案** 8×64b 各不同)② 设备读回同行 A ③ 逐字比对
// 判据:图案逐字一致 + 桥 `o_rd_done=1/o_wr_done=1/o_err=0` + 从内存 `rd=1/wr=1` +
//       **命令映射验证**(从内存看到读=ReadShared 0x1、写=WriteUnique 0x0)+ **地址换算验证**
//       (从内存看到的字节地址 == 行地址 << 6)✓
// 纪律:设备侧 pin 级 = 1 拍/线 ⇒ 握手判据用 §同一 slot 线对(见 cxl2ace_bridge.h 文件头)✓
//============================================================================
#include <systemc.h>
#include <amba_pv.h>
#include <models/amba_pv_ace_protocol_checker.h>
#include "cxl2ace_bridge.h"

static const int LINEW = cxlshim::LINE_BYTES;   // 64B

// 非均匀图案:第 i 个 64b 字(项目教训:恒定载荷会掩盖"取错区间" ✗)
static uint64_t pat_word(int i) { return 0xA5A50000ULL + (uint64_t)(i * 0x0111); }

static uint64_t patB_word(int i) { return 0x5B5B0000ULL + (uint64_t)(i * 0x1111); }
static sc_dt::sc_biguint<512> patB_line() {
    sc_dt::sc_biguint<512> L = 0;
    for (int i = 0; i < 8; i++) L.range(64 * i + 63, 64 * i) = patB_word(i);
    return L;
}

static sc_dt::sc_biguint<512> pat_line() {
    sc_dt::sc_biguint<512> L = 0;
    for (int i = 0; i < 8; i++) L.range(64 * i + 63, 64 * i) = pat_word(i);
    return L;
}

//----------------------------------------------------------------------------
// 设备侧伙伴:先写一行、再读回同一行、逐字比对(判据在此)✓
//----------------------------------------------------------------------------
struct DevPartner: sc_core::sc_module {
    sc_in<bool> clk, rst_n;
    // D2H(驱动桥)
    sc_out<bool>        req0_valid; sc_out<sc_uint<5>> req0_opcode;
    sc_out<sc_uint<12>> req0_cqid;  sc_out<sc_uint<46>> req0_addr;  sc_in<bool> req0_ready;
    sc_out<bool>        req1_valid; sc_out<sc_uint<5>> req1_opcode;
    sc_out<sc_uint<12>> req1_cqid;  sc_out<sc_uint<46>> req1_addr;  sc_in<bool> req1_ready;
    sc_out<bool>        data_valid; sc_out<sc_uint<12>> data_uqid;  sc_out<bool> data_chunk_valid;
    sc_out<sc_biguint<512>> data_payload; sc_in<bool> data_ready;
    // H2D snoop(收桥发来的 snoop)+ D2H Rsp(回 RspI)★ v1.0
    sc_in<bool>         h2d_req_valid;  sc_in<sc_uint<3>>  h2d_req_opcode;
    sc_in<sc_uint<46>>  h2d_req_addr;   sc_in<sc_uint<12>> h2d_req_uqid; sc_out<bool> h2d_req_ready;
    sc_out<bool>        d2h_rsp_valid;  sc_out<sc_uint<5>> d2h_rsp_opcode;
    sc_out<sc_uint<12>> d2h_rsp_uqid;   sc_in<bool>        d2h_rsp_ready;
    // H2D(收桥回程)
    sc_in<bool> h2d_rsp_valid; sc_in<sc_uint<4>> h2d_rsp_opcode;
    sc_in<sc_uint<12>> h2d_rsp_rsp_data; sc_in<sc_uint<12>> h2d_rsp_cqid; sc_out<bool> h2d_rsp_ready;
    sc_in<bool> h2d_data_valid; sc_in<sc_uint<12>> h2d_data_cqid;
    sc_in<bool> h2d_data_chunk_valid; sc_in<sc_biguint<512>> h2d_data_payload; sc_out<bool> h2d_data_ready;

    // ★ v1.0:相位 2 = **连投 3 笔不等完成**(W(cqid11,行0x2000) + R(cqid12,行0x1234)
    //   + R(cqid13,行0x2000))⇒ 真压桥的请求队列(QDEPTH=4)✓
    enum { D_WR_REQ, D_WR_DATA, D_RD_REQ, D_RD_GO, D_RD_DATA,
           D_W2_REQ, D_W2_DATA, D_R2_REQ, D_R3_REQ, D_P_GO, D_P_DATA, D_DONE } ds;
    int p_rsp = 0;                      // 相位 2 已收的读响应数(2 = 完)
    int p_bad = 0;                      // 相位 2 比对失败数
    sc_uint<46> A;                       // 测试行地址(行地址口径 ✓)
    int wr_bad = 0, rd_gobad = 0;
    sc_biguint<512> rd_line; sc_uint<12> cur_rcq = 0;
    bool rv_prev = false;                // 我方 GO-ready 上一拍上线值(§slot 线对)
    bool hd_prev = false;                // 我方 data-ready 上一拍上线值
    // ★ v1.0 snoop 应答(独立子 FSM,与主流程并行)✓
    enum { SN_IDLE, SN_RSP } sn_st = SN_IDLE;
    sc_uint<3> sn_op; sc_uint<46> sn_addr; sc_uint<12> sn_uqid;
    uint32_t sn_cnt = 0; bool sn_ready_prev = false, sn_rsp_v_prev = false;

    void run() {
        if (!rst_n.read()) {
            ds = D_WR_REQ; A = 0x1234; rv_prev = false; hd_prev = false;
            req0_valid.write(false); req1_valid.write(false); data_valid.write(false);
            h2d_rsp_ready.write(false); h2d_data_ready.write(false);
            req0_opcode.write(cxlshim::D2H_REQ_SPEC_RD_SHARED);
            req1_opcode.write(cxlshim::D2H_REQ_MEMWR);
            sn_st = SN_IDLE; sn_cnt = 0; sn_ready_prev = false; sn_rsp_v_prev = false;
            p_rsp = 0; p_bad = 0;
            h2d_req_ready.write(false); d2h_rsp_valid.write(false);
            req0_cqid.write(2); req1_cqid.write(1);
            req0_addr.write(0); req1_addr.write(0);
            data_uqid.write(1); data_chunk_valid.write(true); data_payload.write(pat_line());
            return;
        }
        // ---- 我方 ready 输出(纯函数;收到即撤)✓ ----
        bool rv = (ds == D_RD_GO) || (ds == D_P_GO);          // ★ 相位 2 也要抬(首版漏,卡死)✓
        bool hd = (ds == D_RD_DATA) || (ds == D_P_DATA);
        h2d_rsp_ready.write(rv);
        h2d_data_ready.write(hd);
        // ---- 交付判定(同一 slot 线对:我上一拍输出 ∧ 对方本拍读值)✓ ----
        bool go_xfer = h2d_rsp_valid.read() && rv_prev;
        bool hd_xfer = h2d_data_valid.read() && hd_prev;
        rv_prev = rv; hd_prev = hd;

        switch (ds) {
        case D_WR_REQ:                       // 写请求:MemWr/WrCur(0 0111b)✓
            req1_addr.write(A);
            if (!req1_valid.read()) { req1_valid.write(true); }       // 先抬,下一拍才看 ready ✓
            else if (req1_ready.read()) { req1_valid.write(false); ds = D_WR_DATA; }
            break;
        case D_WR_DATA:                      // 写数据b 整行一拍 ✓
            if (!data_valid.read()) { data_valid.write(true); }
            else if (data_ready.read()) { data_valid.write(false); ds = D_RD_REQ; }
            break;
        case D_RD_REQ:                       // 读请求:RD_SHARED(5'h03)✓
            req0_addr.write(A);
            if (!req0_valid.read()) { req0_valid.write(true); }
            else if (req0_ready.read()) { req0_valid.write(false); ds = D_RD_GO; }
            break;
        case D_RD_GO:                        // 等 GO(查 opcode / 状态)✓
            if (go_xfer) {
                if (h2d_rsp_opcode.read() != cxlshim::H2D_RSP_SPEC_GO) wr_bad++;
                if ((h2d_rsp_rsp_data.read() & 0x7) != cxlshim::CXL_GO_STATE_S) rd_gobad++;
                ds = D_RD_DATA;
            }
            break;
        case D_RD_DATA:                      // 相位 1 收数据(512b 整行一拍)✓
            if (hd_xfer) { rd_line = h2d_data_payload.read(); ds = D_W2_REQ; }
            break;
        // ---- 相位 2:连投(不等完成)----
        case D_W2_REQ:
            req1_addr.write(0x2000); req1_cqid.write(11);
            if (!req1_valid.read()) { req1_valid.write(true); }
            else if (req1_ready.read()) { req1_valid.write(false); ds = D_W2_DATA; }
            break;
        case D_W2_DATA:
            data_payload.write(patB_line()); data_uqid.write(11);
            if (!data_valid.read()) { data_valid.write(true); }
            else if (data_ready.read()) { data_valid.write(false); ds = D_R2_REQ; }
            break;
        case D_R2_REQ:
            req0_addr.write(0x1234); req0_cqid.write(12);
            if (!req0_valid.read()) { req0_valid.write(true); }
            else if (req0_ready.read()) { req0_valid.write(false); ds = D_R3_REQ; }
            break;
        case D_R3_REQ:                       // ③ 第二笔读**立刻跟上**(前两笔还没回来)✓
            req0_addr.write(0x2000); req0_cqid.write(13);
            if (!req0_valid.read()) { req0_valid.write(true); }
            else if (req0_ready.read()) { req0_valid.write(false); ds = D_P_GO; }
            break;
        case D_P_GO:
            if (go_xfer) { cur_rcq = h2d_rsp_cqid.read(); ds = D_P_DATA; }
            break;
        case D_P_DATA:                       // 按 cqid 查期望图案比对 ✓
            if (hd_xfer) {
                sc_dt::sc_biguint<512> exp = (cur_rcq.to_uint() == 12) ? pat_line() : patB_line();
                sc_dt::sc_biguint<512> got = h2d_data_payload.read();
                for (int i = 0; i < 8; i++)
                    if (got.range(64*i+63, 64*i) != exp.range(64*i+63, 64*i)) p_bad++;
                p_rsp++;
                ds = (p_rsp == 2) ? D_DONE : D_P_GO;
            }
            break;
        case D_DONE:
            break;
        }
        // ---- snoop 应答子 FSM(恒 RspI = D2H_RSP_SPEC_I_HIT_I ✓;照 RTL"snoop 恒 RspI")----
        bool sn_rdy = (sn_st == SN_IDLE);
        h2d_req_ready.write(sn_rdy);
        bool sn_rv = (sn_st == SN_RSP);
        d2h_rsp_valid.write(sn_rv);
        if (sn_st == SN_RSP) { d2h_rsp_opcode.write(cxlshim::D2H_RSP_SPEC_I_HIT_I); d2h_rsp_uqid.write(sn_uqid); }
        if (sn_st == SN_IDLE) {
            if (sn_ready_prev && h2d_req_valid.read()) {          // 收 snoop(同 slot 线对)✓
                sn_op = h2d_req_opcode.read(); sn_addr = h2d_req_addr.read();
                sn_uqid = h2d_req_uqid.read(); sn_cnt++;
                sn_st = SN_RSP;
            }
        } else if (sn_rsp_v_prev && d2h_rsp_ready.read()) {        // 桥已收 ⇒ 收工 ✓
            sn_st = SN_IDLE;
        }
        sn_ready_prev = sn_rdy; sn_rsp_v_prev = sn_rv;
    }
    bool check() {                            // ★ 判据:逐字比对 + 正控 ✓
        int bad = rd_gobad + wr_bad;
        sc_dt::sc_biguint<512> exp = pat_line();
        for (int i = 0; i < 8; i++) {
            sc_dt::uint64 got = rd_line.range(64 * i + 63, 64 * i).to_uint64();
            if (got != pat_word(i)) {
                printf("   ✗ [判据] 第 %d 个 64b 字不符:读到 %016llx 期望 %016llx\n",
                       i, (unsigned long long)got, (unsigned long long)pat_word(i));
                bad++;
            }
        }
        printf("   正控:读地址 == 写地址(行 0x%llx)✓ 图案非均匀 ✓\n", (unsigned long long)A.to_uint64());
        printf("   GO 检查:opcode 错 %d 笔 / 状态非-S %d 笔\n", wr_bad, rd_gobad);
        if (p_bad) { printf("   ✗ [判据] 相位 2(多 outstanding)有 %d 个 64b 字不符\n", p_bad); bad += p_bad; }
        if (p_rsp != 2) { printf("   ✗ [判据] 相位 2 只收到 %d/2 笔读响应\n", p_rsp); bad++; }
        printf("   相位 2(连投 3 笔:W/R/R 不等完成):收 %d/2 笔读响应,字比对错 %d ✓\n", p_rsp, p_bad);
        return bad == 0 && ds == D_DONE;
    }
    SC_CTOR(DevPartner) { SC_METHOD(run); sensitive << clk.pos(); }
};

//----------------------------------------------------------------------------
// ACE 从内存(主机侧)b 一拍一行;记录命令映射与地址换算(判据用)✓
//   带延迟标注(10ns)⇒ 桥内 `wait(t)` 被真走到 ✓
//----------------------------------------------------------------------------
struct AceSlaveMem: sc_core::sc_module, amba_pv::amba_pv_ace_slave_base {
    amba_pv::amba_pv_ace_slave_socket<512> ace_s;

    unsigned char mem[4096 * LINEW];
    uint32_t rd_served = 0, wr_taken = 0, bad = 0;
    int last_rd_snoop = -1, last_wr_snoop = -1;
    uint64_t last_rd_addr = ~0ULL, last_wr_addr = ~0ULL;
    uint64_t rd_addrs[8]; int rd_addr_n = 0;      // ★ 全集合(相位 1+2)✓
    uint64_t wr_addrs[8]; int wr_addr_n = 0;

    // ★ v1.0:从端主动发 snoop(验证桥的 snoop 通道)✓
    amba_pv::amba_pv_trans_pool m_pool;
    bool snp_done = false; int snp_bits = -1;
    SC_HAS_PROCESS(AceSlaveMem);                 // 自定义 ctor + SC_THREAD ⇒ 必须 ✓

    AceSlaveMem(sc_core::sc_module_name nm): amba_pv_ace_slave_base("ace_slave"), ace_s("ace_s") {
        ace_s(*this);
        for (int i = 0; i < 4096 * LINEW; i++) mem[i] = 0;
        SC_THREAD(snoop_thread);
    }

    void snoop_thread() {
        wait(5, sc_core::SC_US);                                 // 等回环跑完(单次 sc_start 内)✓
        amba_pv::amba_pv_trans_ptr trans(m_pool.allocate(1, LINEW, NULL, amba_pv::AMBA_PV_INCR));
        amba_pv::amba_pv_extension * ex = NULL;
        trans->get_extension(ex);
        trans->set_address((sc_dt::uint64)0x1234 << 6);          // 与回环同一行 ✓
        trans->set_data_length(LINEW);
        if (ex != NULL) {
            ex->set_snoop((amba_pv::amba_pv_snoop_t)amba_pv::AMBA_PV_READ_UNIQUE);   // → SNP_INV(桥侧映射)✓
        }
        sc_core::sc_time t = sc_core::SC_ZERO_TIME;
        ace_s.b_snoop(*trans, t);                                 // ★ 阻塞:桥服务完才返回 ✓
        wait(t);
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

    void b_transport(int, amba_pv::amba_pv_transaction & trans, sc_core::sc_time & t) override {
        amba_pv::amba_pv_extension * ex = NULL;
        trans.get_extension(ex);
        int snp = (ex != NULL) ? (int)ex->get_snoop() : -1;
        uint64_t a = (uint64_t)trans.get_address();
        size_t off = (size_t)((a >> 6) & 0xFFF) * LINEW;        // 行号取模(地址换算已在桥侧完成 ✓)
        unsigned char * d = trans.get_data_ptr();
        if (trans.is_read()) {
            rd_served++; last_rd_snoop = snp; last_rd_addr = a;
            if (rd_addr_n < 8) rd_addrs[rd_addr_n++] = a;
            for (int i = 0; i < LINEW; i++) d[i] = mem[off + i];
        } else if (trans.is_write()) {
            wr_taken++; last_wr_snoop = snp; last_wr_addr = a;
            if (wr_addr_n < 8) wr_addrs[wr_addr_n++] = a;
            for (int i = 0; i < LINEW; i++) mem[off + i] = d[i];
        } else { bad++; }
        if (ex != NULL) { ex->set_resp(amba_pv::AMBA_PV_OKAY); }
        t += sc_core::sc_time(10, sc_core::SC_NS);              // 从端延迟 ⇒ 桥 `wait(t)` 生效 ✓
    }
    unsigned int transport_dbg(int, amba_pv::amba_pv_transaction &) override { return 0; }
    bool get_direct_mem_ptr(int, amba_pv::amba_pv_transaction &, tlm::tlm_dmi &) override { return false; }
};

//----------------------------------------------------------------------------
int sc_main(int, char **) {
    sc_core::sc_clock clk("clk", 2.0, sc_core::SC_NS);       // 500MHz
    sc_core::sc_signal<bool> rst_n("rst_n");

    Cxl2AceBridge br("br");
    DevPartner    dev("dev");
    AceSlaveMem   slv("slv");
    amba_pv::amba_pv_ace_protocol_checker<512> chk("chk");   // ★ ACE 侧 oracle ✓

    br.clk(clk); br.rst_n(rst_n);
    dev.clk(clk); dev.rst_n(rst_n);

    // 设备侧接线(★ 逐条显式 sc_signal:版图清晰,且避开端口直连的编译限制 ✓)
    sc_core::sc_signal<bool>        n_r0v, n_r0r, n_r1v, n_r1r, n_dv, n_dr, n_gv, n_gr, n_hv, n_hr, n_dcv, n_hcv;
    sc_core::sc_signal<sc_uint<5>>  n_r0op, n_r1op;
    sc_core::sc_signal<sc_uint<4>>  n_gop;
    sc_core::sc_signal<sc_uint<12>> n_r0cq, n_r1cq, n_duq, n_grd, n_gcq, n_hcq;
    sc_core::sc_signal<sc_uint<46>> n_r0ad, n_r1ad;
    sc_core::sc_signal<sc_biguint<512>> n_dpl, n_hpl;

    dev.req0_valid(n_r0v);  br.d2h_req0_valid(n_r0v);
    dev.req0_opcode(n_r0op); br.d2h_req0_opcode(n_r0op);
    dev.req0_cqid(n_r0cq);  br.d2h_req0_cqid(n_r0cq);
    dev.req0_addr(n_r0ad);  br.d2h_req0_addr(n_r0ad);
    dev.req0_ready(n_r0r);  br.d2h_req0_ready(n_r0r);
    dev.req1_valid(n_r1v);  br.d2h_req1_valid(n_r1v);
    dev.req1_opcode(n_r1op); br.d2h_req1_opcode(n_r1op);
    dev.req1_cqid(n_r1cq);  br.d2h_req1_cqid(n_r1cq);
    dev.req1_addr(n_r1ad);  br.d2h_req1_addr(n_r1ad);
    dev.req1_ready(n_r1r);  br.d2h_req1_ready(n_r1r);
    dev.data_valid(n_dv);   br.d2h_data_valid(n_dv);
    dev.data_uqid(n_duq);   br.d2h_data_uqid(n_duq);
    dev.data_chunk_valid(n_dcv); br.d2h_data_chunk_valid(n_dcv);
    dev.data_payload(n_dpl); br.d2h_data_payload(n_dpl);
    dev.data_ready(n_dr);   br.d2h_data_ready(n_dr);
    br.h2d_rsp_valid(n_gv); dev.h2d_rsp_valid(n_gv);
    br.h2d_rsp_opcode(n_gop); dev.h2d_rsp_opcode(n_gop);
    br.h2d_rsp_rsp_data(n_grd); dev.h2d_rsp_rsp_data(n_grd);
    br.h2d_rsp_cqid(n_gcq); dev.h2d_rsp_cqid(n_gcq);
    dev.h2d_rsp_ready(n_gr); br.h2d_rsp_ready(n_gr);
    br.h2d_data_valid(n_hv); dev.h2d_data_valid(n_hv);
    br.h2d_data_cqid(n_hcq); dev.h2d_data_cqid(n_hcq);
    br.h2d_data_chunk_valid(n_hcv); dev.h2d_data_chunk_valid(n_hcv);
    br.h2d_data_payload(n_hpl); dev.h2d_data_payload(n_hpl);
    dev.h2d_data_ready(n_hr); br.h2d_data_ready(n_hr);

    // 观测口(⚠ 也要绑;读数改成读这些信号 ✓)
    sc_core::sc_signal<sc_uint<32>> n_rdd, n_wrd, n_err, n_snp;
    br.o_rd_done(n_rdd); br.o_wr_done(n_wrd); br.o_err(n_err); br.o_snoop_cnt(n_snp);
    // ★ v1.0 新观测口(多笔在飞/槽号口径);观测口也算端口,必须显式绑 ✓
    sc_core::sc_signal<sc_uint<32>> n_ostd;
    sc_core::sc_signal<sc_uint<12>> n_frd, n_fwr, n_rmax, n_wmin;
    br.o_rd_ostd_max(n_ostd); br.o_fst_rd_cqid(n_frd); br.o_fst_wr_cqid(n_fwr);
    br.o_rd_cqid_max(n_rmax); br.o_wr_cqid_min(n_wmin);

    // ★ v1.0:snoop 通道 桥 ↔ 设备伙伴 + D2H Rsp 回程 ✓
    sc_core::sc_signal<bool>         n_sqv, n_sqr, n_drv, n_drr;
    sc_core::sc_signal<sc_uint<3>>   n_sqop;
    sc_core::sc_signal<sc_uint<5>>   n_drop;
    sc_core::sc_signal<sc_uint<46>>  n_sqad;
    sc_core::sc_signal<sc_uint<12>>  n_squq, n_druq;
    br.h2d_req_valid(n_sqv); dev.h2d_req_valid(n_sqv);
    br.h2d_req_opcode(n_sqop); dev.h2d_req_opcode(n_sqop);
    br.h2d_req_addr(n_sqad); dev.h2d_req_addr(n_sqad);
    br.h2d_req_uqid(n_squq); dev.h2d_req_uqid(n_squq);
    br.h2d_req_ready(n_sqr); dev.h2d_req_ready(n_sqr);
    dev.d2h_rsp_valid(n_drv); br.d2h_rsp_valid(n_drv);
    dev.d2h_rsp_opcode(n_drop); br.d2h_rsp_opcode(n_drop);
    dev.d2h_rsp_uqid(n_druq); br.d2h_rsp_uqid(n_druq);
    dev.d2h_rsp_ready(n_drr); br.d2h_rsp_ready(n_drr);

    // 主机侧 ACE:桥 → 检查器 → 从内存 ✓
    br.ace_m.bind(chk.amba_pv_s);
    chk.amba_pv_m.bind(slv.ace_s);

    printf("============================================================\n");
    printf("tb_bridge_loopback — cxl2ace 桥 v1.0(pin 设备侧 ↔ AMBA-PV ACE,512b/行)✓\n");
    printf("============================================================\n");

    rst_n.write(false);
    sc_core::sc_start(10, sc_core::SC_NS);
    rst_n.write(true);
    sc_core::sc_start(20, sc_core::SC_US);                   // 跑完或超时

    bool ok = dev.check();
    printf("   桥计数:rd_done=%u wr_done=%u err=%u snoop=%u | 从内存:rd=%u wr=%u bad=%u\n",
           n_rdd.read().to_uint(), n_wrd.read().to_uint(),
           n_err.read().to_uint(), n_snp.read().to_uint(),
           slv.rd_served, slv.wr_taken, slv.bad);
    printf("   命令映射:读 snoop=0x%x(期望 0x1 ReadShared)/ 写 snoop=0x%x(期望 0x0 WriteUnique)\n",
           slv.last_rd_snoop, slv.last_wr_snoop);
    uint64_t e1 = (uint64_t)(dev.A.to_uint64() << 6), e2 = (uint64_t)0x2000 << 6;
    printf("   地址换算:读 %d 笔 / 写 %d 笔,字节地址全 ∈ {0x%llx, 0x%llx}(行<<6)✓\n",
           slv.rd_addr_n, slv.wr_addr_n, (unsigned long long)e1, (unsigned long long)e2);
    bool map_ok = (slv.last_rd_snoop == ACE_SNOOP_READ_SHARED) &&
                  (slv.last_wr_snoop == ACE_SNOOP_WRITE_UNIQUE);
    bool adr_ok = true;
    for (int i = 0; i < slv.rd_addr_n; i++) if (slv.rd_addrs[i] != e1 && slv.rd_addrs[i] != e2) adr_ok = false;
    for (int i = 0; i < slv.wr_addr_n; i++) if (slv.wr_addrs[i] != e1 && slv.wr_addrs[i] != e2) adr_ok = false;
    adr_ok = adr_ok && (slv.rd_addr_n == 3) && (slv.wr_addr_n == 2);
    bool cnt_ok = (n_rdd.read() == 3) && (n_wrd.read() == 2) &&
                  (n_err.read() == 0) && (slv.rd_served == 3) && (slv.wr_taken == 2) && (slv.bad == 0);
    // ★ v1.0 snoop 判据(设备应答恒 RspI ⇒ CRRESP 应全 0 = I_HIT_I)✓
    printf("   snoop:桥计数=%u | 设备收到 %u 笔(op=0x%x(**databook 1/2/3**)addr=0x%llx uqid=%u)| 从端 CRRESP=0x%x\n",
           n_snp.read().to_uint(), dev.sn_cnt, (unsigned)dev.sn_op.to_uint(),
           (unsigned long long)dev.sn_addr.to_uint(), (unsigned)dev.sn_uqid.to_uint(), slv.snp_bits);
    bool snp_ok = (n_snp.read() == 1) && (dev.sn_cnt == 1) &&
                  (dev.sn_op.to_uint() == 2) &&                       // ★ READ_UNIQUE → SNP_INV = **3'h2**(databook 1/2/3;v1.0 订正)✓
                  (dev.sn_addr.to_uint() == 0x1234) &&
                  (slv.snp_done) && (slv.snp_bits == 0);              // I_HIT_I:全 0 ✓
    if (!snp_ok) printf("   ✗ snoop 判据不符(见上;期望 计数=1/op=1/addr=0x1234/CRRESP=0)\n");
    if (!map_ok) printf("   ✗ 命令映射不符(见上)\n");
    if (!adr_ok) printf("   ✗ 地址换算不符(见上)\n");
    if (!cnt_ok) printf("   ✗ 计数不符(见上)\n");
    bool pass = ok && map_ok && adr_ok && cnt_ok && snp_ok;
    printf("PASS=%d FAIL=%d\n", pass ? 1 : 0, pass ? 0 : 1);
    printf("TB_BRIDGE_LOOPBACK %s\n", pass ? "PASS" : "FAIL");
    printf("============================================================\n");
    return pass ? 0 : 1;
}
