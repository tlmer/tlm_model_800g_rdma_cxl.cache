//============================================================================
// cxl2ace_bridge.cpp — 实现体(v1.0)  [2026-10-04]
//
// 交付结构约定:`.h` = 公开接口(客户 `sc_main` 要有)/ `.cpp` = 实现(编库交付)✓
// 线程结构(⚠ 防死锁,别改回单线程)与纪律全在 **.h 文件头** —— 读它就够 ✓
//============================================================================
#include "cxl2ace_bridge.h"

//----------------------------------------------------------------------------
// 行缓冲 ↔ sc_biguint<512>(小端字节序,照项目 aarch64-LE 口径 ✓)
//----------------------------------------------------------------------------
static void line_to_bytes(const sc_dt::sc_biguint<512> & L, unsigned char * b) {
    for (int i = 0; i < 8; i++) {
        sc_dt::uint64 w = L.range(64 * i + 63, 64 * i).to_uint64();
        for (int k = 0; k < 8; k++) b[i * 8 + k] = (unsigned char)((w >> (8 * k)) & 0xffULL);
    }
}

static sc_dt::sc_biguint<512> bytes_to_line(const unsigned char * b) {
    sc_dt::sc_biguint<512> L = 0;
    for (int i = 0; i < 8; i++) {
        sc_dt::uint64 w = 0;
        for (int k = 0; k < 8; k++) w |= (sc_dt::uint64)b[i * 8 + k] << (8 * k);
        L.range(64 * i + 63, 64 * i) = w;
    }
    return L;
}

//----------------------------------------------------------------------------
Cxl2AceBridge::Cxl2AceBridge(sc_core::sc_module_name nm):
    sc_core::sc_module(nm),
    amba_pv::amba_pv_ace_master_base("cxl2ace_bridge"),
    ace_m("ace_m") {
    ace_m(*this);                             // ★ ACE socket 回程(snoop)自绑 ✓
    ing = ING_IDLE; egr = EGR_IDLE; snp = SNP_IDLE;
    m_qh = m_qt = m_sh = m_st = 0; m_qn = m_sn = 0; m_rd_ostd = 0;
    m_snp_busy = m_snp_done = false; m_snp_have_data = false;
    m_snp_op3 = 0; m_snp_addr = 0; m_snp_uqid = 0; m_snp_d2h_op = 0;
    snp_uqid_gen = 0;
    rd_done = 0; wr_done = 0; err_cnt = 0; snoop_cnt = 0;
    rd_acc = 0; ostd_max_cnt = 0;
    fst_rd = 0xFFF; fst_wr = 0xFFF; rd_cmax = 0; wr_cmin = 0xFFF;
    fst_rd_seen = false; fst_wr_seen = false;
    rdy0_prev = rdy1_prev = dready_prev = go_prev = hd_prev = false;
    rsp_ready_prev = snp_vprev = false;
    SC_THREAD(pin_thread);   sensitive << clk.pos();
    SC_THREAD(issuer_thread);
}

//----------------------------------------------------------------------------
// 反向通道:收 snoop(在**调用者线程**里跑 ⇒ ⛔ 不碰引脚:只填槽 + 等事件)✓
//----------------------------------------------------------------------------
void Cxl2AceBridge::b_snoop(int, amba_pv::amba_pv_transaction & trans, sc_core::sc_time &) {
    amba_pv::amba_pv_extension * ex = NULL;
    trans.get_extension(ex);
    int snp_type = (ex != NULL) ? (int)ex->get_snoop() : 0;

    // ★★ snoop 类型 → 设备侧 3b opcode:**按 databook(Table 3-17)= 1/2/3**(2026-10-04 定案)——
    //   证据(第三方,参照树):`RTL 源`
    //     `H2D_REQ_SNP_DATA/INV/CUR = 3'h1/3'h2/3'h3` ✓ + 参照桥 `RTL 源`
    //     (用同名宏写 `syn_h2d_req.opcode`)✓
    //   ⚠ 原实现按 0/1/2(跟我们 RTL shim 的**错误解码** `RTL 源` 对齐)——
    //     该 shim 解码与 databook 不一致 = **RTL 侧真缺陷**(已报,见 差异清单 D12 + 桥规格 §8.3-6)✗✓
    //   ⇒ 模型**按 databook 走**(交付件要对 IP 正确,不对我们的 bug 正确)✓
    sc_uint<3> op3 = 3;                                   // default → SNP_CUR
    if (snp_type == amba_pv::AMBA_PV_READ_SHARED)      op3 = 1;   // → SNP_DATA
    else if (snp_type == amba_pv::AMBA_PV_READ_UNIQUE) op3 = 2;   // → SNP_INV
    else if (snp_type == amba_pv::AMBA_PV_MAKE_UNIQUE) op3 = 2;   // → SNP_INV(待开卷 ⚠)
    else if (snp_type == amba_pv::AMBA_PV_CLEAN_UNIQUE) op3 = 1;  // → SNP_DATA(待开卷 ⚠)

    m_snp_op3  = op3;
    m_snp_addr = (sc_uint<46>)(trans.get_address() >> 6);    // 字节 → 行地址 ✓
    m_snp_uqid = snp_uqid_gen++;
    m_snp_busy = true; m_snp_done = false;
    snoop_cnt  = snoop_cnt + 1;

    wait(m_ev_snp_done);                                     // 引脚线程服务完通知 ✓

    // ---- CRRESP 5 位回填(位语义 = 库注释;★ 2026-10-04 **位布局已定案**)----
    // ACE CRRESP bit 布局(参照树 libsystemctlm-soc
    //  `namespace CR`  五个提取器):
    //   bit0=DataTransfer · bit1=Error · bit2=PassDirty · bit3=IsShared · bit4=WasUnique ✓
    // 下表 = CXL D2H SnpResp(COH svh 表 3-25)⇒ ACE 置位;⚠ 同名不同码:CXL 侧 I_HIT_I=5'h04
    //   vs ACE 侧全 0 —— 本 switch 即两套 5 位码的**翻译器** ✓(设备不缓存 ⇒ 实际只发 I_HIT_I)
    bool data_xfer = false, is_shared = false, pass_dirty = false, was_uniq = false, is_err = false;
    switch ((int)m_snp_d2h_op) {
    case cxlshim::D2H_RSP_SPEC_I_HIT_I:  break;                       // 全 0 ✓(设备恒回这一路)
    case cxlshim::D2H_RSP_SPEC_I_HIT_SE: is_shared = true; break;
    case cxlshim::D2H_RSP_SPEC_S_HIT_SE: data_xfer = true; is_shared = true; break;
    case cxlshim::D2H_RSP_SPEC_V_HIT_V:  was_uniq = true; break;
    case cxlshim::D2H_RSP_SPEC_S_FWD_M:  data_xfer = true; pass_dirty = true; break;
    case cxlshim::D2H_RSP_SPEC_I_FWD_M:  data_xfer = true; pass_dirty = true; break;   // ★ 转发数据 ⇒ DataTransfer 必置(原缺 ⇒ 数据挂不上)✗✓
    case cxlshim::D2H_RSP_SPEC_V_FWD_V:  was_uniq = true; break;      // 组合按位布局推(本设备不可达,留注)
    default: is_err = true; err_cnt = err_cnt + 1; break;             // 未知 ⇒ 报错位
    }
    if (ex != NULL) {
        ex->set_snoop_data_transfer(data_xfer);
        ex->set_snoop_error(is_err);
        ex->set_pass_dirty(pass_dirty);
        ex->set_shared(is_shared);
        ex->set_snoop_was_unique(was_uniq);
    }
    if (data_xfer && m_snp_have_data) {
        trans.set_data_ptr(m_snp_data);
        trans.set_data_length(cxlshim::LINE_BYTES);
    }
    m_snp_busy = false;                                      // 槽归还(b_snoop 收尾)✓
}

//----------------------------------------------------------------------------
// ACE 事务(★ 只在 issuer 线程调用:阻塞 API)
//----------------------------------------------------------------------------
bool Cxl2AceBridge::ace_read_line(const sc_uint<46> & a, unsigned char * buf) {
    amba_pv::amba_pv_trans_ptr trans(
        m_pool.allocate(1, cxlshim::LINE_BYTES, NULL, amba_pv::AMBA_PV_INCR));
    amba_pv::amba_pv_extension * ex = NULL;
    trans->get_extension(ex);
    trans->set_read();
    trans->set_address((sc_dt::uint64)a.to_uint64() << 6);   // ★ 行地址 → 字节地址 ✓
    trans->set_data_ptr(buf);
    trans->set_data_length(cxlshim::LINE_BYTES);
    trans->set_streaming_width(cxlshim::LINE_BYTES);
    if (ex != NULL) {
        ex->set_snoop((amba_pv::amba_pv_snoop_t)ACE_SNOOP_READ_SHARED);   // 直通无缓存 ⇒ ReadShared ✓
        ex->set_domain((amba_pv::amba_pv_domain_t)amba_pv::AMBA_PV_INNER_SHAREABLE);  // ★ 定论(2026-10-08 双 TRM):IS 有据,见桥规格 §8.3-2
    }
    sc_core::sc_time t = sc_core::SC_ZERO_TIME;
    ace_m.b_transport(*trans, t);
    sc_core::wait(t);
    return (ex == NULL) || (ex->get_resp() != amba_pv::AMBA_PV_OKAY);
}

void Cxl2AceBridge::ace_write_line(const sc_uint<46> & a, const unsigned char * buf) {
    amba_pv::amba_pv_trans_ptr trans(
        m_pool.allocate(1, cxlshim::LINE_BYTES, NULL, amba_pv::AMBA_PV_INCR));
    amba_pv::amba_pv_extension * ex = NULL;
    trans->get_extension(ex);
    trans->set_write();
    trans->set_address((sc_dt::uint64)a.to_uint64() << 6);
    trans->set_data_ptr(const_cast<unsigned char *>(buf));
    trans->set_data_length(cxlshim::LINE_BYTES);
    trans->set_streaming_width(cxlshim::LINE_BYTES);
    if (ex != NULL) {
        ex->set_snoop((amba_pv::amba_pv_snoop_t)ACE_SNOOP_WRITE_UNIQUE);   // 0x0 ✓
        ex->set_domain((amba_pv::amba_pv_domain_t)amba_pv::AMBA_PV_INNER_SHAREABLE);  // ★ 定论(2026-10-08 双 TRM):IS 有据,见桥规格 §8.3-2
    }
    sc_core::sc_time t = sc_core::SC_ZERO_TIME;
    ace_m.b_transport(*trans, t);
    sc_core::wait(t);
    if ((ex == NULL) || (ex->get_resp() != amba_pv::AMBA_PV_OKAY)) {
        err_cnt = err_cnt + 1;                               // 写失败 ⇒ 只计数(照 RTL)✓
    }
}

//----------------------------------------------------------------------------
// issuer 线程:弹请求 → 阻塞 ACE 事务 → 推响应(⚠ 引脚一概不碰 ✓)
//----------------------------------------------------------------------------
void Cxl2AceBridge::issuer_thread() {
    while (true) {
        if (m_qn == 0) { wait(m_ev_req); continue; }         // 无请求(协作式调度,检查与 wait 间无抢占 ✓)
        ReqRec r = m_qreq[m_qh];
        m_qh = (m_qh + 1) % QDEPTH; m_qn--;
        if (r.wr) {
            line_to_bytes(r.line, m_iobuf);
            ace_write_line(r.addr, m_iobuf);
            wr_done = wr_done + 1;                           // posted:发完即计 ✓
        } else {
            RspRec s;
            s.cqid = r.cqid;
            s.err  = ace_read_line(r.addr, m_iobuf);
            if (s.err) { err_cnt = err_cnt + 1; }
            s.line = bytes_to_line(m_iobuf);
            // 响应队列容量 = 读在飞上限(入口按 m_rd_ostd 回压)⇒ 不会溢出 ✓;防御计数兜底 ✗
            if (m_sn >= QDEPTH) { err_cnt = err_cnt + 1; }
            m_qrsp[m_st] = s;
            m_st = (m_st + 1) % QDEPTH; m_sn++;
            m_rd_ostd--;                                         // ★ 一笔读已回响应(腾出一个读在飞位)✓
            m_ev_rsp.notify();
        }
    }
}

//----------------------------------------------------------------------------
// pin_thread:★ 唯一碰引脚者。三路并行子 FSM(snoop 服务 / 请求入队 / 读回程)+ 观测 ✓
//----------------------------------------------------------------------------
void Cxl2AceBridge::pin_thread() {
    // ---- 输出初值 ----
    d2h_req0_ready.write(false); d2h_req1_ready.write(false); d2h_data_ready.write(false);
    d2h_rsp_ready.write(false);
    h2d_rsp_valid.write(false); h2d_rsp_opcode.write(cxlshim::H2D_RSP_SPEC_GO); h2d_rsp_rsp_data.write(0); h2d_rsp_cqid.write(0);
    h2d_data_valid.write(false); h2d_data_cqid.write(0); h2d_data_chunk_valid.write(false); h2d_data_payload.write(0);
    h2d_req_valid.write(false); h2d_req_opcode.write(0); h2d_req_addr.write(0); h2d_req_uqid.write(0);
    o_rd_done.write(0); o_wr_done.write(0); o_err.write(0); o_snoop_cnt.write(0);
    o_rd_ostd_max.write(0); o_fst_rd_cqid.write(0xFFF); o_fst_wr_cqid.write(0xFFF);
    o_rd_cqid_max.write(0); o_wr_cqid_min.write(0xFFF);
    wait();

    while (true) {
        wait();                                              // clk pos
        if (!rst_n.read()) {
            ing = ING_IDLE; egr = EGR_IDLE; snp = SNP_IDLE;
            m_qh = m_qt = m_sh = m_st = 0; m_qn = m_sn = 0; m_rd_ostd = 0;
            m_snp_busy = m_snp_done = false;
            rdy0_prev = rdy1_prev = dready_prev = go_prev = hd_prev = false;
            rsp_ready_prev = snp_vprev = false;
            rd_done = 0; wr_done = 0; err_cnt = 0; snoop_cnt = 0;
            rd_acc = 0; ostd_max_cnt = 0;
            fst_rd = 0xFFF; fst_wr = 0xFFF; rd_cmax = 0; wr_cmin = 0xFFF;
            fst_rd_seen = false; fst_wr_seen = false;
            d2h_req0_ready.write(false); d2h_req1_ready.write(false); d2h_data_ready.write(false);
            d2h_rsp_ready.write(false); h2d_rsp_valid.write(false); h2d_data_valid.write(false);
            h2d_req_valid.write(false);
            o_rd_done.write(0); o_wr_done.write(0); o_err.write(0); o_snoop_cnt.write(0);
            o_rd_ostd_max.write(0); o_fst_rd_cqid.write(0xFFF); o_fst_wr_cqid.write(0xFFF);
            o_rd_cqid_max.write(0); o_wr_cqid_min.write(0xFFF);
            continue;
        }

        // ================= 子 FSM 1:snoop 服务(H2D req 出 / D2H rsp 入)=================
        bool snp_req_out = (snp == SNP_REQ);                 // 我这一拍抬 h2d_req_valid
        bool snp_rdy_out = (snp == SNP_RSP);                 // 我这一拍抬 d2h_rsp_ready
        h2d_req_valid.write(snp_req_out);
        if (snp == SNP_REQ) {
            h2d_req_opcode.write(m_snp_op3);
            h2d_req_addr.write(m_snp_addr);
            h2d_req_uqid.write(m_snp_uqid);
        }
        d2h_rsp_ready.write(snp_rdy_out);
        switch (snp) {
        case SNP_IDLE:
            if (m_snp_busy && !m_snp_done) { snp = SNP_REQ; m_snp_have_data = false; }   // ★ 起始清数据标记 ✓
            break;
        case SNP_REQ:                                                 // 交付 = 我上拍 valid ∧ 本拍 ready
            if (snp_vprev && h2d_req_ready.read()) { snp = SNP_RSP; }
            break;
        case SNP_RSP:                                                 // 交付 = 我上拍 ready ∧ 本拍 valid
            if (d2h_data_chunk_valid.read() && (d2h_data_uqid.read() & 0xFFF) == (m_snp_uqid & 0xFFF)) {
                line_to_bytes(d2h_data_payload.read(), m_snp_data);  // ★ 带数据应答 ⇒ 行数据 ✓
                m_snp_have_data = true;
            }
            if (rsp_ready_prev && d2h_rsp_valid.read()) {
                m_snp_d2h_op = d2h_rsp_opcode.read();                 // 关联:uqid 应回声(⚠ 设备义务)
                m_snp_done = true;
                m_ev_snp_done.notify();                               // 唤醒 b_snoop ✓
                snp = SNP_IDLE;
            }
            break;
        }

        // ================= 子 FSM 2:ingress(设备请求 → 队列)=================
        bool q_full = (m_qn >= QDEPTH);                              // ★ 计数式:满 = 真满(不再少 1)✓
        // ★ 读在飞硬上限:**未回读 + 已排响应 ≤ QDEPTH**(否则响应数 > m_qrsp 容量 ⇒ 覆盖未发条目)✗✓
        bool rd_ok = ((m_rd_ostd + m_sn) < QDEPTH);
        bool rdy0_out = (ing == ING_IDLE) && !q_full && rd_ok;
        // ⚠ **真缺陷修复(v1.0)**:IDLE 时若读/写同拍双 valid,本 FSM 读优先入队、
        //   写**不会**被收 ⇒ rdy1 必须同拍拉低;原实现 rdy0/rdy1 同抬 ⇒ 写**静默丢** ✗✓
        bool rdy1_out = (ing == ING_IDLE) && !q_full && !d2h_req0_valid.read();
        bool dry_out  = (ing == ING_WDATA);
        d2h_req0_ready.write(rdy0_out);
        d2h_req1_ready.write(rdy1_out);
        d2h_data_ready.write(dry_out);
        switch (ing) {
        case ING_IDLE:
            if (rdy0_prev && d2h_req0_valid.read()) {                 // 读:即刻入队 ✓
                ReqRec r; r.wr = false;
                r.op = d2h_req0_opcode.read(); r.addr = d2h_req0_addr.read();
                r.cqid = d2h_req0_cqid.read(); r.line = 0;
                m_qreq[m_qt] = r; m_qt = (m_qt + 1) % QDEPTH; m_qn++;
                m_rd_ostd++;                                         // ★ 读在飞计数(issuer 回响应时减)✓
                m_ev_req.notify();
                // ★ v1.0 观测:读在飞高水位 + 槽号口径见证(读槽应 0..63)✓
                rd_acc++;
                if (!fst_rd_seen) { fst_rd = r.cqid; fst_rd_seen = true; }
                if (r.cqid.to_uint() > rd_cmax.to_uint()) rd_cmax = r.cqid;
                sc_uint<32> ostd = rd_acc - rd_done;
                if (ostd.to_uint() > ostd_max_cnt.to_uint()) ostd_max_cnt = ostd;
            } else if (rdy1_prev && d2h_req1_valid.read()) {          // 写:先收请求,再等数据 ✓
                ing_rec.wr = true;
                ing_rec.op = d2h_req1_opcode.read(); ing_rec.addr = d2h_req1_addr.read();
                ing_rec.cqid = d2h_req1_cqid.read(); ing_rec.line = 0;
                ing = ING_WDATA;
                // ★ v1.0 观测:写槽号应 ≥ 64 ✓
                if (!fst_wr_seen) { fst_wr = ing_rec.cqid; fst_wr_seen = true; }
                if (ing_rec.cqid.to_uint() < wr_cmin.to_uint()) wr_cmin = ing_rec.cqid;
            }
            break;
        case ING_WDATA:
            if (dready_prev && d2h_data_valid.read()) {               // 512b 整行一拍 ✓
                ing_rec.line = d2h_data_payload.read();
                m_qreq[m_qt] = ing_rec; m_qt = (m_qt + 1) % QDEPTH; m_qn++;
                m_ev_req.notify();
                ing = ING_IDLE;
            }
            break;
        }

        // ================= 子 FSM 3:egress(队列 → 设备:GO + 数据)=================
        bool go_out = (egr == EGR_GO);
        bool hd_out = (egr == EGR_DATA);
        h2d_rsp_valid.write(go_out);
        h2d_rsp_opcode.write(cxlshim::H2D_RSP_SPEC_GO);
        h2d_rsp_rsp_data.write(egr_rec.err ? cxlshim::CXL_GO_STATE_ERR : cxlshim::CXL_GO_STATE_S);
        h2d_rsp_cqid.write(egr_rec.cqid);
        h2d_data_valid.write(hd_out);
        h2d_data_cqid.write(egr_rec.cqid);
        h2d_data_chunk_valid.write(true);
        h2d_data_payload.write(egr_rec.line);
        switch (egr) {
        case EGR_IDLE:
            if (m_sn > 0) {                                           // 有响应 ⇒ 起一笔 ✓
                egr_rec = m_qrsp[m_sh]; m_sh = (m_sh + 1) % QDEPTH; m_sn--;
                egr = EGR_GO;
            }
            break;
        case EGR_GO:                                                  // 回 GO(S/ERR)✓
            if (go_prev && h2d_rsp_ready.read()) {
                if (egr_rec.err) { rd_done = rd_done + 1; egr = EGR_IDLE; }   // GO-Err:不回数据 ✓
                else egr = EGR_DATA;
            }
            break;
        case EGR_DATA:                                                // 回数据(512b 一拍)✓
            if (hd_prev && h2d_data_ready.read()) { rd_done = rd_done + 1; egr = EGR_IDLE; }
            break;
        }

        // ---- 保存"本拍上线"值(下一拍读 = 该 slot 线值 ✓;必须与上面的写逐式一致)----
        rdy0_prev = rdy0_out; rdy1_prev = rdy1_out; dready_prev = dry_out;
        go_prev = go_out; hd_prev = hd_out;
        snp_vprev = snp_req_out; rsp_ready_prev = snp_rdy_out;

        o_rd_done.write(rd_done); o_wr_done.write(wr_done);
        o_err.write(err_cnt); o_snoop_cnt.write(snoop_cnt);
        o_rd_ostd_max.write(ostd_max_cnt);
        o_fst_rd_cqid.write(fst_rd); o_fst_wr_cqid.write(fst_wr);
        o_rd_cqid_max.write(rd_cmax); o_wr_cqid_min.write(wr_cmin);
    }
}
