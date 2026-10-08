//============================================================================
// device_tlm.cpp — 设备侧 TLM 实现  [2026-10-04 建 / v1.0 多笔在飞 / v1.0 RQ+SEND / v1.0 snoop 顺序]
//
// 版本轨迹:v1.0 寄存器面 / v1.0 端到端纵切(阻塞单笔) / **v1.0 非阻塞多笔在飞(本版)**✓
// 寄存器语义**逐条照** `规格文档` §8(RW/RO/W1C/门铃;每条有 RTL 出处)✓
// 引擎纪律(§节气)= `重启必读.md` §5-1:每条线 = 1 拍;交付判据 = 「我上拍线上值」∧「对方本拍值」✓
// 槽号/匹配口径(照 RTL)与设计要点 ⇒ 见 .h 文件头 ✓
//============================================================================
#include "device_tlm.h"
#include "roce_pkt.h"      // ★ v1.0:线侧真 RoCE 包(单一定义处)✓
#include <cstring>

//----------------------------------------------------------------------------
// 全局段 RO 判据(照 §8.3 逐项;⛔ 不是"整段 RO"—— 0x144/0x158/0x178 有写分支)✓
// ★ 2026-10-04 修(自测扫出)→ 入参 = **9 位字索引**(`(addr>>2)&0x1FF`,别名口径)——
//   ⚠ 原实现拿**字节偏移**(0x49C 等)直接比字索引(最大 0x1FF)⇒ **恒不成立**、全局 RO 写**全不拦** ✗✗
//   (此前自测只覆盖了 per-QP 的 RO ⇒ 漏网;新"RO 群扫"判据当场抓出)✗✓ ⇒ 统一 **`>>2` 转字索引** ✓
//----------------------------------------------------------------------------
static bool gf_is_ro(sc_uint<32> o) {
    unsigned int x = o.to_uint();                      // 字索引(0..0x1FF)✓
    if (x == (0x01C >> 2)) return true;                // 识别/版本类(VERSION 等):RO 恒 0 ——【刻意不实现】(交付口径:不出 IP 识别面)✓
    if (x == (0x49C >> 2)) return true;                // RD_REQ_CNT(RO)
    if (x >= (0x390 >> 2) && x <= (0x48C >> 2)) return true;   // CNP_SCHD_STS1..64
    static const unsigned int ro1[] = {0x100>>2,0x104>>2,0x108>>2,0x10C>>2,0x110>>2,0x114>>2,0x118>>2,0x11C>>2,
                                       0x120>>2,0x124>>2,0x128>>2,0x12C>>2,0x130>>2,0x134>>2,0x138>>2,0x13C>>2,0x140>>2};
    static const unsigned int ro2[] = {0x148>>2,0x14C>>2,0x150>>2,0x154>>2,0x15C>>2,0x160>>2,0x164>>2,0x168>>2,
                                       0x16C>>2,0x170>>2,0x174>>2,0x17C>>2};
    for (unsigned i = 0; i < sizeof(ro1)/sizeof(ro1[0]); i++) if (x == ro1[i]) return true;
    for (unsigned i = 0; i < sizeof(ro2)/sizeof(ro2[0]); i++) if (x == ro2[i]) return true;
    return false;
}

// per-QP 段 RO(照 §8.4 "Read only" 标注)✓
static bool qp_is_ro(sc_uint<32> o) {
    unsigned int x = o.to_uint();
    return (x == 0x90) || (x == 0x94) || (x == 0x98) || (x == 0x9C) || (x == 0xA0);
}

//----------------------------------------------------------------------------
// 行缓冲 ↔ sc_biguint<512>(小端字节序,照项目 aarch64-LE 口径 ✓)
//----------------------------------------------------------------------------
static void line_to_bytes(const sc_dt::sc_biguint<512> & L, unsigned char * b) {
    for (int i = 0; i < 8; i++) {
        sc_dt::uint64 w = L.range(64*i+63, 64*i).to_uint64();
        for (int k = 0; k < 8; k++) b[i*8+k] = (unsigned char)((w >> (8*k)) & 0xffULL);
    }
}
static sc_dt::sc_biguint<512> bytes_to_line(const unsigned char * b) {
    sc_dt::sc_biguint<512> L = 0;
    for (int i = 0; i < 8; i++) {
        sc_dt::uint64 w = 0;
        for (int k = 0; k < 8; k++) w |= (sc_dt::uint64)b[i*8+k] << (8*k);
        L.range(64*i+63, 64*i) = w;
    }
    return L;
}

//----------------------------------------------------------------------------
DeviceTlm::DeviceTlm(sc_core::sc_module_name nm):
    sc_core::sc_module(nm),
    amba_pv::amba_pv_slave_base<32>("nic_regs"),      // ★ v1.0(A7):从端宿主名 ✓
    reg_s("reg_s") {
    reg_s(*this);                                          // ★ 绑到本类(转译到 read/write)✓
    for (int i = 0; i < NPD; i++) for (int j = 0; j < 8; j++)  m_pd[i][j] = 0;
    for (int i = 0; i < NQP; i++) for (int j = 0; j < 64; j++) m_qp[i][j] = 0;
    for (int i = 0; i < NGF; i++) m_gf[i] = 0;
    m_intr_sts = 0; m_intr_en = 0; m_cq_intr = 0;   // 复位:全屏蔽(照 §10.3)✓
    db_sq_cnt = 0; db_rq_cnt = 0; ro_wr_cnt = 0;
    // ⚠ **不许在 ctor 里写端口**(端口还没绑 ⇒ SystemC 抛 `E112 get interface failed` ✗ 已踩);
    //   端口首值统一在本线程体开头 eng_reset() 里写 ✓
    db_h = db_t = db_n = 0; db_drop_cnt = 0;
    job_seq = 0; goerr_cnt = 0; max_rd_ostd = 0; max_act_per_qp = 0;
    rnr_cnt = 0; opc_simpl_cnt = 0; rr_next = 0;
    for (int i = 0; i < NQP; i++) rq_wrptr[i] = 1;
    tx = TX_IDLE; tx_slot = 0; tx_line = 0;
    tx_reqv_prev = tx_datav_prev = false; h2r_prev = h2d_prev = false;
    rsp_v_last = dat_v_last = false;
    snp_act = false; snp_rdy_prev = false; snp_rspv_prev = false; snp_uqid = 0;
    snp_line = 0; snp_hold_seen = false; snp_held_cnt = 0;
    wqe_cnt = 0; cqe_cnt = 0;
    for (int i = 0; i < NSLOT; i++) sl[i].in_use = false;
    rdf_h = rdf_t = 0; rdf_n = RD_SLOT_CNT;            // 空闲表**满**:64 个读槽全空 ✓
    wrf_h = wrf_t = 0; wrf_n = WR_SLOT_CNT;            // 64 个写槽全空 ✓
    wp_h = wp_t = wp_n = 0;
    for (int i = 0; i < NQP; i++) { cq_hdptr[i] = 1; qp_fatal[i] = false; }
    qp_fatal_cnt = 0;
    for (int j = 0; j < NJOB; j++) jobs[j].act = false;
    SC_METHOD(intr_upd); sensitive << clk.pos();
    SC_THREAD(engine_thread); sensitive << clk.pos();   // ★ 引脚归它(见 .h)✓
    mac_txbusy = false; mac_txlen = mac_txpos = 0; mac_rxlen = 0;
    mac_rxq_ready = false; mac_rxqlen = 0;
    mac_tx_frames = mac_rx_frames = mac_tx_drop_cnt = mac_rx_drop_cnt = 0;
    // ★ v1.0 线侧真包:默认身份 = 基准 配方(本机=设备侧;⚠ 白盒默认,真源待开卷 RTL)✓
    { static const unsigned char DEF_SA[6] = {0x2f,0x76,0x17,0xdc,0x5e,0x9a};
      static const unsigned char DEF_DA[6] = {0xea,0x1e,0x1c,0x69,0xb8,0xed};
      for (int i = 0; i < 6; i++) { lid.sa[i] = DEF_SA[i]; lid.da[i] = DEF_DA[i]; }
      lid.sip = 0x0a000001; lid.dip = 0x00000002 | 0x0a000000;   // 0x0a000002 ✓
      lid.sport = 0x6851;   lid.dport = 0x12b7; }                 // 4791 ✓
    line_tx_pkts = 0; line_rx_acks = 0; line_unexp_cnt = 0; line_big_cnt = 0;
    for (int i = 0; i < CCH_MAX; i++) cch[i].valid = false;   // ★ 行表清零 ✓
    cch_victim = 0; cch_hits = 0; cch_fills = 0; cch_wr_hit = 0; cch_snp_hit = 0;
    snp_op3 = 0; snp_xfer_done = false; snp_rdata = false; snp_data_v = false;
}

//----------------------------------------------------------------------------
// 中断聚合:9 源状态 & 使能 → OR 成一根电平线(照 §10.1/§10.2)✓
//----------------------------------------------------------------------------
void DeviceTlm::intr_upd() {
    if (!rst_n.read()) {
        m_intr_sts = 0; m_intr_en = 0; m_cq_intr = 0;
        // —— 输出首值:intr_upd 只写 **nic_intr 与 o_***;设备侧引脚归 engine_thread ✓
        nic_intr.write(false);
        o_db_sq_cnt.write(0); o_db_rq_cnt.write(0); o_ro_wr_cnt.write(0);
        o_wqe_cnt.write(0); o_cqe_cnt.write(0); o_db_drop_cnt.write(0);
        o_max_rd_ostd.write(0); o_max_act_per_qp.write(0); o_goerr_cnt.write(0);
        o_rnr_cnt.write(0); o_opc_simpl_cnt.write(0); o_snp_held_cnt.write(0);
        o_qp_fatal_cnt.write(0);
        return;
    }
    sc_uint<9> eff = m_intr_sts; eff[4] = (m_cq_intr.to_uint() != 0);   // ★ bit4 = per-QP 汇聚(照 RTL)✓
    nic_intr.write((eff & m_intr_en) != 0);
    o_db_sq_cnt.write(db_sq_cnt); o_db_rq_cnt.write(db_rq_cnt); o_ro_wr_cnt.write(ro_wr_cnt);
    o_wqe_cnt.write(wqe_cnt); o_cqe_cnt.write(cqe_cnt); o_db_drop_cnt.write(db_drop_cnt);
    o_max_rd_ostd.write(max_rd_ostd); o_max_act_per_qp.write(max_act_per_qp);
    o_goerr_cnt.write(goerr_cnt);
    o_rnr_cnt.write(rnr_cnt); o_opc_simpl_cnt.write(opc_simpl_cnt);
    o_snp_held_cnt.write(snp_held_cnt);
    o_qp_fatal_cnt.write(qp_fatal_cnt);
    // ⚠ o_line_* **不在此写** —— 归 engine_thread(mac_engine)(双写 = E115 双驱动,已踩)✗✓
}

void DeviceTlm::set_intr_status(int bit, bool v) {      // ★ 白盒钩子 ⚠
    if (bit == 4) { m_cq_intr = v ? 0xFFFFFFFFu : 0u; return; }   // ★ bit4 = 汇聚 ⇒ 钩子改驱 per-QP 组
    if (bit >= 0 && bit < 9) {
        if (v) m_intr_sts[bit] = 1; else m_intr_sts[bit] = 0;
    }
}

void DeviceTlm::set_rq_pi(int qp, sc_uint<16> v) {      // ★ 白盒钩子(v1.0 起引擎=真源,见 §4.4-⑧)⚠
    if (qp >= 0 && qp < NQP) m_qp[qp][0x9C >> 2] = v;
}

//----------------------------------------------------------------------------
// 寄存器读(照 §8.1 解码:低 21 位;段 = addr[20:19])✓
//----------------------------------------------------------------------------
//============================================================================
// ★ v1.0(A7):AMBA-PV 从口用户层 read/write —— 与直连方法**同源**(逐寄存器一致)✓
//   · 只支持 4B 访问(本寄存器面口径);len ≠ 4 ⇒ SLVERR ✓
//   · 未映射地址 ⇒ DECERR;RO 写 ⇒ SLVERR(先读探映射性再写,区分两类错)✓
//   · t 不加延迟(寄存器面 = 功能级;平台若要时序可自行加)✓
//============================================================================
amba_pv::amba_pv_resp_t DeviceTlm::read(int, const sc_dt::uint64 & addr, unsigned char * data,
                                        unsigned int len, const amba_pv::amba_pv_control *,
                                        sc_core::sc_time &) {
    if (len != 4 || data == nullptr) return amba_pv::AMBA_PV_SLVERR;
    sc_uint<32> v = 0;
    if (!reg_read((sc_dt::uint64)addr, v)) return amba_pv::AMBA_PV_DECERR;
    data[0] = (unsigned char)v.range(7, 0);
    data[1] = (unsigned char)v.range(15, 8);
    data[2] = (unsigned char)v.range(23, 16);
    data[3] = (unsigned char)v.range(31, 24);
    reg_sock_n++;
    return amba_pv::AMBA_PV_OKAY;
}

amba_pv::amba_pv_resp_t DeviceTlm::write(int, const sc_dt::uint64 & addr, unsigned char * data,
                                         unsigned int len, const amba_pv::amba_pv_control *,
                                         unsigned char * byte_en, sc_core::sc_time &) {
    (void)byte_en;                       // 寄存器面 = 4B 全字节(byte_en 忽略,明写)✓
    if (len != 4 || data == nullptr) return amba_pv::AMBA_PV_SLVERR;
    sc_uint<32> v = 0;
    v.range(7, 0)   = (unsigned)data[0];
    v.range(15, 8)  = (unsigned)data[1];
    v.range(23, 16) = (unsigned)data[2];
    v.range(31, 24) = (unsigned)data[3];
    sc_uint<32> probe = 0;
    if (!reg_read((sc_dt::uint64)addr, probe)) return amba_pv::AMBA_PV_DECERR;  // 未映射 ⇒ DECERR ✓
    if (!reg_write((sc_dt::uint64)addr, v)) return amba_pv::AMBA_PV_SLVERR;     // 已映射但拒写(RO)⇒ SLVERR ✓
    reg_sock_n++;
    return amba_pv::AMBA_PV_OKAY;
}

bool DeviceTlm::reg_read(sc_dt::uint64 addr, sc_uint<32> & data) {
    sc_uint<32> a = (sc_uint<32>)((addr & 0x1FFFFF) >> 2 << 2);   // 字对齐
    unsigned int seg = (a.to_uint() >> 19) & 3;
    unsigned int idx = (a.to_uint() >> 8) & 0xFF;
    unsigned int off = a.to_uint() & 0xFF;
    if (seg == 2) {                                   // 全局段(off 只用 [10:2] ⇒ 每 0x800 别名)✓
        unsigned int o = (a.to_uint() >> 2) & 0x1FF;
        if (o == 0x180 >> 2) { data = (sc_uint<32>)m_intr_en.to_uint(); return true; }   // INTR_EN
        if (o == 0x184 >> 2) {                                                            // INTR_STS
            sc_uint<32> v32 = m_intr_sts.to_uint(); v32[4] = (m_cq_intr.to_uint() != 0);  // ★ bit4 = 汇聚 ✓
            data = v32; return true; }
        if (o == 0x290 >> 2) { data = m_cq_intr; return true; }   // ★ CQ_INTR_STS1(0x290;bank0=QP0..31)✓
        data = m_gf[o]; return true;
    }
    if (seg == 3) {                                   // per-QP 段
        if ((int)idx >= NQP || (off & 3)) return false;
        data = m_qp[idx][off >> 2]; return true;
    }
    // PD 段(seg 0/1)✓
    if ((int)idx >= NPD || off > 0x1C || (off & 3)) return false;
    data = m_pd[idx][off >> 2]; return true;
}

//----------------------------------------------------------------------------
// 寄存器写(RO 忽略 + 计数;INTR_STS = **W1C**;门铃 = 存储 + 计数)✓
//----------------------------------------------------------------------------
bool DeviceTlm::reg_write(sc_dt::uint64 addr, const sc_uint<32> & data) {
    sc_uint<32> a = (sc_uint<32>)((addr & 0x1FFFFF) >> 2 << 2);
    unsigned int seg = (a.to_uint() >> 19) & 3;
    unsigned int idx = (a.to_uint() >> 8) & 0xFF;
    unsigned int off = a.to_uint() & 0xFF;
    if (seg == 2) {
        unsigned int o = (a.to_uint() >> 2) & 0x1FF;
        if (o == 0x180 >> 2) { m_intr_en = (sc_uint<9>)data.to_uint(); return true; }    // INTR_EN(RW)✓
        if (o == 0x184 >> 2) {                                                           // W1C ✓
            for (int b = 0; b < 9; b++) if (b != 4 && data[b]) m_intr_sts[b] = 0;        // ★ bit4 例外:
            return true;                                                                 //   写它**不清**(照 RTL 死路 )✗
        }
        if (o == 0x290 >> 2) {                                   // ★ CQ_INTR_STS1:mask 1 = 清对应 QP ✓
            m_cq_intr = (sc_uint<32>)(m_cq_intr.to_uint() & ~data.to_uint());
            return true;
        }
        if (gf_is_ro(o)) { ro_wr_cnt = ro_wr_cnt + 1; return true; }                     // RO:忽略+计数 ✓
        if (o == 0x178 >> 2) { m_gf[o] = m_gf[o] + (sc_uint<32>)(data.to_uint() & 0xFFFF); return true; } // 累加 ✓
        m_gf[o] = data; return true;
    }
    if (seg == 3) {
        if ((int)idx >= NQP || (off & 3)) return false;
        if (qp_is_ro(off)) { ro_wr_cnt = ro_wr_cnt + 1; return true; }                   // RO:忽略+计数 ✓
        m_qp[idx][off >> 2] = data;
        if (off == 0x34) db_rq_cnt = db_rq_cnt + 1;                                      // RQ_CI_DB 门铃 ✓
        if (off == 0x38) { db_sq_cnt = db_sq_cnt + 1;                                     // SQ_PI_DB 门铃 ✓
            // ★ 容量**精确** = DBDEPTH(计数式;不再 (t+1)%N==h 少 1)✓;满 ⇒ 计数,不静默 ✗
            if (db_n < DBDEPTH) { db_q[db_t].qp = (int)idx; db_q[db_t].pi = (sc_uint<16>)(data.to_uint() & 0xFFFF);
                                  db_t = (db_t + 1) % DBDEPTH; db_n++; }
            else { db_drop_cnt = db_drop_cnt + 1; } }
        return true;
    }
    if ((int)idx >= NPD || off > 0x1C || (off & 3)) return false;
    m_pd[idx][off >> 2] = data; return true;
}

//============================================================================
// v1.0 引擎:槽表 + 在飞 job 表 + 单一发射单元(每拍一步,非阻塞)✓
//============================================================================

//----------------------------------------------------------------------------
// ★ v1.0 RQ 环辅助(照 P2 规格 §4.4;每条有 RTL 出处)✓
//----------------------------------------------------------------------------
// 满判据:照 `RTL 源` **原式**(16b 加法,含回绕)✓
bool DeviceTlm::rq_full(int qp) {
    uint32_t depth = rq_depth(qp);
    if (depth == 0) return true;                              // 未配置 ⇒ 视为满(不再猜)✗
    uint16_t ci  = (uint16_t)(m_qp[qp][0x34 >> 2].to_uint() & 0xFFFF);   // RQ_CI_DB 末次写入值 ✓
    uint16_t wr  = rq_wrptr[qp].to_uint();
    return ((uint16_t)(ci) == (uint16_t)(wr + 1)) ||
           ((uint16_t)(ci + depth) == (uint16_t)(wr + 1));
}
// 项地址 = RQ_BUF_BA + ((wrptr==depth)?0:wrptr) × RQ_BUF_SZ × 256
//   (与 RTL 的 ca 递推等价;推导见 §4.4-②;⚠ RQ_BUF_SZ 单位 = 256B)✓
sc_dt::uint64 DeviceTlm::rq_entry_addr(int qp) {
    uint32_t depth = rq_depth(qp);
    sc_dt::uint64 ba  = (sc_dt::uint64)(qp_reg(qp, 0x08) & 0xFFFFFF00u); // [31:8],256B 粒度 ✓
    uint32_t szu      = (qp_reg(qp, 0x00) >> 24) & 0xFF;                 // QP_CONF[31:24] ✓
    uint32_t w        = rq_wrptr[qp].to_uint();
    uint32_t idx      = (w == depth) ? 0 : w;
    return ba + (sc_dt::uint64)idx * (sc_dt::uint64)szu * 256u;
}

// 读/写槽分配(空闲表 FIFO;照 RTL 初值 0..63 / 64..127)✓
int DeviceTlm::alloc_rd() {
    if (rdf_n == 0) return -1;
    int s = rdf_q[rdf_h]; rdf_h = (rdf_h + 1) % RD_SLOT_CNT; rdf_n--; return s;
}
int DeviceTlm::alloc_wr() {
    if (wrf_n == 0) return -1;
    int s = wrf_q[wrf_h]; wrf_h = (wrf_h + 1) % WR_SLOT_CNT; wrf_n--; return s;
}
void DeviceTlm::free_slot(int s) {
    Slot &S = sl[s];
    if (!S.in_use) return;                                   // 防御:不重复归还 ✓
    if (S.job >= 0 && jobs[S.job].act) jobs[S.job].slots_owned--;
    S.in_use = false; S.done = false; S.err = false;
    if (S.is_wr) { wrf_q[wrf_t] = s; wrf_t = (wrf_t + 1) % WR_SLOT_CNT; wrf_n++; }
    else         { rdf_q[rdf_t] = s; rdf_t = (rdf_t + 1) % RD_SLOT_CNT; rdf_n++; }
}

// 每拍:服务 snoop(H2D req → D2H rsp 恒 RspI)
// ★ v1.0 **顺序语义(照 RTL)**:设备恒回 RspI,但**对在飞地址先等完成** ——
//   应答条件 = **无在飞槽(读或写)地址匹配**;出处:`RTL 源`(snoop FIFO +
//   `snp_rsp_pend` 仅当"无在飞匹配")+ `RTL 源` snp_chk 循环(`slot_valid && !slot_done`)✓
//   ⚠ 压住期间 `h2d_req_ready` 保持低 ⇒ **后续 snoop 也按序等**(照 RTL:检查恒指向队头)✓
bool DeviceTlm::snoop_inflight_match() {
    for (int s = 0; s < NSLOT; s++)
        if (sl[s].in_use && sl[s].line == snp_line) return true;      // 读/写槽都在"在飞"集合内 ✓
    return false;
}
void DeviceTlm::snoop_step() {
    bool rdy = !snp_act;
    h2d_req_ready.write(rdy);
    bool hold = snp_act && snoop_inflight_match();       // ★ 在飞地址匹配 ⇒ 压住不应答 ✓

    // ★ 受理后**单次**定应答码 + 行态迁移(照 `规格文档` §1.2-B);默认关 ⇒ I_HIT_I ✓
    if (snp_act && !snp_xfer_done) {
        snp_xfer_done = true;
        snp_rop = cxlshim::D2H_RSP_SPEC_I_HIT_I; snp_rdata = false;
        if (cache_on) {
            int ci = cch_find(snp_line);
            if (ci >= 0) {
                CchLine &CL = cch[ci];
                bool dirty = (CL.st == CCH_M) || (CL.st == CCH_D);
                switch (snp_op3) {
                case 2:                                                // SNP_INV ⇒ 行 → I ✓
                    if (dirty) { snp_rop = cxlshim::D2H_RSP_SPEC_I_FWD_M; snp_rdata = true; }
                    else       { snp_rop = cxlshim::D2H_RSP_SPEC_I_HIT_SE; }
                    CL.valid = false; CL.st = CCH_I;
                    break;
                case 3:                                                // SNP_CUR ✓
                    if (dirty)      { snp_rop = cxlshim::D2H_RSP_SPEC_V_FWD_V; snp_rdata = true; CL.st = CCH_D; }
                    else if (CL.st == CCH_S) { snp_rop = cxlshim::D2H_RSP_SPEC_S_HIT_SE; }
                    else            { snp_rop = cxlshim::D2H_RSP_SPEC_V_HIT_V; CL.st = CCH_F; }
                    break;
                default:                                               // 1 = SNP_DATA ⇒ 行 → I ✓
                    snp_rop = cxlshim::D2H_RSP_SPEC_S_FWD_M; snp_rdata = true;
                    CL.valid = false; CL.st = CCH_I;
                    break;
                }
                if (snp_rdata) for (int k = 0; k < 64; k++) snp_data_buf[k] = CL.data[k];
                cch_snp_hit++;
            }
        }
    }
    // 带数据应答:等槽活动全静(tx == IDLE)才发 —— 数据/请求通道不与写槽竞争(同拍交付)✓
    bool busy_tx = snp_rdata && (tx != TX_IDLE) && snp_act;
    bool rv = snp_act && !hold && !busy_tx;
    d2h_rsp_valid.write(rv);
    if (rv) { d2h_rsp_opcode.write((sc_uint<5>)snp_rop); d2h_rsp_uqid.write(snp_uqid); }
    snp_data_v = rv && snp_rdata;                        // 行数据随应答(§节气:同拍)✓
    if (!snp_act) {
        if (snp_rdy_prev && h2d_req_valid.read()) {
            snp_uqid = h2d_req_uqid.read(); snp_line = h2d_req_addr.read().to_uint64();
            snp_op3  = (int)h2d_req_opcode.read().to_uint();     // ★ M2':记住 snoop 类型 ✓
            snp_act = true; snp_hold_seen = false; snp_xfer_done = false;
        }
    } else if (snp_rspv_prev && d2h_rsp_ready.read()) {
        snp_act = false; snp_data_v = false;
    }
    if (hold && !snp_hold_seen) { snp_hold_seen = true; snp_held_cnt = snp_held_cnt + 1; }  // 每笔记一次 ✓
    snp_rdy_prev = rdy; snp_rspv_prev = rv;
}

void DeviceTlm::eng_reset() {
    for (int s = 0; s < NSLOT; s++) { sl[s].in_use = false; sl[s].is_wr = false;
                                      sl[s].done = false; sl[s].err = false; sl[s].job = -1; }
    rdf_h = rdf_t = 0; rdf_n = RD_SLOT_CNT;          // ⚠ 计数=**已有条目数**(空表初值 = 满)✓
    wrf_h = wrf_t = 0; wrf_n = WR_SLOT_CNT;
    wp_h = wp_t = wp_n = 0;
    for (int i = 0; i < RD_SLOT_CNT; i++) rdf_q[i] = i;              // 初值 0..63 ✓
    for (int i = 0; i < WR_SLOT_CNT; i++) wrf_q[i] = RD_SLOT_CNT + i;// 初值 64..127 ✓
    for (int i = 0; i < RD_SLOT_CNT + WR_SLOT_CNT; i++) wpq[i] = 0;
    for (int j = 0; j < NJOB; j++) { jobs[j].act = false; jobs[j].ph = J_FREE; jobs[j].slots_owned = 0; }
    job_seq = 0; rr_next = 0;
    tx = TX_IDLE; tx_slot = 0; tx_line = 0;
    tx_reqv_prev = tx_datav_prev = false; h2r_prev = h2d_prev = false;
    rsp_v_last = dat_v_last = false;
    db_h = db_t = db_n = 0; db_drop_cnt = 0;
    for (int i = 0; i < NQP; i++) { cq_hdptr[i] = 1; qp_fatal[i] = false; }
    qp_fatal_cnt = 0;
    wqe_cnt = 0; cqe_cnt = 0; goerr_cnt = 0; max_rd_ostd = 0; max_act_per_qp = 0;
    mac_tx_frames = 0; mac_rx_frames = 0; mac_tx_drop_cnt = 0; mac_rx_drop_cnt = 0;
    mac_txbusy = false; mac_txlen = mac_txpos = 0; mac_rxlen = 0; mac_rxq_ready = false; mac_rxqlen = 0;
    cmac_m_axis_tvalid.write(false); cmac_m_axis_tlast.write(false);
    cmac_m_axis_tdata.write(0); cmac_m_axis_tkeep.write(0);
    o_mac_tx_frames.write(0); o_mac_rx_frames.write(0);
    o_line_tx_pkts.write(0); o_line_rx_acks.write(0);
    o_line_unexp_cnt.write(0); o_line_big_cnt.write(0);
    rnr_cnt = 0; opc_simpl_cnt = 0;
    line_tx_pkts = 0; line_rx_acks = 0; line_unexp_cnt = 0; line_big_cnt = 0;   // ★ v1.0 ✓
    for (int i = 0; i < CCH_MAX; i++) cch[i].valid = false;                     // ★ ✓
    cch_victim = 0; cch_hits = 0; cch_fills = 0; cch_wr_hit = 0; cch_snp_hit = 0;
    snp_op3 = 0; snp_xfer_done = false; snp_rdata = false; snp_data_v = false;
    for (int i = 0; i < NQP; i++) rq_wrptr[i] = 1;
    snp_act = false; snp_rdy_prev = false; snp_rspv_prev = false; snp_uqid = 0;
    snp_line = 0; snp_hold_seen = false; snp_held_cnt = 0;
    // 输出初值(本线程独占设备侧引脚;ctor 不能写,见 ctor 注释)✓
    d2h_req0_valid.write(false); d2h_req1_valid.write(false);
    d2h_data_valid.write(false); d2h_data_chunk_valid.write(false);
    h2d_req_ready.write(false); h2d_rsp_ready.write(false); h2d_data_ready.write(false);
    d2h_rsp_valid.write(false);
    d2h_req0_opcode.write(cxlshim::D2H_REQ_SPEC_RD_SHARED);
    d2h_req1_opcode.write(cxlshim::D2H_REQ_MEMWR);
    d2h_req0_cqid.write(0); d2h_req1_cqid.write(0);
    d2h_req0_addr.write(0); d2h_req1_addr.write(0);
    d2h_data_uqid.write(0); d2h_data_payload.write(0);
    d2h_rsp_opcode.write(cxlshim::D2H_RSP_SPEC_I_HIT_I); d2h_rsp_uqid.write(0);
}

void DeviceTlm::eng_step() {
    snoop_step();                                        // 每拍服务 snoop(与主链并行)✓

    // ---------- (1) 采样交付(判据 = 我上拍线上值 ∧ 对方本拍值)----------
    // ★ 回程 valid **上升沿守卫**:桥 egress 的 valid 是 **2 拍宽脉冲**(状态进入时输出晚一拍:
    //   进 EGR_GO 那拍输出仍是 0,故 [转移拍, 转移拍+1] 都挂着 1)⇒ 设备"恒 ready"会把同一笔吃两次 ✗
    //   ⇒ 只在 valid **首次**抬高的那一拍消费(旧版 cxl_read 的 `rp` 臂延迟即同效)✓
    bool rsp_v_now = h2d_rsp_valid.read();
    bool dat_v_now = h2d_data_valid.read();
    bool got_go   = h2r_prev && rsp_v_now && !rsp_v_last;
    bool got_data = h2d_prev  && dat_v_now && !dat_v_last;
    rsp_v_last = rsp_v_now; dat_v_last = dat_v_now;
    bool tx_rd_ok = (tx == TX_RREQ)  && tx_reqv_prev && d2h_req0_ready.read();
    bool tx_wr_ok = (tx == TX_WREQ)  && tx_reqv_prev && d2h_req1_ready.read();
    bool tx_wd_ok = (tx == TX_WDATA) && tx_datav_prev && d2h_data_ready.read();

    // ---------- (2) 应用交付 ----------
    if (got_go) {
        int s = (int)(h2d_rsp_cqid.read().to_uint() & (NSLOT - 1));       // cqid[6:0] = 槽号 ✓
        unsigned st = h2d_rsp_rsp_data.read().to_uint() & 7;
        if (s < NSLOT && sl[s].in_use && !sl[s].is_wr && !sl[s].err) {
            if (st == cxlshim::CXL_GO_STATE_ERR) { sl[s].err = true; goerr_cnt = goerr_cnt + 1; }
            else sl[s].go_state = (unsigned char)(st & 7);                // ★ GO 带回行态 ✓
        } else { goerr_cnt = goerr_cnt + 1; }                             // 无主 GO:计数,不静默 ✗
    }
    if (got_data) {
        int s = (int)(h2d_data_cqid.read().to_uint() & (NSLOT - 1));
        if (s < NSLOT && sl[s].in_use && !sl[s].is_wr && !sl[s].err) {
            line_to_bytes(h2d_data_payload.read(), sl[s].buf);
            sl[s].done = true;
        } else { goerr_cnt = goerr_cnt + 1; }                             // 无主数据:计数 ✗
    }
    if (tx_rd_ok) { tx = TX_IDLE; }                       // 读请求送达:槽留 in_use 等回程 ✓
    else if (tx_wr_ok) { tx = TX_WDATA; }                 // 写请求送达:转数据相 ✓
    else if (tx_wd_ok) {                                  // 写数据送达 ⇒ posted 完成 ✓
        int s = tx_slot;
        int j = sl[s].job; bool is_cqe = (sl[s].kind == SLK_CQE);
        bool live = (j >= 0) && jobs[j].act && (jobs[j].ph != J_DEAD);
        free_slot(s);                                     // 归还槽(在 job 还活着时递减 slots_owned)✓
        wp_h = (wp_h + 1) % NSLOT; wp_n--;                // 写槽服务队列出队 ✓
        tx = TX_IDLE;
        if (live) {
            if (is_cqe) {                                 // CQE 送达 ⇒ 本笔完成(照旧序:计数/头指针/中断)✓
                int qp = jobs[j].qp;
                uint32_t depth = qp_reg(qp, 0x3C) & 0xFFFF;
                uint16_t hd = cq_hdptr[qp].to_uint();
                cq_hdptr[qp] = (sc_uint<16>)((hd == depth) ? 1 : (hd + 1));
                cqe_cnt = cqe_cnt + 1;
                m_cq_intr[qp] = 1;                        // ★ 置**本 QP** 的 CQ 完成位(bit4 = per-QP 汇聚)✓
                jobs[j].act = false; jobs[j].ph = J_FREE;
            } else if (sl[s].kind == SLK_SRC) {           // ★ M3:只有数据写槽计入 wr_done ✓
                jobs[j].wr_done++;
            }                                             // (SLK_WBK 一次性回写不计数)✓
        }
    }

    // ---------- (3) 读槽回填消费(housekeeping)----------
    for (int s = 0; s < NSLOT; s++) {
        Slot &S = sl[s];
        if (!S.in_use || S.is_wr || !S.done) continue;
        int j = S.job;
        if (j < 0) { free_slot(s); continue; }                                 // 无主(防御)✓
        Job &J = jobs[j];
        if (!J.act) { free_slot(s); continue; }                                // 已释放 job 的残留 ✓
        if (S.err) { J.rd_inflight--; J.ph = J_DEAD; free_slot(s); continue; } // GO-Err:本 job 判死 ✓
        if (J.ph == J_DEAD) { J.rd_inflight--; free_slot(s); continue; }       // 死 job 的回程:只回收 ✓
        switch (S.kind) {
        case SLK_WQE: {                                                        // 取 WQE 行 ⇒ 解析 ✓
            for (int k = 0; k < cxlshim::LINE_BYTES; k++) J.wqe_buf[k] = S.buf[k];   // ★ M3:留档(回写用)✓
            J.wqe_line = S.line;
            sc_dt::sc_biguint<512> L = bytes_to_line(S.buf);
            J.opcode = (sc_uint<8>)L.range(135,128).to_uint();                 // 照 §9.1 字段表 ✓
            J.wrid   = (sc_uint<16>)L.range(15,0).to_uint();
            J.len    = (sc_uint<32>)L.range(127,96).to_uint();
            uint64_t lo = L.range(95,32).to_uint64();
            uint64_t ro = L.range(223,160).to_uint64();
            J.dest_qpid = (uint32_t)L.range(143,136).to_uint();       // WQE byte17 qp_id(进包 DestQP)✓
            J.rkey      = (uint32_t)L.range(255,224).to_uint();        // WQE byte28..31 r_key(进包 RETH)✓
            wqe_cnt = wqe_cnt + 1;
            // opcode 分支(§9.1 全集;角色按 RoCE 语义;P2 规格 §4.4 定案):
            //   0x00 WRITE / 0x01 WRITE+IMMDT :local(源)→ remote(目标)✓
            //   0x04 READ                     :remote(源)→ local(目标)**翻转** ✓
            //   0x02 SEND / 0x03 SEND+IMMDT   :local(源)→ **本 QP 的 RQ 项**(dst 在 §MQ 锁定时算)✓
            //   ⚠ opcode[0] = IMMDT 有效位 ⇒ 0x01/0x03 的 `imm_data` 走**独立 immdt FIFO/AXIS**
            //     ),**不在本模型边界** ⇒ 按基类搬数据 + 计 `o_opc_simpl_cnt`(不静默)✗
            //   其余 = 不搬数、不出 CQE、job 判死(⚠ RTL ILL_OPCODE FSM 未开卷 ⇒ 占位)✗
            J.is_send = false;
            uint32_t op = J.opcode.to_uint();
            if (op == 0x00)      { J.src = lo; J.dst = ro;
                                   if (line_mode) {                       // ★ v1.0:M2 三型都走真包 ✓
                                       if (J.len.to_uint() <= (uint32_t)LINE_PAY_MAX) { J.line_wr = true; J.line_op = Job::LINE_WR; }
                                       else { J.ph = J_DEAD; line_big_cnt = line_big_cnt + 1; }  // 超上限 ✗
                                   } }
            else if (op == 0x01) { J.src = lo; J.dst = ro; opc_simpl_cnt = opc_simpl_cnt + 1; }
            else if (op == 0x02) { J.src = lo;
                                   if (line_mode) {                       // ★ M2:SEND 走真包(is_send 保持 false;RQ 本地环不动)✓
                                       if (J.len.to_uint() <= (uint32_t)LINE_PAY_MAX) { J.line_wr = true; J.line_op = Job::LINE_SEND; }
                                       else { J.ph = J_DEAD; line_big_cnt = line_big_cnt + 1; }
                                   } else J.is_send = true; }
            else if (op == 0x03) { J.src = lo; J.is_send = true; opc_simpl_cnt = opc_simpl_cnt + 1; }
            else if (op == 0x04) { J.src = ro; J.dst = lo;                 // ★ M2:READ(RETH.va = ro;响应载荷落 lo)✓
                                   if (line_mode) {
                                       if (J.len.to_uint() <= (uint32_t)LINE_PAY_MAX) { J.line_wr = true; J.line_op = Job::LINE_RD; }
                                       else { J.ph = J_DEAD; line_big_cnt = line_big_cnt + 1; }
                                   } }
            else { J.ph = J_DEAD;
                   // ★ D7-① 开卷定案(2026-10-04):RTL 非法 opcode ⇒ `ILL_OPCODE` 状态 +
                   //   `o_wqe_fatal_qpid <= o_qp_id`)⇒ **该 QP 判 fatal**(不再处理)
                   //   ⇒ 中断 **bit7(fatal_err)**(bit5 `ill_opc_in_sq` 源已禁用恒 0,§10.2)✓
                   qp_fatal[J.qp] = true; qp_fatal_cnt = qp_fatal_cnt + 1;
                   m_intr_sts[7] = 1; }
            if (J.ph != J_DEAD) {                                              // 按 dma_len 逐行搬 ✓
                if (J.line_wr && J.line_op == Job::LINE_RD) {                  // ★ M2:RD **不发源读** ✓
                    J.nlines = 0; J.next_line = 0; J.ph = J_MOVE;             // (nlines 由响应载荷定)
                } else {
                    uint32_t nl = (J.len.to_uint() + 63) / 64; if (nl == 0) nl = 1;
                    if (J.len.to_uint() > 64u * 1024u) nl = 1;                 // 保险:异常长度不炸 ✓
                    J.nlines = (int)nl; J.next_line = 0; J.ph = J_MOVE;
                }
            }
            J.rd_inflight--; free_slot(s); break;
        }
        case SLK_SRC: {                                                        // 源行 ⇒ 转发成写槽 ✓
            if (cache_on) {                                                    // ★ 填表(态 = GO 带回态)✓
                int ci = cch_find(S.line);
                if (ci < 0) { ci = cch_victim; cch_victim = (cch_victim + 1) % cch_n; }   // 简单轮转替换
                CchLine &CL = cch[ci];
                CL.valid = true; CL.tag = S.line;
                CL.st = (S.go_state <= 3) ? S.go_state : 1;                    // 缺省 S ✓
                for (int k = 0; k < cxlshim::LINE_BYTES; k++) CL.data[k] = S.buf[k];
                cch_fills++;
            }
            if (J.line_wr) {                                                   // ★ v1.0:线侧 ⇒ 存载荷 ✓
                int off = S.li * 64;                                           // (不发本地写槽 = 真 RDMA 语义)✓
                for (int k = 0; k < cxlshim::LINE_BYTES; k++)
                    if ((uint32_t)(off + k) < J.len.to_uint() && (off + k) < LINE_PAY_MAX)
                        J.pbuf[off + k] = S.buf[k];
                J.rd_done++; J.rd_inflight--; free_slot(s); break;
            }
            if (cache_on && !J.is_send) {                                      // ★ 写命中 ⇒ 就地 + M ✓
                int ci = cch_find((J.dst + (sc_dt::uint64)S.li * 64) >> 6);
                if (ci >= 0) {                                                 // (不落内存;可见性由 snoop 保证)✓
                    CchLine &CL = cch[ci];
                    for (int k = 0; k < cxlshim::LINE_BYTES; k++) CL.data[k] = S.buf[k];
                    CL.st = CCH_M; cch_wr_hit++;
                    J.fwd_cnt++; J.wr_done++; J.rd_inflight--; free_slot(s); break;
                }
            }
            int w = alloc_wr();
            if (w < 0) break;                                                  // 无写槽:压在读槽,下拍再试 ✓
            Slot &W = sl[w];
            memcpy(W.buf, S.buf, cxlshim::LINE_BYTES);
            W.in_use = true; W.is_wr = true; W.done = false; W.err = false;
            W.job = j; W.li = S.li; W.kind = SLK_SRC;
            W.line = (J.dst + (sc_dt::uint64)S.li * 64) >> 6;
            J.slots_owned++; wpq[wp_t] = w; wp_t = (wp_t + 1) % NSLOT; wp_n++;
            J.fwd_cnt++; J.rd_inflight--; free_slot(s); break;
        }
        case SLK_CQE: {                                                        // CQ 行 ⇒ RMW 后转发 ✓
            int w = alloc_wr();
            if (w < 0) break;
            Slot &W = sl[w];
            memcpy(W.buf, S.buf, cxlshim::LINE_BYTES);
            uint32_t off = (uint32_t)(J.cq_byte & 0x3C);                       // 行内 4B 槽 ✓
            W.buf[off+0] = (unsigned char)J.wrid.range(7,0).to_uint();         // CQE = {7'h0,fatal,opcode,wrid}
            W.buf[off+1] = (unsigned char)J.wrid.range(15,8).to_uint();        // (照 §9.4;⚠ 整行 RMW 代字节使能)✓
            W.buf[off+2] = (unsigned char)J.opcode.to_uint();
            W.buf[off+3] = 0;
            W.in_use = true; W.is_wr = true; W.done = false; W.err = false;
            W.job = j; W.li = 0; W.kind = SLK_CQE;
            W.line = J.cq_byte >> 6;
            J.slots_owned++; wpq[wp_t] = w; wp_t = (wp_t + 1) % NSLOT; wp_n++;
            J.rd_inflight--; free_slot(s); break;
        }
        }
    }

    // ---------- (4) 相位推进与回收 ----------
    for (int j = 0; j < NJOB; j++) {
        Job &J = jobs[j];
        if (!J.act) continue;
        if (J.ph == J_DEAD) {
            if (J.slots_owned <= 0) { J.act = false; J.ph = J_FREE; }          // 死 job 排空 ⇒ 释放 ✓
            continue;
        }
        // ★ v1.0 SEND:RQ 项地址锁定(不满才锁;满 ⇒ RNR 记账 + 中断 bit3,下拍再试)✓
        if (J.ph == J_MOVE && J.is_send && !J.rq_ready) {
            if (rq_full(J.qp)) {
                if (!J.rnr_seen) { J.rnr_seen = true; rnr_cnt = rnr_cnt + 1;
                                   m_intr_sts[3] = 1; }                     // rnr_nack_gen ✓(§10.2 bit3)
            } else {
                J.dst = rq_entry_addr(J.qp);                                // 项地址(引擎锁定时算)✓
                J.rq_ready = true;
            }
        }
        if (J.ph == J_MOVE && J.is_send && !J.rq_ready) continue;           // 未锁定 ⇒ 不推进 ✓
        // ★ M3:超时 ⇒ 重发同一包(RETRY 次;复用原 PSN;首重发时回写 WQE retried)✓
        if (J.ph == J_MOVE && J.line_wr && J.pkt_sent && !J.ack_seen && line_tmo_ticks > 0) {
            if (++J.tmo_cnt > line_tmo_ticks) {
                J.tmo_cnt = 0;
                if (J.retries < (int)qp_retry_max(J.qp)) {
                    J.retries++; line_retries++;
                    if (!J.wqe_retried) { J.rnr_retries = 1;              // 复用"非首包"口径 ⇒ 原 PSN ✓
                                          wqe_writeback_retry(J, j); }
                    J.pkt_sent = false;                                   // ⇒ 组包块重发 ✓
                } else {
                    J.ph = J_DEAD; line_rtry_exh++; goerr_cnt = goerr_cnt + 1;   // 耗尽 ⇒ 异常终结 ✗
                }
            }
        }
        // ★ v1.0(/M2):线侧真包 —— 组包上线路(PSN 取 QP_SQ_PSN 并推进;RNR 重发走同路)✓
        if (J.ph == J_MOVE && J.line_wr && !J.pkt_sent &&
            (J.line_op == Job::LINE_RD || J.rd_done == J.nlines)) {
            unsigned char pkt[rocepkt::PKT_MAX];
            // ★ 重发(RNR 后)复用**原 PSN**(不重读已推进的寄存器 ⇒ 重发包与原包逐字节同)✗✓
            uint32_t psn = (J.rnr_retries > 0) ? J.psn_used
                                               : (qp_reg(J.qp, 0x40) & 0xFFFFFF);   // QP_SQ_PSN(0x40)✓
            // ★ D16:包字段改读**寄存器面**(照 `RTL 源` 偏移表 )✓
            uint32_t r_dqp   = qp_reg(J.qp, 0x48) & 0xFFFFFF;                        // DEST_QP_CONF(0x48)✓
            uint16_t r_pkey  = (uint16_t)((qp_reg(J.qp, 0x04) >> 16) & 0xFFFF);      // QP_ADV_CONF[31:16](0x04)✓
            uint16_t r_sport = (uint16_t)((m_gf[0].to_uint() >> 8) & 0xFFFF);        // NIC_CONF[23:8](全局 0x000)✓
            // ★ D16(续):eth DA ← MAC_REMOTE(0x50/0x54;装配 `{MSB[15:0],LSB[31:0]}`)✓
            //   IP dst ← IP_REMOTE_ADDR1(0x60;IPv4 = 低 32 位 {a4,a3,a2,a1}`)✓
            //   ⚠ 头字节 = 寄存器**低字节先**(照 换序装配)⇒ TB 写字节反序值 ✓
            uint64_t r_mac48 = ((uint64_t)(qp_reg(J.qp, 0x54) & 0xFFFF) << 32)
                             | (uint64_t)qp_reg(J.qp, 0x50);
            unsigned char r_da[6];
            for (int i = 0; i < 6; i++) r_da[i] = (unsigned char)((r_mac48 >> (8 * i)) & 0xFF);
            uint32_t r_dip = qp_reg(J.qp, 0x60);
            unsigned char r_dipb[4];
            for (int i = 0; i < 4; i++) r_dipb[i] = (unsigned char)((r_dip >> (8 * i)) & 0xFF);
            // ★ D16(续②):eth SA ← MAC_NIC_ADDR(全局 0x010/0x014,`21'h10_0010/14`);
            //   IPv4 src ← IPV4_NIC_ADDR(全局 0x070,`21'h10_0070`/"IPv4 (own) address")✓
            uint64_t r_mac48s = ((uint64_t)(m_gf[0x014 >> 2].to_uint() & 0xFFFF) << 32)
                              | (uint64_t)m_gf[0x010 >> 2].to_uint();
            unsigned char r_sa[6];
            for (int i = 0; i < 6; i++) r_sa[i] = (unsigned char)((r_mac48s >> (8 * i)) & 0xFF);
            uint32_t r_sip = m_gf[0x070 >> 2].to_uint();
            unsigned char r_sipb[4];
            for (int i = 0; i < 4; i++) r_sipb[i] = (unsigned char)((r_sip >> (8 * i)) & 0xFF);
            uint32_t r_sip_be = ((uint32_t)r_sipb[0] << 24) | ((uint32_t)r_sipb[1] << 16) |
                                ((uint32_t)r_sipb[2] << 8)  | (uint32_t)r_sipb[3];
            (void)J.dest_qpid;                                                       // (WQE byte17 不再是真源)✗
            int n = 0;
            // ⚠ 头字节序:DA/IP-dst 取**寄存器低字节先**;IP-dst 的 put32 需**反序**传入 ✓
            uint32_t r_dip_be = ((uint32_t)r_dipb[0] << 24) | ((uint32_t)r_dipb[1] << 16) |
                                ((uint32_t)r_dipb[2] << 8)  | (uint32_t)r_dipb[3];
            if (J.line_op == Job::LINE_WR)
                n = rocepkt::build_write_request(pkt, r_da, r_sa, r_sip_be, r_dip_be,
                                                 r_sport, lid.dport, rocepkt::OPC_WRITE_ONLY,
                                                 r_dqp, psn, true,
                                                 J.dst /*RETH.va = remote_offset(roff)*/,
                                                 J.rkey, J.pbuf, (int)J.len.to_uint(), r_pkey);
            else if (J.line_op == Job::LINE_SEND)
                n = rocepkt::build_send_only(pkt, r_da, r_sa, r_sip_be, r_dip_be,
                                             r_sport, lid.dport, r_dqp, psn,
                                             J.pbuf, (int)J.len.to_uint(), r_pkey);
            else
                n = rocepkt::build_read_request(pkt, r_da, r_sa, r_sip_be, r_dip_be,
                                                r_sport, lid.dport, r_dqp, psn,
                                                J.src /*RETH.va = ro(远端);本地落点 = J.dst*/,
                                                J.rkey, J.len.to_uint() /*读长 ⇒ RETH.dma_len*/,
                                                r_pkey);
            if (mac_tx_try_enqueue(pkt, n)) {                                // TX 忙 ⇒ 下拍重试 ✓
                J.pkt_sent = true;
                J.dest_qpid = r_dqp;                                         // ★ D16:匹配键 = **实发** DestQP ✓
                if (J.rnr_retries == 0) {                                    // **首次才推进**(重发不推进)✓
                    J.psn_used = psn;
                    m_qp[J.qp][0x40 >> 2] = (sc_uint<32>)((psn + 1) & 0xFFFFFF);
                }
                line_tx_pkts = line_tx_pkts + 1;
            }
        }
        // ★ M2:RD 响应载荷 ⇒ 逐行写槽(槽紧俏 ⇒ 每拍补一行;照真 RDMA"落远端内存")✓
        if (J.ph == J_MOVE && J.line_wr && J.line_op == Job::LINE_RD &&
            J.ack_seen && J.wr_queued < J.nlines) {
            int w = alloc_wr();
            if (w >= 0) {
                Slot &W = sl[w];
                int off = J.wr_queued * 64;
                for (int k = 0; k < cxlshim::LINE_BYTES; k++)
                    W.buf[k] = ((off + k) < J.paylen) ? J.pbuf[off + k] : 0;
                W.in_use = true; W.is_wr = true; W.done = false; W.err = false;
                W.job = j; W.li = J.wr_queued; W.kind = SLK_SRC;
                W.line = (J.dst + (sc_dt::uint64)J.wr_queued * 64) >> 6;
                J.slots_owned++; wpq[wp_t] = w; wp_t = (wp_t + 1) % NSLOT; wp_n++;
                J.wr_queued++;
            }
        }
        bool move_done;
        if (J.line_wr) {
            if (J.line_op == Job::LINE_RD)                                    // ★ M2:RD 须响应到 + 载荷写完 ✓
                move_done = J.pkt_sent && J.ack_seen && (J.wr_queued == J.nlines) && (J.wr_done == J.nlines);
            else
                move_done = J.pkt_sent && J.ack_seen;                         // WR/SEND:成功 ACK 即完 ✓
        } else {
            move_done = (J.fwd_cnt == J.nlines && J.wr_done == J.nlines);
        }
        if (J.ph == J_MOVE && move_done) {
            // ★ SEND:数据都送达 ⇒ **推进 RQ 写指针**(每消息一次,照)✓
            //   + 引擎写 STAT_RQ_PI_DB(0x9C,RO)= 新 wrptr(生产者指针的 SW 可见面)✓
            if (J.is_send) {
                uint32_t depth = rq_depth(J.qp);
                uint16_t w = rq_wrptr[J.qp].to_uint();
                rq_wrptr[J.qp] = (sc_uint<16>)((w == depth) ? 1 : (w + 1));
                m_qp[J.qp][0x9C >> 2] = (sc_uint<32>)rq_wrptr[J.qp].to_uint();
            }
            // 进入 CQE:算目标字节(照 §9.2:索引 = (hd==depth)?0:hd;指针 1..depth)✓
            uint32_t depth = qp_reg(J.qp, 0x3C) & 0xFFFF;
            uint16_t hd = cq_hdptr[J.qp].to_uint();
            uint32_t cq_idx = (hd == depth) ? 0 : hd;
            J.cq_byte = (sc_dt::uint64)qp_reg(J.qp, 0x18) + (sc_dt::uint64)cq_idx * 4;
            J.ph = J_CQE;
        }
    }

    // ---------- (5) 门铃 → job(同 QP **保序**:在飞 ≤ 1)----------
    if (db_n > 0) {
        // ★ 跳过被"同 QP 保序"挡住的门铃(**不许队头阻塞**:后面的别的 QP 要能先跑)✓
        //   队列按序扫描 ⇒ 同 QP 的门铃仍严格 FIFO ✓
        int pick = -1;
        for (int k = 0; k < db_n; k++) {
            int qp = db_q[(db_h + k) % DBDEPTH].qp;
            if (qp_fatal[qp]) {                                  // ★ fatal QP:丢弃该门铃(计数,不静默)✗
                db_q[(db_h + k) % DBDEPTH] = db_q[(db_h + db_n - 1) % DBDEPTH];
                db_t = (db_t + DBDEPTH - 1) % DBDEPTH; db_n--;
                qp_fatal_cnt = qp_fatal_cnt + 1;                 // ⚠ 与"置 fatal"共用计数(事件数)✓
                k--; continue;
            }
            bool busy = false;
            for (int j = 0; j < NJOB; j++) if (jobs[j].act && jobs[j].qp == qp) busy = true;
            if (!busy) { pick = k; break; }
        }
        int jf = -1;
        for (int j = 0; j < NJOB; j++) if (!jobs[j].act) { jf = j; break; }
        if (pick >= 0 && jf >= 0) {
            int e = (db_h + pick) % DBDEPTH;
            int qp = db_q[e].qp;
            Job &J = jobs[jf];
            J.act = true; J.qp = qp; J.pi = db_q[e].pi; J.ph = J_WQE;
            J.wrid = 0; J.opcode = 0; J.src = J.dst = 0; J.len = 0;
            J.nlines = 0; J.next_line = 0; J.rd_inflight = 0; J.fwd_cnt = 0; J.wr_done = 0;
            J.slots_owned = 0; J.cqe_rd_issued = false; J.cq_byte = 0; J.seq = job_seq; job_seq = job_seq + 1;
            J.is_send = false; J.rq_ready = false; J.rnr_seen = false;
            J.line_wr = false; J.line_op = Job::LINE_WR;
            J.pkt_sent = false; J.ack_seen = false; J.rd_done = 0;
            J.wr_queued = 0; J.paylen = 0; J.rnr_retries = 0;
            J.tmo_cnt = 0; J.retries = 0; J.wqe_retried = false; J.wqe_line = 0;
            J.dest_qpid = 0; J.rkey = 0; J.psn_used = 0;
            // 摘除该条目(后续前移,保持 FIFO 序)✓
            for (int k = pick; k < db_n - 1; k++)
                db_q[(db_h + k) % DBDEPTH] = db_q[(db_h + k + 1) % DBDEPTH];
            db_t = (db_t + DBDEPTH - 1) % DBDEPTH; db_n--;
        }
    }

    // ---------- (6) 读发射(轮转挑 job;无读槽则本拍不发)----------
    if (tx == TX_IDLE) {
        int cj = -1, ckind = 0, cli = 0; sc_dt::uint64 cline = 0;
        for (int k = 0; k < NJOB; k++) {
            int j = (rr_next + k) % NJOB; Job &J = jobs[j];
            if (!J.act || J.ph == J_DEAD) continue;
            if (J.ph == J_WQE && J.rd_inflight == 0) {      // ⚠ WQE 行读**只许在飞一笔**(否则重复发)✓
                uint32_t depth = qp_reg(J.qp, 0x3C) & 0xFFFF;
                uint32_t idx = (J.pi.to_uint() == depth) ? 0 : J.pi.to_uint();       // 1-based ✓
                cline = ((sc_dt::uint64)qp_reg(J.qp, 0x10) + (sc_dt::uint64)idx * 64) >> 6;
                cj = j; ckind = SLK_WQE; cli = 0; break;
            }
            // ⚠ SRC 行读唯一性由 `next_line` 自增保证(发出即推进)✓
            // ⚠ SEND 未锁 RQ 项(dst 未定)⇒ 不发读 ✓
            // ★ M2:**线侧 RD** 不发源读(nlines 由响应载荷定;误发会读 VA 乱地址)—— WR/SEND 照发 ✓
            if (J.ph == J_MOVE && !(J.line_wr && J.line_op == Job::LINE_RD) &&
                (!J.is_send || J.rq_ready) &&
                J.next_line < J.nlines && J.rd_inflight < RD_INFLIGHT_MAX) {
                cline = (J.src + (sc_dt::uint64)J.next_line * 64) >> 6;
                cj = j; ckind = SLK_SRC; cli = J.next_line; break;
            }
            if (J.ph == J_CQE && !J.cqe_rd_issued) {
                cline = J.cq_byte >> 6;
                cj = j; ckind = SLK_CQE; cli = 0; break;
            }
        }
        int s = (cj >= 0) ? alloc_rd() : -1;
        if (s >= 0) {
            Slot &S = sl[s];
            S.in_use = true; S.is_wr = false; S.done = false; S.err = false;
            S.job = cj; S.li = cli; S.kind = ckind; S.line = cline;
            S.go_state = 1;                                                            // 缺省 S ✓
            Job &J = jobs[cj];
            J.slots_owned++; J.rd_inflight++;
            if (ckind == SLK_SRC) J.next_line++;
            if (ckind == SLK_CQE) J.cqe_rd_issued = true;
            int chit = (cache_on && ckind == SLK_SRC) ? cch_find(cline) : -1;
            if (chit >= 0) {                                                           // ★ 命中 ✓
                for (int k = 0; k < cxlshim::LINE_BYTES; k++) S.buf[k] = cch[chit].data[k];
                S.done = true; cch_hits++;                                             // 不落内存(tx 保持 IDLE)✓
            } else {
                tx = TX_RREQ; tx_slot = s; tx_line = cline;                            // cqid = 槽号 ✓
            }
            rr_next = (cj + 1) % NJOB;
        }
    }

    // ---------- (7) 写发射(FIFO:分配序 = 服务序)----------
    if (tx == TX_IDLE && wp_n > 0) { tx = TX_WREQ; tx_slot = wpq[wp_h]; }

    // ---------- (8) 本拍输出(纯状态函数;每通道逐式一致 ⇒ prev 才是"线上值")----------
    bool r0v = false, r1v = false, wdv = false;
    if (tx == TX_RREQ) {
        r0v = true;
        d2h_req0_opcode.write(cxlshim::D2H_REQ_SPEC_RD_SHARED);
        d2h_req0_cqid.write((sc_uint<12>)tx_slot);
        d2h_req0_addr.write((sc_uint<46>)tx_line);
    }
    if (tx == TX_WREQ) {
        r1v = true;
        d2h_req1_opcode.write(cxlshim::D2H_REQ_MEMWR);
        d2h_req1_cqid.write((sc_uint<12>)tx_slot);
        d2h_req1_addr.write((sc_uint<46>)sl[tx_slot].line);
    }
    if (tx == TX_WDATA) {
        wdv = true;
        d2h_data_uqid.write((sc_uint<12>)tx_slot);       // 照 shim:写数据 uqid = tag(槽号)✓
        d2h_data_chunk_valid.write(true);
        d2h_data_payload.write(bytes_to_line(sl[tx_slot].buf));
    } else if (snp_data_v) {                             // ★ M2':snoop 行数据(uqid = snoop uqid)✓
        wdv = true;                                      // (互斥:snp_data_v 只在 tx==IDLE 时抬 ✓)
        d2h_data_uqid.write((sc_uint<12>)(snp_uqid & 0xFFF));
        d2h_data_chunk_valid.write(true);
        d2h_data_payload.write(bytes_to_line(snp_data_buf));
    }
    d2h_req0_valid.write(r0v);
    d2h_req1_valid.write(r1v);
    d2h_data_valid.write(wdv);
    tx_reqv_prev = r0v || r1v;
    tx_datav_prev = wdv;

    h2d_rsp_ready.write(true);  h2r_prev = true;         // 设备恒可吸收 GO/数据(槽已就位)✓
    h2d_data_ready.write(true); h2d_prev  = true;

    // ---------- (9) 观测高水位(多笔在飞 / 同 QP 保序见证)----------
    int ostd = 0;
    for (int s = 0; s < NSLOT; s++) if (sl[s].in_use && !sl[s].is_wr) ostd++;
    if ((sc_uint<32>)ostd > max_rd_ostd) max_rd_ostd = (sc_uint<32>)ostd;
    int per[NQP]; for (int q = 0; q < NQP; q++) per[q] = 0;
    for (int j = 0; j < NJOB; j++) if (jobs[j].act) per[jobs[j].qp]++;
    for (int q = 0; q < NQP; q++) if ((sc_uint<32>)per[q] > max_act_per_qp) max_act_per_qp = (sc_uint<32>)per[q];

    // ---------- (10) MAC 面向(★ v1.0;M2 帧级透传,TB 钩子驱动)----------
    mac_engine();
}

//============================================================================
// MAC 面向引擎(v1.0):TX 流式发帧(2048b AXIS;tready 采样推进)+ RX 采帧入取回槽
// ⚠ M2 边界:本口**不做组帧**(BTH/RETH/ICRC)——帧内容 = TB 注入什么发什么 ✓
//============================================================================
void DeviceTlm::mac_engine() {
    // ---- TX:单帧流式,tready(电平)采样推进 ----
    bool tv = false, tl = false; sc_biguint<2048> td = 0; sc_biguint<256> tk = 0;
    if (mac_txbusy) {
        int nb = mac_txlen - mac_txpos * MAC_AXIS_BYTES;
        if (nb > MAC_AXIS_BYTES) nb = MAC_AXIS_BYTES;
        bool last = (mac_txpos * MAC_AXIS_BYTES + nb >= mac_txlen);
        for (int j = 0; j < nb; j++) {
            td.range(j*8+7, j*8) = mac_txbuf[mac_txpos * MAC_AXIS_BYTES + j];
            tk[j] = true;
        }
        tv = true; tl = last;
        if (cmac_m_axis_tready.read()) {                 // 电平采样:可收 ⇒ 推进一拍
            mac_txpos++;
            if (last) { mac_txbusy = false; mac_tx_frames++; }
        }
    }
    cmac_m_axis_tvalid.write(tv); cmac_m_axis_tlast.write(tl);
    cmac_m_axis_tdata.write(td);  cmac_m_axis_tkeep.write(tk);

    // ---- RX:采 beat(⚠ 无 ready ⇒ 恒收);tlast ⇒ 成帧入取回槽 ----
    if (roce_cmac_s_axis_tvalid.read()) {
        sc_biguint<2048> rd = roce_cmac_s_axis_tdata.read();
        sc_biguint<256>  rk = roce_cmac_s_axis_tkeep.read();
        for (int j = 0; j < MAC_AXIS_BYTES; j++)
            if (rk[j].to_bool()) {
                if (mac_rxlen < (int)sizeof(mac_rxbuf)) mac_rxbuf[mac_rxlen++] = (unsigned char)rd.range(j*8+7, j*8).to_uint();
                else { mac_rx_drop_cnt++; }
            }
        if (roce_cmac_s_axis_tlast.read()) {
            bool consumed = line_mode && line_rx_deliver();       // ★ v1.0:M2 响应匹配 ⇒ 消费 ✓
            if (!consumed) {
                if (!mac_rxq_ready) {
                    int n = (mac_rxlen < (int)sizeof(mac_rxq)) ? mac_rxlen : (int)sizeof(mac_rxq);
                    for (int i = 0; i < n; i++) mac_rxq[i] = mac_rxbuf[i];
                    mac_rxqlen = n; mac_rxq_ready = true;
                } else { mac_rx_drop_cnt++; }            // 取回槽未取走 ⇒ 丢并计数 ✗
            }
            mac_rxlen = 0; mac_rx_frames++;
        }
    }
    o_mac_tx_frames.write(mac_tx_frames); o_mac_rx_frames.write(mac_rx_frames);
    o_line_tx_pkts.write(line_tx_pkts); o_line_rx_acks.write(line_rx_acks);
    o_line_unexp_cnt.write(line_unexp_cnt); o_line_big_cnt.write(line_big_cnt);
}

//============================================================================
// ★ v1.0:线侧真包两件 —— TX 入队(忙不丢,调用方重试)/ RX ACK 匹配(消费)✓
//============================================================================
bool DeviceTlm::mac_tx_try_enqueue(const unsigned char * d, int n) {
    if (mac_txbusy || n <= 0 || n > (int)sizeof(mac_txbuf)) return false;
    for (int i = 0; i < n; i++) mac_txbuf[i] = d[i];
    mac_txlen = n; mac_txpos = 0; mac_txbusy = true;
    return true;
}

//============================================================================
// ★ 缓存行表两件(查/读态;语义照 `规格文档` §1.2)✓
//============================================================================
// ★ M3:2.1j 回写 —— WQE byte2 bit0 = retried(照 `sw/nic_regs.h WQE_RETRY_OFF 2 [16])✓
//   (buf_id[23:17] 模型恒 0:v1 无多缓冲语义;⚠ 明标占位)✓
void DeviceTlm::wqe_writeback_retry(Job &J, int ji) {
    int w = alloc_wr();
    if (w < 0) return;                                    // 无槽 ⇒ 放弃本次回写(重发照走,不阻塞)✓
    Slot &W = sl[w];
    for (int k = 0; k < cxlshim::LINE_BYTES; k++) W.buf[k] = J.wqe_buf[k];
    W.buf[2] |= 0x01;                                     // WQE byte2 bit0 = retried = 1(regs.h)✓
    W.in_use = true; W.is_wr = true; W.done = false; W.err = false;
    W.job = ji; W.li = 0; W.kind = SLK_WBK;               // 一次性回写(不参与数据搬运计数)✓
    W.line = J.wqe_line;
    J.slots_owned++; wpq[wp_t] = w; wp_t = (wp_t + 1) % NSLOT; wp_n++;
    J.wqe_retried = true;
}

int DeviceTlm::cch_find(sc_dt::uint64 line) {
    for (int i = 0; i < cch_n; i++)
        if (cch[i].valid && cch[i].tag == line) return i;
    return -1;
}
int DeviceTlm::cch_state(sc_dt::uint64 byte_addr) {
    int i = cch_find(byte_addr >> 6);
    return (i >= 0) ? (int)cch[i].st : 0;          // 未命中 = I ✓
}

// 成帧后:响应 (DestQP,PSN) 命中在飞线侧 job ⇒ **消费**(返 true);否则计数并留进取回槽 ✗
//   ⚠ 匹配口径 = **等值**(RTL unack-窗口校验在"每 QP 单在飞"下退化;见 c2 规格 §3.1-③)✓
//   · ACK(0x11):syndrome 类型 ACK ⇒ 完成;RNR_NAK ⇒ **重发同一包**(PSN 不推进,封顶 8);
//     NAK(seq err 等)⇒ M2 判死 + 计数 ✗
//   · RD_RSP_ONLY(0x10)⇒ **仅** RD job:载荷落 pbuf(待逐行写回本地目标)✓
bool DeviceTlm::line_rx_deliver() {
    unsigned char opc = 0; uint32_t dq = 0, psn = 0;
    if (!rocepkt::parse_bth(mac_rxbuf, mac_rxlen, opc, dq, psn)) {
        line_unexp_cnt = line_unexp_cnt + 1; return false;
    }
    int hit = -1;
    for (int j = 0; j < NJOB; j++) {
        Job &J = jobs[j];
        if (!(J.act && J.line_wr && J.pkt_sent && !J.ack_seen)) continue;
        if (J.dest_qpid != dq || J.psn_used != psn) continue;             // ★ PSN 等值校验 ✓
        if (opc == rocepkt::OPC_ACK        && J.line_op == Job::LINE_RD) continue;   // 类型须配 ✓
        if (opc == rocepkt::OPC_RD_RSP_ONLY && J.line_op != Job::LINE_RD) continue;
        if (opc != rocepkt::OPC_ACK && opc != rocepkt::OPC_RD_RSP_ONLY) continue;
        hit = j; break;
    }
    if (hit < 0) { line_unexp_cnt = line_unexp_cnt + 1; return false; }   // PSN 乱序/无主 ⇒ 丢弃+计数 ✓
    Job &J = jobs[hit];
    if (opc == rocepkt::OPC_ACK) {
        unsigned ty = (unsigned)((mac_rxbuf[54] >> 5) & 3u);              // AETH.Syndrome 类型)✓
        if (ty == 0) {                                                    // ACK ✓
            J.ack_seen = true; line_rx_acks = line_rx_acks + 1;
        } else if (ty == 1) {                                             // RNR-NAK ⇒ 重发同包 ✓
            if (J.rnr_retries < (int)qp_rnr_retry_max(J.qp)) {            // ★ M3:上限 = RNR_RETRY([13:11])✓
                J.rnr_retries++; J.pkt_sent = false; line_rx_acks = line_rx_acks + 1;
            }
            else { J.ph = J_DEAD; line_rtry_exh++; goerr_cnt = goerr_cnt + 1; }   // 耗尽 ⇒ 异常终结 ✗
        } else {                                                          // NAK ⇒ M2:判死 + 计数 ✗
            J.ph = J_DEAD; goerr_cnt = goerr_cnt + 1;
        }
        return true;
    }
    {                                                                     // RD_RSP_ONLY(且已检 line_op == RD)✓
        int pay = mac_rxlen - 58 - 4;                                     // 载荷 = 总长 - 头(AETH 至 57)- ICRC ✓
        if (pay < 0) pay = 0;
        if (pay > LINE_PAY_MAX) { J.ph = J_DEAD; goerr_cnt = goerr_cnt + 1; return true; }
        for (int k = 0; k < pay; k++) J.pbuf[k] = mac_rxbuf[58 + k];
        J.paylen = pay;
        J.nlines = (pay + 63) / 64; if (J.nlines == 0) J.nlines = 1;
        J.wr_queued = 0; J.ack_seen = true;
        line_rx_acks = line_rx_acks + 1;
        return true;
    }
}

void DeviceTlm::mac_push_tx_frame(const unsigned char* d, int n) {
    if (!mac_tx_try_enqueue(d, n)) mac_tx_drop_cnt++;    // ★ v1.0:与线侧共用一个入队 ✓
}

int DeviceTlm::mac_pop_rx_frame(unsigned char* d, int maxn) {
    if (!mac_rxq_ready) return 0;
    int n = (mac_rxqlen < maxn) ? mac_rxqlen : maxn;
    for (int i = 0; i < n; i++) d[i] = mac_rxq[i];
    mac_rxq_ready = false; mac_rxqlen = 0;
    return n;
}

void DeviceTlm::engine_thread() {
    eng_reset();                                         // 输出初值(本线程独占引脚)✓
    wait();
    while (true) {
        wait();                                          // ★ 每轮必等一拍
        if (!rst_n.read()) { eng_reset(); continue; }
        eng_step();
    }
}
