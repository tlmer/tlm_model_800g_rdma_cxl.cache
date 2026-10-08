//============================================================================
// device_tlm.h — 设备侧 TLM(v1.0:QP fatal + NQP=32)  [2026-10-04 建 / 同日升]
//
// 规格 = `规格文档`(P2.1 边界)+ `规格文档`
//   **§8 寄存器全表**(191 条 / 三段 / 属性 RW·RO·W1C / 门铃清单)/ **§9 WQE·CQE·RQ** /
//   **§10 中断**(9 源 + INTR_EN + INTR_STS(W1C)+ 一根电平线)✓
//
// ★★ v1.0(本版)= **多笔在飞**(v1.0 是"阻塞单笔"):阻塞 SC_THREAD 链 ⇒ **每拍一步的非阻塞 FSM**
//   + **每事务状态**(在飞 job 表,每 WQE 一份)+ **事务槽表**;槽号/匹配口径**逐条照 RTL**:
//   · 槽号:`RTL 源`(读槽 **0..63** / 写槽 **64..127**;空闲表 FIFO,初值 = 0..63/0..63)
//     ⇒ 线上 **cqid = 零扩槽号**(SLOT_IDX_W=7);出处 `RTL 源(tag = 零扩 slot)
//     + `RTL 源`(**cqid = tag**)✓
//   · 应答匹配:回程 GO/数据带 **cqid** ⇒ `cqid[6:0]` 即槽号(`RTL 源` 同口径)✓
//   · 同 QP **保序**:同 QP 在飞 job ≤ 1(RDMA 语义:QP 内 WQE 按序执行)⇒ CQE 序/头指针天然有序 ✓
//   · 发射单元**单一**(req0/req1/写数据 三通道互斥):⚠ 桥 ingress 同拍只收一路(读优先)⇒
//     设备侧**不许同拍抬 req0+req1 valid**(桥侧已加 rdy1 门兜底,见桥 v1.0)✗✓
//
// ★★ v1.0(本版)= **队列语义补全①**:RQ 缓冲环(§9.3)+ SEND/IMMDT opcode;口径逐条见
//   `规格文档` **§4.4**(项地址映射 = SQ 同款 / 指针每消息一次 / 满 ⇒ RNR-NAK + 中断 bit3 /
//   IMMDT 侧带不在模型边界)。⚠ 单设备环回:"对端 RQ" = **本设备该 QP 的 RQ 环**(真 dest_qpid 待线侧 2.4)✓
// ★★ v1.0(本版)= **COH 事务语义补全**:snoop 恒回 RspI,但**对在飞地址先等完成**
//   (出处 `RTL 源` + `RTL 源` snp_chk 循环)⇒ 应答条件 =
//   **无在飞槽(读/写)地址匹配**;压住期间不收新 snoop(照 RTL:检查恒指向队头)✓
//   ⚠ 边界说明:lane 前缀位 = **待核**(本 RTL 路径 shim 直通、低 7 位即槽号);
//     AXI 域 per-ID 重排在**本模型边界(CXL.cache)之外** ⇒ 不建模 ✗
// ★ 测试注入(**白盒钩子,明标**):`set_intr_status(bit, v)` / `set_rq_pi(qp, v)` —— ⛔ 只有 TB 可调;
//   ⚠ `set_rq_pi` 的对象 `STAT_RQ_PI_DB` 自 v1.0 起**引擎即真源**(SEND 送达时自写),钩子退为 TB 注入用 ✗
// ★ 已知简化(明写,见 P2 规格 §4):R_key/VA 不译(offset 直用为字节地址);CQE 4B 用整行 RMW;
//   未知 opcode 的 RTL ILL_OPCODE FSM 未开卷 ⇒ 模型占位(不搬数、不出 CQE、job 判死)✓
//============================================================================
#ifndef DEVICE_TLM_H
#define DEVICE_TLM_H

//============================================================================
// ★ 版本与冻结策略(计划 1.11)⚠ `.h` 一改 = 客户侧重编
//   · **冻结时机** = 客户首次集成前;冻结后**任何接口变更** ⇒ MINOR 进位 + 下表加一行 ✓
//   变更记录:
//     (2026-10-05) ★ **+M2:线侧真 RoCE 包(单包 WRITE/SEND/READ;成功响应才完成)** ——
//                        `set_line_pkt_mode(true)`(**默认关 ⇒ 旧 TB 零影响**):WQE 由引擎**自己组包**
//                        (包库单一定义处 = `peer/roce_pkt.h`;opcode/AETH/PSN 口径全照 RTL 开卷,见
//                        `规格文档` §3.1)经 MAC 面向口发出:
//                        · WR(0x00)⇒ BTH 0x0a;SEND(0x02)⇒ 0x04(无 RETH);RD(0x04)⇒ 0x0c(RETH.dma_len = 读长)✓
//                        · **成功响应(BTH 0x11 且 AETH=ACK / 0x10 RD_RSP_ONLY)且 (DestQP,PSN) 命中**
//                          才判完成 ⇒ CQE + 中断;**RD 响应载荷落本地目标区**(逐行写槽)✓
//                        · **PSN 取 `QP_SQ_PSN`(0x40)并每包推进**(24b);**PSN 不匹配 ⇒ 丢弃 + 计数** ✓
//                        · **RNR-NAK ⇒ 重发同一包(PSN 不推进;重试封顶 8)** ✓
//                        ⚠ 边界:单包(载荷 ≤ `LINE_PAY_MAX`)+ 无超时重传(ACK 不到 ⇒ 停在 J_MOVE;
//                        参数化超时/重传 = M3)✗ ✓
//     (2026-10-05) ★ **A7 口径落地()**:寄存器面加 **AMBA-PV 从口**
//                       (`amba_pv::amba_pv_slave_socket<32> reg_s`;b_transport 经基类转用户层 read/write
//                        ⇒ **与直连方法同源** ⇒ 两路逐寄存器一致;⚠ 直连路径**保留**(TB/驱动沿用))✓
//     (2026-10-04) ★ **CQ 中断清位口径对齐(/D14)**:补 `CQ_INTR_STS1`(写 mask 1=清对应 QP);
//                        **INTR_STS[4] 改 per-QP 汇聚**(读=OR;写它**不再清** —— 照 RTL→ 死路,
//                        真清路 = 写 `CQ_INTR_STS1`→)✓
//     (2026-10-04) ★ **MAC 面向口**(照 `规格文档` §2 契约 = `nic` 实读;M2 = 帧级透传,
//                        TB 钩子注入/取回;⛔ 组帧(BTH/RETH/ICRC)= 后续,勿默认已建 RoCE 发包)✓
//     (2026-10-04) ★ 非法 opcode ⇒ **QP fatal + 中断 bit7**(照 RTL `ILL_OPCODE` 开卷);NQP 8→**32**(对齐 RTL)✓
//     (2026-10-04) COH 事务语义:snoop 对在飞地址先等完成 ✓
//     (2026-10-04) RQ 缓冲环 + SEND/IMMDT opcode ✓
//     (2026-10-04) 非阻塞引擎 + 多笔在飞 + 事务槽表 ✓
//     (2026-10-04) 端到端纵切(阻塞单笔)✓
//     (2026-10-04) 寄存器面 + 门铃 + 中断 ✓
//============================================================================
#define DEVICE_TLM_VERSION_MAJOR 1
#define DEVICE_TLM_VERSION_MINOR 0
#define DEVICE_TLM_VERSION_STR   "device_tlm 1.0 (2026-10-05)"

#include <systemc.h>
#include <amba_pv.h>          // ★ v1.0(A7):寄存器面 AMBA-PV 从口 ✓
#include "cxl_shim_types.h"

struct DeviceTlm: sc_core::sc_module,
                  amba_pv::amba_pv_slave_base<32> {   // ★ v1.0(A7):32 位寄存器面从端 ✓
    // ---------------- ★ v1.0(A7):寄存器面 AMBA-PV 从口 ----------------
    //   用法(平台/客户):`dev.reg_s.bind(平台的 master)` ⇒ b_transport / transport_dbg 由基类
    //   转译到本类 **read()/write()** ⇒ 内部调**既有 reg_read/reg_write**(同源 ⇒ 与直连一致)✓
    //   地址口径:与直连同 —— 低 21 位窗口(段 = addr[20:19]);未映射 ⇒ **DECERR**;RO 写 ⇒ **SLVERR** ✓
    amba_pv::amba_pv_slave_socket<32> reg_s;
    unsigned long reg_sock_n = 0;          // 观测:经 socket 的访问计数(判据用)✓
    virtual amba_pv::amba_pv_resp_t read(int, const sc_dt::uint64 & addr, unsigned char * data,
                                         unsigned int len, const amba_pv::amba_pv_control *,
                                         sc_core::sc_time &) override;
    // ⚠ AMBA-PV 基类的 write() 第 6 参 = **byte_enable 指针**(照 BL 头签名;read() 无此参)✓
    virtual amba_pv::amba_pv_resp_t write(int, const sc_dt::uint64 & addr, unsigned char * data,
                                          unsigned int len, const amba_pv::amba_pv_control *,
                                          unsigned char * byte_en, sc_core::sc_time &) override;

    // ---------------- 时钟/复位 ----------------
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    // ---------------- 软件面:寄存器访问(★ **路径待定**,见 规格文档 §2.3)----
    // ⚠ 候选:① AMBA-PV 从口 ② ACE-Lite 从口 ③ 平台直连 —— **已列入问题清单**(A7);
    //   本轮**不绑任何 socket**,寄存器语义用**直连方法** `reg_read/reg_write` 验 ✓
    //   (⚠ 这是**明写的范围缩减**,不是遗漏)✗

    // ---------------- 设备侧:CXL.cache(与桥的 device 侧**一一对应**,直连)----
    sc_out<bool>        d2h_req0_valid; sc_out<sc_uint<5>>  d2h_req0_opcode;
    sc_out<sc_uint<12>> d2h_req0_cqid;  sc_out<sc_uint<46>> d2h_req0_addr;  sc_in<bool> d2h_req0_ready;
    sc_out<bool>        d2h_req1_valid; sc_out<sc_uint<5>>  d2h_req1_opcode;
    sc_out<sc_uint<12>> d2h_req1_cqid;  sc_out<sc_uint<46>> d2h_req1_addr;  sc_in<bool> d2h_req1_ready;
    sc_out<bool>        d2h_data_valid; sc_out<sc_uint<12>> d2h_data_uqid;
    sc_out<bool>        d2h_data_chunk_valid; sc_out<sc_biguint<512>> d2h_data_payload; sc_in<bool> d2h_data_ready;
    sc_out<bool>        d2h_rsp_valid;  sc_out<sc_uint<5>>  d2h_rsp_opcode;
    sc_out<sc_uint<12>> d2h_rsp_uqid;   sc_in<bool>         d2h_rsp_ready;
    sc_in<bool>         h2d_req_valid;  sc_in<sc_uint<3>>   h2d_req_opcode;
    sc_in<sc_uint<46>>  h2d_req_addr;   sc_in<sc_uint<12>>  h2d_req_uqid;   sc_out<bool> h2d_req_ready;
    sc_in<bool>         h2d_rsp_valid;  sc_in<sc_uint<4>>   h2d_rsp_opcode;
    sc_in<sc_uint<12>>  h2d_rsp_rsp_data; sc_in<sc_uint<12>> h2d_rsp_cqid; sc_out<bool> h2d_rsp_ready;
    sc_in<bool>         h2d_data_valid; sc_in<sc_uint<12>>  h2d_data_cqid;
    sc_in<bool>         h2d_data_chunk_valid; sc_in<sc_biguint<512>> h2d_data_payload; sc_out<bool> h2d_data_ready;

    // ---------------- 中断(★ 一根电平线;照 §10.1)----------------
    sc_out<bool> nic_intr;

    // ---------------- MAC 面向口(★ v1.0;照 [`规格文档`](规格文档) §2 = `nic` 实读)----
    //   TX(NIC→MAC):RoCE 主通道 2048b AXIS + `tready`(入);RX(MAC→NIC):`roce_cmac_s_axis_*`
    //   ⚠ RX **无 ready**(恒收;照 RTL 实证);⚠ M2 = **帧级透传**(钩子注入/取回),
    //     组帧细节(BTH/RETH/ICRC)= 后续任务 ⇒ 勿把本口当"已会发 RoCE 包" ✗
    sc_out<bool>             cmac_m_axis_tvalid;
    sc_out<sc_biguint<2048>> cmac_m_axis_tdata;
    sc_out<sc_biguint<256>>  cmac_m_axis_tkeep;
    sc_out<bool>             cmac_m_axis_tlast;
    sc_in<bool>              cmac_m_axis_tready;
    sc_in<bool>              roce_cmac_s_axis_tvalid;
    sc_in<sc_biguint<2048>>  roce_cmac_s_axis_tdata;
    sc_in<sc_biguint<256>>   roce_cmac_s_axis_tkeep;
    sc_in<bool>              roce_cmac_s_axis_tlast;
    sc_in<bool>              roce_cmac_s_axis_tuser;
    // ★ MAC 面向:TB 钩子(M2 帧级透传;⛔ 只 TB 可调;非最终语义)✓
    void mac_push_tx_frame(const unsigned char* d, int n);   // 注入待发帧(忙则丢并计数)
    int  mac_pop_rx_frame(unsigned char* d, int maxn);       // 取回收帧;返回长度(0 = 无)
    unsigned int mac_tx_frames, mac_rx_frames;               // 计数(白盒)
    unsigned int mac_tx_drop_cnt, mac_rx_drop_cnt;           // 满/溢出丢弃(不静默)✗
    // —— MAC 透传内部态(单帧在飞即可,M2 用) ——
    static const int MAC_AXIS_BYTES = 256;                   // 2048b = 256B/拍
    unsigned char mac_txbuf[4096]; int mac_txlen, mac_txpos; bool mac_txbusy;
    unsigned char mac_rxbuf[4096]; int mac_rxlen;            // 收帧组装
    unsigned char mac_rxq[4096];   int mac_rxqlen; bool mac_rxq_ready;   // 取回槽
    void mac_engine();                                       // 每拍一步(eng_step 末调)

    // ---------------- ★ v1.0:线侧真 RoCE 包 ----------------
    //   用法(TB):`set_line_pkt_mode(true)` + `set_line_id({...})`(默认 = 基准 配方身份值)✓
    //   ⚠ 白盒旋钮:⛔ 只 TB/自测可调;真源(寄存器面 MAC/IP 配置)⏳ 待开卷 RTL(§3-⑤)✗
    static const int LINE_PAY_MAX = 256;                     // M1 单包载荷上限(>256B ⇒ 拒并计数)✗
    struct LineId { unsigned char da[6], sa[6]; uint32_t sip, dip; uint16_t sport, dport; };
    void set_line_pkt_mode(bool on) { line_mode = on; }      // 默认 false ⇒ 旧口径 ✓
    void set_line_id(const LineId & v) { lid = v; }
    bool line_mode = false;
    LineId lid;                                              // ctor 填默认(基准 配方)✓
    bool  mac_tx_try_enqueue(const unsigned char* d, int n); // 忙 ⇒ false(调用方下拍重试)✓
    bool  line_rx_deliver();                                 // 成帧后:响应匹配(ACK/RD_RSP/RNR;命中 ⇒ 消费)✓

    // ---------------- ★ 设备缓存行表(I/S/E/M;**默认关**)----------------
    //   语义全部照参照树(`规格文档` §1.2):**填充态 = GO 带回态**(现成字段);
    //   snoop 转移/应答表 = §1.2-B(落地在 M2')✓
    //   M1' 范围:仅**源数据读**(SLK_SRC)走查表 —— WQE/CQE 行照旧直读
    //   (⛔ 不把 CQE 可见性提前卷进来;CQE 行的缓存语义随 M2'/M3' 的 snoop 应答一起上)✗
    static const int CCH_MAX = 64;
    struct CchLine { bool valid; unsigned char st; sc_dt::uint64 tag; unsigned char data[64]; };
    CchLine cch[CCH_MAX];
    int  cch_n = 16, cch_victim = 0;
    bool cache_on = false;                        // 默认关 ⇒ 旧行为零影响 ✓
    unsigned int cch_hits = 0, cch_fills = 0;     // 观测(白盒直读;省 E109 改口)✓
    void set_cache_mode(bool on, int lines = 16) { cache_on = on; if (lines > 0 && lines <= CCH_MAX) cch_n = lines; }
    int  cch_find(sc_dt::uint64 line);            // 命中返回行号;未命中 −1 ✓
    int  cch_state(sc_dt::uint64 byte_addr);      // TB 查行态(0=I/未命中)✓
    unsigned int cch_wr_hit = 0;                  // ★ M2':写命中(就地更新 + M,不落内存)✓
    // ★ M3(超时/重传;机制照 `QP_TIMEOUT(0x4C)` 开卷字段;⚠ 阈值单位 = 模型拍,
    //   旋钮明标 —— RTL 计时器本版未开卷(只有字段/组合;`//Partial RD_REQ_RETRY` 半成品))✓
    int  line_tmo_ticks = 0;                      // 超时阈值(拍;0 = 关 ⇒ 老行为"一直等")✓
    void set_line_timeout(int ticks) { line_tmo_ticks = ticks; }
    unsigned int line_retries = 0;                // 超时重发总次数(观测)✓
    unsigned int line_rtry_exh = 0;               // 重试耗尽 ⇒ 异常终结计数 ✓
    // 行态常量(照 `RTL 源`)✓
    enum { CCH_I = 0, CCH_S = 1, CCH_E = 2, CCH_M = 3, CCH_F = 4, CCH_D = 5 };
    // ★ snoop 状态化应答(照 `规格文档` §1.2-B;默认关 ⇒ 旧口径 I_HIT_I)✓
    int  snp_op3 = 0;              // 受理时的 snoop 类型(1=DATA/2=INV/3=CUR;照 D12 口径)✓
    bool snp_xfer_done = false;    // 应答码/行态迁移已算(单次;防每拍重算)✓
    bool snp_rdata = false;        // 本应答带数据 ✓
    unsigned char snp_rop = cxlshim::D2H_RSP_SPEC_I_HIT_I;   // 应答码(5b 编码)✓
    bool snp_data_v = false;       // 本拍驱动行数据(与槽活动互斥)✓
    unsigned char snp_data_buf[64];
    unsigned int cch_snp_hit = 0;  // snoop 命中行数(观测)✓

    // ---------------- 观测(判据用)----------------
    sc_out<sc_uint<32>> o_db_sq_cnt;   // SQ_PI_DB 门铃写入次数
    sc_out<sc_uint<32>> o_db_rq_cnt;   // RQ_CI_DB 门铃写入次数
    sc_out<sc_uint<32>> o_ro_wr_cnt;   // 对 RO 寄存器的写(应被忽略;正控用)
    sc_out<sc_uint<32>> o_wqe_cnt;     // 已取 WQE 数
    sc_out<sc_uint<32>> o_cqe_cnt;     // 已写 CQE 数
    sc_out<sc_uint<32>> o_db_drop_cnt; // 门铃丢弃数(满时;恒 0 = 正常)✓
    // ★ v1.0 多笔在飞判据(高水位;>1 = 真多笔)✓
    sc_out<sc_uint<32>> o_max_rd_ostd;   // 读槽在飞高水位(含 WQE/CQE 行读;>1 = 多笔在飞)✓
    sc_out<sc_uint<32>> o_max_act_per_qp;// 同 QP 在飞 job 高水位(**须 = 1** = 保序)✓
    sc_out<sc_uint<32>> o_goerr_cnt;     // GO-Err / 无主回程计数(不许静默)✓
    // ★ v1.0 RQ 环 / 简化 opcode 计数(见 P2 规格 §4.4)✓
    sc_out<sc_uint<32>> o_rnr_cnt;       // RQ 满 ⇒ RNR-NAK 次数(每消息一次)✓
    sc_out<sc_uint<32>> o_opc_simpl_cnt; // 按基类搬数据但侧带未建模的笔数(0x01/0x03 IMMDT)✓
    // ★ v1.0 snoop 顺序语义(照 RTL"对在飞地址先等完成")✓
    sc_out<sc_uint<32>> o_snp_held_cnt;  // 因**在飞地址匹配**被压住过的 snoop 数(每笔记一次)✓
    // ★ v1.0 QP fatal(照 RTL `ILL_OPCODE` ⇒ `o_wqe_fatal_qpid`;fatal ⇒ 中断 **bit7**)✓
    sc_out<sc_uint<32>> o_qp_fatal_cnt;  // fatal 事件数(置 fatal 1 次 + 其后该 QP 的门铃丢弃各 1 次)✓
    sc_out<sc_uint<32>> o_mac_tx_frames; // MAC 面向:已发帧数(透传)✓
    sc_out<sc_uint<32>> o_mac_rx_frames; // MAC 面向:已收帧数 ✓
    // ★ v1.0 线侧真包(判据用)✓
    sc_out<sc_uint<32>> o_line_tx_pkts;   // 线侧:引擎已发请求包数 ✓
    sc_out<sc_uint<32>> o_line_rx_acks;   // 线侧:已匹配(消费)的对端 ACK 数 ✓
    sc_out<sc_uint<32>> o_line_unexp_cnt; // 线侧:非预期帧(非 ACK/不匹配;不静默)✗
    sc_out<sc_uint<32>> o_line_big_cnt;   // 线侧:超 M1 单包上限被拒的 WQE 数 ✗

    // ---------------- 内部:寄存器存储(照 §8 三段)----------------
    // ★ D10 已对齐(2026-10-04):**NQP = 32**(照 RTL `C_NUM_QP=32`;地址索引 `addr[15:8]` ⇒ 0..31)✓
    static constexpr int NPD = 8, NQP = 32, NGF = 512;
    sc_uint<32> m_pd[NPD][8];          // PD 段:块内 off 0x00..0x1C → [off>>2]
    sc_uint<32> m_qp[NQP][64];         // per-QP 段:块内 off 0x00..0xFC → [off>>2]
    sc_uint<32> m_gf[NGF];             // 全局段:off[10:2](每 0x800 别名 ✓)
    sc_uint<9>  m_intr_sts;            // INTR_STS(0x184)9 位(W1C;**bit4 例外 = per-QP 汇聚**,见 m_cq_intr)✓
    sc_uint<32> m_cq_intr;             // ★ CQ 完成中断(bank0 = QP0..31;INTR_STS[4] = 其 OR;
                                       //   清 = 写 `CQ_INTR_STS1`(mask 1=清对应 QP;照 RTL→)✓
    sc_uint<9>  m_intr_en;             // INTR_EN(0x180)9 位 ✓
    sc_uint<32> db_sq_cnt, db_rq_cnt, ro_wr_cnt;

    // ---------------- 方法 ----------------
    SC_HAS_PROCESS(DeviceTlm);
    DeviceTlm(sc_core::sc_module_name nm);

    // 寄存器访问(返回 false = 地址未实现/非法访问)
    bool reg_read (sc_dt::uint64 addr, sc_uint<32> & data);
    bool reg_write(sc_dt::uint64 addr, const sc_uint<32> & data);

    // ★ 白盒钩子(真源 = 队列引擎/事件逻辑)⚠
    void set_intr_status(int bit, bool v);          // 置/清中断状态位(0..8)
    void set_rq_pi(int qp, sc_uint<16> v);          // 更新 STAT_RQ_PI_DB(读回用)✓

    // 中断聚合(每拍重算;**只写 nic_intr 与 o_***,设备侧引脚归引擎线程 ✓)
    void intr_upd();

    //========================================================================
    // v1.0 引擎(★ **唯一驱动设备侧 CXL.cache 引脚者**;每拍一步的非阻塞 FSM)✓
    // 纪律(§节气):引脚每条线 = 1 拍 ⇒ 交付判据 = 「我上一拍在线上输出的值」∧「对方这一拍读到的值」✓
    //========================================================================
    // ---- 事务槽表(照 `RTL 源`:读槽 0..63 / 写槽 64..127;cqid = 零扩槽号)----
    static constexpr int RD_SLOT_CNT = 64, WR_SLOT_CNT = 64, NSLOT = RD_SLOT_CNT + WR_SLOT_CNT;
    enum { SLK_WQE = 0, SLK_SRC, SLK_CQE, SLK_WBK };  // 读槽用途 / SLK_WBK = 一次性回写(M3)✓
    struct Slot {
        bool in_use, is_wr, done, err;
        int  job, li, kind;                          // 属主 job / job 内行号 / 用途
        sc_dt::uint64 line;                          // 行地址(调试/判据)
        unsigned char go_state;                      // ★ 本槽 GO 带回的行态(填充用;缺省 S)✓
        unsigned char buf[cxlshim::LINE_BYTES];
    };
    Slot sl[NSLOT];
    int  rdf_q[RD_SLOT_CNT], rdf_h, rdf_t, rdf_n;     // 读槽空闲表(FIFO;初值 0..63)✓
    int  wrf_q[WR_SLOT_CNT], wrf_h, wrf_t, wrf_n;     // 写槽空闲表(FIFO;槽号 = 表值 + 64)✓
    int  wpq[NSLOT], wp_h, wp_t, wp_n;                // 写槽待发队列(分配序 = 服务序)✓

    // ---- 在飞 job 表(每 WQE 一份事务状态;照 `CXL_MAX_OUTSTANDING_REQ 4`)----
    static constexpr int NJOB = 4;
    static constexpr int RD_INFLIGHT_MAX = 8;         // 单 job 读在飞上限(防独占槽表)✓
    enum { J_FREE = 0, J_WQE, J_MOVE, J_CQE, J_DEAD };
    struct Job {
        bool act; int qp; sc_uint<16> pi; int ph;
        sc_uint<16> wrid; sc_uint<8> opcode;
        sc_dt::uint64 src, dst; sc_uint<32> len;
        int nlines, next_line, rd_inflight, fwd_cnt, wr_done, slots_owned;
        bool cqe_rd_issued;
        // ★ v1.0 SEND 专用(SEND 的目标 = **本 QP 的 RQ 项地址**,不是 WQE.remote_offset)✓
        bool is_send;                                 // opcode 0x02/0x03 ✓
        bool rq_ready;                                // RQ 项地址已锁定(不满 ⇒ 可搬)✓
        bool rnr_seen;                                // 本消息已记过 RNR(每消息计一次)✓
        sc_dt::uint64 cq_byte;                        // CQE 目标字节地址(进入 J_CQE 时定)✓
        sc_uint<32> seq;                              // 全局序(调试用)✓
        // ★ v1.0(/M2)线侧真包(line_mode 时开)✓
        enum { LINE_WR = 0, LINE_SEND = 1, LINE_RD = 2 };
        bool line_wr;                                 // 本 job 走线侧真包 ✓
        int  line_op;                                 // WR / SEND / RD ✓
        bool pkt_sent, ack_seen;                      // 包已上线路 / 成功响应已匹配(ACK 或 RD_RSP)✓
        int  rd_done;                                 // 线侧 WR/SEND:源行读完数 ✓
        int  wr_queued, paylen;                       // 线侧 RD:响应载荷已排队行数 / 载荷长 ✓
        int  rnr_retries;                             // 线侧:RNR-NAK 重发次数(封顶)✓
        int  tmo_cnt, retries;                        // ★ M3:超时计拍 / 超时重发次数 ✓
        bool wqe_retried;                             // ★ M3:retried 已回写 ✓
        unsigned char wqe_buf[64];                    // ★ M3:WQE 行留档(回写用)✓
        sc_dt::uint64 wqe_line;                       // ★ M3:WQE 行地址 ✓
        uint32_t dest_qpid, rkey;                     // WQE 字段(进包:DestQP / RETH.rkey)✓
        uint32_t psn_used;                            // 本包 PSN(= 发包时 QP_SQ_PSN 快照)✓
        unsigned char pbuf[LINE_PAY_MAX];             // 载荷缓冲(WR/SEND 源;RD 响应落地)✓
    };
    Job jobs[NJOB];
    sc_uint<32> job_seq, goerr_cnt, max_rd_ostd, max_act_per_qp;
    sc_uint<32> rnr_cnt, opc_simpl_cnt;               // ★ v1.0 ✓
    sc_uint<32> line_tx_pkts, line_rx_acks, line_unexp_cnt, line_big_cnt;   // ★ v1.0 ✓
    sc_uint<16> rq_wrptr[NQP];                        // ★ RQ 写指针(1..rq_depth→1;每消息推进)✓
    int rr_next;                                      // 读发射轮转(公平)✓

    // ---- 发射单元(单一;req0/req1/写数据 三通道互斥 ⇒ 桥侧不会同拍双 valid)⚠ ----
    enum { TX_IDLE = 0, TX_RREQ, TX_WREQ, TX_WDATA } tx;
    int tx_slot; sc_dt::uint64 tx_line;
    bool tx_reqv_prev, tx_datav_prev;                 // §节气:我上拍在线上输出的值 ✓
    bool h2r_prev, h2d_prev;                          // GO-ready / 数据-ready 的上拍值 ✓
    bool rsp_v_last, dat_v_last;                      // ★ 回程 valid **上拍读值**(上升沿守卫,见 .cpp)✓

    // ---- 门铃事件队列(reg_write 产 / 引擎消)✓ ----
    struct DbEv { int qp; sc_uint<16> pi; };
    static constexpr int DBDEPTH = 8;
    sc_uint<32> db_drop_cnt;                          // ★ 门铃**丢弃计数**(不许静默丢)✓
    DbEv db_q[DBDEPTH]; int db_h, db_t, db_n;

    sc_uint<16> cq_hdptr[NQP];                        // ★ **per-QP** CQE 写指针(1..depth;照 §9.2)✓
    bool        qp_fatal[NQP];                        // ★ 非法 opcode ⇒ 该 QP 判 fatal(照 RTL)✓
    sc_uint<32> qp_fatal_cnt;
    sc_uint<32> wqe_cnt, cqe_cnt;

    // snoop 应答子 FSM(设备恒回 RspI;★ **对在飞地址先等完成** —— 照 RTL)✓
    bool snp_act, snp_rdy_prev, snp_rspv_prev;
    sc_uint<12> snp_uqid;
    sc_dt::uint64 snp_line;                   // 受理的 snoop 地址(行)✓
    bool snp_hold_seen;                       // 本笔是否已记过"被压"(每笔记一次)✓
    sc_uint<32> snp_held_cnt;

    // ---- 引擎方法 ----
    void engine_thread();                      // SC_THREAD@clk.pos:每拍调 eng_step ✓
    void eng_reset();                          // 复位:清表 + 输出初值 ✓
    void eng_step();                           // 每拍一步(采样交付 → 消费 → 决策 → 输出)✓
    void snoop_step();                         // 每拍:服务 snoop(含**在飞地址顺序**)✓
    bool snoop_inflight_match();               // 在飞槽地址命中?(照 RTL snp_chk)✓
    uint32_t rq_depth(int qp) { return (qp_reg(qp, 0x3C) >> 16) & 0xFFFF; }        // Q_DEPTH[31:16] ✓
    bool rq_full (int qp);                     // 照 RTL 原式 ✓
    sc_dt::uint64 rq_entry_addr(int qp);       // 照 SQ 同款映射(见 §4.4-②)✓
    int  alloc_rd();                           // 读槽分配(空闲表 FIFO;−1 = 无)✓
    int  alloc_wr();                           // 写槽分配 ✓
    void free_slot(int s);                     // 归还槽(读/写各自空闲表)✓
    uint32_t qp_reg(int qp, int off) { return m_qp[qp][off >> 2].to_uint(); }   // 读 QP 寄存器 ✓
    // ★ M3:QP_TIMEOUT(0x4C)字段(照 `sw/nic_regs.h:72` + RTL 组合逐位对上)✓
    uint32_t qp_retry_max(int qp)     { return (qp_reg(qp, 0x4C) >> 8)  & 7; }  // [10:8] ✓
    uint32_t qp_rnr_retry_max(int qp) { return (qp_reg(qp, 0x4C) >> 11) & 7; }  // [13:11] ✓
    void wqe_writeback_retry(Job & J, int ji);    // 2.1j:WQE byte2 bit0 = retried(照 regs.h)✓
};

#endif
