//============================================================================
// cxl2ace_bridge.h — cxl2ace 桥 TLM(v1.0)  [2026-10-04 建 / 同日升 / 同日续]
//
// 规格 = `规格文档`(§1.2 主机侧形态 / §8 口径标定+待核 / §9 实测)
//
// ★★ v1.0(配合设备侧 同版多笔在飞):
//   ① **真缺陷修复(静默丢写)**:原 ingress 在 IDLE 时 rdy0/rdy1 **同抬** ⇒ 同拍两个 valid
//      时读优先入队、**写被丢**(设备侧却已按 ready∧valid 判成交)✗ ⇒ rdy1 加 `!req0_valid` 门 ✓
//   ② **队列容量精确**:原 `((t+1)%N==h)` 判满 ⇒ N 深实容 N−1(与设备侧门铃同族缺陷 ✗)
//      ⇒ 改**计数式占用**,容量 = QDEPTH,不再少 1 ✓
//   ③ **观测口(多笔在飞/槽号口径的独立见证;读数在桥侧,不看设备自证)**：
//      `o_rd_ostd_max`(读在飞高水位:>1 = 真多笔在飞)/ `o_fst_rd_cqid` / `o_fst_wr_cqid`
//      / `o_rd_cqid_max` / `o_wr_cqid_min` —— 照 RTL 槽号口径(读 0..63 / 写 64..127)✓
//
// ★★ 本版相对上一版的两项:**snoop 通道打通** + **多 outstanding(请求队列)**
//   ① **snoop 通道**(v1.0 只留了占位口):ACE snoop(`b_snoop`)→ 设备侧 `H2D req`
//      → 设备 `D2H rsp` → **CRRESP 5 位**回填事务 ✓
//      · 设备侧 snoop opcode **照我们 shim 的实际解码**(`RTL 源`:
//        0=SNP_DATA / 1=SNP_INV / 2+=SNP_CUR)⚠ 与 `RTL 源`(1/2/3)
//        **不一致** ⇒ 以实际消费者(shim)为准,已记规格 §8.3 待核 ✗
//      · CRRESP 位语义 = 库注释(`amba_pv_ace_simple_probe.h):
//        [0]DataTransfer [1]Error [2]PassDirty [3]IsShared [4]WasUnique;
//        ⚠ 只有 **I_HIT_I(全 0)** 这一路被 RTL 实证("snoop 恒 RspI",`RTL 源`)
//        并被本 TB 激励;其余组合是**按 ACE 语义的表**(待开卷确认)✗
//   ② **多 outstanding**:设备侧请求入**队列**(深度 = `CXL_MAX_OUTSTANDING_REQ` 4,照 `RTL 源`)
//      ⇒ 设备可连续投递多笔(读/写各自保序完成);响应按 cqid 原样回带 ✓
//
// ★★ 线程结构(⚠ 关键设计,防死锁 —— 别改回单线程):
//   · `pin_thread`(SC_THREAD@clk.pos)= **唯一驱动设备侧引脚的人**,并**就地服务 snoop**
//   · `issuer_thread`(SC_THREAD)= 只做 ACE 事务(**阻塞 `b_transport` 只在这个线程**)
//   ⇒ 互连在"我们的 ACE 读还没回来"时发 snoop 也**不会死锁**:
//     snoop 由 pin 线程服务,不依赖 issuer ✓(单线程写法会死锁:issuer 等互连、互连等 snoop ✗)
//   · `b_snoop()`(在**调用者线程**里跑)= 只填槽 + `wait(事件)`,**不碰引脚** ✓
//
// ★ 实现纪律(同 v1.0,全篇适用):
//   (a) 引脚侧 = `sc_signal`,`SC_THREAD@clk.pos` ⇒ **每条线 = 1 拍**;
//       一次"交付"的判据 = **「我上一拍在线上输出的值」∧「对方这一拍读到的值」**(同一 slot 线对)✓
//   (b) 取数与判据**同一 slot**;⛔ **禁止拿"读回自己输出"当守卫** ✗
//   (c) 队列跨线程**只用成员变量 + sc_event**(SystemC 协作式调度,无真并发 ⇒ 安全 ✓);
//       ⛔ 但**任何引脚**只许 pin_thread 碰 ✗
//============================================================================
#ifndef CXL2ACE_BRIDGE_H
#define CXL2ACE_BRIDGE_H

//============================================================================
// ★ 版本与冻结策略(计划 1.11)⚠ `.h` 一改 = 客户侧重编
//   · **冻结时机** = 客户首次集成前;冻结后**任何接口变更** ⇒ MINOR 进位 + 下表加一行;
//     纯实现变更(不动 .h)不动版本 ✓
//   · 版本 = 桥 TLM 接口版本(与设备侧 `DEVICE_TLM_VERSION_*` 各自独立)✓
//   变更记录:
//     (2026-10-04) snoop opcode 编码订正为 databook 1/2/3(原 0/1/2 = 跟随 RTL shim 缺陷)✓
//     (2026-10-04) 修"静默丢写"+ 修"响应队列被覆盖";加多笔在飞观测口 ✓
//     (2026-10-04) snoop 通道 + 多 outstanding(请求队列)✓
//     (2026-10-04) 主机侧换 AMBA-PV ACE + 512b 整行 ✓
//     (2026-10-04) 数据通路修复(§节气纪律)✓
//     (2026-10-04) 骨架 ✓
//============================================================================
#define CXL2ACE_BRIDGE_VERSION_MAJOR 1
#define CXL2ACE_BRIDGE_VERSION_MINOR 0
#define CXL2ACE_BRIDGE_VERSION_STR   "cxl2ace_bridge 1.0 (2026-10-04)"

#include <systemc.h>
#include <amba_pv.h>
#include "cxl_shim_types.h"

// ---- ACE 侧常量(★ 双信源:AMBA-PV `amba_pv_control.h` + Xilinx `amba-ace.h`,MIT)----
static constexpr int ACE_BUSWIDTH = 512;                                  // 一拍一整行 ✓
static constexpr unsigned char ACE_SNOOP_READ_SHARED  = amba_pv::AMBA_PV_READ_SHARED;   // 0x1
static constexpr unsigned char ACE_SNOOP_READ_UNIQUE  = amba_pv::AMBA_PV_READ_UNIQUE;   // 0x7
static constexpr unsigned char ACE_SNOOP_WRITE_UNIQUE = amba_pv::AMBA_PV_WRITE_UNIQUE;  // 0x0

struct Cxl2AceBridge: sc_core::sc_module, amba_pv::amba_pv_ace_master_base {

    // ---------------- 时钟/复位(设备侧 pin 级需要)----------------
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    // ---------------- 主机侧:AMBA-PV ACE master(512b)----------------
    amba_pv::amba_pv_ace_master_socket<ACE_BUSWIDTH> ace_m;

    // ---------------- 设备侧:CXL.cache pin 级(位宽照 `RTL 源`)----------------
    // D2H Req0(读:RD_SHARED)/ Req1(写:MemWr/WrCur = 0 0111b)—— addr = **行地址[45:0]** ✓
    sc_in<bool>         d2h_req0_valid; sc_in<sc_uint<5>>  d2h_req0_opcode;
    sc_in<sc_uint<12>>  d2h_req0_cqid;  sc_in<sc_uint<46>> d2h_req0_addr;  sc_out<bool> d2h_req0_ready;
    sc_in<bool>         d2h_req1_valid; sc_in<sc_uint<5>>  d2h_req1_opcode;
    sc_in<sc_uint<12>>  d2h_req1_cqid;  sc_in<sc_uint<46>> d2h_req1_addr;  sc_out<bool> d2h_req1_ready;
    // D2H Data(写数据):**512b 整行一拍** ✓
    sc_in<bool>         d2h_data_valid; sc_in<sc_uint<12>> d2h_data_uqid;
    sc_in<bool>         d2h_data_chunk_valid; sc_in<sc_biguint<512>> d2h_data_payload; sc_out<bool> d2h_data_ready;
    // ★ v1.0 新增:D2H Rsp(snoop 应答;Table 3-15:valid·opcode[4:0]·uqid[11:0])✓
    sc_in<bool>         d2h_rsp_valid;  sc_in<sc_uint<5>>  d2h_rsp_opcode;
    sc_in<sc_uint<12>>  d2h_rsp_uqid;   sc_out<bool>       d2h_rsp_ready;
    // H2D Rsp(GO/ExtCmp)/ H2D Data(读回,**512b 整行一拍**)✓
    sc_out<bool>        h2d_rsp_valid;  sc_out<sc_uint<4>>  h2d_rsp_opcode;
    sc_out<sc_uint<12>> h2d_rsp_rsp_data; sc_out<sc_uint<12>> h2d_rsp_cqid; sc_in<bool> h2d_rsp_ready;
    sc_out<bool>        h2d_data_valid; sc_out<sc_uint<12>> h2d_data_cqid;
    sc_out<bool>        h2d_data_chunk_valid; sc_out<sc_biguint<512>> h2d_data_payload; sc_in<bool> h2d_data_ready;
    // H2D Req(snoop;★ v1.0 **真驱动**:opcode 3b 照 shim 口径 / addr = 行地址 / uqid 关联)✓
    sc_out<bool>        h2d_req_valid;  sc_out<sc_uint<3>>  h2d_req_opcode;
    sc_out<sc_uint<46>> h2d_req_addr;   sc_out<sc_uint<12>> h2d_req_uqid; sc_in<bool> h2d_req_ready;

    // ---------------- 观测(判据用)----------------
    sc_out<sc_uint<32>> o_rd_done;    // 完成的读笔数
    sc_out<sc_uint<32>> o_wr_done;    // 完成的写笔数(写 = posted,issuer 完成即计 ✓)
    sc_out<sc_uint<32>> o_err;        // ACE 失败/协议错计数
    sc_out<sc_uint<32>> o_snoop_cnt;  // 收到的 snoop 数 ✓
    // ★ v1.0 多笔在飞 / 槽号口径的**独立见证**(读数在桥侧 = 不看设备自证)✓
    sc_out<sc_uint<32>> o_rd_ostd_max; // 读在飞高水位(= 已收读请求 − 已回读响应;>1 = 真多笔在飞)✓
    sc_out<sc_uint<12>> o_fst_rd_cqid; // 首个**读**请求 cqid(照 RTL 槽号:**应 = 0**,空闲表 FIFO 初值)✓
    sc_out<sc_uint<12>> o_fst_wr_cqid; // 首个**写**请求 cqid(应 = **64** = RD_SLOT_CNT)✓
    sc_out<sc_uint<12>> o_rd_cqid_max; // 读请求 cqid 最大(应 ≤ 63 = 读槽区)✓
    sc_out<sc_uint<12>> o_wr_cqid_min; // 写请求 cqid 最小(应 ≥ 64 = 写槽区)✓

    // ---------------- 内部:跨线程队列(引脚 ↔ issuer)----------------
    // ⚠ 容量口径 = **计数式**(QDEPTH 深就是 QDEPTH 深;不再 `(t+1)%N==h` 少 1)✗✓
    static constexpr int QDEPTH = 4;          // 照 `CXL_MAX_OUTSTANDING_REQ 4` ✓
    struct ReqRec {
        sc_uint<5>  op; sc_uint<46> addr; sc_uint<12> cqid; bool wr;
        sc_biguint<512> line;                 // 写数据(整行)✓
    };
    struct RspRec { sc_uint<12> cqid; bool err; sc_biguint<512> line; };

    ReqRec m_qreq[QDEPTH]; int m_qh, m_qt, m_qn;   // 引脚线程产 / issuer 消 ✓
    RspRec m_qrsp[QDEPTH]; int m_sh, m_st, m_sn;   // issuer 产 / 引脚线程消 ✓
    // ★★ **读在飞硬上限**(= 响应队列容量;⚠ 少了它必溢出,见 .cpp issuer 注释):
    //   在飞读 = m_qreq 里的 + **issuer 手里正在做的那一笔** ⇒ 若只按请求队列回压,
    //   响应数可到 QDEPTH+1 ⇒ 覆盖 m_qrsp 未发条目(实测:3/4/5/6 的响应被吞、7/8/9/10 发两遍)✗✓
    //   ★ 正确不变式 = **「未回响应的读」+「已排进 m_qrsp 未发出的响应」≤ QDEPTH**:
    //     入口按 (m_rd_ostd + m_sn) 回压 ⇒ issuer 推响应时 m_sn 必 < QDEPTH ✓
    int m_rd_ostd;                                 // 已收、尚未回响应(未入 rsp 队)的读笔数 ✓
    sc_core::sc_event m_ev_req, m_ev_rsp;

    // ---------------- 观测累计(引脚线程更新)----------------
    sc_uint<32> rd_acc, ostd_max_cnt;          // 已收读请求数 / 在飞高水位
    sc_uint<12> fst_rd, fst_wr, rd_cmax, wr_cmin;
    bool        fst_rd_seen, fst_wr_seen;

    // ---------------- 内部:snoop 服务(判定"全 0 = I_HIT_I"的单一路径)----------------
    bool        m_snp_busy, m_snp_done;       // b_snoop 填 busy;引脚线程置 done
    sc_uint<3>  m_snp_op3;                    // 设备侧 snoop opcode(**照 shim 口径** ⚠)
    sc_uint<46> m_snp_addr;                   // 行地址
    sc_uint<12> m_snp_uqid;                   // 本桥自造关联号(设备 d2h_rsp 要回声 ✓)
    sc_uint<5>  m_snp_d2h_op;                 // 设备回的 D2H rsp opcode(→ CRRESP)
    bool        m_snp_have_data;              // 设备回带数据?(v1.0 设备恒 RspI ⇒ 否)
    unsigned char m_snp_data[cxlshim::LINE_BYTES];
    sc_core::sc_event m_ev_snp_done;
    sc_uint<12> snp_uqid_gen;

    // ---------------- 引脚线程的每通道 prev(§(a) 纪律)----------------
    bool rdy0_prev, rdy1_prev, dready_prev, go_prev, hd_prev;
    bool rsp_ready_prev, snp_vprev;

    // ---------------- 引脚侧子 FSM(v1.0:三路并行,互不干扰)----------------
    enum { ING_IDLE, ING_WDATA } ing;         // 设备请求 → 队列
    ReqRec ing_rec;
    enum { EGR_IDLE, EGR_GO, EGR_DATA } egr;  // 队列 → 设备(读回程)
    RspRec egr_rec;
    enum { SNP_IDLE, SNP_REQ, SNP_RSP } snp;  // snoop 服务(H2D req / D2H rsp)

    // ---------------- 计数器(各线程更新,引脚线程写输出)----------------
    sc_uint<32> rd_done, wr_done, err_cnt, snoop_cnt;

    // ---------------- ACE 事务资源(仅 issuer 线程使用)----------------
    amba_pv::amba_pv_trans_pool m_pool;
    unsigned char               m_iobuf[cxlshim::LINE_BYTES];

    // ---------------- 方法 ----------------
    SC_HAS_PROCESS(Cxl2AceBridge);
    Cxl2AceBridge(sc_core::sc_module_name nm);
    void pin_thread();                        // ★ 唯一碰引脚者(含 snoop 服务)✓
    void issuer_thread();                     // ★ 唯一做阻塞 ACE 事务者 ✓
    void b_snoop(int, amba_pv::amba_pv_transaction &, sc_core::sc_time &) override;
    bool ace_read_line (const sc_uint<46> & a, unsigned char * buf);   // 返回 err
    void ace_write_line(const sc_uint<46> & a, const unsigned char * buf);
};

#endif
