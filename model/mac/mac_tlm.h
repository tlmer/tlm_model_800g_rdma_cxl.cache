//============================================================================
// mac_tlm.h — MAC TLM(客户 MAC ↔ 本 MAC TLM ↔ NIC TLM)  [2026-10-04 建 / v1.0]
//
// 规格 = `规格文档`(立项/两侧契约实读/实现规格 §7)✓
//   链路:客户 MAC ──LINK 2×512b── [本 MAC TLM] ──2048b AXIS(照 RTL)── NIC TLM
//
// ★ 两侧契约(逐条实读;出处见规格 §2/§3):
//   · NIC 侧(2048b AXIS;出处 = `RTL 源` 顶层 `nic` 端口表):
//     RX(NIC→MAC)= `cmac_m_axis_{tdata[2047:0],tkeep[255:0],tvalid,tlast}` + `tready`(本模块出)
//     TX(MAC→NIC)= `roce_cmac_s_axis_{tdata,tkeep,tvalid,tlast,tuser[0:0]}`;⚠ **无 ready**(恒收)
//   · 线侧 LINK(800G 模式 = 双 lane;出处 = `RTL 源` 已逐条复核):
//     TX:`link_txdval_0/1` · `link_txd_0/1[511:0]` · `link_txc_0/1[63:0]`(皆出)· `txclk_ena_0/1`(入)
//     RX:`link_rxd_0/1[511:0]` · `link_rxc_0/1[63:0]`(入)· `rxclk_ena_0/1`(入)
//     ⚠ clk/ena 原生由 **PCS 侧**给(两 MAC 直连 ⇒ 需 **glue**;样例在 TB,规格 §3/§7.2)✗
//   · `txc` 语义(规格 §3,出处 `notes/stage2_启动.md` §2.2):**0=数据字节 / 1=控制字符**;
//     帧首字节 = **Start 控制字(txc=1)**;帧间/空闲 = **txc 全 1(Idle)**;每字节 1 位 ✓
//
// ★ 行为(帧级功能子集;规格 §4/§7.3):TX = 收 AXIS 帧 → 前导+Start → 数据 → **FCS(CRC32)** →
//   Idle;RX = LINK 按 txc 识别 Start/Idle → 重组(±FCS 校验)→ 2048b AXIS 交付(roce 通道)
//   ⛔ 不做:PCS/66b/gearbox 内部、逐拍 CD、精确 FIFO 深度 ✓
// ★ 白盒观测(⛔ 只 TB 可调/可读;命名 `o_` 照惯例):帧计数 + Idle 拍计数 …… ✓
//============================================================================
#ifndef MAC_TLM_H
#define MAC_TLM_H

//============================================================================
// ★ 版本与冻结策略(同设备/桥:**冻结 = 客户首次集成前**)
//   变更记录:
//     (2026-10-05) ★ 与真 PCS RTL 对拍()修两条:
//       ① `o_fcs_err` 语义扩为 **RX 坏帧计数**(含 FCS 错 + 形状中断 + 异常控制字节)——
//          修前:START 值被打掉 ⇒ 帧静默丢弃、无计数 ⇒ 负控判据随注错落点浮动 ✗✓
//       ③ **RX `R_PAY` 遇非 TERM 控制字符 ⇒ 跳过**(802.3 线开销;修前一遇即 abort ⇒
//          帧收全却不交付 —— cosim 混合仿真实测)✗✓
//       ② **`txdval` 改「持续有效」**
//       (ena=1 ⇒ 恒 1,idle 列也带 valid;照主线 `RTL 源 铁证)——
//       原「仅帧期置 1」在真 PCS 全链环回**不通**(使能空洞);TLM 侧各 TB 判据不变 ✓
//     (2026-10-04) 骨架:两侧端口契约 + 帧级收发 + 自环 TB(依规格 §7 M1)✓
//     v1.0(rdma800 M2,2026-10-08) ★ 800G 档落地:
//       · 新增 **LANE_MODE**:`LM_800G`(默认;1024b/拍 = lane0 低半 + lane1 高半,双 lane 拼接)
//         / `LM_400G`(**512b/拍 仅 lane0 有效**;lane1 输出恒无效 —— 照 MAC 原生 400G 档)✓
//       · 新增**契约观测** `o_lane_mismatch_cnt`:`txclk_ena_0 != txclk_ena_1`(或 rx 侧同)的拍数
//         (契约G 档**双 lane 必须同值** ⇒ 不一致**计数不静默**)✓
//       ⚠ 默认 LM_800G ⇒ 既有 20 台 TB 行为**零变化**(回归护栏)✓
//============================================================================
#define MAC_TLM_VERSION_MAJOR 1
#define MAC_TLM_VERSION_MINOR 0
#define MAC_TLM_VERSION_STR   "mac_tlm 1.0 (2026-10-05)"

#include <systemc.h>

struct MacTlm: sc_core::sc_module {
    // ---------------- 时钟/复位(行为级单时钟;LINK clk/ena 角色 = glue,规格 §3)----------------
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    // ---------------- NIC 侧b AXIS(RoCE 主通道;non-roce 先留桩)----------------
    // RX(NIC→MAC;本模块为从侧:收帧)
    sc_in<bool>               cmac_m_axis_tvalid;
    sc_in<sc_biguint<2048>>   cmac_m_axis_tdata;
    sc_in<sc_biguint<256>>    cmac_m_axis_tkeep;
    sc_in<bool>               cmac_m_axis_tlast;
    sc_out<bool>              cmac_m_axis_tready;
    // TX(MAC→NIC;本模块为主侧:交帧;⚠ 无 ready —— 对面恒收)
    sc_out<bool>              roce_cmac_s_axis_tvalid;
    sc_out<sc_biguint<2048>>  roce_cmac_s_axis_tdata;
    sc_out<sc_biguint<256>>   roce_cmac_s_axis_tkeep;
    sc_out<bool>              roce_cmac_s_axis_tlast;
    sc_out<bool>              roce_cmac_s_axis_tuser;

    // ---------------- 线侧:LINK(800G 模式 = 双 lane;400G 模式只用 lane0)----------------
    // TX(MAC→客户/PCS)
    sc_out<bool>              link_txdval_0,  link_txdval_1;
    sc_out<sc_biguint<512>>   link_txd_0,     link_txd_1;
    sc_out<sc_biguint<64>>    link_txc_0,     link_txc_1;
    sc_in<bool>               link_txclk_ena_0, link_txclk_ena_1;   // PCS/glue 侧给(行为级=节流)
    // RX(客户/PCS→MAC)
    sc_in<sc_biguint<512>>    link_rxd_0,     link_rxd_1;
    sc_in<sc_biguint<64>>     link_rxc_0,     link_rxc_1;
    sc_in<bool>               link_rxclk_ena_0, link_rxclk_ena_1;

    // ---------------- 白盒观测(⛔ 只 TB)----------------
    sc_out<sc_uint<32>> o_frames_tx;      // 已发帧数(线侧)
    sc_out<sc_uint<32>> o_frames_rx;      // 已收帧数(线侧→NIC)
    sc_out<sc_uint<32>> o_idle_beats;     // Idle 拍数(txc 全 1)
    sc_out<sc_uint<32>> o_fcs_err;        // RX **坏帧计数**(FCS 错 + 形状中断/异常控制字节;负控判据用)✓
    sc_out<sc_uint<32>> o_lane_mismatch_cnt; // ★ v1.0:双 lane **契约违例拍数**(ena_0 != ena_1,含 rx)✓

    // ---------------- 构造/运行(实现见 mac_tlm.cpp;照「节气纪律」:SC_METHOD@clk.pos)----------------
    SC_HAS_PROCESS(MacTlm);
    MacTlm(sc_core::sc_module_name nm);
    void mac_step();                       // 每拍一步:线侧收/发 + NIC 侧交/收 + 观测

    // ---------------- ★ 线侧字相位(对拍;默认 1 = 关)----------------
    //   真 PCS 的 LINK 是「2 拍 = 1 个 2048b 字」(link1024_adapt 自注:低 1024b 先、高 1024b 后),
    //   主线 MAC 模型**只从字起始拍(偶拍)起帧**(`RTL 源` `tx_beat_cnt%2==0`)⇒
    //   混合同仿置 **2** 对齐该相位;TLM 侧各 TB 保持默认 1(行为不变)✓
    int line_phase = 1;
    unsigned long ena_beat_cnt = 0;
    void set_line_phase(int n) { line_phase = (n < 1) ? 1 : n; }

    // ---------------- ★ v1.0 线侧 lane 档位(rdma800 M2)----------------
    //   LM_800G(默认)= 1024b/拍:lane0 = 流字节 0..63、lane1 = 64..127(双 lane 拼接;
    //                 契约:双 lane 的 dval/ena **必须同值**)✓
    //   LM_400G      = **512b/拍 仅 lane0 有效**:lane1 输出恒无效(dval_1=0、数据 0),
    //                 RX 不看 lane1(MAC 原生 400G 档)✓
    enum LaneMode { LM_800G = 0, LM_400G = 1 };
    int lane_mode = LM_800G;
    void set_lane_mode(int m) { lane_mode = (m == LM_400G) ? LM_400G : LM_800G; }

    // ---------------- 规模常量 ----------------
    static const int MAX_FRAME   = 4096;   // 帧缓冲上限(功能级;TB/MTU 量级够用)
    static const int AXIS_BYTES  = 256;    // 2048b = 256B/拍
    static const int LINK_BYTES = 128;    // 1024b = 128B/拍(双 lane 各 64B)

    // ---------------- 内部状态(实现细节;⛔ 非接口)----------------
    enum TxSt { T_IDLE, T_EMIT };
    enum RxSt { R_IDLE, R_PRE, R_PAY };
    TxSt tx_state; RxSt rx_state;
    unsigned char tx_buf[MAX_FRAME];   int tx_len, tx_pos;      // NIC 侧收帧
    unsigned char wire_buf[MAX_FRAME]; int wire_len;            // 线帧(含前导/FCS/TERM)
    unsigned char rx_buf[MAX_FRAME];   int rx_len, pre_n;       // 线侧收帧
    bool  d_busy; int dm_bytes, dm_beat_i;                      // AXIS 交付(AXIS 侧 FSM,与 LINK 侧分离)
    unsigned int f_tx, f_rx, f_idle, f_fcs, f_lane_mm;
    void rx_finish(); void dm_step();
};

#endif // MAC_TLM_H
