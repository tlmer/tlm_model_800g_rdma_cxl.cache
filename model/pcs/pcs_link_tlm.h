//============================================================================
// pcs_link_tlm.h — PCS/FEC **link 模型**(非位级转写)  [2026-10-04 建 / v1.0]
//
// 规格 = `规格文档`(**照方案册 `tlm_model.md` §3.1 定论**:PCS「不转」,
//   等价物 = link 模型:通道速率 + 标称延迟 + 误码注入 + lock/align 状态门面位)✓
//
// ★ MAC 侧契约(逐条实读;出处 = `RTL 源`,`PCS 视角`方向):
//   · PCS→MAC:`link_rxd_0/1[511:0]` · `link_rxc_0/1[63:0]` · `link_rxclk_ena_0/1`
//             + `link_txclk_ena_0/1`(**RTL 注释原话:「本拍可收列」= MAC 依它驱动 txdval**)✓
//   · MAC→PCS:`link_txd_0/1[511:0]` · `link_txc_0/1[63:0]` · `link_txdval_0/1`
//   · ⚠ **免 glue 直连**:本口与 `mac_tlm.h` 的线侧**输入输出恰好对偶**(`rxdval` 不进 MAC 侧契约,
//     实件 `RTL 源 头注明写)⇒ 两 TLM 同信号直绑即可 ✓
//   · txc 语义同 规格文档 §3(0=数据/1=控制/空闲全 1)✓
//
// ★ 线侧(验门面,不碰 serdes):`fec_rx_align_status` / `fec_rx_lane_locked[15:0]`
//   —— 照 RTL 端口语义(`RTL 源`);⛔ TLM **不呈现** 16×80b serdes(`:52-53`,位级不在本模型)✗
//
// ★ 行为(M1):MAC 发来的列(txdval=1 拍)进**延迟 FIFO**(标称延迟旋钮)⇒ **环回**发出(线对端 = 本模型);
//   FIFO 空 ⇒ 发 **idle 拍**(字节全 0x07 + txc 全 1,照 TB 定义);误码注入 = 逐字节按 BER 翻位;
//   `txclk_ena/rxclk_ena` 行为级恒 1(0.98 节奏以节流旋钮表示,M1 不细分)✓
//   ⛔ 不做:64b66b 编解码 / 扰码 / AM / gearbox / transcode / RS-FEC 内部(全位级;方案册 §3.1)✗
//============================================================================
#ifndef PCS_LINK_TLM_H
#define PCS_LINK_TLM_H

//============================================================================
// ★ 版本与冻结策略(同全线:**冻结 = 客户首次集成前**)
//   变更记录:
//     (2026-10-04) 骨架:LINK 契约 + 延迟/误码旋钮 + 环回 + lock 门面位 ✓
//============================================================================
#define PCS_LINK_TLM_VERSION_MAJOR 1
#define PCS_LINK_TLM_VERSION_MINOR 0
#define PCS_LINK_TLM_VERSION_STR   "pcs_link_tlm 1.0 (2026-10-04)"

#include <systemc.h>
#include <deque>

struct PcsLinkTlm: sc_core::sc_module {
    // ---------------- 时钟/复位(行为级单时钟;LINK clk/ena 角色见规格 §3)----------------
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    // ---------------- MAC 侧(方向 = PCS 视角;照 `RTL 源`)----------------
    sc_out<sc_biguint<512>> link_rxd_0,  link_rxd_1;
    sc_out<sc_biguint<64>>  link_rxc_0,  link_rxc_1;
    sc_out<bool> link_rxclk_ena_0, link_rxclk_ena_1;
    sc_out<bool> link_txclk_ena_0, link_txclk_ena_1;    // ★「本拍可收列」(MAC 依它驱动 txdval)✓
    sc_in<sc_biguint<512>>  link_txd_0,  link_txd_1;
    sc_in<sc_biguint<64>>   link_txc_0,  link_txc_1;
    sc_in<bool> link_txdval_0, link_txdval_1;

    // ---------------- 线侧门面(照 RTL 端口语义;TLM = 状态位)----------------
    sc_out<bool>        fec_rx_align_status;
    sc_out<sc_uint<16>> fec_rx_lane_locked;

    // ---------------- 旋钮(SE 直调;⛔ 非接口面)----------------
    void set_latency(int beats);      // 标称延迟(拍;0 = 直通)✓
    void set_ber(double p);           // 误码率(逐**位**翻转概率;0 = 干净)✓
    void set_loopback(bool on);       // 线侧环回(M1 默认开;**off ⇒ 只吞不发**,模拟断线)✓
    void set_seed(unsigned s);        // 误码 RNG 种子(可复现)✓

    // ---------------- 观测(判据用)----------------
    sc_out<sc_uint<32>> o_beats_in;    // 收到 MAC 的列数(txdval 拍)✓
    sc_out<sc_uint<32>> o_beats_out;   // 向 MAC 发出的列数(含 idle)✓
    sc_out<sc_uint<32>> o_bit_errs;    // 注入的位翻转数(BER 见证)✓

    SC_HAS_PROCESS(PcsLinkTlm);
    PcsLinkTlm(sc_core::sc_module_name nm);
    void link_step();                  // 每拍一步(照「节气纪律」)✓

    // ---------------- 内部态 ----------------
    struct Beat { sc_biguint<512> d0, d1; sc_biguint<64> c0, c1; };
    struct Pend { Beat b; int wait; };        // ★ 逐列倒计时(延迟旋钮)
                                              //   ⚠ 不能用"size()>lat":单列帧(1 拍铺完)**永远**不满足 ⇒ 卡死 ✗✓
    std::deque<Pend> fifo;
    int      lat = 0;                  // 延迟拍数
    double   ber = 0.0;                // 误码率
    bool     lb  = true;               // 环回
    unsigned rng = 0x12345678u;        // xorshift32(种子可设)✓
    unsigned b_in = 0, b_out = 0, b_err = 0;
    unsigned xrand();
    int      inject(unsigned char* buf);   // 对 128 字节流按 BER 翻位;返回翻位数
};

#endif // PCS_LINK_TLM_H
