//============================================================================
// mac_tlm.cpp — MacTlm 实现(v1.0:帧级收发 + 自环;规格 = 规格文档 §7)✓
//
// ★ 线帧格式与字节序(全部实读,出处 = `RTL 源`):
//   · 常量(:44-45):IDLE=0x07 · START=0xFB · TERM=0xFD · SFD=0xD5 · PRE=0x55 ✓
//   · 帧格式(:50-53 写侧 ~410 判据 FSM 逐字对照):
//       START(ctrl) + 6×PRE(data) + SFD(data) + payload(data…) + [FCS 4B(data)] + TERM(ctrl)
//     ⚠ FCS = 以太网 CRC32(反射;LSB 先出)= 实件 MAC 职责;本 TLM 默认生成/校验 ✓
//   · idle 拍 = **字节全 0x07 且 txc 全 1** 判据原话)✓
//   · 字内字节映射):**默认 REV 序**(vendor 原生,`posn(i)=(i/8)*8+(7-(i%8))`);
//     `+define MAC_TLM_STD_BYTE_ORDER` 切标准序(XGMII 档;⚠ 与对端必须同档)✗
//   · 1024b/拍 = lane0(流字节 0..63)+ lane1(流字节 64..127);i<64→lane0、i≥64→lane1 ✓
//
// ★ 行为(帧级;照「节气纪律」:SC_METHOD@clk.pos,每拍一步):
//   TX:NIC AXIS(2048b/拍,tkeep 逐字节)收帧 ⇒ 组线帧(前导+SFD+FCS+TERM)⇒ LINK 字流;
//      空闲/帧间 = idle 拍;`txclk_ena`=0 ⇒ 本拍冻结(行为级节流)✓
//   RX:LINK 字流按 txc 识别(START/TERM/idle)⇒ 重组 + FCS 校验 ⇒ 剥 FCS ⇒ AXIS 2048b 流式交付
//      (roce 通道;**无 ready ⇒ 恒收**;交付 FSM 与 LINK 侧 FSM **分离**)✓
//   ⛔ 明写简化:帧自**字边界**起始(实件可任意字节对齐 ⇒ PCS 域细节,本 link 不建)✗
//============================================================================
#include "mac_tlm.h"

namespace {

// ---- 线帧常量(RTL 源 实读)----
const unsigned char C_IDLE  = 0x07;
const unsigned char C_START = 0xFB;
const unsigned char C_TERM  = 0xFD;
const unsigned char C_SFD   = 0xD5;
const unsigned char C_PRE   = 0x55;

// ---- 字内字节映射(默认 REV;切档见文件头)----
#ifdef MAC_TLM_STD_BYTE_ORDER
inline int posn(int i) { return (i/8)*8 + (i    %8); }   // 标准序(XGMII 档)
#else
inline int posn(int i) { return (i/8)*8 + (7-(i%8)); }   // ★ REV 序(vendor 原生,默认)
#endif

// ---- 以太网 FCS:CRC32(反射,初值/终值取反;线上 LSB 先出)----
inline unsigned int eth_crc32(const unsigned char* d, int n) {
    unsigned int c = 0xFFFFFFFFu;
    for (int i = 0; i < n; i++) {
        c ^= (unsigned int)d[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return c ^ 0xFFFFFFFFu;
}

} // namespace

MacTlm::MacTlm(sc_core::sc_module_name nm) : sc_module(nm) {
    SC_METHOD(mac_step);
    sensitive << clk.pos();
    dont_initialize();

    tx_state = T_IDLE; rx_state = R_IDLE;
    tx_len = tx_pos = wire_len = rx_len = pre_n = 0;
    d_busy = false; dm_bytes = dm_beat_i = 0;
    f_tx = f_rx = f_idle = f_fcs = 0; f_lane_mm = 0;
    // ⚠ 构造期**不写端口**(端口未绑定 ⇒ E112 实踩:第一个 write=tready=port_6 ✗✓);
    //   端口初值全在 mac_step 的复位分支里给(TB 复位期即写入)✓
}

void MacTlm::mac_step() {
    if (!rst_n.read()) {
        tx_state = T_IDLE; rx_state = R_IDLE;
        tx_len = tx_pos = wire_len = rx_len = pre_n = 0;
        d_busy = false; dm_bytes = dm_beat_i = 0;
        f_tx = f_rx = f_idle = f_fcs = 0;
        cmac_m_axis_tready.write(true);
        roce_cmac_s_axis_tvalid.write(false); roce_cmac_s_axis_tlast.write(false); roce_cmac_s_axis_tuser.write(false);
        roce_cmac_s_axis_tdata.write(0); roce_cmac_s_axis_tkeep.write(0);
        link_txdval_0.write(false); link_txdval_1.write(false);
        link_txd_0.write(0); link_txd_1.write(0); link_txc_0.write(0); link_txc_1.write(0);
        o_frames_tx.write(0); o_frames_rx.write(0); o_idle_beats.write(0); o_fcs_err.write(0);
        o_lane_mismatch_cnt.write(0);
        return;
    }

    //====================================================================
    // ① NIC→MAC:AXIS 收帧(2048b/拍;tkeep 逐字节;tready 恒持)
    //====================================================================
    if (tx_state == T_IDLE) {
        cmac_m_axis_tready.write(true);
        if (cmac_m_axis_tvalid.read()) {
            sc_biguint<2048> d = cmac_m_axis_tdata.read();
            sc_biguint<256>  k = cmac_m_axis_tkeep.read();
            for (int j = 0; j < AXIS_BYTES; j++) {
                if (k[j].to_bool() && tx_len < MAX_FRAME)
                    tx_buf[tx_len++] = (unsigned char)d.range(j*8+7, j*8).to_uint();
            }
            if (cmac_m_axis_tlast.read()) {
                // 组线帧:START + 6×PRE + SFD + payload + FCS + TERM(常量照 tb,FCS 标准以太网)
                unsigned int crc = eth_crc32(tx_buf, tx_len);
                int n = 0;
                wire_buf[n++] = C_START;
                for (int i = 0; i < 6; i++) wire_buf[n++] = C_PRE;
                wire_buf[n++] = C_SFD;
                for (int i = 0; i < tx_len && n < MAX_FRAME-8; i++) wire_buf[n++] = tx_buf[i];
                wire_buf[n++] = (unsigned char)(crc & 0xff);
                wire_buf[n++] = (unsigned char)((crc >> 8) & 0xff);
                wire_buf[n++] = (unsigned char)((crc >> 16) & 0xff);
                wire_buf[n++] = (unsigned char)((crc >> 24) & 0xff);
                wire_buf[n++] = C_TERM;
                wire_len = n;
                cmac_m_axis_tready.write(false);       // 发射期不收
                tx_state = T_EMIT; tx_pos = 0;
            }
        }
    } else {
        cmac_m_axis_tready.write(false);
    }

    //====================================================================
    // ② MAC→线:LINK 发射(1024b/拍;`txclk_ena`=0 ⇒ 本拍冻结)
    //====================================================================
    {
        bool ena0 = link_txclk_ena_0.read();
        bool ena1 = link_txclk_ena_1.read();
        // ★ v1.0G 档契约 = 双 lane 的 ena **必须同值**;不一致 ⇒ 计数(不静默)且按"两 lane 都放行"才动
        if (lane_mode == LM_800G && ena0 != ena1) { f_lane_mm++; o_lane_mismatch_cnt.write(f_lane_mm); }
        bool ena = (lane_mode == LM_800G) ? (ena0 && ena1) : ena0;
        const int NBYTES = (lane_mode == LM_800G) ? LINK_BYTES : 64;   // 400G 档 = 512b/拍(仅 lane0)✓
        if (ena) {
            sc_biguint<512> d0 = 0, d1 = 0; sc_biguint<64> c0 = 0, c1 = 0;
            bool fresh = (line_phase <= 1) || ((ena_beat_cnt % (unsigned long)line_phase) == 0);
            bool has_frame = (tx_state == T_EMIT);
            // ★ 相位门(line_phase>1):帧**起点**只在"字起始拍"落(照 RTL 源)✓
            if (has_frame && tx_pos == 0 && !fresh) has_frame = false;
            ena_beat_cnt++;
            for (int i = 0; i < NBYTES; i++) {
                unsigned char b = C_IDLE; bool c = true;                 // 默认 idle 填充
                if (has_frame && tx_pos < wire_len) {
                    int idx = tx_pos;                                     // ★ 控制位**按位置**标(仅首=START/末=TERM);
                    b = wire_buf[tx_pos];                                 //   按值法会把载荷中 0xFB/0xFD 误标 ✗✓
                    c = (idx == 0 || idx == wire_len - 1);
                    tx_pos++;
                }
                if (i < 64) { int p = posn(i);    d0.range(p*8+7, p*8) = b; c0[p] = c; }
                else        { int p = posn(i-64); d1.range(p*8+7, p*8) = b; c1[p] = c; }
            }
            if (lane_mode == LM_400G) { d1 = 0; c1 = 0; }        // ★ 400G 档:lane1 恒无效(数据 0)✓
            link_txd_0.write(d0); link_txd_1.write(d1);
            link_txc_0.write(c0); link_txc_1.write(c1);
            // ★ 2026-10-05(对拍修):txdval = **持续有效**(ena=1 ⇒ 恒 1;idle 列也带 valid)——
            //   照主线 RTL 源 `mac_txdval = link_rxclk_ena_0`  注释
            //   (「MAC 自猜节奏 ⇒ TX codec 使能带空洞」);原「仅帧期置 1」在真 PCS 环回上不通 ✗✓
            link_txdval_0.write(true);
            link_txdval_1.write(lane_mode == LM_800G);          // ★ 400G 档 lane1 恒不有效 ✓
            if (has_frame && tx_pos >= wire_len) {                       // 本帧已发完(含 TERM)
                tx_state = T_IDLE; tx_len = 0; tx_pos = 0; f_tx++; o_frames_tx.write(f_tx);
            }
        } else {
            ena_beat_cnt = 0;                          // ★ ena 洞 ⇒ 相位重新起算(照 TB beat_cnt 只在 ena 推进)✓
            link_txdval_0.write(false); link_txdval_1.write(false);
            link_txd_0.write(0); link_txd_1.write(0); link_txc_0.write(0); link_txc_1.write(0);
        }
    }

    //====================================================================
    // ③ 线→MAC:LINK 接收(rxclk_ena=1 拍处理;按 txc 识别 START/TERM/idle)
    //====================================================================
    {
    bool rx0 = link_rxclk_ena_0.read(), rx1 = link_rxclk_ena_1.read();
    if (lane_mode == LM_800G && rx0 != rx1) { f_lane_mm++; o_lane_mismatch_cnt.write(f_lane_mm); }
    if ((lane_mode == LM_800G) ? (rx0 && rx1) : rx0) {
        const int RBYTES = (lane_mode == LM_800G) ? LINK_BYTES : 64;    // 400G 档:只看 lane0 ✓
        sc_biguint<512> d0 = link_rxd_0.read(), d1 = link_rxd_1.read();
        sc_biguint<64>  c0 = link_rxc_0.read(), c1 = link_rxc_1.read();
        bool all_idle = true;                                            // 全 idle 拍快计 判据)
        for (int i = 0; i < RBYTES && all_idle; i++) {
            unsigned char b; bool c;
            if (i < 64) { int p = posn(i);    b = (unsigned char)d0.range(p*8+7, p*8).to_uint(); c = c0[p].to_bool(); }
            else        { int p = posn(i-64); b = (unsigned char)d1.range(p*8+7, p*8).to_uint(); c = c1[p].to_bool(); }
            if (!(b == C_IDLE && c)) all_idle = false;
        }
        if (all_idle) { f_idle++; o_idle_beats.write(f_idle); }
        for (int i = 0; i < RBYTES; i++) {
            unsigned char b; bool c;
            if (i < 64) { int p = posn(i);    b = (unsigned char)d0.range(p*8+7, p*8).to_uint(); c = c0[p].to_bool(); }
            else        { int p = posn(i-64); b = (unsigned char)d1.range(p*8+7, p*8).to_uint(); c = c1[p].to_bool(); }
            if (b == C_IDLE && c) continue;                              // idle 字节:各状态一律跳过
            switch (rx_state) {
            case R_IDLE:
                if (b == C_START && c) { rx_state = R_PRE; pre_n = 0; rx_len = 0; }
                // ★ 2026-10-05(对拍):异常控制字节(非 START)⇒ **坏帧计数**
                //   (线侧注错打掉 START 值 ⇒ 帧静默丢弃的路径,原先无见证 ⇒ 负控判据不稳)✗✓
                else if (c) { f_fcs++; o_fcs_err.write(f_fcs); }
                break;
            case R_PRE:
                if (pre_n < 6 && b == C_PRE && !c) pre_n++;
                else if (pre_n == 6 && b == C_SFD && !c) rx_state = R_PAY;
                else { f_fcs++; o_fcs_err.write(f_fcs); rx_state = R_IDLE; }   // 形状错 ⇒ 丢 + 计数
                break;
            case R_PAY:
                if (c) {
                    if (b == C_TERM) rx_finish();
                    // ★ 2026-10-05(对拍):**非 TERM 控制字符 ⇒ 跳过**(802.3 线开销/OS 码;
                    //   照 XGMII:帧 = S…T 之间,中插控制码由线侧/对端产生,不判死)——
                    //   修前:R_PAY 一遇非 TERM 控制码即 abort ⇒ 帧明明收全却**不交付** ✗✓
                    //   (cosim 实测:MAC 序探针见帧字节全对 + 多 7 个杂散控制码 ⇒ 收长 0)
                }
                else if (rx_len < MAX_FRAME) rx_buf[rx_len++] = b;
                break;
            }
        }
    }
    }   // ★ v1.0:外层块(ena 契约采样在块首)✓

    //====================================================================
    // ④ 交付步:上一帧校验通过 ⇒ 每拍交一个 2048b beat(流式;tlast 收尾)
    //====================================================================
    dm_step();
}

//----------------------------------------------------------------------------
// 收帧完成:FCS 校验(剥 4B)⇒ 起交付(⚠ 交付在飞时不收新帧 —— M1 单帧在飞即为足)
//----------------------------------------------------------------------------
void MacTlm::rx_finish() {
    rx_state = R_IDLE;
    if (d_busy) return;                                         // 交付在飞 ⇒ 丢本帧(注:TB 节奏下单帧)
    if (rx_len < 4) { f_fcs++; o_fcs_err.write(f_fcs); return; }
    int pay = rx_len - 4;
    unsigned int crc = eth_crc32(rx_buf, pay);
    unsigned int got = (unsigned int)rx_buf[pay] | ((unsigned int)rx_buf[pay+1] << 8)
                     | ((unsigned int)rx_buf[pay+2] << 16) | ((unsigned int)rx_buf[pay+3] << 24);
    if (crc != got) { f_fcs++; o_fcs_err.write(f_fcs); return; }
    d_busy = true; dm_bytes = pay; dm_beat_i = 0;               // 下一个 mac_step 的 ④ 起交
}

//----------------------------------------------------------------------------
// 交付步(每拍一次):组 2048b beat + tkeep;末拍 tlast;无 ready ⇒ 每拍推进
//----------------------------------------------------------------------------
void MacTlm::dm_step() {
    if (!d_busy) { roce_cmac_s_axis_tvalid.write(false); roce_cmac_s_axis_tlast.write(false); return; }
    sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
    int rem = dm_bytes - dm_beat_i * AXIS_BYTES;
    int nb  = (rem > AXIS_BYTES) ? AXIS_BYTES : rem;
    bool last = (dm_beat_i * AXIS_BYTES + nb >= dm_bytes);
    for (int j = 0; j < nb; j++) {
        d.range(j*8+7, j*8) = rx_buf[dm_beat_i*AXIS_BYTES + j];
        k[j] = true;
    }
    roce_cmac_s_axis_tdata.write(d); roce_cmac_s_axis_tkeep.write(k);
    roce_cmac_s_axis_tvalid.write(true); roce_cmac_s_axis_tlast.write(last);
    roce_cmac_s_axis_tuser.write(false);
    dm_beat_i++;
    if (last) { d_busy = false; f_rx++; o_frames_rx.write(f_rx); }
}
