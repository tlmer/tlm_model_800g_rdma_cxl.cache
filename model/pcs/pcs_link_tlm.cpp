//============================================================================
// pcs_link_tlm.cpp — PcsLinkTlm 实现(v1.0;规格 = 规格文档 §3)✓
//   行为(照方案册 §3.1 四条):① 速率 = 行为级(ena 恒 1;节流旋钮留 M2)② 标称延迟 = FIFO 拍数
//   ③ 误码注入 = 逐位翻转(BER)④ lock/align 门面位 = 常锁(M2 再给时序)✓
//   ⚠ 空闲拍 = 字节全 **0x07** + txc 全 1(照 RTL 源 原判据)✓
//   ⚠ 字节序:本模型**不解释**字节序(纯搬运列);两端(本模型 ↔ MAC TLM)自洽,REV/标准由 MAC 侧定 ✓
//============================================================================
#include "pcs_link_tlm.h"

namespace {
const unsigned char C_IDLE = 0x07;
const int LINK_BYTES = 128;          // 1024b = 128B/拍(双 lane 各 64B)✓
}

PcsLinkTlm::PcsLinkTlm(sc_core::sc_module_name nm): sc_module(nm) {
    SC_METHOD(link_step);
    sensitive << clk.pos();
    dont_initialize();
    // ⚠ 构造期**不写端口**(绑定前 ⇒ E112 纪律,同全线)✓;初值在 link_step 复位分支 ✓
}

unsigned PcsLinkTlm::xrand() {         // xorshift32(确定性,可复现)✓
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
}

// 对 128 字节流按 BER 翻位(逐位概率 = ber);返回翻位数 ✓
int PcsLinkTlm::inject(unsigned char* buf) {
    if (ber <= 0.0) return 0;
    int n = 0;
    for (int i = 0; i < LINK_BYTES; i++) {
        unsigned r = xrand();
        // 每字节 8 位:用 (r & 0xFFFFFF) / 2^24 < ber*8 近似"至少一位翻" ⇒ 简单且可复现
        if ((double)(r & 0xFFFFFFu) / 16777216.0 < ber * 8.0) {
            buf[i] ^= (unsigned char)(1u << (r >> 24 & 7));
            n++;
        }
    }
    return n;
}

void PcsLinkTlm::link_step() {
    if (!rst_n.read()) {
        fifo.clear(); b_in = b_out = b_err = 0;
        link_rxd_0.write(0); link_rxd_1.write(0);
        link_rxc_0.write(0); link_rxc_1.write(0);
        link_rxclk_ena_0.write(false); link_rxclk_ena_1.write(false);
        link_txclk_ena_0.write(false); link_txclk_ena_1.write(false);
        fec_rx_align_status.write(false); fec_rx_lane_locked.write(0);
        o_beats_in.write(0); o_beats_out.write(0); o_bit_errs.write(0);
        return;
    }

    // ---- ① 收 MAC 的列(txdval 拍 ⇒ 进延迟 FIFO;含误码注入)----
    // ⚠ 行为级:以 lane0 的 dval 为准(照契约G 模式双 lane 同值)✓
    if (link_txdval_0.read()) {
        Beat b;
        unsigned char buf[LINK_BYTES];
        sc_biguint<512> t0 = link_txd_0.read(), t1 = link_txd_1.read();
        sc_biguint<64>  c0 = link_txc_0.read(), c1 = link_txc_1.read();
        sc_biguint<512> n0 = t0, n1 = t1;
        sc_biguint<64>  m0 = c0, m1 = c1;
        for (int i = 0; i < 64; i++) {
            buf[i]      = (unsigned char)t0.range(i*8+7, i*8).to_uint();
            buf[64 + i] = (unsigned char)t1.range(i*8+7, i*8).to_uint();
        }
        int nerr = inject(buf);            // ★ 误码(注意:直接按**位下标**翻转,与映射无关)✓
        b_err += (unsigned)nerr;
        for (int i = 0; i < 64; i++) {
            n0.range(i*8+7, i*8) = buf[i];
            n1.range(i*8+7, i*8) = buf[64 + i];
        }
        b.d0 = n0; b.d1 = n1; b.c0 = m0; b.c1 = m1;
        Pend p; p.b = b; p.wait = lat; fifo.push_back(p);   // ★ 倒计时 = 标称延迟 ✓
        b_in++;
    }

    // ---- ② 出(倒计时到点才发;空/未到点 ⇒ idle 拍)----
    sc_biguint<512> rd0 = 0, rd1 = 0; sc_biguint<64> rc0 = 0, rc1 = 0;
    bool have = false;
    if (!fifo.empty()) {
        if (!lb) { fifo.clear(); }                       // 断线:只吞不发(队列不积压)✓
        else {
            if (fifo.front().wait > 0) fifo.front().wait--;   // ★ 每拍减一(到达标称延迟才发)✓
            if (fifo.front().wait <= 0) {
                Beat b = fifo.front().b; fifo.pop_front();
                rd0 = b.d0; rd1 = b.d1; rc0 = b.c0; rc1 = b.c1;
                have = true;
            }
        }
    }
    if (!have) {                           // idle 拍:全 0x07 + 全 1 控制 ✓
        for (int i = 0; i < 64; i++) {
            rd0.range(i*8+7, i*8) = (unsigned)C_IDLE;
            rd1.range(i*8+7, i*8) = (unsigned)C_IDLE;
            rc0[i] = true; rc1[i] = true;
        }
    } else {
        b_out++;                           // 只对**真列**计数(idle 不算)✓
    }
    link_rxd_0.write(rd0); link_rxd_1.write(rd1);
    link_rxc_0.write(rc0); link_rxc_1.write(rc1);

    // ---- ③ ena/门面位(行为级)----
    link_txclk_ena_0.write(true);  link_txclk_ena_1.write(true);    // 「本拍可收列」✓
    link_rxclk_ena_0.write(true);  link_rxclk_ena_1.write(true);    // RX 有效 ✓
    fec_rx_align_status.write(true);                                   // M1:常锁(M2 给过程)✓
    fec_rx_lane_locked.write(0xFFFF);                                  // 16 lane 全锁 ✓

    o_beats_in.write(b_in); o_beats_out.write(b_out); o_bit_errs.write(b_err);
}

void PcsLinkTlm::set_latency(int beats) { lat = (beats < 0) ? 0 : beats; }
void PcsLinkTlm::set_ber(double p)      { ber = (p < 0.0) ? 0.0 : p; }
void PcsLinkTlm::set_loopback(bool on)  { lb = on; }
void PcsLinkTlm::set_seed(unsigned s)   { rng = s ? s : 1u; }
