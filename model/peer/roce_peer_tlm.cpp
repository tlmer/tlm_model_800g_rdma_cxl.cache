//============================================================================
// roce_peer_tlm.cpp — 最小 RoCEv2 对端实现(v1.0;包配方照 基准 TB 原码移植)✓
//   ⚠ 包库(构造/解析/ICRC)已抽出 ⇒ `peer/roce_pkt.h`(**单一定义处**,设备引擎真包侧共用)✓
//============================================================================
#include "roce_peer_tlm.h"

//----------------------------------------------------------------------------
RocePeerTlm::RocePeerTlm(sc_core::sc_module_name nm): sc_module(nm) {
    SC_METHOD(peer_step);
    sensitive << clk.pos();
    dont_initialize();          // ⚠ 构造期不写端口(E112 纪律)✓
}

// PSN 偏置(负控用):改 BTH.PSN 后**重算 ICRC**(PSN 不在 IPv4 校验范围内)✓
void RocePeerTlm::patch_psn(unsigned char * p, int n, int bias) {
    uint32_t psn = ((uint32_t)p[51] << 16) | ((uint32_t)p[52] << 8) | p[53];
    psn = (psn + (uint32_t)bias) & 0xFFFFFFu;
    p[51] = (unsigned char)(psn >> 16); p[52] = (unsigned char)(psn >> 8); p[53] = (unsigned char)psn;
    uint32_t ic = rocepkt::icrc(p, n - 4);
    p[n-4] = (unsigned char)(ic >> 24); p[n-3] = (unsigned char)(ic >> 16);
    p[n-2] = (unsigned char)(ic >> 8);  p[n-1] = (unsigned char)ic;
}

bool RocePeerTlm::push_out(const unsigned char * f, int n) {
    if (tx_busy || n > PKT_MAX) return false;
    std::memcpy(tx_buf, f, n); tx_len = n; tx_off = 0; tx_busy = true;
    return true;
}

void RocePeerTlm::peer_step() {
    if (!rst_n.read()) {
        rx_len = 0; last_rx_len = 0; tx_busy = false; tx_off = tx_len = 0;
        req_cnt = ack_cnt = bad_cnt = 0;
        roce_cmac_s_axis_tvalid.write(false); roce_cmac_s_axis_tlast.write(false);
        roce_cmac_s_axis_tdata.write(0);  roce_cmac_s_axis_tkeep.write(0);
        o_req_cnt.write(0); o_ack_sent.write(0); o_bad_cnt.write(0);
        return;
    }

    // ---- ① 收(MAC→对端):装配一帧 ⇒ 解析 ⇒ 若是 WRITE 请求 ⇒ 备好 ACK ✓
    if (cmac_m_axis_tvalid.read()) {
        sc_biguint<2048> d = cmac_m_axis_tdata.read();
        sc_biguint<256>  k = cmac_m_axis_tkeep.read();
        for (int j = 0; j < 256 && rx_len < PKT_MAX; j++)
            if (k[j].to_bool()) rx_buf[rx_len++] = (unsigned char)d.range(j*8+7, j*8).to_uint();
        if (cmac_m_axis_tlast.read()) {
            unsigned char opc = 0; uint32_t qp = 0, psn = 0;
            last_rx_len = rx_len;                        // TB 抽包判据用 ✓
            if (parse_bth(rx_buf, rx_len, opc, qp, psn)) {
                bool is_req = (opc == OPC_WRITE_ONLY) || (opc == OPC_SEND_ONLY) || (opc == OPC_RD_REQ);
                if (is_req && drop_times > 0) {          // ★ M3:静默丢(留档供"重发包==原包"判据)✗
                    drop_times--;
                    for (int i = 0; i < rx_len; i++) rnr_rx_buf[i] = rx_buf[i];
                    rnr_rx_len = rx_len;
                }
                else if (is_req && rnr_times > 0) {      // ★ M2:RNR-NAK(不计请求数;之后重发会再收)✓
                    rnr_times--; rnr_cnt++;
                    for (int i = 0; i < rx_len; i++) rnr_rx_buf[i] = rx_buf[i];
                    rnr_rx_len = rx_len;                 // 原包留档(判据用)✓
                    int n = build_rnr_ack(tx_buf, rx_buf, rx_len, 0x02);
                    tx_len = n; tx_off = 0; tx_busy = true;
                } else if (opc == OPC_WRITE_ONLY) {      // WRITE ⇒ ACK(M1 口径)✓
                    req_cnt++;
                    int n = build_ack(tx_buf, rx_buf, rx_len);
                    if (psn_bias) patch_psn(tx_buf, n, psn_bias);
                    tx_len = n; tx_off = 0; tx_busy = true; ack_cnt++;
                } else if (opc == OPC_SEND_ONLY) {       // ★ M2:SEND ⇒ ACK ✓
                    send_cnt++;
                    int n = build_ack(tx_buf, rx_buf, rx_len);
                    if (psn_bias) patch_psn(tx_buf, n, psn_bias);
                    tx_len = n; tx_off = 0; tx_busy = true; ack_cnt++;
                } else if (opc == OPC_RD_REQ) {          // ★ M2:READ ⇒ READ RESPONSE(单包)✓
                    rd_req_cnt++;
                    uint32_t rdlen = ((uint32_t)rx_buf[66] << 24) | ((uint32_t)rx_buf[67] << 16) |
                                     ((uint32_t)rx_buf[68] << 8)  | (uint32_t)rx_buf[69];
                    if (rdlen > (uint32_t)pmem_len) rdlen = (uint32_t)pmem_len;   // 内存不够 ⇒ 截断
                    int n = build_read_response(tx_buf, rx_buf, rx_len, pmem, (int)rdlen);
                    if (psn_bias) patch_psn(tx_buf, n, psn_bias);
                    tx_len = n; tx_off = 0; tx_busy = true; rd_rsp_cnt++;
                } else bad_cnt++;
            } else bad_cnt++;
            rx_len = 0;
        }
    }
    (void)cmac_m_axis_tready.read();             // MAC 侧受理指示(对端只读;M1 不用)✓

    // ---- ② 发(对端→MAC):一帧一 beat(≤256B;本 M1 的 ACK=62B)✓
    if (tx_busy) {
        sc_biguint<2048> d = 0; sc_biguint<256> k = 0;
        int nb = (tx_len - tx_off > 256) ? 256 : (tx_len - tx_off);
        for (int j = 0; j < nb; j++) { d.range(j*8+7, j*8) = tx_buf[tx_off + j]; k[j] = true; }
        tx_off += nb;
        roce_cmac_s_axis_tdata.write(d);  roce_cmac_s_axis_tkeep.write(k);
        roce_cmac_s_axis_tvalid.write(true);
        roce_cmac_s_axis_tlast.write(tx_off >= tx_len);
        if (tx_off >= tx_len) tx_busy = false;
    } else {
        roce_cmac_s_axis_tvalid.write(false);
        roce_cmac_s_axis_tlast.write(false);
    }

    o_req_cnt.write(req_cnt); o_ack_sent.write(ack_cnt); o_bad_cnt.write(bad_cnt);
}
