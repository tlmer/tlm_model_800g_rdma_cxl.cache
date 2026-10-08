//============================================================================
// roce_pkt.h — RoCEv2 **包库**(纯 C++;单一定义处)  [2026-10-05 建 / ]
//
// 出处(只抽不发明;逐条):
//   · 布局/字段 = 厂商 `send_rdresp_ack_pkt_gen` 配方(**主线 基准 TB 逐字节照抄**:
//     `RTL 源`;BTH opcode 0x0a = WRITE ONLY)✓
//   · ICRC = **非反射 CRC32**(poly `0x04C11DB7` MSB-first / init `0xFFFFFFFF` / 末**取反**;
//     追加 = **MSB 先**)—— 照 基准 `crc32_step`)/`func_icrc`)/
//     `func_pkt_byte`)原码移植 ✓(⚠ 与以太网 FCS 的**反射** CRC32 **不同族**,别混)✗
//   · IPv4 校验和 = 标准 16 位反码和(基准 `ipv4_cksum`)✓
//
// ★ 用户(两个,同一份源):① `peer/roce_peer_tlm.cpp`(对端 TLM)
//   ② `device/device_tlm.cpp`(设备引擎真包侧,)—— ⚠ 本库**不依赖 SystemC** ✓
//============================================================================
#ifndef ROCE_PKT_H
#define ROCE_PKT_H

#include <cstdint>
#include <cstring>

namespace rocepkt {

static const int PKT_MAX = 2048;
static const unsigned char OPC_WRITE_ONLY = 0x0a;   // RDMA WRITE ONLY(配方)✓
static const unsigned char OPC_SEND_ONLY  = 0x04;   // ★ M2:WQE opc 0x02 ⇒ {7'h02,opc[0]}(RTL)✓
static const unsigned char OPC_RD_REQ     = 0x0c;   // ★ M2:RDMA READ REQUEST(WQE opc 0x04;RTL)✓
static const unsigned char OPC_RD_RSP_ONLY= 0x10;   // ★ M2:READ RESPONSE ONLY(单包;RTL)✓
static const unsigned char OPC_ACK        = 0x11;   // ACKNOWLEDGE ✓

// ---- AETH.Syndrome(byte54;{1'b0, 2'b TYPE, 5'b CODE};RTL)----
inline unsigned char aeth_ack(unsigned char credits = 0x1F) { return (unsigned char)((0u << 5) | (credits & 0x1Fu)); }
inline unsigned char aeth_rnr(unsigned char tval    = 0x02) { return (unsigned char)((1u << 5) | (tval    & 0x1Fu)); }
inline unsigned char aeth_nak(unsigned char code)           { return (unsigned char)((3u << 5) | (code    & 0x1Fu)); }

// 非反射 CRC32 单步(基准 `crc32_step`)✓
inline uint32_t crc_icrc_step(uint32_t c, unsigned char b) {
    for (int i = 7; i >= 0; i--) {
        uint32_t bin = (b >> i) & 1u;
        c = (c << 1) ^ (((c >> 31) ^ bin) ? 0x04C11DB7u : 0u);
    }
    return c;
}

inline uint32_t icrc(const unsigned char * content, int n) {
    uint32_t c = 0xFFFFFFFFu;
    for (int k = 0; k < n; k++) c = crc_icrc_step(c, content[k]);
    return ~c;                       // ★ 末取反(照 func_icrc)✓
}

inline uint16_t ipv4_cksum(const unsigned char * p) {
    uint32_t s = 0;
    for (int k = 14; k < 34; k += 2) s += (uint32_t)((p[k] << 8) | p[k + 1]);
    while (s >> 16) s = (s & 0xFFFFu) + (s >> 16);
    return (uint16_t)(~s);
}

// ---- 内部小工具(主机序 → 线序/线序 → 主机序)✓ ----
inline void put32(unsigned char * p, uint32_t v) {
    p[0] = (unsigned char)(v >> 24); p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);  p[3] = (unsigned char)v;
}
inline uint32_t get24(const unsigned char * p) {
    return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

// ---- 请求包通用构造(内容+ICRC;返回总长)----
//   `has_reth` = 带 RETH 的请求(WRITE/READ);SEND 无 RETH(RTL has_reth 判据)✓
//   布局:eth14+ip20+udp8+BTH12(+RETH16)+payload+ICRC4(照 基准 配方,逐字节一致)✓
inline int build_req(unsigned char * out,
                     const unsigned char * da, const unsigned char * sa,
                     uint32_t src_ip, uint32_t dst_ip,
                     uint16_t src_port, uint16_t dst_port,
                     unsigned char opcode, bool has_reth,
                     uint32_t dest_qp, uint32_t psn, bool ack_req,
                     uint64_t va, uint32_t rkey, uint32_t reth_dma_len,
                     const unsigned char * payload, int pay_len,
                     uint16_t pkey_reg = 0x6f66) {          // ★ D16:寄存器形态(头字节 = 低字节先;RTL 换序)✓
    const int pay_off = has_reth ? 70 : 54;
    std::memset(out, 0, pay_off + pay_len + 4);
    for (int i = 0; i < 6; i++) out[i] = da[i];          // DA
    for (int i = 0; i < 6; i++) out[6 + i] = sa[i];      // SA
    out[12] = 0x08; out[13] = 0x00;                      // etype 0x0800 ✓
    out[14] = 0x45; out[15] = 0xb8;                      // IPv4 ver/IHL=5, DSCP ✓
    out[18] = 0x55; out[19] = 0x55;                      // ID(配方)
    out[20] = 0x40; out[21] = 0x00;                      // DF ✓
    out[22] = 0xba; out[23] = 0x11;                      // TTL / proto=UDP ✓
    put32(out + 26, dst_ip); put32(out + 30, src_ip);    // 26..29=对端,30..33=本机 ✓
    out[34] = (unsigned char)(src_port >> 8); out[35] = (unsigned char)src_port;
    out[36] = (unsigned char)(dst_port >> 8); out[37] = (unsigned char)dst_port;
    out[40] = 0; out[41] = 0;                            // UDP cksum = 0(IPv4 合法)✓
    int content_len = pay_off + pay_len;
    int total       = content_len + 4;                   // 含 ICRC ✓
    int ip_total    = total - 14;
    int udp_len     = total - 34;
    out[16] = (unsigned char)(ip_total >> 8); out[17] = (unsigned char)ip_total;
    out[38] = (unsigned char)(udp_len >> 8);  out[39] = (unsigned char)udp_len;
    // BTH(42..53)✓
    out[42] = opcode;
    out[43] = 0x30;                                      // flags(ver=3)✓
    out[44] = (unsigned char)(pkey_reg & 0xFF);          // ★ D16:P_Key ← QP_ADV_CONF[31:16](换序装配)✓
    out[45] = (unsigned char)(pkey_reg >> 8);            // (默认 0x6f66 ⇒ 头字节 66 6f = 原配方值 ✓)
    out[46] = 0x05;                                      // reserved(配方)✓
    out[47] = (unsigned char)(dest_qp >> 16); out[48] = (unsigned char)(dest_qp >> 8);
    out[49] = (unsigned char)dest_qp;                    // DestQP(3B)✓
    out[50] = ack_req ? 1 : 0;                           // AckReq ✓
    out[51] = (unsigned char)(psn >> 16); out[52] = (unsigned char)(psn >> 8);
    out[53] = (unsigned char)psn;                        // PSN(3B)✓
    // RETH(54..69;仅 has_reth)✓
    if (has_reth) {
        for (int i = 0; i < 8; i++) out[54 + i] = (unsigned char)(va >> (8 * (7 - i)));
        put32(out + 62, rkey);
        put32(out + 66, reth_dma_len);                   // dma_len ✓
    }
    if (pay_len > 0 && payload) std::memcpy(out + pay_off, payload, pay_len);
    // IP 校验和(24/25)✓
    out[24] = 0; out[25] = 0;
    uint16_t ck = ipv4_cksum(out);
    out[24] = (unsigned char)(ck >> 8); out[25] = (unsigned char)ck;
    // ICRC(**MSB 先**追加;照 基准 `func_pkt_byte`)✓
    uint32_t ic = icrc(out, content_len);
    out[content_len + 0] = (unsigned char)(ic >> 24);
    out[content_len + 1] = (unsigned char)(ic >> 16);
    out[content_len + 2] = (unsigned char)(ic >> 8);
    out[content_len + 3] = (unsigned char)ic;
    return total;
}

// 构造 WRITE 请求(内容+ICRC;返回总长)—— 头部字段照配方,载荷/长度/VA/rkey/PSN 为入参 ✓
inline int build_write_request(unsigned char * out,
                               const unsigned char * da, const unsigned char * sa,
                               uint32_t src_ip, uint32_t dst_ip,
                               uint16_t src_port, uint16_t dst_port,
                               unsigned char opcode,
                               uint32_t dest_qp, uint32_t psn, bool ack_req,
                               uint64_t va, uint32_t rkey,
                               const unsigned char * payload, int pay_len,
                               uint16_t pkey_reg = 0x6f66) {
    return build_req(out, da, sa, src_ip, dst_ip, src_port, dst_port,
                     opcode, /*has_reth=*/true, dest_qp, psn, ack_req,
                     va, rkey, (uint32_t)pay_len, payload, pay_len, pkey_reg);
}

// ★ M2:RDMA READ REQUEST(无载荷;RETH.dma_len = **读长**;RTL)✓
inline int build_read_request(unsigned char * out,
                              const unsigned char * da, const unsigned char * sa,
                              uint32_t src_ip, uint32_t dst_ip,
                              uint16_t src_port, uint16_t dst_port,
                              uint32_t dest_qp, uint32_t psn,
                              uint64_t va, uint32_t rkey, uint32_t read_len,
                              uint16_t pkey_reg = 0x6f66) {
    return build_req(out, da, sa, src_ip, dst_ip, src_port, dst_port,
                     OPC_RD_REQ, /*has_reth=*/true, dest_qp, psn, true,
                     va, rkey, read_len, nullptr, 0, pkey_reg);
}

// ★ M2:SEND ONLY(无 RETH;载荷紧随 BTH;RTL)✓
inline int build_send_only(unsigned char * out,
                           const unsigned char * da, const unsigned char * sa,
                           uint32_t src_ip, uint32_t dst_ip,
                           uint16_t src_port, uint16_t dst_port,
                           uint32_t dest_qp, uint32_t psn,
                           const unsigned char * payload, int pay_len,
                           uint16_t pkey_reg = 0x6f66) {
    return build_req(out, da, sa, src_ip, dst_ip, src_port, dst_port,
                     OPC_SEND_ONLY, /*has_reth=*/false, dest_qp, psn, true,
                     0, 0, 0, payload, pay_len, pkey_reg);
}

// 构造 ACK(62B;DA/SA·IP·端口互换;DestQP/PSN 取自请求;AETH syndrome=0 ⏳ 待与真 RTL 核对)✓
inline int build_ack(unsigned char * out, const unsigned char * req, int req_len) {
    (void)req_len;
    std::memset(out, 0, 62);
    for (int i = 0; i < 6; i++) { out[i] = req[6 + i]; out[6 + i] = req[i]; }   // DA/SA 互换 ✓
    out[12] = 0x08; out[13] = 0x00;
    out[14] = 0x45; out[15] = 0xb8;
    out[18] = 0x55; out[19] = 0x55;
    out[20] = 0x40; out[21] = 0x00;
    out[22] = 0xba; out[23] = 0x11;
    for (int i = 0; i < 4; i++) { out[26 + i] = req[30 + i]; out[30 + i] = req[26 + i]; }  // IP 互换 ✓
    out[34] = req[36]; out[35] = req[37]; out[36] = req[34]; out[37] = req[35];  // 端口互换 ✓
    int total = 62, ip_total = total - 14, udp_len = total - 34, content_len = 58;
    out[16] = (unsigned char)(ip_total >> 8); out[17] = (unsigned char)ip_total;
    out[38] = (unsigned char)(udp_len >> 8);  out[39] = (unsigned char)udp_len;
    out[42] = OPC_ACK;                       // BTH opcode = 0x11(ACKNOWLEDGE)✓
    out[43] = 0x30;
    out[44] = 0x66; out[45] = 0x6f;
    out[46] = 0x05;
    out[47] = req[47]; out[48] = req[48]; out[49] = req[49];   // DestQP 回抄 ✓
    out[50] = 0;
    out[51] = req[51]; out[52] = req[52]; out[53] = req[53];   // ★ PSN 取自请求 ✓
    out[54] = aeth_ack();                                            // ★ AETH:ACK + credits=5'b11111 = 0x1F(RTL)✓
    out[55] = 0x00; out[56] = 0x00; out[57] = 0x00;                  // MSN = 0 ✓
    out[24] = 0; out[25] = 0;
    uint16_t ck = ipv4_cksum(out);
    out[24] = (unsigned char)(ck >> 8); out[25] = (unsigned char)ck;
    uint32_t ic = icrc(out, content_len);
    out[content_len + 0] = (unsigned char)(ic >> 24);   // MSB 先(同上)✓
    out[content_len + 1] = (unsigned char)(ic >> 16);
    out[content_len + 2] = (unsigned char)(ic >> 8);
    out[content_len + 3] = (unsigned char)ic;
    return total;
}

// ★ M2:READ RESPONSE(单包;内容 = eth14+ip20+udp8+BTH12+**AETH4**+payload;RTL)
//   BTH opcode = RD_RSP_ONLY(0x10);DestQP/PSN **回抄请求**(RTL)✓
inline int build_read_response(unsigned char * out, const unsigned char * req, int req_len,
                               const unsigned char * payload, int pay_len) {
    (void)req_len;
    std::memset(out, 0, 58 + pay_len + 4);
    for (int i = 0; i < 6; i++) { out[i] = req[6 + i]; out[6 + i] = req[i]; }   // DA/SA 互换 ✓
    out[12] = 0x08; out[13] = 0x00;
    out[14] = 0x45; out[15] = 0xb8;
    out[18] = 0x55; out[19] = 0x55;
    out[20] = 0x40; out[21] = 0x00;
    out[22] = 0xba; out[23] = 0x11;
    for (int i = 0; i < 4; i++) { out[26 + i] = req[30 + i]; out[30 + i] = req[26 + i]; }  // IP 互换 ✓
    out[34] = req[36]; out[35] = req[37]; out[36] = req[34]; out[37] = req[35];  // 端口互换 ✓
    int content_len = 58 + pay_len;
    int total       = content_len + 4;
    int ip_total    = total - 14;
    int udp_len     = total - 34;
    out[16] = (unsigned char)(ip_total >> 8); out[17] = (unsigned char)ip_total;
    out[38] = (unsigned char)(udp_len >> 8);  out[39] = (unsigned char)udp_len;
    out[42] = OPC_RD_RSP_ONLY;                        // BTH opcode = 0x10 ✓
    out[43] = 0x30;
    out[44] = 0x66; out[45] = 0x6f;
    out[46] = 0x05;
    out[47] = req[47]; out[48] = req[48]; out[49] = req[49];   // DestQP 回抄 ✓
    out[50] = 0;
    out[51] = req[51]; out[52] = req[52]; out[53] = req[53];   // PSN 回抄 ✓
    out[54] = aeth_ack();                             // AETH:ACK + credits 0x1F ✓
    out[55] = 0x00; out[56] = 0x00; out[57] = 0x00;   // MSN = 0 ✓
    if (pay_len > 0 && payload) std::memcpy(out + 58, payload, pay_len);
    out[24] = 0; out[25] = 0;
    uint16_t ck = ipv4_cksum(out);
    out[24] = (unsigned char)(ck >> 8); out[25] = (unsigned char)ck;
    uint32_t ic = icrc(out, content_len);
    out[content_len + 0] = (unsigned char)(ic >> 24);
    out[content_len + 1] = (unsigned char)(ic >> 16);
    out[content_len + 2] = (unsigned char)(ic >> 8);
    out[content_len + 3] = (unsigned char)ic;
    return total;
}

// ★ M2:RNR-NAK 包(ACK 包头 + syndrome {1'b0,01,tval};RTL)✓
inline int build_rnr_ack(unsigned char * out, const unsigned char * req, int req_len,
                         unsigned char tval = 0x02) {
    int n = build_ack(out, req, req_len);
    out[54] = aeth_rnr(tval);
    out[24] = 0; out[25] = 0;                          // 重算 IP 校验和(syndrome 变了)✓
    uint16_t ck = ipv4_cksum(out);
    out[24] = (unsigned char)(ck >> 8); out[25] = (unsigned char)ck;
    uint32_t ic = icrc(out, 58);                       // ICRC 覆盖内容(不含自身)✓
    out[58] = (unsigned char)(ic >> 24); out[59] = (unsigned char)(ic >> 16);
    out[60] = (unsigned char)(ic >> 8);  out[61] = (unsigned char)ic;
    return n;
}

// 解析(最小):取 BTH opcode / DestQP / PSN(未命中返回 false)✓
inline bool parse_bth(const unsigned char * p, int n,
                      unsigned char & opc, uint32_t & dest_qp, uint32_t & psn) {
    if (n < 54) return false;
    opc = p[42];
    dest_qp = get24(p + 47);
    psn = get24(p + 51);
    return true;
}

} // namespace rocepkt

#endif // ROCE_PKT_H
