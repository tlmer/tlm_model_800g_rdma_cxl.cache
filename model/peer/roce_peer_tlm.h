//============================================================================
// roce_peer_tlm.h — ★ 最小 RoCEv2 **对端**(第三方 NIC 的替身)  [2026-10-05 建 / v1.0]
//
// 规格 = `规格文档`(C2 定稿:线侧对端 = 第三方 NIC ⇒ 真包语义)✓
// 本册 = §5 的「最小对端」:**先自洽自测;真第三方模型到场后替换** ✓
//
// ★ 包配方(只抽不发明;出处逐条):
//   · 布局/字段 = 厂商 `send_rdresp_ack_pkt_gen` 配方(**主线 基准 TB 逐字节照抄**:
//     `RTL 源`,BTH opcode=0x0a=WRITE ONLY)✓
//   · **ICRC = 非反射 CRC32**(poly `0x04C11DB7`,MSB-first,init `0xFFFFFFFF`,末**取反**;
//     追加 = **MSB 先**;残差(内容+ICRC)须 == `0xc704dd7b`)—— 照 基准 TB `crc32_step`/
//     `func_icrc`)+ 追加序照 `func_pkt_byte`)原码移植 ✓(⚠ 与以太网 FCS 的**反射** CRC32 **不同族**,别混)✗
//   · IPv4 校验和 = 标准 16 位反码和(基准 `ipv4_cksum`)✓
//
// ★ 行为(M1):收 **WRITE 请求**(opcode 0x0a)⇒ 回 **ACK**(opcode 0x11;DestQP/PSN 取自请求;
//   AETH syndrome=0)⚠ AETH 细节 ⏳ **待与真 RTL 配方核对**(下一步开卷 基准 的 ACK 逐字节)✓
//   另附**静态构造器**(`build_write_request`/`build_ack`)供 TB/设备引擎复用(单一定义处)✓
//============================================================================
#ifndef ROCE_PEER_TLM_H
#define ROCE_PEER_TLM_H

//============================================================================
// ★ 版本与冻结策略(同全线:冻结 = 客户首次集成前)
//   变更记录:
//     (2026-10-05) ★ ****:SEND⇒ACK / **READ⇒READ RESPONSE(单包,载荷取对端内存)** /
//                       **RNR-NAK**(旋钮)/ **PSN 偏置旋钮**(乱序负控);计数 +rnr/rd/send ✓
//     (2026-10-05) 骨架:包库(构造/解析/ICRC)+ WRITE 请求→ACK 收发 + 观测 ✓
//============================================================================
#define ROCE_PEER_TLM_VERSION_MAJOR 1
#define ROCE_PEER_TLM_VERSION_MINOR 0
#define ROCE_PEER_TLM_VERSION_STR   "roce_peer_tlm 1.0 (2026-10-05)"

#include <systemc.h>
#include <cstring>
#include "roce_pkt.h"      // ★ 包库**单一定义处**(纯 C++;设备引擎真包侧共用,)✓

struct RocePeerTlm: sc_core::sc_module {
    // ---------------- 时钟/复位 ----------------
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    // ---------------- NIC 侧(AXIS;与 MacTlm 的 NIC 面同口,直连即可)----------------
    // RX(对端→MAC 方向;本模块为输出侧)⚠ 与 MacTlm 的 roce_cmac_s_axis_* 同形 ✓
    sc_out<bool>              roce_cmac_s_axis_tvalid;
    sc_out<sc_biguint<2048>>  roce_cmac_s_axis_tdata;
    sc_out<sc_biguint<256>>   roce_cmac_s_axis_tkeep;
    sc_out<bool>              roce_cmac_s_axis_tlast;
    sc_in<bool>               roce_cmac_s_axis_tuser;   // ⚠ 它是 **MAC 的输出**(侧带)⇒ 对端为**输入** ✓
    // TX(MAC→对端;本模块为输入侧)
    sc_in<bool>               cmac_m_axis_tvalid;
    sc_in<sc_biguint<2048>>   cmac_m_axis_tdata;
    sc_in<sc_biguint<256>>    cmac_m_axis_tkeep;
    sc_in<bool>               cmac_m_axis_tready;   // ⚠ MAC 的输出(受理指示)⇒ 对端为**输入** ✓
    sc_in<bool>               cmac_m_axis_tlast;

    // ---------------- 观测(判据用)----------------
    sc_out<sc_uint<32>> o_req_cnt;   // 收到的 WRITE 请求数
    sc_out<sc_uint<32>> o_ack_sent;  // 发出的 ACK 数
    sc_out<sc_uint<32>> o_bad_cnt;   // 非预期包(opcode/长度)计数

    // ---------------- 包库(静态;**单一定义处 = `peer/roce_pkt.h`**)----------------
    //   ⚠ 实现在 `rocepkt::`(纯 C++ 头,设备引擎真包侧同一份源)⇒ 此处只留**薄转发**(TB 原调用不变)✓
    static const int PKT_MAX = rocepkt::PKT_MAX;
    static const unsigned char OPC_WRITE_ONLY  = rocepkt::OPC_WRITE_ONLY;
    static const unsigned char OPC_SEND_ONLY   = rocepkt::OPC_SEND_ONLY;    // ★ M2 ✓
    static const unsigned char OPC_RD_REQ      = rocepkt::OPC_RD_REQ;       // ★ M2 ✓
    static const unsigned char OPC_RD_RSP_ONLY = rocepkt::OPC_RD_RSP_ONLY;  // ★ M2 ✓
    static const unsigned char OPC_ACK         = rocepkt::OPC_ACK;
    // ★ M2 包构造转发 ✓
    static int build_read_request(unsigned char * out, const unsigned char * da, const unsigned char * sa,
                                  uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport,
                                  uint32_t qp, uint32_t psn, uint64_t va, uint32_t rkey, uint32_t rdlen) {
        return rocepkt::build_read_request(out, da, sa, sip, dip, sport, dport, qp, psn, va, rkey, rdlen);
    }
    static int build_send_only(unsigned char * out, const unsigned char * da, const unsigned char * sa,
                               uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport,
                               uint32_t qp, uint32_t psn, const unsigned char * pay, int n) {
        return rocepkt::build_send_only(out, da, sa, sip, dip, sport, dport, qp, psn, pay, n);
    }
    static int build_read_response(unsigned char * out, const unsigned char * req, int rn,
                                   const unsigned char * pay, int n) {
        return rocepkt::build_read_response(out, req, rn, pay, n);
    }
    static int build_rnr_ack(unsigned char * out, const unsigned char * req, int rn, unsigned char tval = 0x02) {
        return rocepkt::build_rnr_ack(out, req, rn, tval);
    }

    // 非反射 CRC32(基准 同款)✓
    static uint32_t crc_icrc_step(uint32_t c, unsigned char b) { return rocepkt::crc_icrc_step(c, b); }
    static uint32_t icrc(const unsigned char * content, int n) { return rocepkt::icrc(content, n); }
    static uint16_t ipv4_cksum(const unsigned char * p)        { return rocepkt::ipv4_cksum(p); }

    // 构造 WRITE 请求(内容+ICRC;返回总长)✓
    static int build_write_request(unsigned char * out,
                                   const unsigned char * da, const unsigned char * sa,
                                   uint32_t src_ip, uint32_t dst_ip,
                                   uint16_t src_port, uint16_t dst_port,
                                   unsigned char opcode,
                                   uint32_t dest_qp, uint32_t psn, bool ack_req,
                                   uint64_t va, uint32_t rkey,
                                   const unsigned char * payload, int pay_len,
                                   uint16_t pkey_reg = 0x6f66) {
        return rocepkt::build_write_request(out, da, sa, src_ip, dst_ip, src_port, dst_port,
                                            opcode, dest_qp, psn, ack_req, va, rkey, payload, pay_len,
                                            pkey_reg);
    }

    // 构造 ACK(62B;DestQP/PSN 取自请求;AETH syndrome=0 ⏳ 待核对)✓
    static int build_ack(unsigned char * out,
                         const unsigned char * req, int req_len) { return rocepkt::build_ack(out, req, req_len); }

    // 解析(最小):取 BTH opcode / DestQP / PSN(未命中返回 false)✓
    static bool parse_bth(const unsigned char * p, int n,
                          unsigned char & opc, uint32_t & dest_qp, uint32_t & psn) {
        return rocepkt::parse_bth(p, n, opc, dest_qp, psn);
    }

    // ---------------- 构造/运行 ----------------
    SC_HAS_PROCESS(RocePeerTlm);
    RocePeerTlm(sc_core::sc_module_name nm);
    void peer_step();

    // ---------------- ★ M2 旋钮/资源(⚠ 白盒:只 TB 可调)----------------
    int  psn_bias  = 0;          // 响应 PSN 偏置(负控:模拟乱序/重复响应;0 = 正常回抄)✗
    int  rnr_times = 0;          // 前 N 个请求回 RNR-NAK(之后正常;每收到一次减一)✓
    int  drop_times = 0;         // ★ M3:前 N 个请求**静默丢**(不回任何响应 ⇒ 对端超时重发)✗
    unsigned char pmem[1024];    // 对端内存(READ RESPONSE 的载荷源)✓
    int  pmem_len = 0;
    void set_pmem(const unsigned char * d, int n) {
        int m = (n < (int)sizeof(pmem)) ? n : (int)sizeof(pmem);
        for (int i = 0; i < m; i++) pmem[i] = d[i];
        pmem_len = m;
    }
    unsigned int rd_req_cnt = 0, rd_rsp_cnt = 0, send_cnt = 0, rnr_cnt = 0;   // ★ M2 计数 ✓
    unsigned char rnr_rx_buf[PKT_MAX]; int rnr_rx_len = 0;   // 回 RNR 的那一包留档(判"重发 == 原包")✓
    static void patch_psn(unsigned char * p, int n, int bias);                // 改 PSN + 重算 ICRC ✓

    // 内部态
    unsigned char rx_buf[PKT_MAX]; int rx_len = 0;
    int last_rx_len = 0;             // 最近一帧的长度(TB 抽包判据用;rx_len 处理完即清零)✓
    bool          tx_busy = false;   int tx_off = 0; int tx_len = 0;
    unsigned char tx_buf[PKT_MAX];
    unsigned int  req_cnt = 0, ack_cnt = 0, bad_cnt = 0;

    // TB 钩子(直发一个包;诊断用)✓
    bool push_out(const unsigned char * f, int n);
};

#endif // ROCE_PEER_TLM_H
