//============================================================================
// cxl_shim_types.h — CXL.cache(`syn_*`)通道的 **C++ 侧字段/编码镜像**  [2026-10-04 建 / v1.0 补全]
//
// ⚠⚠ 本文件是 `RTL 源` 的【逐字段镜像】—— 权威仍是那份 RTL;
//     改 RTL 定义 ⇒ 必须回来改这里(并走 §对拍)✗✓
// ★ v1.0 按 §8.1 口径标定**补全字段**(原来只镜像了要搬的子集):
//   · 一拍 = **一整 64B 行**(`CXL_CACHE_LINE_BITS 512`,payload[511:0])—— 不再拆 8×64b ✓
//   · 地址 = **行地址**(byte_addr[51:6]);ACE 侧字节地址 = 行地址 << 6 ✓
//     出处(双信源):① `RTL 源` 注释;② **已验参照桥** `RTL 源`
//     `e.addr = {vif.syn_d2h_req0.addr, 6'd0}` ✓
//     ⚠⚠ **待核(RTL 侧)**:我们 COH 的 shim 把**字节地址**直接塞进该字段
//     (`RTL 源` ← `RTL 源` ext_addr ← `RTL 源`
//      `base + beat<<6` = 字节地址)⇒ 与参照桥口径**差 64 倍**;已记入桥规格 §8.3 待核清单 ✗
//   · 两套编码方言:桥接路径用 **spec 真实编码**(svh 注释明写);COH 方言同值不同名 ⚠
//============================================================================
#ifndef CXL_SHIM_TYPES_H
#define CXL_SHIM_TYPES_H

#include <cstdint>

namespace cxlshim {

// ---- 拍宽/行口径(照 `RTL 源`)----
static constexpr int LINE_BYTES = 64;         // CXL_CACHE_LINE_SIZE
static constexpr int LINE_BITS  = 512;        // CXL_CACHE_LINE_BITS

// ---- D2H(设备→主机)----
struct D2hReq {           // Table 3-13(79b):valid·opcode[4:0]·cqid[11:0]·nt·addr[45:0]·rsvd
    bool     valid  = false;
    uint8_t  opcode = 0;      // 5b 有效;**桥接路径用 spec 编码**(见下方常量)
    uint16_t cqid   = 0;      // 12b 有效(含 N1 的 lane 位 [8:7]? —— 本桥不解释,直通 ✓)
    bool     nt     = false;  // v1.0 镜像但不用(shim 恒 0:`RTL 源`)✓
    uint64_t addr   = 0;      // 46b 有效 = **行地址**(byte_addr[51:6])✓
};
struct D2hRsp {           // Table 3-15(20b):valid·opcode[4:0]·uqid[11:0]·rsvd
    bool     valid  = false;
    uint8_t  opcode = 0;
    uint16_t uqid   = 0;
};
struct D2hData {          // Synopsys 593b(扁平,无 hdr 子层)✓
    bool     valid       = false;
    uint8_t  opcode      = 0;    // 5b;shim 写 5'h07(MemWr/WrCur 数据槽,= 请求 opcode)✓
    uint16_t uqid        = 0;
    uint8_t  cache_id    = 0;
    uint16_t spid        = 0;
    uint16_t dpid        = 0;
    bool     chunk_valid = false;
    bool     poison      = false;
    bool     bogus       = false;
    uint64_t payload[8]  = {0,0,0,0,0,0,0,0};   // 512b = 8×64b
};

// ---- H2D(主机→设备)----
struct H2dReq {           // Table 3-17(64b,snoop):valid·opcode[2:0]·addr[45:0]·uqid[11:0]·rsvd
    bool     valid  = false;
    uint8_t  opcode = 0;      // 3b(SNP_DATA/INV/CUR)
    uint64_t addr   = 0;      // 行地址 ✓
    uint16_t uqid   = 0;
};
struct H2dRsp {           // Synopsys 64b:valid·opcode[3:0]·rsp_data[11:0]·cqid[11:0]·rsvd
    bool     valid    = false;
    uint8_t  opcode   = 0;
    uint16_t rsp_data = 0;    // [2:0] = GO 状态(照 `CXL_GO_STATE_*`)✓
    uint16_t cqid     = 0;
};
struct H2dData {          // hdr(24b:valid·cqid·chunk_valid·poison·go_err·rsvd)+ payload[511:0]
    bool     hdr_valid   = false;
    uint16_t hdr_cqid    = 0;
    bool     chunk_valid = false;
    bool     poison      = false;
    bool     go_err      = false;
    uint64_t payload[8]  = {0,0,0,0,0,0,0,0};
};

// ---- 编码常量(**spec 真实编码**;出处 `RTL 源` "cxl2ace 桥接路径使用" 段)----
// D2H REQ(5b;MemWr = 5'h07 = 旧名 MemWr / 3.2 名 WrCur(表 3-22 = 0 0111b;2026-10-08 按规范对齐)✓)
static constexpr uint8_t  D2H_REQ_SPEC_RD_CURR  = 0x01;
static constexpr uint8_t  D2H_REQ_SPEC_RD_OWN   = 0x02;
static constexpr uint8_t  D2H_REQ_SPEC_RD_SHARED= 0x03;   // ← 我们的读用这条 ✓
static constexpr uint8_t  D2H_REQ_SPEC_RD_ANY   = 0x04;
static constexpr uint8_t  D2H_REQ_MEMWR         = 0x07;   // 旧名 MemWr / CXL 3.2 名 WrCur(表 3-22 = 0 0111b);原 0x00 未分配,2026-10-08 按规范修 ✓
static constexpr uint8_t  D2H_REQ_WR_CUR        = D2H_REQ_MEMWR;   // ★ 别名:CXL 3.2 新名(WrCur)✓
// D2H RSP(5b,SnpResp;)—— ★ 设备对 snoop 的应答编码 ✓
static constexpr uint8_t  D2H_RSP_SPEC_S_HIT_SE   = 0x01;
static constexpr uint8_t  D2H_RSP_SPEC_I_HIT_I    = 0x04;   // ← 我们 RTL 恒回这一路("snoop 恒 RspI")✓
static constexpr uint8_t  D2H_RSP_SPEC_I_HIT_SE   = 0x05;
static constexpr uint8_t  D2H_RSP_SPEC_V_HIT_V    = 0x06;
static constexpr uint8_t  D2H_RSP_SPEC_S_FWD_M    = 0x07;
static constexpr uint8_t  D2H_RSP_SPEC_I_FWD_M    = 0x0F;
static constexpr uint8_t  D2H_RSP_SPEC_V_FWD_V    = 0x16;
// H2D RSP(4b)
static constexpr uint8_t  H2D_RSP_SPEC_WRITE_PULL = 0x1;
static constexpr uint8_t  H2D_RSP_SPEC_GO         = 0x4;  // ← 读回程用这条 ✓
static constexpr uint8_t  H2D_RSP_SPEC_GO_WP      = 0x5;
static constexpr uint8_t  H2D_RSP_SPEC_EXT_CMP    = 0x6;  // 全观测完成(写完成语义候选 ⚠ 待核)
static constexpr uint8_t  H2D_RSP_SPEC_GO_ERR_WP  = 0xF;
// GO 状态(3b)
static constexpr uint16_t CXL_GO_STATE_I   = 0x0;
static constexpr uint16_t CXL_GO_STATE_S   = 0x1;         // ← 读回程默认 S ✓
static constexpr uint16_t CXL_GO_STATE_E   = 0x2;
static constexpr uint16_t CXL_GO_STATE_M   = 0x3;
static constexpr uint16_t CXL_GO_STATE_ERR = 0x7;

} // namespace cxlshim
#endif
