# NIC TLM 模型 —— 客户用户指南  [2026-10-06 建 / v1]

> **本文面向客户集成工程师**:从零把模型跑起来、接进平台、读懂口径 ✓
> 一页总览 ⇒ [`README.md`](../README.md);任务进度 ⇒ [`开发计划.md`](开发计划.md);
> **与 RTL 的差异** ⇒ [`差异清单.md`](差异清单.md)(交付件,集成前必读)✓

## §0 交付形态一览

**四件模型 = 四个静态库**(`.h` 接口 + `.a` 实现;**改实现不动客户代码**):

| 库 | 接口头 | 模型 | 角色 |
|---|---|---|---|
| `libcxl2ace_tlm.a` | `bridge/cxl2ace_bridge.h` | cxl2ace 桥 **v1.0** | 主机侧 **ACE socket** ⇄ 设备侧 **pin 级 CXL.cache** |
| `libnic_tlm.a` | `device/device_tlm.h` | 设备侧 TLM **v1.0** | NIC/COH 设备(寄存器面 + 队列引擎 + 线侧真包) |
| `libmac_tlm.a` | `mac/mac_tlm.h` | MAC TLM **v1.0** | NIC 侧 AXIS ⇄ 线侧 **LINK 2×512b** |
| `libpcs_tlm.a` | `pcs/pcs_link_tlm.h` | PCS/FEC link **v1.0** | 线侧 link 抽象(延迟/误码/环回旋钮;⛔ 不做位级转写) |

> 另有**非库**参考件(自测/快启用,随源码交付):`top/`(装配样例)、`tb/`(自测 + 绑定范例)、
> `peer/`(RoCE 线侧对端替身,自测用)、`sw/`(软件侧头/驱动/设备树草案)✓

## §1 Filelist(交付清单)

```text
tlm_model/
├── model/
│   ├── bridge/   cxl2ace_bridge.h/.cpp   cxl_shim_types.h        ← 库 1(含 pin 级类型定义)
│   ├── device/   device_tlm.h/.cpp                               ← 库 2
│   ├── mac/      mac_tlm.h/.cpp                                  ← 库 3
│   ├── pcs/      pcs_link_tlm.h/.cpp                             ← 库 4
│   ├── peer/     roce_peer_tlm.h/.cpp  roce_pkt.h                ← 线侧对端(自测;roce_pkt.h 亦被库 2 编译依赖)
│   ├── top/      stack_top.h  dual_mac_top.h                     ← ★ 装配样例(快启用)
│   ├── tb/       tb_*.cpp  dummy_amba_master.h                   ← 自测 24 台 + 绑定范例
│   └── Makefile  regress.sh                                      ← 构建/回归
└── sw/           nic_regs.h  nic_hw.h  nic_drv.c  nic.dtsi   ← 软件可见面(与模型实测对拍)
```

⚠ **编译库 1/2 时**:`device_tlm.cpp` 内含 `peer/roce_pkt.h`(包库,单一定义处)⇒ filelist 需带
`+incdir+…/model/peer`;⚠ **只链 `.a` 而不再编译**的客户**不需要** peer/ ✓

## §2 依赖性

| 依赖 | 要求 | 说明 |
|---|---|---|
| **SystemC** | **2.3.4**(2.3.x 均可;实测 2.3.4-Accellera) | `SYSTEMC_HOME` 可覆盖(`Makefile` 默认 `/opt/tools/systemc2.34`)✓ |
| **AMBA-PV** | **VDK 2024.03 自带**(header-only) | `AMBA_PV_HOME` 指向 `…/SLS/linux/IP_common/AMBA_TLM2_BL/AMBA-PV`;⚠ 仅**编译期**需要(桥/设备头里 `#include <amba_pv.h>`)✓ |
| 编译器 | **g++ 且 -std=c++14 起** | 客户平台口径 = **VDK 2024.03 自带 gcc-9.2/9.5**(实测可用);VCS-SC 混仿另有白名单要求(9.2/9.5/12.3/13.2)✓ |
| 可选(仅自测) | `amba_pv_ace_protocol_checker`(AMBA-PV 例程)、Xilinx `libsystemctlm-soc`(TBs 的 `-I`) | 不集成可不备 ✓ |
| 平台 | **VDK 2024.03 + Fast Model**(客户定案口径) | 模型为**标准 SystemC/TLM-2.0**;不绑平台,VDK 仅作运行宿主 ✓ |

## §3 构建命令

```sh
cd model
make libs                    # 出四静态库(交付形态)✓
sh regress.sh                # 24 台自测全跑(自带 make -q 守卫:过期先自动重建)✓

# 覆盖依赖路径(客户环境):
make libs SYSTEMC_HOME=<你的 systemc> AMBA_PV_HOME=<你的 amba-pv>
```

单库构建:`make libcxl2ace_tlm.a` / `libnic_tlm.a` / `libmac_tlm.a` / `libpcs_tlm.a` ✓
客户侧链接示例:`g++ main.cpp -I<交付头> -L. -lcxl2ace_tlm -lnic_tlm -lmac_tlm -lpcs_tlm $(SYSTEMC) …` ✓

## §4 例化模板与绑定要点

### 4.1 快启(**推荐先跑这个**):完整端点栈

```cpp
#include "stack_top.h"                 // 设备 + 桥 + MAC,内部接线全封装 ✓
StackTop stack("stack");
stack.clk(clk); stack.rst_n(rst_n);    // 时钟/复位
// ① 主机侧:桥 → (检查器) → 平台 ACE 从口/主机内存
stack.br.ace_m.bind(平台 ACE 从口);      // AMBA-PV ACE master socket<512>(full:带 snoop 回程)✓
// ② 寄存器面(A7):平台 AMBA-PV master → 设备从口(不用则绑占位主端,见 tb/dummy_amba_master.h)
平台master.m.bind(stack.dev.reg_s);
// ③ 线侧:stack.mac 的 LINK 口外露 ⇒ 接 PCS link / 真 PCS / 对端(见 tb_dual_stack.cpp 交叉线范例)
// ④ 中断:stack.sig_intr(电平线);观测:stack.sig_* 一组
```

### 4.2 单件例化(按需组合)

```cpp
Cxl2AceBridge br("br");  br.clk(clk); br.rst_n(rst_n);
//   主机侧:br.ace_m(ACE full socket)  ↔ 平台
//   设备侧:pin 级 CXL.cache(d2h_req0/1·data·rsp / h2d_req·rsp·data,逐个显式 sc_signal 绑设备)
//           接线清单照抄 top/stack_top.h 构造体(逐条对应,勿少绑)

DeviceTlm dev("dev");    dev.clk(clk); dev.rst_n(rst_n);
//   软件面:dev.reg_s(AMBA-PV 32b 从口;与直连方法 reg_read/reg_write 同源)✓
//   线侧 :dev.cmac_m_axis_*(→MAC) / roce_cmac_s_axis_*(←MAC;无 ready,恒收)
//   中断 :dev.nic_intr(电平线)

MacTlm mac("mac");       mac.clk(clk); mac.rst_n(rst_n);
//   NIC 侧:cmac_m_axis_*/roce_cmac_s_axis_*(与设备直连);
//   线侧    :link_{txd,txc,txdval,rxd,rxc}[1:0] + txclk_ena/rxclk_ena(★ 4 个 ena 是 MAC 输入)
//   ⚠ 裸连两个 MAC 时须给 ena 恒 1(TB 范式:tb_dual_mac.cpp)✓

PcsLinkTlm pcs("pcs");   // link 抽象:延迟/误码/环回/种子旋钮 + lock/align 门面位
//   pcs 的 MAC 面与 MacTlm 线侧 1:1 免 glue 直连(tb_mac_pcs_link.cpp)✓
```

**三条绑定纪律(踩过的坑,客户侧同样适用)**:
1. **每个端口都要绑**——含观测口/未用口;sc_out 各自独立信号、⛔ 一个信号不许两个写者(E115)✗
2. **AMBA-PV 从口 socket 必须绑**(策略 = ONE_OR_MORE);不用时绑占位主端(`tb/dummy_amba_master.h`)✓
3. **握手判据 = 上拍我输出值 ∧ 本拍对方值**(本项目"§节气"):消费者防 2 拍宽 valid 重吃
   (valid 上升沿守卫或"消费后撤 ready")✓

## §5 集成步骤(建议顺序)

```text
① 出库:make libs(§3)
② 平台接线:
   · 主机:br.ace_m ↔ 平台 ACE 从口(CCI-550/CMN-700 侧;full ACE:snoop 回程已实现)✓
   · 寄存器:平台 master ↔ dev.reg_s(AMBA-PV 32b;未映射⇒DECERR,RO 写⇒SLVERR)✓
   · 线侧:mac 的 LINK ↔ 你的 PCS/对端;⚠ 接**真 PCS** 时注意线侧节拍口径
     (差异清单 **D15**:须 TB 同款稀疏节拍,现提供适配层范式见 cosim/)✓
   · 中断:nic_intr 接你的中断控制器
③ 自检:sh regress.sh(24/24)⇒ 再跑你的平台用例 ⇒ 对读数
```

**软件面**:寄存器/WQE/CQE 布局 = `sw/nic_regs.h`(与模型实测对拍);驱动骨架 `sw/nic_drv.c`
(本机内核树编译通过,上板待搭台)✓

## §6 接口契约与版本

- **版本宏**:各头文件顶部(桥 1.0 / 设备 1.0 / MAC 1.0 / PCS-link 1.0);**冻结 = 客户首次集成前**;
  冻结后 `.h` 一改 = MINOR 进位 + 变更记录(头文件内)✓
- **时钟/复位**:各模块 `clk` 单时钟域(`SC_METHOD/SC_THREAD @clk.pos`);`rst_n` 低有效;
  设备另有 `intr_upd`(中断聚合)与 `engine_thread`(引脚驱动)两进程
- **两种可选能力(默认关,开法见头文件)**:
  · 设备 `set_line_pkt_mode(true)` = **线侧真 RoCE 包**(WRITE/SEND/READ + ACK/PSN/超时重传)✓
  · 设备 `set_cache_mode(true, L)` = **MESI 缓存态**(I/S/E/M + snoop 状态化应答;)✓

## §7 已知差异与边界(摘要;**集成前读全表** ⇒ [`差异清单.md`](差异清单.md))

| 类别 | 要点(编号见差异清单) |
|---|---|
| 一致性 | 默认**全 I**(设备直通无缓存)与 RTL 现默认一致;MESI 为**可选能力**(D17);snoop 顺序语义已建 ✓ |
| 地址译码 | R_key/VA **不译**(offset 直用为字节地址;D3);MTU/分片不建(D9,按行搬) |
| 立即数 | IMMDT 侧带不在模型边界(D2,按基类搬数据 + 计数不静默) |
| 线侧 | 真包字段全部**寄存器真源**(MAC/IP/端口/P_Key/DestQP);⚠ 单包 SEND/READ 的 `bth_hdr[15]` 位
  ⏳ 待与 RTL 对表(影响面:SE 位一个 bit);超时阈值单位 = 模型拍(旋钮,`set_line_timeout`) |
| 规模 | QP 32 / PD 8 块 / 主机内存 4MB(样例);CQE 4B 整行 RMW ✓ |

## §8 自检与回归(交付验收自带)

- `sh regress.sh` ⇒ 一行摘要 + 退出码;**判据行 = 各 TB 自带 `TB_xxx PASS`**(脚本只收集不解释)✓
- **24 台覆盖**:上列 17 类 + ★ **800G 档新台 4 台** —— `tb_mac_lane800`(lane 契约)/
  `tb_mac_pcs_lane`(双档)/ **`tb_800g_bpwm`(L4-3 背压·水位·丢弃;正控+负控)** /
  **`tb_800g_ostd`(L4-4 多笔在飞·桥侧口径 + 槽号)** ✓
- 单台跑:`cd model && make tb_xxx && ./tb_xxx | tail -3`(判据行在末尾)✓

## §9 常见问题(FAQ)

| 症状 | 处理 |
|---|---|
| 编译找不到 SystemC | `SYSTEMC_HOME` 指对(`make libs SYSTEMC_HOME=…`)✓ |
| `E109 port not bound` | 端口(含观测口/socket)漏绑 ⇒ 对照模块端口清单逐条绑(§4.3-1)✗ |
| 接真 PCS 后帧异常/0xFE | 线侧节拍不匹配 ⇒ 见 **D15**:须 TB 同款稀疏节拍(适配层范式在 `cosim/`)✓ |
| 读数"零增长"像挂了 | 先自检进程 **cputime** 增不增(别看 `%CPU` 均值)✓ |
| 想要线侧真包/一致性场景 | 开 §6 的两个旋钮;差异与边界见 §7 ✓ |
| 读厂商/产品 ID 或**版本寄存器** | **模型刻意不实现**(可读但恒 0、写忽略)——交付的是标准 SystemC 参考模型,**不出 IP 识别/版本面** ✓ |

---
*时间戳口径 `YYYY-MM-DD-HH:MM`;模型不绑平台,VDK/FM 仅作运行宿主 ✓*
