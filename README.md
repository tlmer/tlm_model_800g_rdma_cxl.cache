# tlm_model_800g_rdma_cxl.cache

**800G RDMA(ROCEv2)+ CXL.cache 系统级 TLM 参考模型** —— SystemC/TLM-2.0,面向 800G RDMA NIC 与
CXL.cache 设备的早期软件开发、系统验证与回归测试。

- **不绑平台**:模型为标准 SystemC/TLM-2.0,桥的主机侧为 **AMBA-PV ACE**,可直接接入 Synopsys VDK、
  ARM Fast Models 等虚拟平台;
- **双向帧路**:既含设备侧(CXL.cache 语义 + 400G MAC/PCS),也含对端(RoCEv2 数据面 + RDMA-CM 建链);
- **自带回归**:模型自带 **24 个自测用例**,克隆即可复现 `全部 PASS (24/24)`。

## Features

- **The only device model with CXL.cache support in the industry today**;
- **Builds in both Arm Fast Models and Synopsys VDK environments**;
- **AMBA-PV ACE host interface — integrates directly with CCI-550 / CMN-700**;
- **Compatible with Arm ACE and Intel/AMD CXL coherence semantics**;
- **Built-in MESI** cache-coherence state model;
- **No QEMU-based simulation approximation required**.

## 目录结构

```text
.
├── model/                 # 模型本体
│   ├── device/            # 设备 TLM(CXL.cache 设备 + NIC 寄存器面)
│   ├── bridge/            # cxl2ace 桥(CXL.cache ⇄ AMBA-PV ACE)
│   ├── mac/               # 800G MAC TLM(LINK 1024b 双 lane 线侧)
│   ├── pcs/               # PCS / RS-FEC link TLM
│   ├── peer/              # 对端模型(RoCEv2 数据面 + RDMA-CM)
│   ├── top/               # 双实例顶层(dual MAC / dual stack)
│   ├── tb/                # 自测 TB + regress.sh 回归脚本
│   └── Makefile
├── sw/                    # 软件面素材(寄存器定义 nic_regs.h 等)
└── docs/
    └── user_guide.md      # 用户指南(filelist / 依赖 / 编译 / 例化 / 集成 / FAQ)
```

## 环境要求

| 项 | 要求 |
|---|---|
| SystemC | 2.3.4(Accellera),含 TLM 2.0 |
| 编译器 | g++ ≥ 9(C++14;部分用例 C++17) |
| 可选 | AMBA-PV 库(仅桥的 ACE 口 socket 需要) |
| OS | Linux(x86_64) |

## 构建与自测

```sh
cd model
make            # 构建模型库与全部自测程序
sh regress.sh   # 回归:24 个用例,~数秒
```

预期输出:

```text
全部 PASS ✓(24/24)
```

## 在虚拟平台中构建与运行(Fast Models / VDK)

本模型**不绑平台**:既可在本机构建自测(上节),也可作为组件集成进 **ARM Fast Models** 或
**Synopsys VDK** 平台。两个平台的接线口是同一份标准接口 —— 桥的主机侧
`amba_pv::amba_pv_ace_master_socket<512>`(**AMBA-PV ACE**)。

### 方式 A:本机构建(任意 Linux + SystemC)

```sh
cd model && make && sh regress.sh      # 回归 21/21(~数秒)
```

仅需:SystemC 2.3.4(Accellera)+ g++ ≥ 9。

### 方式 B:集成到 ARM Fast Models

**依赖**

- Fast Models **≥ 11.31**(含 SystemC Export 与 AMBA-PV);
- SystemC 2.3.4(Accellera);
- g++ 9.x(对应构建档 `rel_gcc93_64`)。

**环境(三件套缺一不可)**

```sh
export PVLIB_HOME=<FastModels 安装目录>
export MAXCORE_HOME=$PVLIB_HOME          # simgen 位于 $MAXCORE_HOME/bin
export SYSTEMC_HOME=<SystemC 安装目录>
export PATH=<gcc-9 所在目录>:$PATH
```

**步骤**

1. **构建承载平台**(或并入贵方既有平台工程):

   ```sh
   cd <平台工程>/Build_<CPU>
   make rel_gcc93_64       # 目标名 = {dbg,rel}_gcc{93,103,123}_64,与所用 gcc 档对应
   ```

2. **接入本模型**:在平台 SystemC 工程中例化本模型顶层;桥的主机侧
   `amba_pv_ace_master_socket<512>` 经 Fast Models **自带的官方桥**
   `$PVLIB_HOME/examples/SystemCExport/Bridges/AMBAPVACE2PVBus.lisa`
   接到平台 PVBus 互连;线侧(MAC / PCS / RoCEv2 对端)由本模型内部闭环,不占平台资源。

3. **运行**:`./<平台可执行>`

> 实测参考环境:Fast Models 11.31.28 + SystemC 2.3.4 + gcc 9.5 ⇒ 目标档 `rel_gcc93_64`。
> 平台库生成(simgen 解析 + 编译)耗时与机器性能强相关(单核机器可达小时量级)。

### 方式 C:集成到 Synopsys VDK

**依赖**

- VDK **≥ 2024.03**(`vsmake` = Virtualizer Studio 的**命令行无头版**,随安装提供);
- 许可:检出 `Virtualizer-Elite-Tool`(或对应档位)。

**步骤(全命令行,无需 GUI)**

1. **作为 TLM 工程供入**:新建/复用 TLM Model 工程,把 `model/` 源码挂入(工程描述文件 `.tlmc`);

2. **命令行构建 + 测试**:

   ```sh
   cd <TLM 工程目录>
   $VDK/SLS/linux/virtualizerstudio/vsmake --package . --test \
        --workdir <临时工作区> --testdir <测试输出>
   ```

   产出:`./vsmake/output/<类型>/<名>`(LIBRARY / UNITTEST 包);测试结果:
   `./vsmake/testresults/<名>.log`;成败看**退出码**(0 = 过);

3. **平台集成与打包**:在 VDK 平台工程中例化本模型,桥的 ACE 主机侧接互连的
   **ACE slave 口**(BCA `tlm2_amba_ace`,同为 AMBA-PV);打包平台:

   ```sh
   vsmake --package <平台工程> --config <配置名>    # ★ 多打包配置时必须 --config
   ```

> 提示:无头模式**首次**启动需初始化 Eclipse 工作区(视机器,~10 分钟量级);**复用同一
> `--workdir`** 可显著加速后续构建。

### 两平台对照

| | **Fast Models** | **VDK** |
|---|---|---|
| 模型形态 | LISA/SystemC Export 平台 + SystemC 组件 | TLM 工程(`.tlmc`)⇒ `vsmake` 打包/测试 |
| 平台侧接线 | 官方桥 `AMBAPVACE2PVBus` → PVBus | 互连 `ACE_S_x` 口(BCA) |
| 构建命令 | `make rel_gcc93_64` | `vsmake --package . --test` |
| 版本建议 | FM ≥ 11.31 | VDK ≥ 2024.03 |

## 版本

本仓库按版本发布(快照式):

| 版本 | 说明 |
|---|---|
| **1.0** | 初始发布:全栈模型 + 21/21 回归 |
| 1.1+ | 后续回归完成后依次追加 |

## 文档

- **用户指南**:[`docs/user_guide.md`](docs/user_guide.md)(filelist、依赖、编译、例化、集成、FAQ)
- 其余设计/接口文档按需提供。

## 许可与联系

- 本仓库内容为交付物,使用与分发请遵循与提供方的约定;
- 联系:(待填)。
