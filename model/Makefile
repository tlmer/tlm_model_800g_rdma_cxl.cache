# Makefile — tlm_model 模型骨架(SystemC)  [2026-10-04 建 / 同日改交付结构]
# 用法:make(默认构建 tb_bridge_loopback 并跑);make libcxl2ace_tlm.a(交付形态样板)
# ⚠ SYSTEMC_HOME 可用环境变量覆盖(交付件不绑死本机路径 ✓;客户用 VDK/SystemC 时指向其安装)
SYSTEMC_HOME ?= /opt/tools/systemc2.34
# ★ 主机侧落地库 = VDK 自带 AMBA-PV(header-only;见桥规格 §1.2 实读)✓
AMBA_PV_HOME ?= /opt/AMBA-PV   # 指向你的 AMBA-PV(VDK 自带 header-only:$(VDK)/SLS/linux/IP_common/AMBA_TLM2_BL/AMBA-PV)
# 参考件(Xilinx libsystemctlm-soc,MIT):ACE 编码互印证 + pin 级桥参考 ✓

CXX      := g++
CXXFLAGS := -std=c++14 -O1 -g -Wall \
            -I$(SYSTEMC_HOME)/include -Ibridge -Idevice -Imac -Ipcs -Ipeer -Itop -Itb -I../sw -I$(AMBA_PV_HOME)/include
LDFLAGS  := -L$(SYSTEMC_HOME)/lib-linux64 -lsystemc -Wl,-rpath,$(SYSTEMC_HOME)/lib-linux64

BRIDGE_OBJS := bridge/cxl2ace_bridge.o
TB_OBJS     := tb/tb_bridge_loopback.o
DEVICE_OBJS := device/device_tlm.o
TBR_OBJS    := tb/tb_device_regs.o
TBF_OBJS    := tb/tb_driver_flow.o
TBW_OBJS    := tb/tb_800g_bpwm.o        # ★ L4-3 背压/水位/丢弃(2026-10-09)
TBO_OBJS    := tb/tb_800g_ostd.o        # ★ L4-4 多笔在飞(桥侧口径)+槽号(2026-10-09)
MAC_OBJS    := mac/mac_tlm.o
PCS_OBJS    := pcs/pcs_link_tlm.o
TBFULL_OBJS := tb/tb_full_chain.o
TB2MAC_OBJS := tb/tb_dual_mac.o
TB2STK_OBJS := tb/tb_dual_stack.o
TBAPV_OBJS  := tb/tb_device_amba_pv.o
PEER_OBJS   := peer/roce_peer_tlm.o
TBPEER_OBJS := tb/tb_roce_peer_m1.o

all: run

# ★ 交付形态(2026-10-04 用户指正后立):`.h` 接口 + `.cpp` 实现 ⇒ 编静态库交付客户 ✓
#   (客户侧只 include 头、链我们的 .a;改实现不动客户代码)
#   ★ 两个模块各一库(桥可单用:P1.8 与真 CCI 联调;设备与桥合用 ⇒ E2E)✓
libcxl2ace_tlm.a: $(BRIDGE_OBJS)
	ar rcs $@ $^
libnic_tlm.a: $(DEVICE_OBJS)
	ar rcs $@ $^
libmac_tlm.a: $(MAC_OBJS)
	ar rcs $@ $^
libpcs_tlm.a: $(PCS_OBJS)
	ar rcs $@ $^
libs: libcxl2ace_tlm.a libnic_tlm.a libmac_tlm.a libpcs_tlm.a
	@echo "== 交付库 =="; ls -l libcxl2ace_tlm.a libnic_tlm.a libmac_tlm.a libpcs_tlm.a

# ★ P5.4 回归(一条命令跑全部 TB + 摘要 + 退出码;⚠ 脚本无执行位 ⇒ 用 sh 跑)✓
regress:
	sh ./regress.sh

tb_bridge_loopback: $(BRIDGE_OBJS) $(TB_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# ★ P2 设备侧 TLM(v1.0:寄存器面 + 门铃 + 中断)⇒ 自测 ✓
tb_device_regs: $(DEVICE_OBJS) $(TBR_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# ★ P2 v1.0 端到端:设备 + 桥 + ACE 主机内存(引脚直连)✓
tb_device_e2e: $(DEVICE_OBJS) $(BRIDGE_OBJS) tb/tb_device_e2e.o
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_device_e2e.o: tb/tb_device_e2e.cpp device/device_tlm.h bridge/cxl2ace_bridge.h bridge/cxl_shim_types.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ P4:驱动流程自测(序列 = sw/nic_hw.h;原语 = 模型直连)✓
tb_driver_flow: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(TBF_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_driver_flow.o: tb/tb_driver_flow.cpp device/device_tlm.h bridge/cxl2ace_bridge.h ../sw/nic_regs.h ../sw/nic_hw.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ M3/L4-3G 背压·水位·丢弃(正控+负控;判据照 md/L4_背压与多笔在飞_TB设计.md)✓
tb_800g_bpwm: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(TBW_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_800g_bpwm.o: tb/tb_800g_bpwm.cpp device/device_tlm.h bridge/cxl2ace_bridge.h ../sw/nic_regs.h ../sw/nic_hw.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ M3/L4-4G 多笔在飞(★ 桥侧口径)+ 槽号(单笔下界对照)✓
tb_800g_ostd: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(TBO_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_800g_ostd.o: tb/tb_800g_ostd.cpp device/device_tlm.h bridge/cxl2ace_bridge.h ../sw/nic_regs.h ../sw/nic_hw.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ MAC TLM(帧级收发 + 自环;规格 规格文档 §7)⇒ 自测 ✓
tb_mac_loopback: $(MAC_OBJS) tb/tb_mac_loopback.o
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_mac_loopback.o: tb/tb_mac_loopback.cpp mac/mac_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ rdma800 M2:线侧 lane 档位(800G 双 lane 拼接 / 400G 仅 lane0;契约观测+负控)⇒ 自测 ✓
TBL8_OBJS := tb/tb_mac_lane800.o
tb_mac_lane800: $(MAC_OBJS) $(TBL8_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_mac_lane800.o: tb/tb_mac_lane800.cpp mac/mac_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
mac/mac_tlm.o: mac/mac_tlm.cpp mac/mac_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ NIC(设备 TLM)↔ MAC TLM 对接(帧级透传)⇒ 对接 TB ✓
tb_mac_nic_dock: $(DEVICE_OBJS) $(MAC_OBJS) tb/tb_mac_nic_dock.o
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_mac_nic_dock.o: tb/tb_mac_nic_dock.cpp device/device_tlm.h mac/mac_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ 四模型整机串联(e2e 基座 + 真 MAC/PCS link 接线;撤哑绑定)⇒ 参考装配 TB ✓
tb_full_chain: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(PCS_OBJS) $(TBFULL_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_full_chain.o: tb/tb_full_chain.cpp device/device_tlm.h bridge/cxl2ace_bridge.h bridge/cxl_shim_types.h mac/mac_tlm.h pcs/pcs_link_tlm.h ../sw/nic_regs.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ 最小 RoCEv2 对端(真包:WRITE 请求 -> ACK)⇒ 自测 TB ✓
tb_roce_peer_m1: $(MAC_OBJS) $(PEER_OBJS) $(TBPEER_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# ★ (引擎真包侧):WQE ⇒ 设备引擎自己组包上线 ⇒ ACK ⇒ CQE ⇒ 中断 ✓
TBE1_OBJS := tb/tb_roce_engine_m1.o
tb_roce_engine_m1: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(PEER_OBJS) $(TBE1_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_roce_engine_m1.o: tb/tb_roce_engine_m1.cpp top/stack_top.h device/device_tlm.h mac/mac_tlm.h peer/roce_peer_tlm.h ../sw/nic_regs.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ CM 建链 M1:两端会合 ⇒ 自动配 QP ⇒ 一笔真 RDMA(sw/nic_cm.c 首个 TB 消费者)✓
TBCM_OBJS := tb/tb_cm_m1.o ../sw/nic_cm.o
tb_cm_m1: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(PEER_OBJS) $(TBCM_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_cm_m1.o: tb/tb_cm_m1.cpp top/stack_top.h device/device_tlm.h mac/mac_tlm.h peer/roce_peer_tlm.h ../sw/nic_regs.h ../sw/nic_hw.h ../sw/nic_cm.h tb/cm_pipe.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
../sw/nic_cm.o: ../sw/nic_cm.c ../sw/nic_cm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ M2:priv 往返 + 三段(INIT→RTR→RTS)+ 拒绝/超时分支 ✓
tb_cm_m2: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(PEER_OBJS) tb/tb_cm_m2.o ../sw/nic_cm.o
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_cm_m2.o: tb/tb_cm_m2.cpp top/stack_top.h device/device_tlm.h mac/mac_tlm.h peer/roce_peer_tlm.h ../sw/nic_regs.h ../sw/nic_hw.h ../sw/nic_cm.h tb/cm_pipe.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ M3:验收场景(CM 建链 ⇒ B5 最小集工作负载)✓
tb_cm_m3: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(PEER_OBJS) tb/tb_cm_m3.o ../sw/nic_cm.o
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_cm_m3.o: tb/tb_cm_m3.cpp top/stack_top.h device/device_tlm.h mac/mac_tlm.h peer/roce_peer_tlm.h ../sw/nic_regs.h ../sw/nic_hw.h ../sw/nic_cm.h tb/cm_pipe.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ 超时重发 + 2.1j 回写 + RNR_RETRY 参数化 ✓
TBE3_OBJS := tb/tb_roce_engine_m3.o
tb_roce_engine_m3: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(PEER_OBJS) $(TBE3_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_roce_engine_m3.o: tb/tb_roce_engine_m3.cpp top/stack_top.h device/device_tlm.h mac/mac_tlm.h peer/roce_peer_tlm.h ../sw/nic_regs.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ 写命中⇒M + snoop 状态化应答 + 脏数据转发 ✓
TBC2_OBJS := tb/tb_cache_m2.o
tb_cache_m2: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(TBC2_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_cache_m2.o: tb/tb_cache_m2.cpp top/stack_top.h device/device_tlm.h bridge/cxl2ace_bridge.h mac/mac_tlm.h ../sw/nic_regs.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ 设备缓存行表(I/S/E/M;默认关;命中不落内存)✓
TBC1_OBJS := tb/tb_cache_m1.o
tb_cache_m1: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(TBC1_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_cache_m1.o: tb/tb_cache_m1.cpp top/stack_top.h device/device_tlm.h bridge/cxl2ace_bridge.h mac/mac_tlm.h ../sw/nic_regs.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ (双向):READ/SEND + PSN 乱序被拒 + RNR-NAK 重发同包 ✓
TBE2_OBJS := tb/tb_roce_engine_m2.o
tb_roce_engine_m2: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(PEER_OBJS) $(TBE2_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_roce_engine_m2.o: tb/tb_roce_engine_m2.cpp top/stack_top.h device/device_tlm.h mac/mac_tlm.h peer/roce_peer_tlm.h ../sw/nic_regs.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
tb/tb_roce_peer_m1.o: tb/tb_roce_peer_m1.cpp top/dual_mac_top.h mac/mac_tlm.h peer/roce_peer_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
peer/roce_peer_tlm.o: peer/roce_peer_tlm.cpp peer/roce_peer_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ (A7):设备寄存器面 AMBA-PV 从口 ⇒ 自测 TB ✓
tb_device_amba_pv: $(DEVICE_OBJS) $(TBAPV_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_device_amba_pv.o: tb/tb_device_amba_pv.cpp device/device_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ 双实例 top(`top/dual_mac_top.h`:2× MacTlm + LINK 交叉 + ena glue)⇒ 互测 TB ✓
tb_dual_mac: $(MAC_OBJS) $(TB2MAC_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_dual_mac.o: tb/tb_dual_mac.cpp top/dual_mac_top.h mac/mac_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ 双实例**完整栈**互测(`top/stack_top.h`:每侧 = 设备+桥+MAC;用户令)dock TB ✓
tb_dual_stack: $(DEVICE_OBJS) $(BRIDGE_OBJS) $(MAC_OBJS) $(TB2STK_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_dual_stack.o: tb/tb_dual_stack.cpp top/stack_top.h device/device_tlm.h bridge/cxl2ace_bridge.h mac/mac_tlm.h ../sw/nic_regs.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ MAC TLM ⇄ PCS/FEC link TLM 对接(免 glue 直连)⇒ 对接 TB ✓
tb_mac_pcs_link: $(MAC_OBJS) $(PCS_OBJS) tb/tb_mac_pcs_link.o
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_mac_pcs_link.o: tb/tb_mac_pcs_link.cpp mac/mac_tlm.h pcs/pcs_link_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ★ M3:PCS×Lane 双档台(800G/400G 对照 + 速率档 + 误码负控)✓
tb_mac_pcs_lane: $(MAC_OBJS) $(PCS_OBJS) tb/tb_mac_pcs_lane.o
	$(CXX) -o $@ $^ $(LDFLAGS)
tb/tb_mac_pcs_lane.o: tb/tb_mac_pcs_lane.cpp mac/mac_tlm.h pcs/pcs_link_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
pcs/pcs_link_tlm.o: pcs/pcs_link_tlm.cpp pcs/pcs_link_tlm.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

device/device_tlm.o: device/device_tlm.cpp device/device_tlm.h bridge/cxl_shim_types.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
tb/tb_device_regs.o: tb/tb_device_regs.cpp device/device_tlm.h bridge/cxl_shim_types.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

bridge/cxl2ace_bridge.o: bridge/cxl2ace_bridge.cpp bridge/cxl2ace_bridge.h bridge/cxl_shim_types.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
tb/tb_bridge_loopback.o: tb/tb_bridge_loopback.cpp bridge/cxl2ace_bridge.h bridge/cxl_shim_types.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

run: tb_bridge_loopback tb_device_regs tb_device_e2e tb_driver_flow tb_mac_loopback tb_mac_lane800 tb_mac_nic_dock tb_mac_pcs_link tb_mac_pcs_lane tb_full_chain tb_dual_mac tb_dual_stack tb_device_amba_pv tb_roce_peer_m1 tb_roce_engine_m1 tb_roce_engine_m2 tb_roce_engine_m3 tb_cache_m1 tb_cache_m2 tb_cm_m1 tb_cm_m2 tb_cm_m3 tb_800g_bpwm tb_800g_ostd
	./tb_bridge_loopback
	./tb_device_regs
	./tb_device_e2e
	./tb_driver_flow
	./tb_800g_bpwm
	./tb_800g_ostd
	./tb_mac_loopback
	./tb_mac_lane800
	./tb_mac_nic_dock
	./tb_mac_pcs_link
	./tb_mac_pcs_lane
	./tb_full_chain
	./tb_dual_mac
	./tb_dual_stack
	./tb_device_amba_pv
	./tb_roce_peer_m1
	./tb_roce_engine_m1
	./tb_roce_engine_m2
	./tb_roce_engine_m3
	./tb_cache_m1
	./tb_cache_m2
	./tb_cm_m1
	./tb_cm_m2
	./tb_cm_m3

clean:
	rm -f tb_bridge_loopback tb_device_regs tb_device_e2e tb_driver_flow tb_800g_bpwm tb_800g_ostd tb_mac_loopback tb_mac_nic_dock tb_mac_pcs_link tb_mac_pcs_lane tb_full_chain tb_dual_mac tb_dual_stack tb_device_amba_pv tb_roce_peer_m1 tb_roce_engine_m1 tb_roce_engine_m2 tb_roce_engine_m3 tb_cache_m1 tb_cache_m2 tb_cm_m1 tb_cm_m2 tb_cm_m3 \
	      tb/tb_device_e2e.o tb/tb_driver_flow.o tb/tb_800g_bpwm.o tb/tb_800g_ostd.o tb/tb_mac_loopback.o tb/tb_mac_nic_dock.o tb/tb_mac_pcs_link.o tb/tb_mac_pcs_lane.o tb/tb_full_chain.o tb/tb_dual_mac.o tb/tb_dual_stack.o tb/tb_device_amba_pv.o tb/tb_roce_peer_m1.o tb/tb_roce_engine_m1.o tb/tb_roce_engine_m2.o tb/tb_cache_m1.o tb/tb_cache_m2.o tb/tb_roce_engine_m3.o tb/tb_cm_m1.o tb/tb_cm_m2.o tb/tb_cm_m3.o ../sw/nic_cm.o $(PEER_OBJS) $(BRIDGE_OBJS) $(PCS_OBJS) $(TB_OBJS) $(DEVICE_OBJS) $(TBR_OBJS) $(TBF_OBJS) $(TBW_OBJS) $(TBO_OBJS) $(MAC_OBJS) \
	      libcxl2ace_tlm.a libnic_tlm.a libmac_tlm.a libpcs_tlm.a
.PHONY: all run clean libs regress
