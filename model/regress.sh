#!/bin/sh
#============================================================================
# regress.sh — tlm_model 回归:一条命令跑全部 TB(20 个)+ 紧凑摘要 + 退出码  [2026-10-04 建](计划 5.4)
#
# 用法:sh regress.sh        (或 cd model && make regress)
# ⚠ 脚本无执行位 ⇒ 用 `sh` 跑(项目惯例)✓
#
# 判据口径(**只认读数**):每个 TB 的**自带终判行**(`TB_xxx PASS`)为准;
# 本脚本只做收集/汇总,**不重新解释判据** ✓;任一 TB 失败 ⇒ 退出码 = 1 ✓
#============================================================================
cd "$(dirname "$0")" || exit 1

# ★ 2026-10-05 加(实踩教训):**防陈旧二进制读数** —— 本脚本只跑现成二进制,
#   若源码/头比二进制新,读数就是旧口径的(后曾据此报过含陈旧 device 台的读数)✗✓
if ! make -q >/dev/null 2>&1; then
	echo "[构建] 有目标过期 ⇒ 先 make(防陈旧二进制读数)✗"
	make -s || { echo "构建失败,回归中止"; exit 1; }
fi

echo "==== tlm_model 回归($(date '+%Y-%m-%d %H:%M'))===="
printf "%-20s %-6s %-8s %s\n" "TB" "结果" "用时" "自带读数行(节选)"
fail=0

run_one() {
	tb="$1"; key="$2"
	if [ ! -x "./$tb" ]; then
		printf "%-20s %-6s %-8s %s\n" "$tb" "缺" "-" "未构建?先 make"
		fail=1; return
	fi
	t0=$(date +%s%N)
	out=$(./"$tb" 2>&1)
	t1=$(date +%s%N)
	ms=$(( (t1 - t0) / 1000000 ))
	if echo "$out" | grep -q "^TB_.* PASS"; then
		res="PASS"; npass=$((npass+1))
	else
		res="FAIL"; fail=1
	fi
	read=$(echo "$out" | grep -E "$key" | head -1 | sed 's/^[ \t]*//')
	printf "%-20s %-6s %-8s %s\n" "$tb" "$res" "${ms}ms" "$read"
}

run_one tb_bridge_loopback '桥计数'
run_one tb_device_regs     '读数:用例'
run_one tb_device_e2e      '^   v (多笔在飞|槽号)'
run_one tb_driver_flow     'RQ 生产指针回读'
run_one tb_800g_bpwm       '\[读数\] o_max_rd_ostd'
run_one tb_800g_ostd       '\[读数\] 桥 o_rd_ostd_max|桥槽号'
run_one tb_mac_loopback    '环回: 逐字节一致'
run_one tb_mac_lane800     'lane1|lanemm|START 落'
run_one tb_mac_nic_dock   '帧A'
run_one tb_mac_pcs_link     '经 PCS link'
run_one tb_mac_pcs_lane     '速率档|双档'
run_one tb_full_chain       '经 PCS link 环回'
run_one tb_dual_mac         '双向同刻'
run_one tb_dual_stack       '双栈|帧路 A→B'
run_one tb_device_amba_pv   'socket 写 QP_CONF'
run_one tb_roce_peer_m1     '对端收到 WRITE 请求'
run_one tb_roce_engine_m1   '引擎:WQE ⇒ 真包 ⇒ ACK ⇒ CQE|对端收到请求包数'
run_one tb_roce_engine_m2   'READ 请求数|重发包逐字节'
run_one tb_roce_engine_m3   'retried|line_retries|重试耗尽'
run_one tb_cache_m1        '桥读增量|cch_hits|cch_fills'
run_one tb_cache_m2        'cch_wr_hit|snoop A:数据|snop'
run_one tb_cm_m1           'CM 三消息|读回对表|一笔真 RDMA|TB_CM_M1'
run_one tb_cm_m2           '三段读回|拒绝|超时|TB_CM_M2'
run_one tb_cm_m3           '工作负载|CQE 序列|清后 INTR|TB_CM_M3'

# 失败时把末段贴出来(定位用)✓
if [ "$fail" != 0 ]; then
	echo "---- 有失败:失败 TB 末 25 行 ----"
	for tb in tb_bridge_loopback tb_device_regs tb_device_e2e tb_driver_flow tb_800g_bpwm tb_800g_ostd tb_mac_loopback tb_mac_nic_dock tb_mac_pcs_link tb_mac_pcs_lane tb_full_chain tb_dual_mac tb_dual_stack tb_device_amba_pv tb_roce_peer_m1 tb_roce_engine_m1 tb_roce_engine_m2 tb_roce_engine_m3 tb_cache_m1 tb_cache_m2 tb_cm_m1 tb_cm_m2 tb_cm_m3; do
		if [ -x "./$tb" ] && ! ./"$tb" 2>&1 | grep -q "^TB_.* PASS"; then
			echo "==== $tb ===="
			./"$tb" 2>&1 | grep -E '^   (x|✗)|FAIL' | head -25
		fi
	done
	echo "回归失败 ✗"
	exit 1
fi

echo "全部 PASS ✓(${npass}/24)"
exit 0
