#!/bin/sh
TBS="tb_bridge_loopback tb_device_regs tb_device_e2e tb_driver_flow tb_mac_loopback tb_mac_nic_dock tb_mac_pcs_link tb_full_chain tb_dual_mac tb_dual_stack tb_device_amba_pv tb_roce_peer_m1 tb_roce_engine_m1 tb_roce_engine_m2 tb_roce_engine_m3 tb_cache_m1 tb_cache_m2 tb_cm_m1 tb_cm_m2 tb_cm_m3"
for tb in $TBS; do
  (cd <内部路径> && timeout 600 ./$tb) > /tmp/cmp4_$tb.txt 2>&1
  r4=$?
  (cd <内部路径> && timeout 600 ./$tb) > /tmp/cmp8_$tb.txt 2>&1
  r8=$?
  if diff -q /tmp/cmp4_$tb.txt /tmp/cmp8_$tb.txt > /dev/null 2>&1; then
    echo "IDENTICAL $tb (rc $r4/$r8)"
  else
    n=$(diff /tmp/cmp4_$tb.txt /tmp/cmp8_$tb.txt | grep -c '^[<>]')
    echo "DIFF $tb (rc $r4/$r8; $n 行)"
  fi
done
echo CMP_ALL_DONE
