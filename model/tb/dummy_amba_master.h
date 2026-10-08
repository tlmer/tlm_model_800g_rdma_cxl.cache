//============================================================================
// dummy_amba_master.h — AMBA-PV **占位主端**(只为满足 socket 绑定;不发事务)  [2026-10-05 建]
//
// ⚠ 起因(实踩):设备 v1.0 加 `reg_s`(A7)后,`amba_pv::amba_pv_slave_socket` 内部
//   `sc_port` 的绑定策略 = **ONE_OR_MORE**(tlm_utils simple_target_socket 默认)⇒
//   **不用它的 TB 不绑就 elaboration 报 E109**(`reg_s_port_0`)**✗
//   —— 且**旧二进制不会报**(链的是 v1.0 之前的设备)⇒ 本轮重链才现形 = 典型的
//   「陈旧二进制读数」陷阱(教训入册)✗✓
// ⇒ 纪律:凡例化 `DeviceTlm` 而不走寄存器 socket 的 TB,**一律绑本占位件**(E109 的 socket 版)✓
//   (真事务路径 = `tb_device_amba_pv`;本件被 `m(*this)` 绑上即满足契约,不发任何事务)✓
//============================================================================
#ifndef DUMMY_AMBA_MASTER_H
#define DUMMY_AMBA_MASTER_H

#include <systemc.h>
#include <amba_pv.h>

struct DummyAmbaMaster: sc_core::sc_module,
                        amba_pv::amba_pv_master_base {
    amba_pv::amba_pv_master_socket<32> m;

    SC_HAS_PROCESS(DummyAmbaMaster);
    DummyAmbaMaster(sc_core::sc_module_name nm):
        sc_module(nm), amba_pv::amba_pv_master_base("dummy_mb"), m("m") {
        m(*this);                    // 反向路径接口绑到自己(照 VDK dma 例子 `dma.cpp:53`)✓
    }
    // ⛔ 无行为:只为 socket 绑定;⛔ 不发任何事务 ✗
};

#endif // DUMMY_AMBA_MASTER_H
