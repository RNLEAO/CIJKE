# 负压风扇 V3 台架兼容说明

## 基线与范围

- 基线：`feature/v3.2-four-inductance-upgrade` 的 `53e7947`。
- 验证硬件：主板、核心板、IPS114 屏幕、P33/Q2 负压风扇。
- 本阶段不依赖电感、编码器、行走电机或环岛状态。
- 未修改 `element.c`、`inductance4.c`、`motion_runtime.c`、`out_control.c` 和 `control_four.c`。

## 固定参数

- PWM：`PWMB_CH3_P33`，17 kHz。
- 占空比：30%，V3 PWM 标度下为 3000/10000。
- 输出时间：5 ms 状态机的 100 个 PREPARE 周期加 100 个 HOLD 周期，约 1 秒。
- 每次触发后冷却 30 秒，并要求手动 RESET 后才能再次布防。

## 台架操作

在第 17 页 `NP BENCH` 中，用 P73 选择项目、P72执行操作：

1. `EN`
2. `ARM`
3. `FIRE`
4. 状态显示 `WAIT` 时等待冷却结束。
5. 选择 `RESET` 并按 P72；成功后状态显示 `READY`。
6. 再次执行 `ARM`、`FIRE`。

一次触发结束后 `ARM` 自动变为 `OFF`，属于防止重复启动的安全行为。

## 已完成验证

- Keil C251 完整重建：0 Error(s), 0 Warning(s)。
- 风扇能够按固定参数启动并自动停止。
- 屏幕、主控工作正常。
- 风扇声音正常、无异味，Q2和线路无明显升温。

## 后续整车阶段

`negative_pressure.c/.h` 中的P33驱动、状态机、冷却和锁定逻辑继续保留。
`negative_pressure_menu.c/.h` 和屏幕手动触发属于台架验收入口；接入电感触发后应关闭或改成只读诊断页。
