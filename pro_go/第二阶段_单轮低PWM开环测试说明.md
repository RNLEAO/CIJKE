# 第二阶段：单轮低 PWM 开环测试

## 本版本边界

- 仅开放无线串口命令 `MTEST L`、`MTEST R`、`MTEST STOP`。
- 每次只允许一个车轮正转。
- 目标 PWM 固定为 300，最长运行 500 ms，随后自动停止。
- PWM 从 0 按每个 5 ms 最大增加 25 的斜率上升，不会瞬间跳到 300。
- 普通 `RUN`、P70 长按启动和元素控制仍然锁定。
- `ELEMENTS=OFF`，直角和环岛不会参与本阶段测试。
- IMU660RB 必须为 `STATE=OK`，并且 `PROTECT=NONE`，否则拒绝启动。
- 启动后若被测编码器连续无有效脉冲、方向为负或出现异常大脉冲，程序会提前停止并锁定保护。

## 烧录文件

```text
C:\Users\70731\Desktop\21届此间客电磁组（飞檐走壁）\Project\mdk\out_file\CIJIANKE_EM.hex
```

烧录后先关闭驱动板电源，将车辆架空，确认两个车轮均不会接触桌面或车架。

## 串口设置

```text
波特率：115200
数据位：8
停止位：1
校验位：无
流控：无
HEX 发送：关闭
自动发送：关闭
```

## 串口显示模式

本版本默认关闭连续诊断刷屏。上电后只显示一行 `BOOT`，之后根据命令输出：

```text
STATUS       单次显示4行精简状态
STATUS FULL  单次显示完整诊断帧
STREAM ON    每5秒显示一次完整诊断帧
STREAM OFF   关闭连续显示，恢复静默
```

单轮 MTEST 结束后会自动显示一份固定结果，包含测试轮、正转方向、累计脉冲、峰值、结果、保护状态和最终 PWM，不需要抢着看运行中的瞬时帧。

## 上电静态检查

1. 先保持驱动板电源关闭。
2. IMU660RB 平放、文字朝上。
3. 核心板上电后的前 3 秒不要移动车辆。
4. 必须看到：

```text
BOOT: IMU660RB=OK AXIS=Z MOTOR=LOCKED ELEMENTS=OFF
```

5. 发送 `STATUS`，必须同时满足：

```text
STATUS: IMU=OK MOTOR=LOCKED PROTECT=NONE ELEMENTS=OFF STREAM=OFF
MTEST: SIDE=NONE RESULT=IDLE PULSES=0 PEAK=0
OUTPUT: LPWM=0 RPWM=0
```

这里的 `RUNLOCK=1` 表示普通行驶仍被锁定，属于本阶段的正确状态。

## 左轮测试

1. 车辆继续架空，手离开两个车轮。
2. 打开驱动板电源。
3. 串口只发送一次：

```text
MTEST L
```

4. 应立即收到：

```text
OK: MTEST L START PWM=300 TIME=500MS AUTO_STOP
```

5. 左轮应按车辆前进方向短暂转动，右轮必须保持不动。
6. 最迟约 500 ms 后会自动显示固定结果，例如：

```text
MTEST RESULT: SIDE=L DIR=FORWARD RESULT=DONE PULSES=... PEAK=...
MTEST SAFE: PROTECT=NONE LPWM=0 RPWM=0
ACTION: TEST COMPLETE; TURN DRIVER OFF
```

7. 如需再次确认，发送一次 `STATUS`，精简状态应包含：

```text
MTEST: SIDE=L REMAIN_MS=0 PWM=300 PULSES=... PEAK=... RESULT=DONE
OUTPUT: LPWM=0 RPWM=0 LAPPLIED=0 RAPPLIED=0
```

其中 `PULSES` 是本次 500 ms 测试累计收到的编码器脉冲，`PEAK` 是单个 5 ms 采样周期内的最大脉冲数。正常转动时两者都应大于 0。

8. 立即关闭驱动板电源，触摸检查电机只能是常温或无明显升温。

## 右轮测试

只有左轮测试完全通过后才进行：

1. 车辆继续架空。
2. 打开驱动板电源。
3. 串口只发送一次：

```text
MTEST R
```

4. 应收到启动提示，右轮按车辆前进方向短暂转动，左轮保持不动。
5. 最迟约 500 ms 后必须自动停止并显示 `RESULT=DONE`。
6. 立即关闭驱动板电源并检查右电机温度。

## 立即停止条件

出现以下任一情况，立刻关闭驱动板电源，不要再次发送 MTEST：

- 非测试侧车轮转动。
- 车轮向后转。
- 500 ms 后仍持续转动。
- 电机、驱动板明显发热、异响、焦味或机械卡滞。
- 串口出现 `L_STALL`、`R_STALL`、`L_DIR`、`R_DIR`、`ENC_SPIKE`、`IMU` 或其他保护原因。

可随时发送：

```text
MTEST STOP
```

程序会立即将两路 PWM 清零。保护触发后，在排除原因且驱动板电源关闭的情况下发送 `CLEAR`，不要直接重试。

## 本阶段验收数据

每个车轮分别保留以下串口行：

```text
MTEST:
ENC:
OUTPUT:
SAFE:
```

同时人工记录：测试轮、实际旋转方向、另一侧是否静止、是否自动停止、是否有异响和是否升温。
