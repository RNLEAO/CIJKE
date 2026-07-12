# 21届此间客电磁组（飞檐走壁）

这是从 `258-test` 独立迁移出的 STC32G12K128 四路电感差速车工程。原工程未被覆盖，本目录可以单独打开、编译和烧录。

## 工程入口

- Keil 工程：`Project\mdk\seekfree.uvproj`
- 编译目标：`STC32G12K128_CIJIANKE_EM`
- 输出文件：`Project\mdk\out_file\CIJIANKE_EM.hex`
- 逐飞库版本：`Libraries\doc\version.txt`，V3.2.4

## 四路电感接线

| 逻辑位置 | ADC 引脚 | 代码索引 |
| --- | --- | --- |
| L，左外侧 | P06 | `INDUCTANCE4_L` |
| LM，左内侧 | P00 | `INDUCTANCE4_LM` |
| RM，右内侧 | P01 | `INDUCTANCE4_RM` |
| R，右外侧 | P05 | `INDUCTANCE4_R` |

`P10` 不参与本工程的初始化、采样和元素判断。`MID` 变量仅为兼容旧菜单保留，运行时固定为 0，不能接入控制公式。

## 电机与编码器

| 功能 | 引脚/通道 |
| --- | --- |
| 左轮 PWM 正反桥臂 | P60 / P66 |
| 右轮 PWM 正反桥臂 | P62 / P64 |
| 左编码器计数 | CTIM0，P34 |
| 左编码器方向 | P35 |
| 右编码器计数 | CTIM3，P04 |
| 右编码器方向 | P53 |

工程保持原来的单脉冲计数方式，通过方向输入修正编码器正负号。

## 外设接线

- IPS114：SPI2，SCK=P25，MOSI=P23，RST=P20，DC=P21，CS=P22，BLK=P27。
- IMU660RA：软件 SPI，SCK=P40，MOSI=P41，MISO=P42，CS=P43。
- 无线串口：UART4，模块 TX 接 P02，模块 RX 接 P03，RTS 接 P07，115200 baud。

## 元素处理

普通循迹使用四路归一化值计算偏差：

```text
error = A * (L - R) + B * (LM - RM)
```

直角和环岛由 `Project\code\element.c` 的独立状态机处理：

- 直角：`TRACK -> RIGHT_ANGLE_CONFIRM -> RIGHT_ANGLE_TURN -> RIGHT_ANGLE_RECOVER`。
- 触发后采用差速硬转，一侧反转、一侧前进；陀螺仪角度、重新捕线和超时共同决定退出。
- 环岛：`TRACK -> RING_CONFIRM -> RING_ENTER -> RING_HOLD -> RING_EXIT -> RING_RECOVER`。
- 环岛结合四路电感形态、编码器累计距离和 IMU 角度，不依赖第五路中间电感。
- 所有元素都有确认计数、恢复阶段和超时保护，避免单次 ADC 抖动直接触发。

## 首次上车流程

1. 先架空车轮上电，确认菜单、IPS114、IMU 和四路 ADC 均能刷新。
2. 进入 `I4 CAL` 页面，执行四路电感地面/赛道标定并保存。
3. 确认四路状态均为 `VALID`，再允许电机运行。
4. 低速调整普通循迹的 A/B/P 参数。
5. 分别低速测试左直角、右直角和环岛，再调整元素阈值、硬转速度、角度与距离参数。
6. 最后提高基础速度，不要在未标定时直接闭环运行。

## 编译说明

工程采用 C251 `XSMALL` 模型。为兼容 V3.2.4 和旧应用层：

- 大缓冲和格式化临时数据显式放入 XDATA。
- EDATA 上电清零范围设为完整 4KB。
- 硬件栈保留 1KB；本工程无 RTOS，函数大数组不占硬件栈。
- 官方 `zf_device_config.lib` 只提供 IMU/DL1B 常量表，按 LARGE 模型预编译。链接器仅对该无 ABI 风险的模型提示关闭 L14。

最后一次完整重编结果：

```text
Program Size: data=8.4 edata+hdata=4062 xdata=3104 const=5670 code=56849
0 Error(s), 0 Warning(s)
```

## 验证边界

已完成源码检查、全量 C251 编译、链接和 HEX 生成。尚未在实车上烧录验证，因此电感阈值、轮速 PID、直角角度、环岛距离和电机正反方向仍需按实际机械结构现场校准。
