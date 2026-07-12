# 第四阶段负压自动控制测试工程

## 工程定位

- 工程入口：`NEGATIVE_PRESSURE_STAGE4_TEST.uvproj`
- Keil 目标：`STC32G12K128_NEG_PRESS_STAGE4_TEST`
- HEX 名称：`NEG_PRESS_STAGE4_TEST.hex`
- 输出目录：`Out_File/NEG_PRESS_STAGE4_TEST/`
- 代码基线：旧版 CIJKE 的第四阶段检查点 `205fcfe`

该工程只用于第四阶段负压自动控制测试，不是队长 V3.2.4 主工程，也不能作为最终整车工程提交。工程直接引用当前测试源码，没有复制第二套源码；因此所有第四阶段修改只有一个来源，不会出现两个工程中的代码逐渐不一致。

## 当前测试范围

1. 第一层：纯软件状态机，正在验收。
   - 检查 `OFF -> PREPARE -> HOLD -> RELEASE -> OFF`。
   - 屏幕菜单模拟请求和故障状态。
   - 真实 P33 PWM 输出保持关闭。
2. 第二层：P33 空载 PWM，尚未开始。
   - 不接风扇。
   - 验证 17 kHz、5% PWM 和自动关闭时间。
3. 第三层：真实风扇低 PWM，尚未开始。
   - 仅在前两层通过后进行。
   - 限制为 5% 至 10%、100 ms 至 300 ms。

## 第四阶段相关文件

- `negative_pressure.c/.h`：状态机、保护条件、输出门控。
- `menu.c/.h`：`NP Auto` 和 `NP Output` 测试页面。
- `main.c`：测试页面注册和按键事件处理。
- `isr.c`：5 ms 周期中的请求更新与状态机调用。
- `control.c/.h`：按键事件和整车运行状态接口。
- `element.c/.h`：旧版 `cir_flag` 测试触发来源。

其余电机、屏幕、IMU 和逐飞库文件是上述测试代码能够编译运行所需的旧工程依赖，不代表这些模块属于负压功能本身。

## 安全边界

- `NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE` 必须保持为 `0`，直到明确进入第二层空载测量。
- `NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE` 必须保持为 `0`，直到明确进入第二层空载测量。
- 上电默认 OFF；不得直接使用 100% 占空比。
- P07 映射只属于旧版阶段测试。新版主工程中 P07 是无线串口 RTS，禁止原样迁移。
- 当前旧版使用 `pwm_duty()` 和旧 PWM 尺度。迁入 V3.2.4 时必须改为原生 `pwm_set_duty()`。
- 当前 `cir_flag` 触发只用于旧版测试。迁入 V3.2.4 时必须适配 `Element4State`。

## 后续迁移原则

第四阶段测试完成后，从队长最新 `origin/master` 新建集成分支。只迁移负压模块及必要接点，不覆盖新版的 `main.c`、`isr.c`、`menu_four.c` 或 `control_four.c`，并重新完成编译、菜单、按键、无线串口和分层硬件验收。
