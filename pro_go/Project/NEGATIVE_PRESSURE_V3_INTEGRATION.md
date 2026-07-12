# V3 负压自动控制集成说明

## 基线与用途

- 队长主程序基线：`origin/master` 的 `8d77cab`
- 集成分支：`feature/negative-pressure-v3-integrate`
- 正式 Keil 工程：`mdk/CIJIANKE_EM.uvproj`
- 当前目标：完成第四阶段第一层的软件接入，不进行真实风扇输出

截至 2026-07-13，`origin/master` 仍停留在 `8d77cab`。远程
`feature/v3.2-four-inductance-upgrade` 上未合并的 WIP 提交不属于本次
正式基线，待队长合入 `master` 后再单独同步。

第四阶段旧版测试代码保留在 `C:\Users\lucky\Desktop\CIJKE` 的
`feature/negative-pressure-auto-control` 分支。V3 集成目录不复制旧版
`main.c`、`isr.c`、菜单或控制文件，只迁移负压模块和必要接点。

## 本次接入内容

- `code/negative_pressure.c/.h`：非阻塞状态机、锁定、冷却和故障保护。
- `code/negative_pressure_menu.c/.h`：`NP AUTO` 与 `NP OUTPUT` 两个诊断页。
- `code/element.c/.h`：用 `Element4State` 提供语义化环岛请求。
- `user/isr.c`：在 5 ms 周期中更新请求并运行状态机。
- `code/menu.h` 与 `user/main.c`：统一 18 页枚举和循环翻页。
- `code/control_four.c`：恢复阶段二按键映射并移除 P07 输出冲突。

## 当前安全边界

以下两个宏均为 `0`：

```c
#define NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE            0
#define NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE  0
```

因此当前构建不会初始化 P33，也不会生成负压模块到 `pwm_set_duty()` 的
调用。状态机可以在屏幕上完成 `OFF -> PREP -> HOLD -> RELEASE -> OFF`
的软件验收，但 `REAL` 输出始终为 0。

- P07 保留给无线串口 RTS，负压模块不再使用 P07。
- 后续启用 PWM 时只使用 V3 原生 `pwm_set_duty()` 的 `0..10000` 尺度。
- 环岛请求仅在 `RING_ENTER`、`RING_HOLD`、`RING_EXIT` 为真。
- 即使以后开启物理编译门，自动环岛模式仍不允许真实输出；当前物理
  输出路径只允许停车状态下的菜单手动测试。

## 按键与页面

- P70 短按：上一页；长按：RUN/STOP。
- P71 短按：下一页。
- P72：增加；P75+P72：减少。
- P73 短按：选择项目；长按：SAVE。
- P70+P73 长按：CLEAN。
- 原有页面为 0 至 15；新增 16 `NP AUTO`、17 `NP OUTPUT`。

## 已完成验证

- Keil Rebuild：`0 Error(s), 0 Warning(s)`。
- 程序大小：`data=9.4`、`edata+hdata=2238`、`xdata=2520`、
  `const=7411`、`code=53680`。
- `CIJIANKE_EM.uvproj` 的 86 个文件引用全部存在。
- P07 无负压占用，链接调用图中负压对象无物理 PWM 驱动依赖。

## 尚未完成

1. 18 页与全部按键的新版实机验收。
2. 屏幕状态机时序验收。
3. P33 空载 17 kHz、5% PWM 验收。
4. 真实风扇 5% 至 10%、100 ms 至 300 ms 低功率验收。

完成前不得把两个物理输出宏改为 `1`，也不得把本分支合并到 `master`。
