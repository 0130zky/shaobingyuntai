# 模板装配层

公共入口 `Application/robot.h` 由 `robots/<所选机器人>/robot.c` 实现。
`SysInit → Robot_Init`；控制任务调用 `Robot_ControlStep`；后台任务调用机器人后台钩子；
CAN、USB 与 USART1 输入通过机器人钩子分发。回调在中断上下文时只允许有界、非阻塞工作。
`ROBOT` 决定机器人装配，`FIRMWARE_PROFILE` 决定 robot_template 的运行模式，两者分开选择。
`minimal` 不创建裁判任务，不编入发射、云台或电机驱动；板型外设和裁判接收代码仍保留。
目前是 F407 板型模板，BSP/供应商代码保留原路径；换 MCU 仍需重新适配 CubeMX、启动文件与链接脚本。

下面保留发射模块内部架构说明（其中“仅四电机”描述不再代表整套固件）。

# 当前架构

## 一套硬件，三个使用模式

固件只包含三轮 M3508 和一个 DM4310。`Shooter / Bench / Calibration` 区分输入权限与是否出力，
不再区分拨盘电机型号。旧整车代码在 `legacy/robot/` 只供查阅，恢复整车功能需要单独设计并验证。

## 谁负责什么

```text
SBUS / USB ISR → 输入快照或命令邮箱
CAN1 ISR → 电机反馈缓存（拨盘位置逐帧展开）
                    ↓
发射控制任务：输入仲裁 → 反馈采样 → 时间检查 → 动作接受/丢弃
            → 三轮控制 → 拨盘控制 → CAN 提交 → 发布只读状态
                    ↓
遥测任务：复制状态 → 编码 → UART DMA / USB
裁判任务：接收解析 → 独立裁判观测
```

只有控制任务修改目标、动作状态和 PID。中断不推进动作，显示不访问内部控制结构。
内部 `BOOSTER` 仅在 `shooter_internal.h` 中声明，外部通过 `Booster_ReadStatus()` 复制结果。
反馈接收和控制状态分开，控制执行期间新到的帧留给下一拍，不改变这一拍的测量。

## 文件指引

| 修改内容 | 文件 |
| --- | --- |
| 三个任务与调度外壳 | `Application/app_tasks.c` |
| 启动顺序与 CAN 路由绑定 | `Sys/sys.c`、`Application/app_can_router.c` |
| 控制拍顺序、CAN 输出提交 | `Application/booster_task.c` |
| 遥控通道与轮速映射 | `Application/RC_task.c` |
| 输入仲裁、触发边沿、USB 邮箱 | `modules/shooter/shooter_command.c` |
| 时间有效性与异常间隔处理 | `modules/shooter/control_time.h`、`shooter_time.c` |
| 三轮参考、同步、保护和诊断 | `modules/shooter/friction_control.c` |
| 拨盘动作、回拨、锁定 | `modules/shooter/dial_control.c` |
| 反馈接收与采样 | `modules/shooter/shooter_feedback.c` |
| 对外只读类型与发布 | `modules/shooter/shooter_status.h/.c` |
| 两种遥测编码 | `modules/shooter/shooter_telemetry.c` |
| DM 连续位置、保持闭环与力矩提交 | `Components/Devices/dial_port_dm.c` |
| 机构参数与 DM 量程 | `Components/Devices/dial_config.h` |
| 无 HAL/RTOS 的 PID 数学 | `Components/Controller/controller.h/.c` |

## 边界和约定

- `control_algorithms` 是纯 C 数学目标，不包含 STM32、DWT 或 RTOS 头文件。只保留实际使用的 PI/PID、积分限幅和输出滤波；旧模糊控制、LQR 等在历史归档。
- `shooter_core` 作为独立对象目标构建，内部头文件和设备依赖使用私有 include 路径；板级应用负责装配。
- BSP CAN 不包含业务对象，只负责 HAL 收包与帧类型/长度检查。绑定在开启接收前完成。
- 连续输入只保留最新值；离散动作即时接受或丢弃，没有动作 FIFO。失能、失联、超时和异常周期不会补发动作。
- 同一拍的 PID、斜坡与持续计时采用同一个实测 dt。初拍采用名义 3ms，之后使用两次控制入口的 DWT 周期差。
- 超过 20ms 的空档或非正、非有限 dt 不参与积分，当前拍撤拨盘输出并丢弃动作；
  三轮摩擦轮在 20~30ms 的单次抖动里保留上一拍输出，更大的空档或连续 3 拍异常才回零。
  有效拍恢复后重新对齐，遥控触发重新解锁。
- 拨盘仍保留锁定起终点、45° 网格、一次有限回拨、到位保持与越界保护。运动完成、保持和故障阈值分别处理。
- 电机帧直接有界提交，不排队积压旧电流。提交成功不等于执行器已执行，真实总线和机械表现需要上板验证。
- 快照包含控制采样时间、序号及各反馈接收时间。控制停住时数据冻结，遥测不会伪造新采样时间。
- UART/USB 发送缓冲区为静态存储，忙时跳过本次遥测；USB 编码前先检查发送是否空闲。
- 裁判观测与电机控制采样独立，裁判帧年龄在打包时计算，不拿较早的控制时间减较新的裁判时间。

## 上位机分层

| 层 | 文件 | 依赖 |
| --- | --- | --- |
| 协议 | `tools/telemetry_protocol.py` | Python 标准库，无串口/平台模块 |
| 串口 | `tools/serial_transport.py` | POSIX 标准库或 Windows pyserial |
| 进程与目录所有权 | `tools/process_support.py` | 平台锁、进程创建时间 |
| 板载时基 | `tools/board_clock.py` | 标准库，重复帧不推进时间 |
| 数值分析 | `tools/analysis_core.py` | NumPy，无 GUI、无串口 |
| 采集与保存 | `tools/shot_monitor.py` | 单串口所有者，CSV/JSONL |
| 有限采集 | `tools/capture_friction.py` | 共用协议与串口层 |
| 展示与回放 | `tools/shot_scope_vscode.py`、`shot_scope.py` | 共用时基和分析库 |

`telemetry_io.py`、`wheel_measurement.py` 保留为旧导入入口。独立评估工具保留其不同的统计口径，
例如“恢复到共同目标”与“恢复到发前基线”不会合并为一个含义不清的指标。

## 验证边界

主机回归直接链接真实纯 C PID 和电机收发实现，并模拟时间、HAL、串口或 VS Code API。
`tests/check_firmware.py` 检查最终构建源文件、目标依赖与 ELF 符号，防止旧整车或 RM 拨盘分支重新进入固件。
这些检查覆盖软件行为与构建集成，不能替代真实 DMA、CAN 拥塞、接收机和机械响应测试。
