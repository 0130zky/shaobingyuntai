# 当前哨兵主控：达妙 DM-MC02 / STM32H723

默认哨兵预设 `Sentry` 已迁移到 H723，见 [MC02 说明](boards/dm_mc02/README.md)。
产物 `build/SentryH7/sentry_dm_mc02.elf/.hex/.bin`。下面保留原 F407 示例文档，其接线和烧录命令不适用于 MC02。

# 机器人工程模板（STM32F407）

已增加机器人装配层：默认 `example` 保留当前发射机构与 Pitch 云台，`minimal` 提供无电机输出的起步工程。
模板使用方法见 [TEMPLATE.md](TEMPLATE.md)。原有目录名和产物名暂时保留，以兼容工具。

> 下文是原发射机构使用说明；当前构建还包含云台，硬件配置以 `robots/example/` 和实际源文件为准。

# 四电机发射机构

当前固件只服务于 **三个 M3508 摩擦轮 + 一个 DM4310 拨盘**。旧 M3508 拨盘实现已删除。
云台转轴、IMU、底盘通信等整车逻辑已移入 `legacy/robot/`，不参与当前任何构建。

## 硬件与接口

| 设备 | 总线/接收接口 | 反馈 ID | 控制 |
| --- | --- | --- | --- |
| 12 点、4 点、8 点摩擦轮 | CAN1，1 Mbps | 0x201、0x202、0x203 | 0x200 的前三槽；第四槽恒为零 |
| DM4310 拨盘 | CAN1，1 Mbps | 0x12 | 0x02 MIT 纯力矩帧，板端位置/速度闭环 |
| SBUS 遥控接收机 | USART3 | 25 字节 SBUS | CH6 总使能、CH8 轮速、CH5 模式、CH7 触发 |
| 裁判模块 | USART6，115200 8N1 | 裁判协议 v1.7 | 原始弹速与接收计数 |
| 遥测/相位校准 | USART1，460800 8N1 | JustFloat | 25 通道主帧、24 通道诊断、校准应答 |

拨盘外部减速比 3:1，8 槽，每格 45°；默认相位 −1°。DM 量程为 ±12.5 rad、±30 rad/s、±10 N·m。
这些值以及原 PI 增益、输出限幅、同步补偿和保护阈值均保留，实际硬件参数仍需实物匹配。

遥控操作：CH6 >1500 使能；CH8 逆时针死区停轮，中位目标 5560 rpm，最大 10000 rpm。
CH5 低位单步进弹、中位按住连发、高位单步退弹。连发等待到位并停留 150ms。
失联/重新使能后先松开 CH7 再按，忙碌或卡弹期间的正向请求不积压。

## 构建配置

在本文件所在目录执行，需要 CMake、Ninja 和 Arm GNU 工具链：

```sh
cmake --preset Shooter
cmake --build --preset Shooter
```

| 配置 | 控制来源 | 电机输出 |
| --- | --- | --- |
| `Shooter` | 遥控；USB 控制帧和 USB 零点设置被拒绝 | 正常运行 |
| `Bench` | 遥控在线时优先遥控；无遥控时接受有效 USB 台架指令 | 正常运行 |
| `Calibration` | 只观测和校准 | 三轮电流恒为零；拨盘发零力矩，支持手转观测 |

每种配置使用独立的 `build/<配置>/` 目录，产物为 `robot_template.elf/.hex/.bin`。
`DM4310`、`Debug` 是 `Shooter` 的兼容名称，`DM4310Calib` 是 `Calibration` 的兼容名称；
`Release` 使用同一套硬件与遥控模式。旧 RM3508 拨盘缓存会在配置阶段报错，不会回退构建。
不要把 `Calibration` 当作正常运行固件。

系统只创建三个任务：发射控制、遥测、裁判接收。USB 独立初始化；启动不等待 IMU，CAN2 不启动。
应用任务定义在 `Application/app_tasks.c`，CubeMX 的 `freertos.c` 仅在保留区调用应用入口；
`.ioc` 不再重复定义应用线程。板级生成代码保留供应商目录结构。

## 时间与控制

控制任务保持每轮完成后相对延时 3ms，**3ms 是名义间隔，不是假定的实际周期**。
同一拍的 PID、参考斜坡、堵转和越界持续计时使用实测周期；链路、停留和停稳判定使用实际时钟差。
轮速斜坡对应原来的 25000 rpm/s 加速、50000 rpm/s 减速。

发现不合法周期或超过 20ms 的调度空档时，本拍撤销使能、丢弃本拍动作，拨盘当拍断输出；
三轮摩擦轮则在 20~30ms 的单次抖动里沿用上一拍输出，不降速再重新爬坡，更大的空档或连续 3 拍异常仍回零停机。
空档不会积成积分或一次大幅运动。下一有效拍重新对齐；遥控需松开再触发。该策略有离线回归，尚未上板验证。
CAN 发送不忙等、不在每拍重开总线。DM 连续发送失败保护按实际 15ms 计时。

## 上位机

Windows、macOS、Linux 共用协议、时基和分析库；Windows 串口使用 pyserial，POSIX 串口保留标准库实现。
先建立 Python 环境并安装依赖：

```sh
python -m venv .venv
# Windows
.venv/Scripts/python -m pip install -r tools/requirements.txt
# macOS/Linux 使用 .venv/bin/python
```

后台记录器是日常使用的唯一串口所有者。例如 Windows：

```sh
.venv/Scripts/python tools/shot_monitor.py start --port COM3
.venv/Scripts/python tools/shot_monitor.py status
.venv/Scripts/python tools/shot_monitor.py events --lines 20
.venv/Scripts/python tools/shot_monitor.py stop
```

macOS/Linux 换用对应的虚拟环境 Python 与串口路径。后台停止采用会话专属请求，正常关闭文件和串口；
重复启动受目录锁保护，PID 身份校验包含进程创建时间，避免误认复用的 PID。

VS Code 扩展 `tools/vscode-shot-scope/` 提供实时图、逐发回放、导出和相位调节。
哨兵折叠云台使用只读扩展 `tools/vscode-gimbal-encoder/`：直接解析 USB CDC 的
`SENTRY` 状态行，显示折叠与 Pitch 两台 DM4310 的原始编码、角度、目标与反馈状态。
该扩展独占串口，不能和 `shot_monitor` 同时打开同一端口；说明见
[tools/vscode-gimbal-encoder/README.md](tools/vscode-gimbal-encoder/README.md)。
它读取后台 CSV，不打开串口；关闭面板只结束分析进程。Tk 界面 `tools/shot_scope.py --monitor`
同样读取后台记录，直接连接模式也使用统一解码器。有限采集使用 `tools/capture_friction.py`。

原 CSV 列名、25 通道和诊断布局保留。主采样时间是控制拍时间；重复收到同一控制拍不会制造新时间，
也不会刷新后台的“最后新采样时间”。不同设备反馈仍保留各自接收时间，软件快照不表示物理同步采样。
原始记录、逐轮测量和离线评估继续可用；历史 CSV 可回放。

## 离线验证

无需串口即可执行固件核心回归：

```sh
python tests/run_shooter_tests.py
```

安装上位机依赖后执行全部 Python/主机 C 测试：

```sh
.venv/Scripts/python -X utf8 -m unittest discover -s tests -p "test_*.py"
python tests/check_firmware.py build/Shooter
```

回归覆盖真实 PID 与电机编码、反馈展开、变周期、堵转/越界持续时间、异常周期撤输出、
三种控制模式、数据快照、CAN 错误路径、协议重同步、逐发测量、CSV 回放、后台生命周期和扩展启动/关闭。
Windows 使用虚拟串口回环测试后台；仅 POSIX PTY 专用测试在 Windows 跳过。

本轮验证不包含烧录、实机转动、真实串口通信或机械响应验证。
架构与修改位置见 [ARCHITECTURE](ARCHITECTURE.md)，历史试验记录见 [docs/history](docs/history)。
