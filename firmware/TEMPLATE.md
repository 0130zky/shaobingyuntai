> 当前哨兵工程已切换至达妙 MC02/H723，使用 `Sentry` 预设。见 [MC02 说明](boards/dm_mc02/README.md)。本文的 Minimal/example/Shooter 均为原F407兼容配置。

# 使用机器人模板

## 编译

在本目录使用 CMake、Ninja 和完整 Arm GNU 工具链：

```sh
cmake --preset Minimal
cmake --build --preset Minimal
python3 tests/check_template.py build/Minimal

cmake --preset Shooter
cmake --build --preset Shooter
python3 tests/run_shooter_tests.py
python3 tests/check_template.py build/Shooter
```

复制工程后若旧 build 缓存仍引用原路径，先用 `cmake --fresh --preset <名称>` 重新配置，再构建。
所有配置仍输出 `build/<配置>/robot_template.elf/.hex/.bin`，以兼容原脚本。
`Minimal` 是独立构建目录，不要用原 flash.sh 的 Shooter 默认参数烧录它。
本次仅完成离线构建与回归，没有烧录和实机验证。

## 文件职责

| 位置 | 职责 |
| --- | --- |
| `Application/robot.h` | 启动、控制、后台、USB/UART/CAN 统一装配接口 |
| `robots/minimal/robot.c` | 新机器人的最小起点：USB、LED、SBUS 接收，无电机输出 |
| `robots/example/robot.c` | 当前机器人的模块装配、控制与反馈分发 |
| `robots/example/remote_mapping.c` | 当前机器人的遥控使能与轮速映射 |
| `robots/example/gimbal_config.h` | 当前云台的 ID、零点、方向、限位、增益 |
| `robots/example/dial_config.h` | 当前拨盘机构参数 |
| `robots/example/friction_sync.h` | 当前摩擦轮同步配置与辅助计算 |
| `Components/Controller/` | 无 HAL/RTOS 依赖的控制算法 |
| `Components/Devices/` | 设备协议驱动；旧配置路径保留兼容转发头 |
| `Bsp/`、`Core/`、`Drivers/` | 当前 F407 板型接口与供应商代码 |

## 新增机器人

1. 复制 `robots/minimal/` 到 `robots/my_robot/`。
2. 在 CMakeLists.txt 的 ROBOT 合法列表加入 `my_robot`，为它增加显式源文件、include 和模块链接分支。
3. 在 CMakePresets.json 增加独立配置，设置 `ROBOT=my_robot` 与独立 build 目录。
4. 在 robot.c 实现设备初始化、反馈分发、控制拍和后台逻辑；需要的设备驱动与功能模块显式加入构建。
5. 为新机器人建立自己的硬件参数与遥控映射，不复用 robot_template 的机械零点、ID、限位和增益。

当前模板不是任意模块开关的组合系统；每台机器人通过显式 CMake 装配选择依赖。
默认 robot_template 的 Shooter / Bench / Calibration 模式保留；云台沿用其现有独立控制逻辑，
不能将 Calibration 模式解读成整机所有执行器的无输出保证。

## 控制与通信约定

- 初始化反馈状态后再绑定路由、开启 CAN 接收，控制任务随后启动。
- 控制周期名义为 3ms 相对延时，实际 dt 由 DWT 测量；各机构负责异常周期与失联保护。
- ISR 不执行闭环、不忙等、不推进离散动作；控制任务持有控制状态，后台读取快照。
- 当前 BSP 仅启动 CAN1。CAN2 钩子已预留，使用前仍需配置外设、过滤器、启动与通知。
- 最小配置沿用 F407 外设初始化及 SBUS 接收基础；裁判协议源文件仍保留，但没有裁判任务。
- 现有遥控端点/通道常量部分仍在 RC_task.h，摩擦轮 ID 常量仍在 app_RM_Motor.h；后续扩展其他接收机或电机组时需继续下沉到实例配置。

先通过离线行为与构建边界验证，再针对新硬件验证总线、反馈、失联、零点、方向与限位。
