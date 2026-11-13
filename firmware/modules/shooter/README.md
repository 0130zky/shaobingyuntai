# 发射机构模块

当前只支持三个 M3508 摩擦轮和一个 DM4310 拨盘。旧 M3508 拨盘实现、型号选择宏和构建分支已删除。

完整文件指引、数据流、时钟和依赖规则见 [当前架构](../../ARCHITECTURE.md)，构建、遥控与上位机入口见 [README](../../README.md)。

控制入口是 `Application/booster_task.c::Booster_ControlStep(dt_s)`。输入、反馈、动作、闭环、发送和只读状态发布在同一任务中完成。
外部只能提交输入或复制 `shooter_status_t`；不得包含 `shooter_internal.h` 修改内部状态。

`Shooter` 为遥控正常模式，`Bench` 允许 USB 台架输入，`Calibration` 禁止三轮电流并对拨盘输出零力矩。
使用实测周期进行 PID、斜坡及堵转/越界持续计时；20ms 以上的控制空档本拍撤输出，不补动作。
任务保持相对延时 3ms，这不是实际周期保证。PI、同步、机械保护等现有参数未调优。

```sh
python tests/run_shooter_tests.py
cmake --preset Shooter
cmake --build --preset Shooter
python tests/check_firmware.py build/Shooter
```

本轮只做离线测试和编译，没有烧录或实机验证。
