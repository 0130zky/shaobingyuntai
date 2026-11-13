# MC02云台：S2顺序折叠与展开

| 操作 | 行为 |
|---|---|
| S1上/中 | 总失能，两轴禁用 |
| S1下、S2上 | 两轴禁用 |
| S1下、S2中 | 保持Pitch，先展开折叠轴；实际到位后再调Pitch到34370 |
| S1下、S2下 | Pitch先到31401，折叠轴折叠到位，再调Pitch到29763 |
| 右杆上下 | 展开流程完成后独立调整Pitch |
| 左杆 | 不控制折叠 |

开机或故障恢复先让S1上位（或S2上位），右摇杆回中，再允许动作。失能会取消当前流程。
同一S2档保持不会重复触发已完成的动作；换相反档按对应顺序执行。

## 当前参数

折叠预备Pitch为用户指定的31401；折叠轴实际到位后最终Pitch目标29763；展开水平目标34370。直接使用原值，不换算偏移。
折叠轴顺序端点仍为展开38800、折叠45628；正常序列目标保留64计数内缩，即38864和45564。
两轴到位允许96计数误差，低速稳定100ms后进入下一步。Pitch准备/调平最多20秒。

按用户要求，已取消两轴实际编码超出历史标定范围导致的失能/拒绝使能，以及最终位置命令按历史端点绝对限幅。
Pitch独立摇杆也不再使用历史端点限位；仅保留16位协议的0～65535编码范围。
历史端点用于确定顺序动作目标和USB标定配置，不再作为实际位置的停机边界。
S1/S2失能、遥控失联、反馈过期、周期异常、驱动故障/过温、发送失败、堵转及阶段超时保护保留。
保持Pitch时偏差超过128计数，暂停折叠轴；持续20秒停机。

## 接口与构建

DM-MC02 / STM32H723，CAN1 PD0/PD1，1Mbps经典CAN。
折叠控制/反馈ID=1/0x11，Pitch=2/0。DT7板载接口UART5 PD2，100k 8E1。
板载蜂鸣器PB15/TIM12_CH2，启动响两下，每下100ms；后台执行，不阻塞控制任务。
每次上电/复位都会响，表示程序启动；编程成功仍以OpenOCD校验为准。
USB CDC每100ms输出一行`SENTRY`状态，其中`fold_raw`/`pitch_raw`是两轴原始编码。
VS Code上位机见[tools/vscode-gimbal-encoder](../../tools/vscode-gimbal-encoder/README.md)，
串口只读显示编码值，不与`shot_monitor`同时运行；同一面板可调用
[tools/sentry_build_flash.sh](../../tools/sentry_build_flash.sh)编译并烧录。

```sh
cd firmware
cmake --preset Sentry
cmake --build --preset Sentry
python3 tests/test_z_linkage.py
python3 tests/test_sentry.py
python3 tests/test_mc02_transport.py
python3 tests/check_template.py build/SentryH7
```

产物build/SentryH7/sentry_dm_mc02.elf/.bin/.hex。
当前部署证据在outputs/three_stage_fold/，包含编程校验复位、固件哈希和运行状态。
已模拟验证旧范围之外的反馈可使能并保持原位，不会因位置命令限幅发生目标跳变；实机动作尚未自动测试。

## 状态与标定

z_mode：0失能、1准备折叠Pitch、2正在折叠、3展开完成、4折叠完成、5正在展开、6展开后Pitch调平、7保持、8折叠轴到位后最终Pitch动作。
s1、s2均为上1、中3、下2。fold_pitch_target和open_pitch_target直接报告已加载的两种Pitch目标。
故障：1遥控、2周期、3反馈、4驱动状态/温度/使能超时、6发送、7堵转、9右杆未回中、10阶段超时、11配置/输入无效；原位置越界故障5不再产生。
USB支持STATUS、CAL BEGIN、CAL FOLD A/B、CAL PITCH A/B、CAL SAVE、CAL CANCEL、RUN。
USB重新标定仍需要失能、新鲜静止反馈和完整有效配置。

参数集中在robots/sentry/gimbal_config.h：SENTRY_SEQ_FOLD_PITCH_RAW=29763，SENTRY_SEQ_OPEN_PITCH_RAW=34370；二者为动作目标。SENTRY_PITCH_A/B_RAW保留为历史标定配置。

## 三步折叠修正

现场故障7时，Fold38889仍在展开端，Pitch31401已顶住机械止挡，最终目标29763尚不可达。
按用户确认采用31401作预备角，仅在实际Fold达到折叠端附近后才允许Pitch继续至29763。
模拟测试验证提前进入最终Pitch阶段被拦截，并验证折叠时预备/折叠轴/最终Pitch三阶段。
展开按用户最终指令维持原流程：保持Pitch不动，展开折叠轴，再调Pitch34370；SENTRY_SEQ_UNFOLD_ALLOWED=1。
S1/S2控制、位置限位取消、堵转/通信/过温保护和启动两声提示保持。
当前已烧录并校验、复位，运行读回预备31401/折叠29763/展开34370、展开启用。未由调试器执行机构运动。

## CAN Bus-Off恢复

现场多次出现两轴反馈全无、CAN1 CCCR.INIT=1、PSR.BO=1，导致armed=0而不能使能。
现增加Board_CanRecover，每个控制周期在解析遥控后检查；仅在有效S1上/中或S2上失能档恢复。
恢复前撤销排队TX并使旧反馈失效，再清除INIT请求硬件恢复；至少间隔250ms重试。
使能档与遥控失效时不会自动恢复，反馈恢复后仍按原门控重新允许使能。
本轮已手动恢复现场总线、验证两轴反馈回来，并将自动恢复修复烧录、校验、复位。
部署证据outputs/can_busoff_recovery/；重启后fault=0、armed=1、S1/S2上位，两轴保持失能。
主机测试覆盖使能档禁止恢复、有效失能放行、过期遥控拒绝恢复、旧反馈失效和重试间隔。
