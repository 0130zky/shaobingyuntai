# MC02云台：S2并行折叠/展开

| 操作 | 行为 |
|---|---|
| S1上/中 | 总失能，两轴禁用 |
| S1下、S2上 | 两轴禁用 |
| S1下、S2中 | Fold朝展开端、Pitch朝34370，同一控制周期开始运动 |
| S1下、S2下 | Fold朝折叠端、Pitch朝29763，同一控制周期开始运动 |
| 右杆上下 | 展开完成后按杆量线性调整Pitch速度 |
| 左杆 | 不控制折叠，自动动作由S2选择 |

两轴独立规划、独立限速和加速度；先到的一轴保持，另一轴继续。两轴实际反馈均在到位容差内、低速稳定100ms，才报告动作完成。
不再串行等待Pitch预备或折叠轴到位后再动Pitch，31401预备角已不参与控制。
换S2方向同时改变两轴目标；失能取消当前动作。启动与故障恢复先观察到失能档、右杆回中，再允许动作。

## 参数位置

robots/sentry/gimbal_config.h：

| 参数 | 用途 |
|---|---|
| SENTRY_FOLD_SPEED | 折叠轴速度上限设置 |
| SENTRY_PITCH_SPEED | Pitch速度上限设置 |
| SENTRY_FOLD_ACCEL | 仅折叠轴加速度 |
| SENTRY_PITCH_ACCEL | 仅Pitch加速度，包括右杆调角 |
| SENTRY_STICK_DEADZONE | 摇杆中心死区，默认0.05（5%） |
| SENTRY_SEQ_FOLD_PITCH_RAW | 折叠Pitch目标29763 |
| SENTRY_SEQ_OPEN_PITCH_RAW | 展开Pitch目标34370 |
| SENTRY_Z_LEVEL_TOL_COUNTS | Pitch到位容差96计数 |
| SENTRY_SEQ_FOLD_TOL_COUNTS | Fold到位容差96计数 |
| SENTRY_Z_SETTLE_MS | 双轴稳定时间100ms |
| SENTRY_Z_ALIGN_TIMEOUT_MS | 整体动作超时20000ms |

折叠轴端点仍为展开38800、折叠45628；动作目标保留64计数内缩，即38864/45564。
Pitch直接采用上述原始目标，不作偏移换算。

右杆中心死区外重新映射为连续0～100%杆量，速度按该比例给定；松杆按Pitch自己的加速度减速，不积累未来位置目标。
满杆采用速度设置与现有跟随/制动包络中较小的值，各杆量缩放同一包络，因此小杆不会提前饱和成满杆。
SPEED数值并不保证实际机械速度等于该数值，实际还受跟随误差、加速度和驱动能力限制。
修改后先保存文件，再编译烧录；旧共用SENTRY_ACCEL已替换为两轴独立参数。

## 保护和已知诊断

按用户先前要求，运行中的历史编码范围越界失能及最终命令历史范围限幅已取消。保留16位协议范围和相对跟随约束。
S1/S2失能、遥控失联、反馈过期、周期异常、驱动错误/过温、发送失败、堵转、动作超时保护保留。
CAN Bus-Off只在有效失能档恢复，先撤销TX和废弃旧反馈，重试间隔至少250ms。

本轮按用户要求暂停电机上电坐标问题排查，未改电机参数/零点或加入自动偏移换算。
同姿态试验记录在outputs/coordinate_reset_check：只复位MCU编码基本不变，电机重新上电后Fold增加16383、Pitch减少1647计数。
这项问题未宣称已解决，本次并行、线性摇杆和独立加速度修改与其分开记录。

## 构建、状态与部署

```sh
cd firmware
cmake --preset Sentry
cmake --build --preset Sentry
python3 tests/test_z_linkage.py
python3 tests/test_manual_pitch.py
python3 tests/test_automatic_speed.py
python3 tests/test_sentry.py
python3 tests/test_mc02_transport.py
python3 tests/check_template.py build/SentryH7
```

产物build/SentryH7/sentry_dm_mc02.elf/.bin/.hex。当前部署证据outputs/parallel_motion/。
模拟验证覆盖双向同拍启动、运动重叠、单轴先到位、双轴完成门控、反向、失能、超时、旧范围外保持、25/50/100%杆量速度比例，以及单轴加速度变更不影响另一轴。
编程/校验/复位已验证，运行parallel_motion=1、prep_pitch_target=0、故障0，两轴失能且反馈刷新；实际机构手感与动作效果未由调试器测试。

z_mode：0失能、2并行折叠中、3展开完成、4折叠完成、5并行展开中、7保持；1/6/8为旧版本兼容保留值。
fold_pitch_target/open_pitch_target报告两种Pitch目标，parallel_motion=1表示并行版本。
板载蜂鸣器PB15/TIM12_CH2正常启动响两下，每声100ms，后台执行；每次上电/复位都会响。

## S1重新使能保持原位

S1上/中位失能后再拨下位，两轴以当时反馈编码作为保持目标，自动动作请求清零，不把S2既有档位当作新的动作。
S1已下位时再切换S2才启动动作；S2上位失能后切中/下仍是明确的新动作请求。
同时改变S1和S2时先按重新使能保持处理，需后续再切一次S2。
这能保留失能前/失能期间手动改变的Pitch姿态，不自动拉回34370或29763。
部署证据outputs/reenable_hold/。主机测试覆盖S2中/下固定时反复切S1、Pitch手动位置保留、S1/S2同时切换，以及S1已使能时的S2命令。
现场S2固定中位的一轮S1上→下→上测试，62个有效使能样本：Pitch反馈/目标34392，Fold反馈38857、目标38858，轨迹速度0，自动请求0、故障0。
记录开始时另有已锁存的遥控故障1，回有效失能档后清除；本轮有效使能阶段未再出现。不能据此认定更早每次切换的全部原因都相同。

## 自动动作速度修正

自动轨迹原先把反馈附近的跟随窗口当成终点，反复减速并清零速度；高加速度时更容易发生。
现在按真实动作终点计算制动距离，跟随窗口仅限制位置超前，不作为停车点。
自动与满杆遥控共用 min(SPEED, sqrt(2*ACCEL*ERROR_RAD)) 速度包络及位置跟随积分；两轴加速度仍各自独立。
四个参数保留用户当前的1200，未更改目标编码。对应包络约10.95 rad/s，增大SPEED不会突破此包络。
test_automatic_speed.py覆盖固定3 rad/s反馈下800/800、1200/800、1200/1200三组参数不再反复清零、自动与满杆一致、调速、到位、反向和阻塞反馈限幅。
这些是软件轨迹测试，实际机构速度需现场验证。
