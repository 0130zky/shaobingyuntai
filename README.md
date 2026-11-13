# 哨兵折叠云台固件

主控：达妙DM-MC02（STM32H723VGT6），两台DM4310用于折叠和Pitch。

- [并行控制、S1/S2操作与参数](firmware/robots/sentry/README.md)
- [MC02接口和构建](firmware/boards/dm_mc02/README.md)

S1上/中总失能；S1下位时，S2上失能、中展开、下折叠。
折叠、展开均由Pitch和折叠轴同时开始运动，各轴独立限速和加速度，双轴到位后完成。
左摇杆不参与折叠，右摇杆仅在展开完成后按杆量线性调Pitch。

在firmware内执行cmake --preset Sentry、cmake --build --preset Sentry。
部署证据见outputs/parallel_motion；模拟验证通过，现场运动效果待验证。
