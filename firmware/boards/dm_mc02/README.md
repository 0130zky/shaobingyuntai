# 达妙 DM-MC02 / STM32H723 哨兵双轴固件

当前工作平台为已确认的 **DM-MC02（STM32H723VGT6）**。
`Sentry` 预设已切换至本板；F407 兼容配置另命名为 `SentryF407`。
折叠/Pitch 控制、四端点标定和 USB 命令共用 `modules/folding_gimbal/`，不会引入夹爪或发射机构逻辑。

## 接口

| 功能 | MC02 接口 | 实现 |
|---|---|---|
| 两台 DM4310 | CAN1（PD0 RX / PD1 TX） | FDCAN1，经典 CAN 1Mbps，8字节帧，关闭 FD/BRS |
| DT7/DR16 接收机 | 板载 SBUS 接口（经反相电路接 UART5 RX / PD2） | 100000 baud，18字节DBUS，8数据位、偶校验、1停止位；UART本身不再反相 |
| 标定/遥测 | 板载 USB | USB OTG_HS 控制器的 FS 模式，CDC虚拟串口 |
| 标定参数 | Bank1 sector7，0x080E0000–0x080FFFFF | 独占128KiB，32字节 Flashword 写入 |

SBUS接口的电平/引脚来自达妙厂商《DM-MC-Board02 电机开发板使用说明书》，公开镜像：
[厂商说明书](https://community.st.com/ysqtg83639/attachments/ysqtg83639/stm32-mcu-products-forum/279593/1/%E8%BE%BE%E5%A6%99%E7%A7%91%E6%8A%80DM-MC-Board02%E7%94%B5%E6%9C%BA%E5%BC%80%E5%8F%91%E6%9D%BF%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6V1.0.pdf)。
接收机插板载 SBUS 插座，不沿用旧 F407 的 USART3 接线。当前接收协议已改为18字节DT7/DR16 DBUS，原25字节SBUS接收机不再适用。

板级初始化沿用本机 `dmzhuajia` 工程的 HSE24MHz、CPU480MHz、FDCAN内核80MHz及USB初始化；相关供应商版权头保留。
PC13/PC14/PC15 会拉高以打开板载两路电源和5V，电机使能仍由控制逻辑独立管理。
D-cache 保持关闭，I-cache 开启；串口使用单字节中断接收，不使用DMA，避免DTCM/DMA及缓存一致性问题。

## 构建与操作

在 `firmware/` 执行：

```sh
cmake --preset Sentry
cmake --build --preset Sentry
python3 tests/test_sentry.py
python3 tests/test_mc02_transport.py
python3 tests/check_template.py build/SentryH7
```

产物为 `build/SentryH7/sentry_dm_mc02.elf/.hex/.bin`，与原F407产物分离。
手动烧录可执行 `bash tools/flash_sentry_h7.sh`，默认不整片擦除，保留独占标定扇区。
脚本使用 `stm32h7x.cfg`，不要使用原F407的 `tools/flash.sh`。
当前已完成正常固件烧录、写入校验、复位、DT7真实接收与两轴CAN反馈验证。四端点随程序Flash加载。USB枚举及负载机构运动尚未验证。

[四端点标定和控制操作](../../robots/sentry/README.md)不变。
配置仍位于 `robots/sentry/gimbal_config.h`。正常构建已设 `SENTRY_HARDWARE_READY=1`，四个实测编码作为硬限位及默认标定编入固件。
开机必须先观察S1上/中位、有效禁用反馈及摇杆回中，S1下位才允许使能；观察版本可显式设为0。
原F407的Flash标定记录不能直接搬到H723，迁移后需要重新手动采集四端点。

H723参数提交区独占最后一个Flashword，头尾提交值和数据校验共同判断完整性。
模拟测试覆盖每个写入单元、每个字节位置的中断写入；真实掉电/ECC行为尚未在硬件上测试。
改变电机ID或MIT量程会使已存记录失效。电机零点改变也必须重新标定。

FDCAN停机时撤销尚未发送的控制帧，避免总线恢复后重放旧位置命令；接收FIFO丢帧会令反馈失效。
通信中断时无法保证禁用帧送达，需核对电机自身通信超时功能；固件不会自动修改驱动器参数。
