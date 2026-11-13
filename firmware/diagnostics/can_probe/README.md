# MC02 RAM CAN 诊断

此程序独立于正常双轴固件，链接地址为 AXI SRAM `0x24000000`，数据/栈在 DTCM。
用于 DAPLink/SWD 读取 CAN 地址、反馈 ID 与电机参数，不写开发板 Flash。
默认只发送 `0x7FF` 上的 `0x33` 参数读取及 `0xCC` 状态读取。无电机使能、位置、速度、力矩、清零或参数写入路径。

`PROBE_DISABLE_READ=ON` 为单独的兼容配置：只在明确允许电机松力后使用，向各标准 ID 发送 `FF FF FF FF FF FF FF FD` 失能帧触发旧固件反馈。
这一兼容配置只扫 1Mbps。正常只读配置检查多个波特率，其他活跃设备共享总线时不宜直接使用。

板载电源打开后等待1.8秒再扫描，避免在电机启动完成前漏掉低 ID。扫描完成后 `state=3`，停止发送。
上电反馈 CAN ID 为0也能被记录，不要求反馈高半字节为1。

构建（在仓库根目录）：

```sh
cmake -S firmware/diagnostics/can_probe -B firmware/build/CanProbeRAM -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/firmware/cmake/gcc-arm-h723.cmake" -DCMAKE_BUILD_TYPE=Debug
cmake --build firmware/build/CanProbeRAM
```

加载前确认芯片及机构安全状态；中断当前 MCU 程序会改变其原有实时控制。
通过 OpenOCD `load_image` 加载该 ELF 并 `verify_image`，确认所有载入区间均为RAM。
从 RAM 向量表第二个字读取入口，或用 `arm-none-eabi-nm` 找当前 ELF 的 `Reset_Handler`。
设置 VTOR=`0x24000000` 后从这个 **Reset_Handler** 恢复执行，不能从向量表基地址执行，也不能套用旧构建入口。
不要对这个 ELF 使用 `program` 或 Flash 烧录脚本。

`can_probe` 为调试快照，包含状态、波特率、计数、寄存器错误统计和最多16组回复。
`probe_stop=1` 可停止后续发送。诊断不修改 Flash，重启后仍是板上原程序。

完整参数 RID：7主机/反馈ID，8电机ID，9通信超时，10控制模式，21/22/23位置/速度/力矩量程，35CAN速率。
原始寄存器值和反馈帧应一并保存；仅根据请求候选地址猜测电机ID是不可靠的。
旧版电机可能匹配控制地址低8位，从多个相隔256的地址返回同一组反馈；这些不是多台电机。
物理轴归属需通过单独连接核实；同一总线不应让两轴保持相同控制ID。

## 受控ID修改工具（单独构建）

`ID_EDITOR=ON` 使用 `id_editor.c`，不会在启动时写任何电机参数。
仅支持明确授权的 ESC_ID 1→2 修改；通过SWD邮箱逐条触发，不自动执行后续步骤。
`ID_EDITOR_FEEDBACK_ID` 必须匹配已单独连接的物理电机。收到其他反馈ID会禁止写入。
CMD3写入前需旧地址的实际状态反馈；CMD6保存前需新地址的实际状态反馈。
保存按厂商协议发送4字节 `02 00 AA 01`，并接收4字节保存应答；不能把没有保存应答判成已持久化。
修改后还需验证旧地址无响应、新地址寄存器读回及电机断电重启后的地址保持。

当前现场结果见仓库 `outputs/can_discovery/CAN实测结果.md`。
