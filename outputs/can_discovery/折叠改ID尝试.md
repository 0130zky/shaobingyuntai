# 折叠电机改 ID 尝试（未生效）

目标：控制 ID 1 改为 2，反馈 ID 0x11 保留。
执行前用失能帧确认 ID1 的回复来自0x11，未收到其他反馈ID。
按厂商SDK，在标准帧0x7FF发送 `01 00 55 08 02 00 00 00`。
发送后ID2失能探测无回复，ID1仍返回首字节01的状态帧，参数读取也无回复。
因此未判定修改成功，未发送保存命令，也未改变正常控制固件的电机配置。
完整命令前后计数见 `id_change_before.json`、`id_change_attempt.json`。

协议依据：[达妙官方SDK](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)。
