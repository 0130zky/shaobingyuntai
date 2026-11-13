# robot_template 发弹监测

在VS Code编辑器标签页中显示三摩擦轮上位机。无需单独Python GUI，不使用外部网络或CDN。

安装本地VSIX后，在工程窗口执行命令 **robot_template: 打开发弹监测**，或点击状态栏“发弹监测”。也可以运行工程中的 `tools/shot_scope_vscode.sh`（Windows 可直接使用 VS Code 命令）。

功能：实时三轮转速和电流叠加；按掉速起点对齐的逐发三轮变化、掉速/恢复统计；CSV回放和导出；初始偏移调节及板端确认。

扩展在 Windows 使用工程 `.venv/Scripts/python.exe`，macOS/Linux 使用 `.venv/bin/python`，运行`tools/shot_scope_vscode.py`。后台`shot_monitor`独占串口并记录CSV；扩展通过本地子进程JSON消息读取记录。关闭面板仅结束分析进程，不停止后台记录。

偏移调节需先关闭CH6，收到板端确认后重新使能。回放模式禁止下发调节。界面显示采样新鲜度，串口连接状态不等同于正在接收数据。

0.2.0新增控制诊断：三轮反馈年龄、实际同步补偿和PID时序叠加曲线，实际PID参考及逐拍限幅计数。需要对应固件与新版后台记录；旧CSV仍可回放，缺失诊断显示为空。诊断约20ms更新，期间保持最近观测；窗口峰值与拍数由控制任务逐拍统计。
