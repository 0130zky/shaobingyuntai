# 云台编码器 · VS Code 内嵌上位机

只读显示哨兵折叠云台两台 DM4310（折叠 / Pitch）的原始编码值。

数据源是 DM-MC02 USB CDC 的 `SENTRY` 状态行，由 `modules/folding_gimbal/sentry.c`
的 `Sentry_Background()` 每 100 ms 输出一次。串口方向只读：扩展只解析状态行，
不会通过串口使能电机、写入标定或触发动作。面板另提供编译与烧录按钮，
经 DAPLink/SWD 写入固件，见下文。

## 安装

在扩展目录打包，再回到 `firmware/` 安装：

```sh
cd tools/vscode-gimbal-encoder
npx @vscode/vsce package --no-dependencies -o ../../vscode-gimbal-encoder.vsix
cd ../..
code --install-extension vscode-gimbal-encoder.vsix
```

也可以直接在 VS Code 里按 F5 调试加载 `tools/vscode-gimbal-encoder/`。
安装后在工作区执行命令 **robot_template: 打开云台编码器**，或点击状态栏
“云台编码器”，或运行：

```sh
tools/gimbal_encoder_vscode.sh
```

扩展使用工程 `.venv` 的 Python 运行 `tools/gimbal_encoder_vscode.py`。
打开面板会自动连接第一个 USB 串口；串口下拉框可切换，刷新重新扫描。

## 显示内容

| 区域 | 内容 |
|---|---|
| 折叠轴 / Pitch 轴 | `fold_raw` / `pitch_raw` 原始编码 0–65535，以及 0–65535 位置条 |
| 输出角 | 板端 `fold_mrad` / `pitch_mrad` 换算的角度 |
| 目标 / 误差 | `*_goal` 换算的目标编码和目标角；误差 = 当前编码 − 目标编码 |
| 反馈年龄 | 板端 CAN 反馈年龄；`4294967295` 表示尚未收到该电机反馈 |
| 驱动状态 | DM4310 `ERR`：失能 / 使能 / 过压、欠压、过流、过温、通讯丢失、过载 |
| 温度 | MOS 与线圈中的较高温度 |
| 状态行 | `z_mode`、S1/S2、标定有效、控制就绪、遥控在线、主控允许、故障码、`seq` |
| 曲线 | 最近 30 s 两轴编码值与目标；悬停显示该时刻两轴编码 |

编码值来自驱动器 MIT 位置字；当前量程为 ±12.5 rad 对应 0–65535
（`robots/sentry/gimbal_config.h` 的 `SENTRY_P_MAX`）。角度直接使用板端 mrad，
不依赖上位机量程假设。

## 编译与烧录

面板底部“编译与烧录”调用 `firmware/tools/sentry_build_flash.sh`（相对工程根目录），
命令行也可以单独用：

```sh
tools/sentry_build_flash.sh build      # cmake --preset Sentry + 编译
tools/sentry_build_flash.sh flash      # 编译后写入并校验，再复位
tools/sentry_build_flash.sh program    # 只写现有 build/SentryH7/sentry_dm_mc02.elf
```

| 按钮 | 行为 |
|---|---|
| 编译 | 只编译，不碰硬件 |
| 编译并烧录 | 编译 + DAPLink 写入校验 + 复位；弹出确认框 |
| 仅烧录 | 跳过编译，直接写现有固件；弹出确认框 |
| 取消 | 终止当前任务（编译中途取消不会留下半截固件） |

烧录开始前扩展会断开串口；复位后 USB CDC 重新枚举，端口名可能变化，
成功后最多轮询 15 s 自动重连，失败时面板提示重新上电后点“刷新”。
串口占用的是板载 USB CDC，烧录走 DAPLink/SWD，两者互不影响。
`--speed` 默认 100 kHz（与已验证的部署一致），`--dry-run` 只打印命令不执行。

## 注意

- 面板直接占用串口，不能和 `tools/shot_monitor.py`、VOFA+ 等同时打开同一端口。
- 串口连接成功不等于正在收到数据：标题下会显示数据年龄和接收行数。
- 状态行是 ASCII、无校验；上位机逐行校验必需字段、编码范围和成对字段，
  格式不符的行计入“无效行”，不会被显示为编码值。
- 旧固件若没有 `fold_raw` / `pitch_raw` 字段，面板会保持“等待状态行”，不会猜测数据。
