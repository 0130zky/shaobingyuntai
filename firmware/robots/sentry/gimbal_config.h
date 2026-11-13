#ifndef SENTRY_CONFIG_H
#define SENTRY_CONFIG_H
/* 硬件验收开关：确认电机 ID、MIT 量程和两轴行程不碰撞之后才置 1。
 * ID 于 2026-11-12 验证：Fold 控制 1 / 反馈 0x11；Pitch 改为 2 / 0，保存后
 * 断电重启复核通过。四个端点已采集；Fold 的 MIT 物理量程仍未核实。
 * 置 0 时 ConfigOK() 恒为假，不会发出任何使能或运动命令。 */
#ifndef SENTRY_HARDWARE_READY
#define SENTRY_HARDWARE_READY 1
#endif
/* CAN 标识符：ESC 是控制帧 ID（即电机 ID），MST 是反馈帧 ID。
 * 取值须小于 0x7FF，且 ESC/MST 之间四个 ID 两两不同，否则 ConfigOK() 拒绝运行。 */
#define SENTRY_FOLD_ESC  0x01U   /* 折叠轴控制帧 ID（电机 ID） */
#define SENTRY_FOLD_MST  0x11U   /* 折叠轴反馈帧 ID（1 + 0x10） */
#define SENTRY_PITCH_ESC 0x02U   /* Pitch 轴控制帧 ID（电机 ID） */
#define SENTRY_PITCH_MST 0x00U   /* Pitch 轴反馈帧 ID（改 ID 后为 0） */
/* MIT 模式量程，均按输出轴计：位置 rad、速度 rad/s、力矩 N·m。
 * Pitch 量程已回读确认；Fold 量程仍未核实。
 * 整量程 2*P_MAX = 25 rad 对应 16 位编码 0..65535，32768 约等于 0 rad，
 * 1 计数约 0.3815 mrad。这三个值连同上面四个 CAN ID 一起写入 Flash 标定的
 * identity，任何一项改动都会让已保存的标定失效并需要重新标定。 */
#define SENTRY_P_MAX 12.5f       /* 位置量程 ±12.5 rad */
#define SENTRY_V_MAX 30.0f       /* 速度量程 ±30 rad/s，超出的速度指令由驱动夹住 */
#define SENTRY_T_MAX 10.0f       /* 力矩量程 ±10 N·m */
#define SENTRY_KP 20.0f          /* MIT 位置增益，允许范围 0..500 */
#define SENTRY_KD 1.0f           /* MIT 阻尼增益，允许范围 0..5 */
#define SENTRY_ERROR_RAD 0.05f   /* 最大跟随误差：位置指令最多超前反馈 0.05 rad（约 131 计数） */
/* 速度、加速度单位分别为 rad/s、rad/s²，代码里乘 65535/(2*P_MAX) 换算成计数。
 * 自动动作和满杆遥控共用 min(SPEED, sqrt(2*ACCEL*ERROR_RAD)) 速度包络。
 * 当前 SPEED=1200、ACCEL=1200、ERROR_RAD=0.05 时包络约 10.95 rad/s；
 * 继续增大 SPEED 不会再提速，机构实际速度还取决于反馈跟随和驱动能力。 */
#define SENTRY_FOLD_SPEED 1000.00f    /* 折叠轴速度上限 */
#define SENTRY_PITCH_SPEED 1000.00f   /* Pitch 轴速度上限，右杆调角也共用此值 */
#define SENTRY_FOLD_ACCEL 1000.00f    /* 折叠轴加速度上限 */
#define SENTRY_PITCH_ACCEL 1000.00f   /* Pitch 轴加速度上限，含右杆调角 */
#define SENTRY_MARGIN_RAD 0.02f       /* 标定校验余量（rad）：两端点间距须大于 2*0.02+0.02 rad */
#define SENTRY_MARGIN_COUNTS 64U      /* 折叠轴动作目标相对端点的内缩量（计数），即 38800+64 / 45628-64；也用于两端点最小间距校验 */
#define SENTRY_MAX_TRAVEL_RAD 6.0f    /* 单轴两端点允许的最大行程（rad，约 344°），超过判标定无效 */
#define SENTRY_FB_MS 100U             /* 电机反馈超过 100 ms 未刷新即判过期，触发 fault 3 */
#define SENTRY_TEMP_C 65U             /* MOS 或线圈温度达到 65 ℃ 触发过温 fault 4 */
/* DT7 开关档位编码：1=上、2=下、3=中。
 * S1 上/中全局失能，S1 下才允许 S2 操作。
 * S2 上失能；S2 中并行展开（Pitch 目标 34370）；S2 下并行折叠（Pitch 目标 29763）。
 * 右摇杆上下只在完全展开后按杆量线性调 Pitch；左摇杆不参与折叠。 */
#define SENTRY_STICK_DEADZONE 0.05f   /* 摇杆中心死区，归一化 [-1,1] 的 5%，死区外重新映射为连续杆量 */
#define SENTRY_SLOW_ZONE_RAD 0.10f    /* 软停区：距端点 0.10 rad（约 262 计数）内速度按剩余距离线性衰减 */
/* 1 = 以下四个端点作为“已验收限位”：保存的标定必须与之一致，标定缺失时也先用它们补全；
 * 0 = 只做几何合法性检查，不强制与这些端点一致。 */
#ifndef SENTRY_USE_COMMISSIONED_LIMITS
#define SENTRY_USE_COMMISSIONED_LIMITS 1
#endif
/* 已保存的标定/动作端点（16 位编码）：A = 折叠位，B = 展开位。
 * 运行时的位置限位联锁已取消，这些值只用于标定校验和缺省填充。 */
#define SENTRY_FOLD_A_RAW 45628U      /* 折叠轴折叠端 */
#define SENTRY_FOLD_B_RAW 38800U      /* 折叠轴展开端 */
#define SENTRY_PITCH_A_RAW 29762U     /* Pitch 折叠端（与折叠目标 29763 基本一致） */
#define SENTRY_PITCH_B_RAW 36039U     /* Pitch 展开端，机械端点，高于展开目标 34370 */
/* 旧版 Fold 电机使能后错误码仍报 0（失能）；置 1 时，发出使能命令且收到一帧
 * 新反馈后就把该状态视为就绪，仅对 Fold 轴生效。 */
#define SENTRY_FOLD_LEGACY_STATUS 1
/* 并行折叠/展开动作参数；端点姿态采集于 2026-11-13，运动过程中没有水平约束。 */
#define SENTRY_SEQ_PREP_PITCH_RAW 31401.0f /* 旧串行流程的 Pitch 预备角，已不参与控制，状态上报固定为 0 */
#define SENTRY_SEQ_UNFOLD_ALLOWED 1        /* 1 = S2 中位触发并行展开；0 = 中位视为失能且禁止展开 */
#define SENTRY_SEQ_FOLD_PITCH_RAW 29763.0f /* 折叠完成时 Pitch 的目标编码 */
#define SENTRY_SEQ_OPEN_PITCH_RAW 34370.0f /* 展开完成时 Pitch 的目标编码，也是右杆调角的水平基准 */
#define SENTRY_Z_LEVEL_TOL_COUNTS 96.0f    /* Pitch 到位容差 96 计数（约 0.037 rad） */
#define SENTRY_Z_TRACK_TOL_COUNTS 128.0f   /* 跟踪容差；目前只参与参数合法性校验（须大于到位容差），运动判定未使用 */
#define SENTRY_SEQ_FOLD_TOL_COUNTS 96.0f   /* 折叠轴到位容差 96 计数 */
#define SENTRY_Z_ALIGN_TIMEOUT_MS 20000U   /* 单次并行动作总超时，超时触发 fault 10（坐标非法为 fault 11） */
#define SENTRY_Z_SETTLE_MS 100U            /* 双轴都进容差且速度低于 80 计数/秒后，再稳定 100 ms 才算完成 */
#endif
