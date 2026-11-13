/**
 *  ███╗   ██╗███████╗██╗  ██╗████████╗  ███████╗
 *  ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝  ██╔════╝
 *  ██╔██╗ ██║█████╗   ╚███╔╝    ██║     █████╗  
 *  ██║╚██╗██║██╔══╝   ██╔██╗    ██║     ██╔══╝  
 *  ██║ ╚████║███████╗██╔╝ ██╗   ██║     ███████╗
 *  ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝     ╚══════╝
 *
 *  @Author : Kaiyue Zhang
 *  @Email  : kaiyue.link@gmail.com
 *  @date   : 2026-10-30
 *  @description ：拨盘机械与闭环参数：角度、角速度和输出统一到拨盘坐标。
 */
#ifndef DIAL_CONFIG_H
#define DIAL_CONFIG_H
/* DM4310 dial: degrees, degrees/second, normalized torque. Parameters preserved. */
#define DIAL_DEG_PER_SHOT   ( 45.0f)                           /* 原 60.15 = 360/6 */
#define DIAL_MOVE_MAX_DEG   ( 45.0f)   /* 单条指令目标跨度硬上限，不能加到位容差 */
#define DIAL_OVERRUN_FAULT_DEG (10.0f)
#define DIAL_OVERRUN_MS        (  20)   /* ms：超过故障门槛后要持续这么久才锁存 */
#define DIAL_ROLLBACK_DEG   ( 22.5f)
#define DIAL_ZERO_DEFAULT_DEG (0.0f)
#define DIAL_PHASE_DEFAULT_DEG (-1.0f) /* 保留2026-10-28板端确认的顺时针1度标定 */
#define DIAL_JAM_ERR_DEG    (  4.0f)    /* 度：小于它就算"到位" */
#define DIAL_JAM_MS         ( 300)      /* ms：给带载顶推留时间，持续堵转仍回拨 */
#define DIAL_JAM_ROLLBACK_TIMEOUT_MS (500)
#define DIAL_ARRIVED_DEG    (DIAL_JAM_ERR_DEG)
#define DIAL_FB_TIMEOUT_MS  (150)
#define DIAL_MAX_SHOT_BURST 6
#define DIAL_ALIGN_ERR_DEG  (   3.0f)   /* 度 */
#define DIAL_ACC_DEG_S2     (3200.0f)   /* 拨盘 deg/s² */
#define DIAL_HOLD_SPD_DPS   ( 8.0f)     /* dps：转速小于它才认为停稳 */
#define DIAL_AUTO_DWELL_MS  (150)
#define DIAL_TELEM_SCALE    (16384.0f)
#define DIAL_DM_ESC_ID      (0x02)
#define DIAL_DM_MST_ID      (0x12)
#define DIAL_DM_BUS         (hcan1)
#define DIAL_DM_RATIO_GEAR  (3.0f)
#define DIAL_DEG_PER_RAD    (180.0f / (3.14159265f * DIAL_DM_RATIO_GEAR))   /* ≈ 19.0986 */
#define DIAL_RAD_PER_DEG    (1.0f / DIAL_DEG_PER_RAD)                       /* ≈ 0.0523599 */
#define DIAL_DM_P_MIN       (-12.5f)    /* 输出轴 rad   */
#define DIAL_DM_P_MAX       ( 12.5f)
#define DIAL_DM_V_MIN       (-30.0f)    /* 输出轴 rad/s */
#define DIAL_DM_V_MAX       ( 30.0f)
#define DIAL_DM_T_MIN       (-10.0f)    /* 输出轴 N·m   */
#define DIAL_DM_T_MAX       ( 10.0f)
#define DIAL_DM_KP_MAX      (500.0f)
#define DIAL_DM_KD_MAX      (  5.0f)
#define DIAL_DM_ANGLE_KP        ( 12.0f)    /* dps/deg */
#define DIAL_DM_ANGLE_KI        (  0.5f)    /* dps/(deg·s)，主要靠 P，积分只消静差 */
#define DIAL_DM_ANGLE_I_LIMIT   ( 70.0f)    /* dps */
#define DIAL_DM_ANGLE_MAX       (500.0f)    /* dps */
#define DIAL_DM_SPEED_KP        (  0.00225f) /* 归一化 / dps */
#define DIAL_DM_SPEED_KI        (  0.0001f) /* 归一化 / (dps·s) */
#define DIAL_DM_SPEED_I_LIMIT   (  0.30f)   /* 归一化：0.3×10N·m×3 ≈ 9 N·m 拨盘，够顶静摩擦 */
#define DIAL_DM_SPEED_MAX       (  1.0f)
#define DIAL_ALIGN_SPD_DPS      (450.0f)    /* 一格 45° 的峰值约 sqrt(ACC×45)≈380 dps，这个封顶只管长距离 */
#define DIAL_DM_INTEGRAL_EPS_DEG (0.10f)
#define DIAL_DM_TERMINAL_DEG       ( 6.0f)  /* 最后6°不再积累两级积分 */
#define DIAL_DM_TERMINAL_SPD_DPS   (60.0f)  /* 末段位置P速度给定上限 */
#define DIAL_DM_STICTION_OUT       (0.14f)  /* 最大附加补偿，输出仍夹在±1 */
#define DIAL_DM_STICTION_FADE_DPS  (20.0f)  /* 达到此速度补偿降至0，保留刹车 */
#define DIAL_DM_SETTLE_DEG         ( 0.5f)  /* 进入精确到位带 */
#define DIAL_DM_SETTLE_SPD_DPS     ( 2.0f)
#define DIAL_DM_SETTLE_MS          (  60U)  /* 实际HAL时间，连续满足才允许下一发 */
#define DIAL_DM_HOLD_EXIT_DEG      ( 0.9f)  /* 退出保持的滞回门槛 */
#define DIAL_DM_DIR_SIGN    (+1.0f)
#define DIAL_DM_VEL_SIGN    (DIAL_DM_DIR_SIGN)
#define DIAL_DM_TORQUE_SIGN (+1.0f)
#define DIAL_DM_TX_FAIL_MS (15U)
#define DIAL_JAM_SPD_DPS    (10.0f)    /* 低速顶推不急于回拨，接近停转才计时 */
#ifndef DIAL_DM_CALIB_FREE
#define DIAL_DM_CALIB_FREE 0
#endif
#endif
