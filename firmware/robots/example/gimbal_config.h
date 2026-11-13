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
 *  @date   : 2026-11-08
 *  @description ：云台两轴 DM4310 的 CAN 标识、零点、方向、限位与 MIT 量程。
 */
#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

/* 编译期总开关：0 = 控制拍只收不发（IDs/零点/方向还没上台架确认时用）。
   为 0 时 ConfigValid() 恒假，任何帧都不会发出。 */
#define GIMBAL_READY 1

/* CAN1：Pitch 读回 ESC=1、MST=0。Yaw 未接，当前控制入口不发 Yaw 帧。 */
#define GIMBAL_BUS          hcan1
#define GIMBAL_PITCH_ESC_ID 0x01U
#define GIMBAL_PITCH_MST_ID 0x00U
#define GIMBAL_YAW_ESC_ID   0x04U
#define GIMBAL_YAW_MST_ID   0x14U

/* Pitch 第二次标定：下限 -1.759 rad，上限 -2.767 rad，以物理下限 -15° 为基准。 */
#define GIMBAL_PITCH_ZERO_RAD (-2.020799388f)
#define GIMBAL_YAW_ZERO_RAD   0.0f

/* 安装方向：+1 = 电机读数增大时坐标角增大。旧版 pitch 力矩正方向为正，
   换到位置模式后必须手转确认；反了摇杆会反向（限位仍然有效）。 */
#define GIMBAL_PITCH_SIGN (-1.0f)
#define GIMBAL_YAW_SIGN   1.0f

/* Pitch 2026-11-11 两次读回：PMAX=12.5, VMAX=30, TMAX=10。
   Yaw 未接，接入前必须单独核对量程。 */
#define GIMBAL_P_MIN (-12.5f)
#define GIMBAL_P_MAX (12.5f)
#define GIMBAL_V_MIN (-30.0f)
#define GIMBAL_V_MAX (30.0f)
#define GIMBAL_T_MIN (-10.0f)
#define GIMBAL_T_MAX (10.0f)
#define GIMBAL_KP_MAX 500.0f
#define GIMBAL_KD_MAX 5.0f

/* 机械角范围(deg)，由已记录的 Pitch 端点确定。 */
#define GIMBAL_PITCH_MIN (-15.0f)
#define GIMBAL_PITCH_MAX (42.754146f)
#define GIMBAL_PITCH_STOP_MARGIN_DEG 1.0f
#define GIMBAL_PITCH_SLOW_ZONE_DEG 5.0f
#define GIMBAL_YAW_MIN (-60.0f)
#define GIMBAL_YAW_MAX (60.0f)

/* 摇杆满量程时目标角速度(deg/s) */
#define GIMBAL_RATE_DPS 30.0f

/* 位置目标与实测角最大偏差。它和 KP 一起决定推回力矩上限：
   KP × 窗口 = 手推时能吃住的最大力矩，超过就变成恒定弹簧。
   台架起步档 120 × 3.33° ≈ 7.0 N·m。 */
#define GIMBAL_PITCH_MAX_ERROR_DEG (10.0f / 3.0f)

/* MIT 位置环增益（电机内部执行），单位 N·m/rad、N·m·s/rad。
   硬保持调参阶梯（每档都要静止听、手推、摇杆走满行程）：
     1) 120 / 2.0 / 3.33° ≈ 7.0 N·m   ← 当前
     2) 200 / 2.5 / 2.50° ≈ 8.7 N·m
     3) 250 / 3.0 / 2.00° ≈ 8.7 N·m
   出现持续嗡嗡或低频来回摆就退回上一档；先加 KD，再按比例降 KP、放大窗口
   （保持 KP×窗口 不变），否则就是机械柔度问题。 */
#define GIMBAL_KP 120.0f
#define GIMBAL_KD 2.0f

/* 反馈超时(ms)：超时即松力，不拿过期位置继续顶 */
#define GIMBAL_FB_TIMEOUT_MS 100U
#define GIMBAL_FB_POLL_AGE_MS 30U
#define GIMBAL_FB_POLL_INTERVAL_MS 25U
#define GIMBAL_TX_RETRY_MS 12U

/* 控制周期上限(s)：调度抖动时不让目标一次性跳很远 */
#define GIMBAL_MAX_DT_S 0.020f

#endif
