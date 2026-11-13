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
 *  @description ：发射机构对外接口：控制拍入口、遥测/指令协议与拨盘查询。
 */
#ifndef __BOOSTER_TASK_H
#define __BOOSTER_TASK_H

#include <stdint.h>
#include "shooter_status.h"

/* 合体版默认仅遥控；USB 台架调试需显式 -DBOOSTER_RC_ONLY=OFF。 */
#ifndef BOOSTER_RC_ONLY
#define BOOSTER_RC_ONLY 1
#endif

/* ---------------- 摩擦轮上位机指令（USB CDC） ---------------- */
/* 摩擦轮索引顺序：0=12点 1=4点 2=8点 */
#define FRI_IDX_12H         0
#define FRI_IDX_4H          1
#define FRI_IDX_8H          2
#define FRI_WHEEL_NUM       SHOOTER_WHEEL_COUNT

/* Fixed hardware: RM current slots 1..3, DM dial uses an independent frame. */

#define PC_CMD_HEADER       0x5A     /* 上位机 -> 板 包头 */
#define PC_TELEM_HEADER     0x5B     /* 板 -> 上位机 包头 */
#define PC_TELEM_TAIL       0x0D     /* 回传帧尾哨兵（抗错位） */

#define PC_CMD_LEN          12       /* 指令帧长度: PC -> 板 */
#define PC_TELEM_LEN        52       /* 回传帧长度: 板 -> PC */

/* 指令帧布局（小端）:
     0      header      0x5A
     1      enable      bit0=12点 bit1=4点 bit2=8点
     2-3    spd_12H     int16
     4-5    spd_4H      int16
     6-7    spd_8H      int16
     8      seq         帧序号
     9      shot_counter  进弹计数, 上位机每点一次"进弹"加 1
                          非零计数差只尝试一格；忙碌、失能或卡弹时丢弃，不补发
     10     zero_byte   拨弹盘零点: 0~255 → 0~45°, 步进 45/256 ≈ 0.176°
                          固件把它交给 Dial_Port_SetZero(), 由 port 在算角度时减掉。
                          掉电不保存, 所以上位机每帧都要带(反正 50Hz 一直在发)。
     11     checksum    XOR(字节 1..10)                                       */
#define PC_CMD_SHOT_OFF     9
#define PC_CMD_ZERO_OFF     10

/* 回传帧布局（小端）:
     0      header      0x5B
     1      status      bit0=link_ok, bit1..3=生效使能
                        bit4=三轮轮速就绪(见 Booster_WheelsReady)
                        bit5=任一速度环输出饱和(≥97% 满量程)
                        bit6=三轮反馈都新鲜(≤20ms)
                        bit7=CAN 错误(ESR 报警/被动/bus-off 或最近 1s TX 失败)
     2      seq_echo    板最近一次接受的指令 seq
     3      jam_retry   本次是否触发过回拨（0/1）；回拨后锁定，不自动前顶
     4-7    timestamp   uint32 微秒 (DWT)，横轴按它画，不受投递抖动影响
     8..    三轮，每轮 10 字节: speed, ref, out, cur, temp 各 int16
     38..   拨弹盘，10 字节: angle, target, speed, out, cur 各 int16
                          angle/target 单位: 度
                          **speed 单位是拨盘 deg/s**（原来是转子 rpm；数值小 8.86 倍，
                          上位机是纯透传、不拿它判阈值，所以 GUI 逻辑不用改，
                          只有右轴量程看着变扁了一点）
                          **out/cur 单位是"±16384 = 该电机满量程"**，3508 下和原来逐值相同
     48     shots       Dial_Shoot 低 8 位（硬夹之后它就等于"实际推进到第几格"，
                        不再等于"共点了几下" —— 被丢掉/被夹掉的不会记进去）
     49     dial_state  bit0=卡弹处理中, bit1=拨弹盘正在被驱动,
                        bit2=拨盘 CAN 反馈超时(已切输出)
     50     checksum    XOR(字节 1..49)
     51     帧尾        0x0D                                                  */
#define PC_TELEM_WHEEL_OFF  8        /* 第一个轮子的起始偏移 */
#define PC_TELEM_WHEEL_LEN  10       /* 每个轮子占的字节数 */
#define PC_TELEM_DIAL_OFF   38       /* 拨弹盘块的起始偏移 */
#define PC_TELEM_DIAL_LEN   10
#define PC_TELEM_SHOTS_OFF  48
#define PC_TELEM_DSTATE_OFF 49

/* 回传帧 byte1(status) 的位定义，上位机 fri_ctl_gui.py 必须保持一致 */
#define PC_TELEM_ST_LINK         0x01
#define PC_TELEM_ST_EN_12H       0x02
#define PC_TELEM_ST_EN_4H        0x04
#define PC_TELEM_ST_EN_8H        0x08
#define PC_TELEM_ST_WHEELS_READY 0x10   /* 三轮都到速并稳住 */
#define PC_TELEM_ST_SAT_ANY      0x20   /* 任一速度环输出打满 */
#define PC_TELEM_ST_FB_OK_ALL    0x40   /* 三轮反馈都新鲜 */
#define PC_TELEM_ST_CAN_ERR      0x80   /* CAN 总线/TX 异常 */

#define SCOPE_DIV           2        /* 每几个 Module_Task 周期发一帧回传(周期 3ms) -> ~166Hz */

#define FRI_CMD_TIMEOUT_MS  300      /* 指令超时自动停机，可调 */
#define FRI_SPD_LIMIT       10000    /* 转速给定安全钳位，需与上位机 SPEED_MAX 保持一致 */
#define FRI_READY_RPM       50      /* 三轮瞬时误差容差，仅用于到速门控，不是弹速精度保证 */
#define FRI_READY_MS        100     /* 连续稳速时间 */
#define FRI_FB_FRESH_MS     20

/* Only PIDTask calls this; dt_s is the measured interval, in seconds. */
void Booster_ControlStep(float dt_s);
void BoosterInit(void);
void Booster_RequestDialPhase(int16_t centidegrees);
uint8_t Booster_TakeDialPhaseReply(int16_t *phase, uint8_t *type);
uint8_t Booster_DialFault(void);

/* 上位机指令解析（由 USB 中断调用，必须短且不可阻塞） */
void    Booster_UsbCmdHandler(const uint8_t *buf, uint32_t len);
/* 组回传帧，填充 PC_TELEM_LEN 字节，返回长度 */
uint8_t Booster_BuildTelemetry(uint8_t *out);
/* 独立诊断帧：主25通道不变；窗口计数在控制任务内逐拍累计。 */
#define FRI_DIAG_CHANNELS 24
void Booster_RecordPIDTiming(uint32_t period_us, uint32_t runtime_us);
void Booster_TakeFrictionDiagnostics(float out[FRI_DIAG_CHANNELS]);

/* CAN1 ISR input; updates device feedback only, never control state. */
void Booster_OnCanRx(uint16_t id, const uint8_t *data, uint8_t length);
#endif
