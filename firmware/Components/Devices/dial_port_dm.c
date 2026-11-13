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
 *  @description ：DM4310 拨盘实现：连续位置展开、板端串级 PI 与 MIT 力矩提交。
 */
#include <math.h>
/* DM4310 axis: continuous feedback, board-side cascaded PI and MIT torque output. */
#include "dial_port.h"
#include "app_DM4310.h"
#include "controller.h"

/* MIT 帧量程 = 电机里配的 PMAX/VMAX/TMAX。量程配错不会报错、电机照转，
   只是速度/力矩全按比例缩放 —— 所以这几个数必须和上位机读出来的一致。 */
static const dm4310_range_t s_rng =
{
  DIAL_DM_P_MIN, DIAL_DM_P_MAX,
  DIAL_DM_V_MIN, DIAL_DM_V_MAX,
  DIAL_DM_T_MIN, DIAL_DM_T_MAX,
  DIAL_DM_KP_MAX, DIAL_DM_KD_MAX
};

/* ---------------- 接收侧 ---------------- */
static DM4310_MotorMeasure_t s_meas;
static dial_fb_t   s_fb;
static float       s_zero_deg;
static volatile uint32_t s_rx_tick;      /* 最近一帧的 HAL_GetTick()，中断里写 */
static volatile uint8_t  s_err;          /* 最近一帧的故障码，中断里写 */

/* 多圈解卷绕（在原始整数上做，见 OnCanRx 的注释） */
static int32_t s_prev_pos_int;
static int32_t s_wrap;                   /* 已经跨过几个 ±P_MAX 窗口 */
static uint8_t s_seeded;

/* ---------------- 控制侧 ---------------- */
static PID_t   s_pid_angle;
static PID_t   s_pid_speed;
static uint8_t s_active;                 /* 1 = 本轮在控制 / 0 = 断控，发零力矩帧 */
static uint32_t s_tx_fail_since;
static uint8_t s_tx_failing;

/* 末段状态只服务于当前绝对目标；换目标、断控、清积分均使其失效。 */
typedef struct {
  float target;
  uint32_t since_ms, updated_ms;
  uint8_t valid, tracking, holding;
} dial_terminal_t;
static dial_terminal_t s_terminal;

/**
 * @description  : 初始化（BoosterInit 里调）
 * @return        {*}
 */
void Dial_Port_Init(void)
{
  s_zero_deg = DIAL_ZERO_DEFAULT_DEG;

  /* 发送侧初值。s_seeded=0 是关键：首帧必须拿来 seed 解卷绕，
     否则 s_prev_pos_int 初值 0 会让上电第一帧凭空算出一个跨窗。 */
  s_seeded       = 0;
  s_wrap         = 0;
  s_prev_pos_int = 0;
  s_active       = 0;
  s_terminal     = (dial_terminal_t){0};
  s_tx_failing = 0;
  s_tx_fail_since = 0;
  s_err          = (uint8_t)ERR_Disable;

  /* 角度环：输入 deg、输出 dps。DeadBand 必须是 0 —— PID_Calculate 在
     |Err| <= DeadBand 时整个更新被跳过、Output 冻在上一次的值。 */
  PID_Init(&s_pid_angle,DIAL_DM_ANGLE_MAX,DIAL_DM_ANGLE_I_LIMIT,
           DIAL_DM_ANGLE_KP,DIAL_DM_ANGLE_KI,0,0,Integral_Limit);
  /* 速度环：输入 dps、输出归一化力矩 [-1,1] */
  PID_Init(&s_pid_speed,DIAL_DM_SPEED_MAX,DIAL_DM_SPEED_I_LIMIT,
           DIAL_DM_SPEED_KP,DIAL_DM_SPEED_KI,0,0,Integral_Limit);

  /* s_rx_tick 保持 0（和 3508 版一致）。这个 0 会让上电头 150ms 被判"反馈新鲜"，
     但 **Update() 里 fb_ok 另有 s_seeded 门控**，首帧真实反馈到达前不会放行 ——
     否则零初值的 s_meas 会翻出 −238.7°，变成一条跨 5 个槽的垃圾指令（详见 Update）。
     不在这里发使能帧：使能由 Tx() 在第一拍发（要 CAN 已经起来）。 */
  s_rx_tick = 0;
}

/**
 * @description  : 上位机下发零点（USB 中断里调）。float 在 M4 上是单指令存储，不会撕裂
 * @param         {float} zero_deg
 * @return        {*}
 */
void Dial_Port_SetZero(float zero_deg)
{
  s_zero_deg = zero_deg;
}

/**
 * @description  : CAN 收到一帧拨盘反馈。**只在中断里调**，必须短
 * @param         {const uint8_t*} data  8 字节原始数据
 * @return        {*}
 */
void Dial_Port_OnCanRx(const uint8_t *data)
{
  DM4310_Get_Ranged(&s_meas, data, &s_rng);
  s_err = s_meas.ERR;

  /* ---- 多圈解卷绕 ----
     反馈里的 POS 是 16 位，映射范围就是 [P_MIN, P_MAX]（出厂 ±12.5 rad 输出轴），
     **驱动里没有任何累加**（DM4310_Get_Ranged 是一次线性映射，和 3508 那边
     Get_RM_MotorInfo_total 的 round*8192+encoder 完全不同）。
     拨盘是单向持续旋转的，不解卷绕的话 angle_deg 每绕一圈就跳回去，
     位置环会在窗口边界看到一整圈的误差。

     在**原始整数**上判绕（半窗 65536/2 = 32768），不碰浮点阈值：
       原始值往回跳 32768 以上 = 拨盘向前越过了 +P_MAX → 窗口数 +1
       原始值往前跳 32768 以上 = 拨盘向后越过了 -P_MAX → 窗口数 -1 */
  if (!s_seeded)
  {
    /* 首帧 seed。**不能省**：漏了的话上电第一帧就会凭空多算/少算一圈。 */
    s_prev_pos_int = (int32_t)s_meas.Pos_int;
    s_wrap         = 0;
    s_seeded       = 1;
  }
  else
  {
    int32_t d = (int32_t)s_meas.Pos_int - s_prev_pos_int;
    if      (d >  32767) s_wrap--;
    else if (d < -32767) s_wrap++;
    s_prev_pos_int = (int32_t)s_meas.Pos_int;
  }

  s_rx_tick = HAL_GetTick();
}

/**
 * @description  : 把原始反馈翻成拨盘坐标（每个控制拍开始调用）
 * @return        {*}
 */
void Dial_Port_Update(void)
{
  /* 解卷绕后的连续位置（输出轴 rad）：
       本窗内的实测位置 + 已经跨过的窗口数 × 一个窗口的宽度

     **必须是 "+ s_wrap × span" 这个形式，不能写成
     `(s_wrap * 65536 + Pos_int) × span / 65535`**：后者把"跨一个窗口"当成
     65536 个计数，而映射用的是 65535（`DM4310_uint_to_float` 的分母），
     每跨一次窗口就系统性差 span/65535 = 0.0073 拨盘度（正好 1 个 LSB）。 */
  const float span   = s_rng.p_max - s_rng.p_min;                /* 25.0 rad */
  const float raw    = s_rng.p_min + (float)s_meas.Pos_int * span / 65535.0f;
  const float rad    = raw + (float)s_wrap * span;               /* 输出轴 rad，连续多圈 */

  s_fb.angle_deg = rad * DIAL_DM_DIR_SIGN * DIAL_DEG_PER_RAD - s_zero_deg;
  s_fb.speed_dps = s_meas.Velocity * DIAL_DM_VEL_SIGN * DIAL_DEG_PER_RAD;
  s_fb.current   = s_meas.Torque / DIAL_DM_T_MAX;                /* 归一化 [-1,1] */

  /* 两个温度取高的那个报出去。T_MOS / T_Rotor 都是无符号字节（摄氏） */
  s_fb.temp_c = (float)((s_meas.T_MOS > s_meas.T_Rotor) ? s_meas.T_MOS : s_meas.T_Rotor);

  /* fb_ok = **收到过真实反馈** 且 反馈新鲜 且 无故障码 且 发送没在连续失败。
     故障码那条（过压/欠压/过流/MOS 过温/线圈过温/通讯丢失/过载 0x08~0x0E）
     以前全工程定义了没人读，出故障时我们只会继续灌力矩。
     0x00 失能 / 0x01 使能 是正常态，其余一律算故障。

     **`s_seeded` 这条门控是 DM 版独有的**：s_rx_tick 初值 0 会让上电头
     DIAL_FB_TIMEOUT_MS 内被判"反馈新鲜"，而此时 s_meas 还是全零 →
     raw = P_MIN = −12.5 rad → angle_deg = −238.7° → 共享层算出
     Dial_Shoot = −5（跨越 5 个弹槽的垃圾指令）。 */
  s_fb.fb_ok = ((s_seeded) &&
                ((HAL_GetTick() - s_rx_tick) <= DIAL_FB_TIMEOUT_MS) &&
                (s_err == (uint8_t)ERR_Enable || s_err == (uint8_t)ERR_Disable) &&
                (!s_tx_failing || (uint32_t)(HAL_GetTick()-s_tx_fail_since) < DIAL_DM_TX_FAIL_MS)) ? 1 : 0;
}

/**
 * @description  : 取反馈（只读）
 * @return        {const dial_fb_t*}
 */
const dial_fb_t *Dial_Port_Get(void)
{
  return &s_fb;
}

/**
 * @description  : 让拨盘跟到指定角度（拨盘绝对角度，deg）。
 *                 **只算不发** —— 结果存在 s_fb.out，由同拍的 Tx() 发出去。
 *                 DM串级：积分方向校正 → 角度环 → 速度包线 → 速度环。
 * @param         {float} dial_deg
 * @return        {*}
 */
void Dial_Port_SetTarget(float dial_deg, float dt_s)
{
  float err = dial_deg - s_fb.angle_deg;
  const float ae = fabsf(err);
  const float speed = fabsf(s_fb.speed_dps);
  const uint32_t now = HAL_GetTick();
  if (!s_terminal.valid || dial_deg != s_terminal.target) {
    s_terminal = (dial_terminal_t){.target=dial_deg, .valid=1};
  }
  /* 未观察到的调度间隙不计作连续停稳，恢复后的旧就绪状态也不能沿用。 */
  if ((uint32_t)(now-s_terminal.updated_ms) > 10U)
    s_terminal.tracking = s_terminal.holding = 0;
  s_terminal.updated_ms = now;
  const uint8_t terminal = ae <= DIAL_DM_TERMINAL_DEG;
  if (!terminal || ae > DIAL_DM_HOLD_EXIT_DEG || speed > DIAL_HOLD_SPD_DPS)
    s_terminal.holding = 0;
  if (!s_terminal.holding) {
    if (ae <= DIAL_DM_SETTLE_DEG && speed <= DIAL_DM_SETTLE_SPD_DPS) {
      if (!s_terminal.tracking) {
        s_terminal.tracking = 1;
        s_terminal.since_ms = now;
      }
      if ((uint32_t)(now-s_terminal.since_ms) >= DIAL_DM_SETTLE_MS)
        s_terminal.holding = 1;
    } else {
      s_terminal.tracking = 0;
    }
  }

  /* 末段不让两级积分继续攒推力：停住后输出不再随等待时间增大。
     保持态仍有位置恢复力和速度阻尼，并非切断总力矩。 */
  const float angle_ki = s_pid_angle.Ki;
  const float speed_ki = s_pid_speed.Ki;
  if (terminal) {
    s_pid_angle.Iout = s_pid_angle.ITerm = s_pid_angle.Last_ITerm = 0.0f;
    s_pid_speed.Iout = s_pid_speed.ITerm = s_pid_speed.Last_ITerm = 0.0f;
    s_pid_angle.Ki = s_pid_speed.Ki = 0.0f;
  }

  /* 非末段只释放与当前位置纠偏方向相反的积分，避免跨目标后仍正向顶推。
     非末段同向积分保留；末段已在上面禁用积分，改用受限摩擦补偿。
     这里不扩大目标/越界容差，也不在到位附近关闭速度闭环。 */
  if (fabsf(err) > DIAL_DM_INTEGRAL_EPS_DEG || fabsf(s_fb.speed_dps) > DIAL_HOLD_SPD_DPS) {
    const float correction = (err == 0.0f) ? -s_fb.speed_dps : err;
    if (s_pid_angle.Iout * correction < 0.0f) {
      s_pid_angle.Iout = s_pid_angle.ITerm = s_pid_angle.Last_ITerm = 0.0f;
    }
    if (s_pid_speed.Iout * correction < 0.0f) {
      s_pid_speed.Iout = s_pid_speed.ITerm = s_pid_speed.Last_ITerm = 0.0f;
    }
  }
  PID_CalculateDt(&s_pid_angle, s_fb.angle_deg, dial_deg, dt_s);
  /* 通用PID在精确零误差时会跳过更新；位置环不能沿用旧的前进速度。 */
  if (err == 0.0f) {
    s_pid_angle.Iout = s_pid_angle.ITerm = s_pid_angle.Last_ITerm = 0.0f;
    s_pid_angle.Pout = s_pid_angle.Output = s_pid_angle.Last_Output = 0.0f;
  }

  float spd_set = s_pid_angle.Output;                          /* 单位：dps */
  if (!terminal && fabsf(err) > DIAL_ALIGN_ERR_DEG)
  {
    /* 速度上限 = sqrt(2*a*|误差|)，"现在开始全力刹车刚好停在目标上"的速度。
       没有它，硬给高速会在目标前刹不住、冲过发位再倒回来。
       淡入：3°..6° 之间从 0 线性升到 1，避免在到位点附近踩一脚油门。 */
    float v_max = sqrtf(2.0f * DIAL_ACC_DEG_S2 * fabsf(err));   /* dps */
    if (v_max > DIAL_ALIGN_SPD_DPS)
      v_max = DIAL_ALIGN_SPD_DPS;

    float k = (fabsf(err) - DIAL_ALIGN_ERR_DEG) / DIAL_ALIGN_ERR_DEG;
    if (k > 1.0f) k = 1.0f;
    v_max *= k;

    /* 只抬"给得太小"的，不去压 P 环本来就给得更大的量 */
    if (fabsf(spd_set) < v_max)
      spd_set = (err > 0.0f) ? v_max : -v_max;
  }

  if (terminal) {
    if (spd_set > DIAL_DM_TERMINAL_SPD_DPS) spd_set = DIAL_DM_TERMINAL_SPD_DPS;
    if (spd_set < -DIAL_DM_TERMINAL_SPD_DPS) spd_set = -DIAL_DM_TERMINAL_SPD_DPS;
  }

  PID_CalculateDt(&s_pid_speed, s_fb.speed_dps, spd_set, dt_s);
  /* 通用PID在零误差时保留旧Output，这里不能留下上一拍加速/刹车值。 */
  if (spd_set == s_fb.speed_dps) {
    s_pid_speed.Pout = 0.0f;
    s_pid_speed.Output = s_pid_speed.Last_Output = s_pid_speed.Iout;
  }
  s_pid_angle.Ki = angle_ki;
  s_pid_speed.Ki = speed_ki;

  /* 归一化力矩给定 [-1,1]。Tx() 再乘 T_MAX 填进 MIT 帧的 torq 字段。
     遥测的 out 字段就是它（× DIAL_TELEM_SCALE）。 */
  s_fb.out = s_pid_speed.Output;
  if (terminal && !s_terminal.holding && ae > DIAL_DM_SETTLE_DEG) {
    float k = 1.0f-speed/DIAL_DM_STICTION_FADE_DPS;
    if (k > 0.0f) {
      /* 已在主动刹车时不叠加推进补偿，过目标后补偿方向随误差反向。 */
      const float correction = err > 0.0f ? 1.0f : -1.0f;
      if (s_fb.out*correction >= 0.0f)
        s_fb.out += correction*DIAL_DM_STICTION_OUT*k;
    }
  }
  if (s_fb.out > DIAL_DM_SPEED_MAX) s_fb.out = DIAL_DM_SPEED_MAX;
  if (s_fb.out < -DIAL_DM_SPEED_MAX) s_fb.out = -DIAL_DM_SPEED_MAX;

  s_active = 1;
}

uint8_t Dial_Port_Settled(void)
{
  return s_active && s_terminal.valid && s_terminal.holding &&
         (uint32_t)(HAL_GetTick()-s_terminal.updated_ms) <= 10U &&
         fabsf(s_terminal.target-s_fb.angle_deg) <= DIAL_DM_HOLD_EXIT_DEG &&
         fabsf(s_fb.speed_dps) <= DIAL_HOLD_SPD_DPS;
}

/**
 * @description  : 只清积分，PID 继续跑。卡弹锁存/解卡/重新对齐时调
 * @return        {*}
 */
void Dial_Port_ClearIntegral(void)
{
  s_pid_angle.Iout = s_pid_angle.ITerm = s_pid_angle.Last_ITerm = 0.0f;
  s_pid_speed.Iout = s_pid_speed.ITerm = s_pid_speed.Last_ITerm = 0.0f;
  s_terminal = (dial_terminal_t){0};
}

/**
 * @description  : 断控：清积分 + 切输出。本拍起改发"零力矩"帧（拨盘失力）。
 *                 **故意不发电机自己的失能帧**，见 Tx() 的注释。
 * @return        {*}
 */
void Dial_Port_Stop(void)
{
  s_pid_angle.Output = 0.0f;
  s_pid_speed.Output = 0.0f;
  s_pid_angle.Iout   = 0.0f;
  s_pid_speed.Iout   = 0.0f;
  s_fb.out           = 0.0f;
  s_active           = 0;
  s_terminal         = (dial_terminal_t){0};

  /* 断控期间 SetTarget 不跑，DWT 时间基会停在最后一次调用上。恢复后第一拍
     的 dt 可能是几百 ms，Ki*Err*dt 会甩出一个不小的积分冲击（力矩模式会直接
     体现成一下猛推）。停控时把时间基追到当前，恢复后的第一拍就是正常的一个
     3ms 周期。 */
}

/**
 * @description  : 每控制周期发一帧（PID_Task 3ms 调）。**必须每拍都发**：
 *                 电机内部有 TIMEOUT(0x09) 保护，停发就自动失能。
 * @return        {*}
 */
void Dial_Port_Tx(void)
{
  uint8_t ok;
  float   torq;

  /* ---- 使能 ----
     只在"电机反馈说自己没使能"时补一帧使能。ERR_Disable(0x00) 正好就是
     "没在使能态、也没报故障"—— 报故障时 ERR 是 0x08~0x0E，这里就不会去戳它。
     收到过反馈之前 s_err 的初值也是 ERR_Disable，所以第一拍就会发一帧使能。 */
  if (s_err == (uint8_t)ERR_Disable)
  {
    DM4310_SetEnable(&DIAL_DM_BUS, DIAL_DM_ESC_ID, 1);
  }

  /* ---- 控制帧 ----
     正常：kp=kd=0 + torq=串级输出 → 纯力矩。位置环在我们这边，
     所以不存在"目标越过 ±12.5 rad 窗口"的问题。
     断控 / 台架标定（DIAL_DM_CALIB_FREE=1）：torq=0，电机使能但零力矩。

     **为什么断控不是发电机失能帧**：DM 失能之后大概率就不再发反馈了，于是
     fb_ok 永久拉低 → 共享层永远走断控分支 → 再也不重新使能，直接死锁。
     发零力矩 MIT 帧既真的失力，又让反馈继续流，恢复时是连续的。 */
  torq = (s_active && !DIAL_DM_CALIB_FREE)
           ? (s_fb.out * DIAL_DM_T_MAX * DIAL_DM_TORQUE_SIGN) : 0.0f;
  ok = DM4310_Send_Ranged(&DIAL_DM_BUS, DIAL_DM_ESC_ID,
                          0.0f, 0.0f, torq,
                          0.0f, 0.0f, &s_rng);

  if (ok) s_tx_failing = 0;
  else if (!s_tx_failing) {
    s_tx_failing = 1;
    s_tx_fail_since = HAL_GetTick();
  }
}
