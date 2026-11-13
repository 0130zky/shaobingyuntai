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
 *  @description ：三轮摩擦轮控制：参考斜坡、同步补偿、反馈看门狗和发散保护。
 */
#include <math.h>
#include "shooter_internal.h"
#include "bsp_dwt.h"
#include "friction_sync.h"
#include "FreeRTOS.h"
#include "task.h"

static const int8_t Fri_DirSign[FRI_WHEEL_NUM] = { -1, 1, -1 };  /* 12点/4点/8点 摩擦轮的正转方向 */

static int16_t s_ready_target[FRI_WHEEL_NUM];
static uint8_t s_wheels_ready, s_ready_tracking;
static uint32_t s_ready_since, s_ready_updated;

static uint8_t FrictionReadyConditions(void)
{
  const uint32_t now = HAL_GetTick();
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; i++) {
    const RM_Motor_t *m = &BOOSTER.RM_Motor[Fri_MotorID[i] - 1];
    const int16_t target = Booster_GetWheelSetpoint(i);
    if (!target || target != s_ready_target[i] ||
        fabsf(m->MotorCtrl_Speed.Ref - target) > FRI_SYNC_TRIM_RPM ||
        !m->RM_MotorMeasure.rx_tick ||
        now - m->RM_MotorMeasure.rx_tick > FRI_FB_FRESH_MS ||
        fabsf((float)m->RM_MotorMeasure.speed - target) > FRI_READY_RPM)
      return 0;
  }
  return 1;
}

void Booster_UpdateWheelReadiness(void)
{
  const uint32_t now = HAL_GetTick();
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; i++) {
    const int16_t target = Booster_GetWheelSetpoint(i);
    if (target != s_ready_target[i]) {
      s_ready_tracking = s_wheels_ready = 0;
      s_ready_target[i] = target;
    }
  }
  s_ready_updated = now;
  if (!FrictionReadyConditions()) {
    s_ready_tracking = s_wheels_ready = 0;
    return;
  }
  if (!s_ready_tracking) {
    s_ready_tracking = 1;
    s_ready_since = now;
  }
  s_wheels_ready = ((uint32_t)(now - s_ready_since) >= FRI_READY_MS);
}

uint8_t Booster_WheelsReady(void)
{
  return s_wheels_ready && HAL_GetTick() - s_ready_updated <= 10U &&
         FrictionReadyConditions();
}

/* ---------------- 摩擦轮速度环整定参数 ----------------
   M3508 官方挂载：轮子在转子上，无减速箱，C620 转速读数即轮速。
   C620 自己**没有速度环**，给多少电流就是多少电流 —— 整个速度环在本文件，
   所以下面 Output 的单位就是电流计数，±16384 ≈ ±20A。

   旧值 Kp=3 / Ki=0.0005 / Kd=0.001 的实测问题：给定 5000 空载在 4950~5050 之间
   飘，**而且转得越久飘得越大**。不是振荡，是"环太软 + 等于没有积分"：

     1) 纯 P 环必然有稳态误差。稳态时 Output 要正好等于维持该转速所需的电流，
        于是 Err = 维持电流 / Kp。空载转 5000 大约只要 150 个计数(≈0.18A)，
        150/3 = 50rpm —— 正好对上实测的"50+"。
        维持电流随温度/润滑漂多少，误差就跟着漂多少，这就是"转久了更大"，
        不是故障，是纯 P 环的固有性质，**只有积分能消掉**。
     2) Ki=0.0005 时积分每秒只攒 0.0005*Err(Err=50rpm 即 0.025/s)，
        攒到能顶事的量级要几十分钟，实际等于不存在。
     3) Kd=0.001 淹没在 1rpm 的量化台阶里，也没用。
     4) 旧 integral_limit=163.84 是死参数(improve=0 时 f_Integral_Limit 从不被调用)，
        且 163.84/16384 = 满量程 1%，扛不住静摩擦。

   下面的值是按 J≈2e-5(转子+55mm胶轮) 推的起点，必须在实机上按
   "带宽/超调" 实测微调，尤其是 FRI_KP 和 FRI_KI。

   *** 注意：这几个宏以前只定义、没接进 PID_Init，三轮一直跑的是旧增益。
   现已接上 —— 以后调参就改这里。*** */
#define FRI_KP          10.0f     /* 原 3.0。离散稳定上限约 55，先 10，可上探到 20 */
#define FRI_KI          20.0f     /* 原 0.0005。目标积分时间常数 Kp/Ki ≈ 0.5s */
#define FRI_KD          0.0f      /* 原 0.001。对误差的无滤波微分，噪声大收益小 */
#define FRI_I_LIMIT     4000.0f   /* 原走 M3508_SpeedMax_I_Out=163.84。按实测保持电流的 1.5~2 倍设 */
#define FRI_OUT_LPF_RC  0.001f    /* 三轮同用1ms输出低通，减小原2ms滤波滞后；PI与电流上限不变 */
/* Integral_Limit 必须开：Ki 调大后 0->5000 的启动过程会积分饱和 */
#define FRI_PID_IMPROVE (Integral_Limit | OutputFilter)

/* ---------------- 摩擦轮安全层 ----------------
   2026-10-17 实测事故：旋钮从中位直接拧到 -100（目标 0）时，三轮"失控反向全速转"，
   拨 CH6 总使能也停不下来。**速度环本身是稳定的**（把目标降到 0 时 P 项随转速
   线性减小，不会 bang-bang），所以下面全是按"命令/反馈链路断了"来兜底的：

   1) 目标为 0 时**不跑 PID、直接发 0 电流**。
      以前这里会按误差打一个满量程刹车电流（5000rpm 误差 → P 项 -50000 饱和到
      -16384）。C620 没有命令超时：一旦 CAN 发不出去（bus-off/线松）或反馈冻住，
      ESC 就一直保持这最后一条"满刹车"电流 —— 轮子减速到 0 后继续被推成**反向
      全速**，而此时把目标改成 0 也还是 0，所以"失能没用"。滑停虽然慢一点，
      但任何链路故障下都不可能把轮子推起来。
   2) 反馈看门狗：100ms 没收到这块电机的 CAN 反馈就断输出。冻住的 speed 会让
      PID 永远追一个假误差。
   3) 参考斜率限制：旋钮从 10000 一步拧到 0 时，内部目标按 -150rpm/3ms 降下去
      （约 0.2s），PID 永远看不到几千转的阶跃，任何一拍的最大电流都很小。
   4) 发散锁存：实测 |speed| 明显超过目标 +2000rpm 且输出打满持续 200ms ——
      正常环不可能这样，只可能是反馈符号/接线反了。断输出并锁存，目标回 0 才解锁。 */
#define FRI_FB_TIMEOUT_MS   (100)
#define FRI_REF_SLEW_UP     (25000.0f)   /* rpm/s */
#define FRI_REF_SLEW_DN     (50000.0f)   /* rpm/s */
#define FRI_DIV_MARGIN_RPM  (2000)
#define FRI_DIV_MS          (200.0f)

void Shooter_FrictionInit(void)
{
  s_wheels_ready = s_ready_tracking = 0;
  for (uint8_t i=0;i<FRI_WHEEL_NUM;i++)
    PID_Init(&BOOSTER.RM_Motor[Fri_MotorID[i]-1].MotorCtrl_Speed,
             M3508_SpeedMaxOut,FRI_I_LIMIT,FRI_KP,FRI_KI,FRI_KD,FRI_OUT_LPF_RC,FRI_PID_IMPROVE);

}

void Shooter_ResetReadiness(void)
{
  s_ready_tracking = s_wheels_ready = 0;
}

/* 把速度环整个按到"还没开始跑"的状态：积分清零、上次误差对齐当前值，
   DWT_CNT 重打时间戳（否则停了几百拍后 dt 会是一个巨大的值）。 */
static void FrictionPid_Reset(PID_t *pid, int16_t meas)
{
  pid->Ref           = 0.0f;
  pid->Measure       = (float)meas;
  pid->Last_Measure  = (float)meas;
  pid->Err           = -(float)meas;
  pid->Last_Err      = pid->Err;
  pid->Pout          = 0.0f;
  pid->ITerm         = 0.0f;
  pid->Last_ITerm    = 0.0f;
  pid->Iout          = 0.0f;
  pid->Dout          = 0.0f;
  pid->Last_Dout     = 0.0f;
  pid->Output        = 0.0f;
  pid->Last_Output   = 0.0f;
}

/**
 * @description  : 第 idx 个摩擦轮的速度环（含全部安全层）
 *                 以前的写法是 PID_FrictionCtrl(电机, 给定)，直接把给定丢给
 *                 PID_Calculate。事故原因不在 PID 稳定性，而在"给定为 0 时仍然
 *                 按误差打满刹车电流"，一旦反馈/CAN 断掉就是永久满刹车 →
 *                 轮子过零后反向全速。安全层的设计理由见文件顶部注释。
 * @param         {uint8_t} idx   0=12点 1=4点 2=8点
 * @return        {*}
 */
static float s_fri_ref_ramp[FRI_WHEEL_NUM];
static uint8_t s_fri_div_latch[FRI_WHEEL_NUM];
static float s_fri_applied_trim[FRI_WHEEL_NUM];

static void Booster_FrictionCtrl(uint8_t idx, const friction_sample_t *sample,
                                  uint32_t now, float sync_trim)
{
  static float div_cnt [FRI_WHEEL_NUM] = {0};
  static int16_t  div_abs_pre[FRI_WHEEL_NUM] = {0};

  if (idx >= FRI_WHEEL_NUM) return;
  s_fri_applied_trim[idx]=0.0f;

  RM_Motor_t *m   = &BOOSTER.RM_Motor[Fri_MotorID[idx] - 1];
  PID_t      *pid = &m->MotorCtrl_Speed;
  const int16_t meas   = sample->speed;
  const int16_t target = sample->target;

  /* 给定回到 0（含 CH6 失能、旋钮死区、接收机掉线）：解除发散锁存 */
  if (target == 0) { s_fri_div_latch[idx] = 0; div_cnt[idx] = 0; div_abs_pre[idx] = 0; }

  /* ---- 反馈看门狗：没有新鲜反馈就绝不给电流 ---- */
  const uint8_t fb_ok = (sample->rx_tick != 0) &&
                        ((now - sample->rx_tick) <= FRI_FB_TIMEOUT_MS);
  if (!fb_ok)
  {
    s_fri_ref_ramp[idx] = 0;
    FrictionPid_Reset(pid, meas);
    return;
  }

  /* ---- 目标为 0：断力矩让轮子滑停，不再主动刹车 ---- */
  if (target == 0)
  {
    s_fri_ref_ramp[idx] = 0;
    FrictionPid_Reset(pid, meas);
    return;
  }

  /* ---- 参考斜率限制：几千转的阶跃永远进不到 PID ---- */
  float r = s_fri_ref_ramp[idx];
  /* 两轮反向安装：升/降速按转速绝对值判定，不能按原始正负号。
     三轮启动均75rpm/拍，降速均150rpm/拍。 */
  const float step = (fabsf((float)target) > fabsf(r)
                         ? FRI_REF_SLEW_UP : FRI_REF_SLEW_DN) * Shooter_ControlDt();
  if      (target > r + step) r += step;
  else if (target < r - step) r -= step;
  else                       r = target;
  s_fri_ref_ramp[idx] = r;

  /* ---- 发散锁存：转速已经远超目标、输出打满、而且**还在越转越快**持续 200ms ----
     "越转越快"这一条是必须的：正常降速时输出也会打满（刹车），但那时 |speed|
     是在**减小**的，不能误判。只有反馈/电流符号反了这种正反馈才会让转速在
     输出打满的同时继续远离 0。M3508 满电流下加速度极大，200ms 足够认出来。
     一旦锁存，只有目标回 0 才解锁（见上面）。 */
  {
    const int16_t a_meas = (meas >= 0) ? meas : (int16_t)(-meas);
    const int16_t a_ref  = (r    >= 0) ? r    : (int16_t)(-r);
    if (!s_fri_div_latch[idx] &&
        a_meas > a_ref + (int16_t)FRI_DIV_MARGIN_RPM &&
        fabsf(pid->Output) > pid->MaxOut * 0.9f)
    {
      if      (a_meas > div_abs_pre[idx] + 2) { div_cnt[idx] += Shooter_ControlDt() * 1000.0f; }
      else if (a_meas < div_abs_pre[idx] - 5) { div_cnt[idx] = 0; }   /* 在减速：不是发散 */
      if (div_cnt[idx] >= FRI_DIV_MS) s_fri_div_latch[idx] = 1;
    }
    else
    {
      div_cnt[idx] = 0;
    }
    div_abs_pre[idx] = a_meas;
  }

  if (s_fri_div_latch[idx])
  {
    s_fri_ref_ramp[idx] = 0;
    FrictionPid_Reset(pid, meas);
    return;
  }

  /* 只有内部参考已完成斜坡才加入同步补偿；所有停机/故障分支先返回。 */
  float ref = (float)r;
  if (r == target) ref += (target > 0 ? sync_trim : -sync_trim);
  if (ref > FRI_SPD_LIMIT) ref = FRI_SPD_LIMIT;
  if (ref < -FRI_SPD_LIMIT) ref = -FRI_SPD_LIMIT;
  s_fri_applied_trim[idx]=(target>0 ? 1.0f : -1.0f)*(ref-(float)r);
  PID_CalculateDt(pid, (float)meas, ref, Shooter_ControlDt());
}

static float s_fri_diag[FRI_DIAG_CHANNELS];
static uint32_t s_fri_diag_seq;

void Booster_RecordPIDTiming(uint32_t period_us, uint32_t runtime_us)
{
  /* 此处在PID_Task末尾发布，本窗口峰值不依赖串口能否及时发送。 */
  taskENTER_CRITICAL();
  s_fri_diag[0]=1.0f; /* 协议版本 */
  s_fri_diag[1]=(float)(HAL_GetTick() & 0xffffU);
  s_fri_diag[2]=(float)period_us;
  if (s_fri_diag[3]<period_us) s_fri_diag[3]=(float)period_us;
  s_fri_diag[4]=(float)runtime_us;
  if (s_fri_diag[5]<runtime_us) s_fri_diag[5]=(float)runtime_us;
  s_fri_diag[22]+=1.0f;
  s_fri_diag[23]=(float)(++s_fri_diag_seq & 0xffffU);
  taskEXIT_CRITICAL();
}

void Booster_TakeFrictionDiagnostics(float out[FRI_DIAG_CHANNELS])
{
  taskENTER_CRITICAL();
  for (uint8_t i=0;i<FRI_DIAG_CHANNELS;i++) out[i]=s_fri_diag[i];
  s_fri_diag[3]=s_fri_diag[5]=0.0f;
  for (uint8_t i=18;i<=22;i++) s_fri_diag[i]=0.0f;
  taskEXIT_CRITICAL();
}

void Booster_FrictionGroupCtrl(void)
{
  friction_sample_t sample[FRI_WHEEL_NUM];
  float trim[FRI_WHEEL_NUM];
  /* 三轮测量、时间戳和目标用同一次快照，避免三次计算间CAN更新造成额外偏差。 */
  taskENTER_CRITICAL();
  const uint32_t now = HAL_GetTick();
  for (uint8_t i=0;i<FRI_WHEEL_NUM;i++) {
    const RM_Motor_t *m=&BOOSTER.RM_Motor[Fri_MotorID[i]-1];
    sample[i].speed=m->RM_MotorMeasure.speed;
    sample[i].rx_tick=m->RM_MotorMeasure.rx_tick;
    sample[i].target=Booster_GetWheelSetpoint(i);
  }
  taskEXIT_CRITICAL();
  uint32_t sync_block=FrictionSync_Compute(sample, now, trim);
  uint8_t ramp_ready=1;
  for (uint8_t i=0;i<FRI_WHEEL_NUM;i++)
    if (s_fri_ref_ramp[i]!=sample[i].target || s_fri_div_latch[i]) ramp_ready=0;
  if (!ramp_ready) sync_block |= 64U;
  if (!ramp_ready)
    for (uint8_t i=0;i<FRI_WHEEL_NUM;i++) trim[i]=0.0f;
  for (uint8_t i=0;i<FRI_WHEEL_NUM;i++)
    Booster_FrictionCtrl(i, &sample[i], now, trim[i]);
  for (uint8_t i=0;i<FRI_WHEEL_NUM;i++)
    if (s_fri_div_latch[i]) sync_block |= 64U;
  uint32_t youngest=65535U, oldest=0U, limit_mask=0U;
  taskENTER_CRITICAL();
  for (uint8_t i=0;i<FRI_WHEEL_NUM;i++) {
    uint32_t age=sample[i].rx_tick ? now-sample[i].rx_tick : 65535U;
    if (age>65535U) age=65535U;
    if (age<youngest) youngest=age;
    if (age>oldest) oldest=age;
    const PID_t *pid=&BOOSTER.RM_Motor[Fri_MotorID[i]-1].MotorCtrl_Speed;
    s_fri_diag[6+i]=(float)age;
    s_fri_diag[11+i]=s_fri_applied_trim[i];
    s_fri_diag[14+i]=Fri_DirSign[i]*pid->Ref;
    if (fabsf(pid->Output)>=0.98f*16384.0f) {
      limit_mask |= 1U<<i;
      s_fri_diag[18+i]+=1.0f;
    }
  }
  s_fri_diag[9]=(float)(oldest-youngest);
  s_fri_diag[10]=(float)sync_block;
  s_fri_diag[17]=(float)limit_mask;
  if (sync_block==0U) s_fri_diag[21]+=1.0f;
  taskEXIT_CRITICAL();
}

