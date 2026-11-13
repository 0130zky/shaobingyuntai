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
 *  @description ：三轮摩擦轮同步补偿计算接口。
 */
#ifndef FRICTION_SYNC_H
#define FRICTION_SYNC_H
#include <stdint.h>

/* 小幅交叉耦合：目标仍相同，只增强三轮之间的差速抑制。
   使用同一次快照；反馈时差过大时禁用，避免追逐旧速度。 */
#define FRI_SYNC_GAIN          (0.30f)
#define FRI_SIDE_SYNC_GAIN     (0.20f)
#define FRI_SIDE_LOAD_DROP_RPM (150.0f)
#define FRI_SYNC_TRIM_RPM      (100.0f)
#define FRI_SYNC_DEADBAND_RPM  (30.0f)
#define FRI_SYNC_FRESH_MS      (5U)
#define FRI_SYNC_SKEW_MS       (1U)

typedef struct {
  int16_t speed, target;
  uint32_t rx_tick;
} friction_sample_t;

static inline uint32_t FrictionSync_Compute(const friction_sample_t sample[3],
                                       uint32_t now, float trim[3])
{
  float speed[3], mean=0.0f, lo=0.0f, hi=0.0f;
  uint32_t youngest=0xffffffffU, oldest=0;
  int32_t common=sample[0].target;
  if (common<0) common=-common;
  for (uint8_t i=0;i<3;i++) trim[i]=0.0f;
  if (common<1000) return 1U; /* 停轮/低目标 */
  for (uint8_t i=0;i<3;i++) {
    int32_t target=sample[i].target;
    if (target<0) target=-target;
    uint32_t age=now-sample[i].rx_tick;
    if (target!=common) return 2U;
    if (!sample[i].rx_tick || age>FRI_SYNC_FRESH_MS) return 4U;
    if (age<youngest) youngest=age;
    if (age>oldest) oldest=age;
    speed[i]=(sample[i].target>0 ? 1.0f : -1.0f)*sample[i].speed;
    /* 启动低速、反转或明显超速时只保留原速度环和保护。 */
    if (speed[i]<0.5f*common || speed[i]>1.25f*common) return 16U;
    if (i==0 || speed[i]<lo) lo=speed[i];
    if (i==0 || speed[i]>hi) hi=speed[i];
    mean+=speed[i]/3.0f;
  }
  if (oldest-youngest>FRI_SYNC_SKEW_MS) return 8U;
  if (hi-lo<=FRI_SYNC_DEADBAND_RPM) return 32U;
  for (uint8_t i=0;i<3;i++)
    trim[i]=(mean-speed[i])*FRI_SYNC_GAIN;

  /* 两侧都明显掉速时，优先压缩侧轮差速。等量反向修正保留共同目标。 */
  if (speed[1]<=common-FRI_SIDE_LOAD_DROP_RPM &&
      speed[2]<=common-FRI_SIDE_LOAD_DROP_RPM) {
    const float side_trim=(speed[2]-speed[1])*FRI_SIDE_SYNC_GAIN;
    trim[1]+=side_trim;
    trim[2]-=side_trim;
  }

  /* 全组同比缩放，保持补偿和为零。 */
  float largest=0.0f;
  for (uint8_t i=0;i<3;i++) {
    const float magnitude=trim[i]<0 ? -trim[i] : trim[i];
    if (magnitude>largest) largest=magnitude;
  }
  if (largest>FRI_SYNC_TRIM_RPM)
    for (uint8_t i=0;i<3;i++) trim[i]*=FRI_SYNC_TRIM_RPM/largest;
  return 0U; /* 差速补偿实际通过门槛 */
}
#endif
