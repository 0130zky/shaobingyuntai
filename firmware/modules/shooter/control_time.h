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
 *  @description ：控制拍时间有效性：周期检查、异常间隔和积分授权。
 */
#ifndef CONTROL_TIME_H
#define CONTROL_TIME_H
#include <stdint.h>
#include <math.h>

#define SHOOTER_MAX_CONTROL_GAP_MS 20U
#define SHOOTER_MAX_CONTROL_DT_S 0.020f
typedef struct {
  uint32_t previous_ms;
  float dt_s;
  uint8_t started, fault;
} control_time_t;

/* No unobserved pause is integrated into a reference ramp or fault timer. */
static inline uint8_t ControlTime_Update(control_time_t *clock, uint32_t now_ms, float dt_s)
{
  const uint8_t valid = isfinite(dt_s) && dt_s > 0.0f && dt_s <= SHOOTER_MAX_CONTROL_DT_S &&
    (!clock->started || (uint32_t)(now_ms-clock->previous_ms) <= SHOOTER_MAX_CONTROL_GAP_MS);
  clock->previous_ms = now_ms;
  clock->started = 1;
  clock->fault = !valid;
  clock->dt_s = valid ? dt_s : 0.0f;
  return valid;
}
#endif
