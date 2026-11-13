#ifndef SENTRY_MOTION_H
#define SENTRY_MOTION_H
#include <stdint.h>
typedef struct { float end[2][2]; uint32_t raw[2][2]; uint32_t mask; } SentryCalibration;
float Sentry_Clamp(float x, float a, float b);
int Sentry_CalValid(const SentryCalibration *c, float range, float max_travel, float margin);
/* acceleration-limited velocity, with exact endpoint clamp */
void Sentry_Move(float *p, float *v, float goal, float speed, float accel, float dt);
void Sentry_Track(float *p,float *v,float goal,float measured,float speed,float accel,
                  float low,float high,float error,float dt);
void Sentry_Jog(float *p,float *v,float *goal,float measured,float stick,float sign,
                float speed,float accel,float low,float high,float error,float slow,float dt);
#endif
