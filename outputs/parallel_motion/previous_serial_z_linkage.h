#ifndef SENTRY_Z_LINKAGE_H
#define SENTRY_Z_LINKAGE_H
#include <stdint.h>
/* Raw encoder coordinates. Only one axis advances in each automatic phase. */
typedef enum { Z_OFF=0,Z_LEVELING=1,Z_LINKED=2,Z_OPEN=3,Z_FOLDED=4,
               Z_UNFOLDING=5,Z_OPEN_LEVELING=6,Z_HOLD=7,Z_FOLD_PITCH=8 } ZMode;
/* Legacy telemetry names: LEVELING=prepare clearance Pitch, LINKED=closing Fold. */
typedef struct {float p,v,goal;} ZTrajectory;
typedef struct {
 ZTrajectory fold,pitch;
 ZMode mode;uint8_t initialized;
 uint32_t align_started,settled;
} ZLinkage;
typedef struct {
 float fold_low,fold_high,pitch_low,pitch_high,margin;
 float fold_speed,pitch_speed,accel,error,slow,level_tolerance,track_tolerance,still_rate,end_tolerance;
 uint32_t settle_ms,align_timeout_ms;
} ZConfig;
float ZLink_OperatingLow(const ZConfig *c);
float ZLink_OperatingHigh(const ZConfig *c);
int ZLink_ConfigValid(const ZConfig *c);
void ZLink_Reset(ZLinkage *z);
/* 0=OK, 1=invalid/protocol coordinate, 2=phase settling timeout. */
int ZLink_Step(ZLinkage *z,const ZConfig *c,float fold,float pitch,float vf,float vp,
               float left,float right,float dt,uint32_t now);
#endif
