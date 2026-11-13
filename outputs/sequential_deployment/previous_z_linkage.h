#ifndef SENTRY_Z_LINKAGE_H
#define SENTRY_Z_LINKAGE_H
#include <stdint.h>
/* Coordinates are raw encoder counts. The map is not a gear ratio. */
typedef enum { Z_OFF=0,Z_LEVELING=1,Z_LINKED=2,Z_OPEN=3,Z_FOLDED=4 } ZMode;
typedef struct {float p,v,goal;} ZTrajectory;
typedef struct {
 ZTrajectory fold,pitch;
 ZMode mode;uint8_t initialized,align_from_open;
 uint32_t align_started,settled,open_settled;
} ZLinkage;
typedef struct {
 float fold_low,fold_high,pitch_low,pitch_high,margin;
 float fold_speed,pitch_speed,accel,error,slow,level_tolerance,track_tolerance,still_rate,end_tolerance;
 uint32_t settle_ms,align_timeout_ms;
} ZConfig;
float ZLink_Level(float fold_raw);
float ZLink_Inverse(float pitch_raw);
float ZLink_OperatingLow(const ZConfig *c);
float ZLink_OperatingHigh(const ZConfig *c);
int ZLink_ConfigValid(const ZConfig *c);
void ZLink_Reset(ZLinkage *z);
/* 0=OK, 1=invalid/unreachable path, 2=leveling timeout. */
int ZLink_Step(ZLinkage *z,const ZConfig *c,float fold,float pitch,float vf,float vp,
               float left,float right,float dt,uint32_t now);
#endif
