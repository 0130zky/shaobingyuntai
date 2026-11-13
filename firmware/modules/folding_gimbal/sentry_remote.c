/* DJI DT7/DR16 DBUS, following the supplied DemoCode/Bsp/remote.c layout.
 * All publication happens in the control task, not the UART ISR. */
#include "sentry_remote.h"
#include "gimbal_config.h"
#include <math.h>
SentryRemoteInput sentry_remote;
static unsigned consecutive;
int SentryRemote_Decode(const uint8_t f[18],uint32_t tick)
{
 SentryRemoteInput next=sentry_remote;
 for(unsigned i=0;i<4;i++) {
  unsigned bit=i*11U,offset=bit/8U;
  uint32_t word=f[offset]|((uint32_t)f[offset+1]<<8)|((uint32_t)f[offset+2]<<16);
  next.channel[i]=(word>>(bit%8U))&2047U;
  if(next.channel[i]<364 || next.channel[i]>1684)goto invalid;
 }
 next.s1=(f[5]>>6)&3;next.s2=(f[5]>>4)&3;
 if(!next.s1||!next.s2||f[12]>1||f[13]>1)goto invalid;
 if(!sentry_remote.frames || tick-sentry_remote.tick>40U)consecutive=0;
 if(consecutive<2)consecutive++;
 next.seen=consecutive>=2;next.tick=tick;next.frames++;
 sentry_remote=next;return 1;
invalid:
 sentry_remote.invalid++;SentryRemote_Invalidate();return 0;
}
void SentryRemote_Invalidate(void){sentry_remote.seen=0;consecutive=0;}
int SentryRemote_Fresh(uint32_t now){return sentry_remote.seen && now-sentry_remote.tick<100U;}
int SentryRemote_Enabled(uint32_t now){return SentryRemote_Fresh(now)&&sentry_remote.s1==2&&(sentry_remote.s2==2||(SENTRY_SEQ_UNFOLD_ALLOWED&&sentry_remote.s2==3));}
int SentryRemote_Disabled(uint32_t now){return SentryRemote_Fresh(now)&&(sentry_remote.s1==1||sentry_remote.s1==3||sentry_remote.s2==1||(!SENTRY_SEQ_UNFOLD_ALLOWED&&sentry_remote.s2==3));}
float SentryRemote_Stick(unsigned axis)
{
 /* Raw channel amplitude -> continuous signed value after a rescaled dead zone. */
 float x=((float)sentry_remote.channel[axis?1:3]-1024.0f)/660.0f;
 x=fminf(fmaxf(x,-1),1);
 const float dead=SENTRY_STICK_DEADZONE;
 if(fabsf(x)<=dead)return 0;
 return copysignf((fabsf(x)-dead)/(1.0f-dead),x);
}
#ifndef SENTRY_REMOTE_LEGACY
void SentryRemote_Poll(uint32_t now){(void)now;}
#endif
