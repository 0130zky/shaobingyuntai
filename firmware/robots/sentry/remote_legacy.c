/* Original F407 Sentry build adapter; MC02 uses DT7 DBUS instead. */
#include "sentry_remote.h"
#include "RC_task.h"
void SentryRemote_Poll(uint32_t now)
{
 sentry_remote.seen=RC_LinkFresh();sentry_remote.tick=now;
 sentry_remote.s1=RC_Data.ET07_DATA.SB_CH6>1500?2:1;
 sentry_remote.channel[3]=RC_Data.ET07_DATA.LeftDirection_CH3;
 sentry_remote.channel[1]=RC_Data.ET07_DATA.RightDirection_CH2;
}
