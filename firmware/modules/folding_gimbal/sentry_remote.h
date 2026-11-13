#ifndef SENTRY_REMOTE_H
#define SENTRY_REMOTE_H
#include <stdint.h>
typedef struct {
 uint16_t channel[4]; uint8_t s1,s2,seen;
 uint32_t tick,frames,invalid;
 int16_t mouse[3]; uint16_t keys; uint8_t buttons[2];
} SentryRemoteInput;
extern SentryRemoteInput sentry_remote;
int SentryRemote_Decode(const uint8_t frame[18],uint32_t arrival_ms);
void SentryRemote_Invalidate(void);
int SentryRemote_Fresh(uint32_t now);
int SentryRemote_Enabled(uint32_t now);
int SentryRemote_Disabled(uint32_t now);
float SentryRemote_Stick(unsigned axis);
uint16_t SentryRemote_Keys(uint32_t now);
void SentryRemote_Poll(uint32_t now);
#endif
