#include "sentry.h"
#ifdef SENTRY_STM32H7
#include "board.h"
#endif
#include "motion.h"
#include "z_linkage.h"
#include "storage.h"
#include "gimbal_config.h"
#include "app_DM4310.h"
#include "sentry_remote.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* All state transitions and flash writes belong to the control task.
 * CAN/USB interrupts publish feedback/bytes only. */
typedef struct { DM4310_MotorMeasure_t m; uint32_t tick,counter; uint8_t seen; } Feedback;
typedef struct { float p,v,goal; uint32_t settled,blocked,stalled; uint8_t active; } Axis;
typedef struct {
    int pos[2],raw[2],goal[2],end[4],endraw[4],err[2],temp[2];
    uint32_t age[2],mask,seq;
    unsigned mode,valid,armed,fault,result,state,rc,master,ready,s1;
    int stick[2];
    unsigned z_mode;int level_raw,level_error_raw;unsigned s2,fold_pitch_target,open_pitch_target,prep_pitch_target,unfold_allowed,parallel_motion;
} Status;
static Feedback feedback[2];
static Axis axis[2];
static ZLinkage linkage;
static SentryCalibration cal;
static Status status;
static uint8_t calibration=1,valid,armed,fault;
static uint8_t neutral[2],enable_sent[2];
static uint32_t enable_rx_count[2];
static uint32_t enabling_since;
static uint32_t mode_tick,tx_fail[2],poll_tick,enable_tick[2],disable_tick[2];
static const uint16_t esc[2]={SENTRY_FOLD_ESC,SENTRY_PITCH_ESC};
static const uint16_t mst[2]={SENTRY_FOLD_MST,SENTRY_PITCH_MST};
static const dm4310_range_t range={-SENTRY_P_MAX,SENTRY_P_MAX,-SENTRY_V_MAX,SENTRY_V_MAX,
    -SENTRY_T_MAX,SENTRY_T_MAX,500,5};
static volatile uint16_t rx_write,rx_read;
static uint8_t rx_ring[256];
static volatile uint8_t rx_overflow;
static char line[48];
static unsigned line_used;
static uint8_t line_bad;
static unsigned result,sequence;
/* result: 0 OK, 1 unknown, 2 requires RC online/master off, 3 requires CAL,
 * 4 feedback/motor not disabled/stationary, 5 invalid endpoints, 6 flash failure,
 * 7 dirty/unsaved, 8 input overflow. fault: 1 link,2 timing,3 feedback,
 * 4 drive/temperature,5 travel,6 TX,7 stall/move timeout. */

static float RawPosition(uint32_t raw){return range.p_min+(range.p_max-range.p_min)*(float)raw/65535.0f;}
static uint32_t HardRaw(unsigned i,unsigned end)
{
    const uint32_t points[2][2]={{SENTRY_FOLD_A_RAW,SENTRY_FOLD_B_RAW},{SENTRY_PITCH_A_RAW,SENTRY_PITCH_B_RAW}};
    return points[i][end];
}
static ZConfig LinkSettings(const SentryCalibration *c)
{
    const float counts=65535.0f/(range.p_max-range.p_min);
    ZConfig z={
        fminf(c->raw[0][0],c->raw[0][1]),fmaxf(c->raw[0][0],c->raw[0][1]),
        fminf(c->raw[1][0],c->raw[1][1]),fmaxf(c->raw[1][0],c->raw[1][1]),
        SENTRY_MARGIN_COUNTS,SENTRY_FOLD_SPEED*counts,SENTRY_PITCH_SPEED*counts,
        SENTRY_FOLD_ACCEL*counts,SENTRY_PITCH_ACCEL*counts,SENTRY_ERROR_RAD*counts,SENTRY_SLOW_ZONE_RAD*counts,
        SENTRY_Z_LEVEL_TOL_COUNTS,SENTRY_Z_TRACK_TOL_COUNTS,80.0f,SENTRY_SEQ_FOLD_TOL_COUNTS,
        SENTRY_Z_SETTLE_MS,SENTRY_Z_ALIGN_TIMEOUT_MS
    };
    return z;
}
static int CalValid(const SentryCalibration *c)
{
    if(!Sentry_CalValid(c,SENTRY_P_MAX,SENTRY_MAX_TRAVEL_RAD,SENTRY_MARGIN_RAD))return 0;
#if SENTRY_USE_COMMISSIONED_LIMITS
    for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++) {
        uint32_t lo=HardRaw(i,0)<HardRaw(i,1)?HardRaw(i,0):HardRaw(i,1);
        uint32_t hi=HardRaw(i,0)>HardRaw(i,1)?HardRaw(i,0):HardRaw(i,1);
        if((c->end[i][1]-c->end[i][0])*((float)HardRaw(i,1)-(float)HardRaw(i,0))<=0)return 0;
        if(c->raw[i][j]<lo || c->raw[i][j]>hi || fabsf(c->end[i][j]-RawPosition(c->raw[i][j]))>.001f)return 0;
    }
#endif
    for(unsigned i=0;i<2;i++) {
        if(fabsf((float)c->raw[i][0]-(float)c->raw[i][1])<2*SENTRY_MARGIN_COUNTS+16)return 0;
    }
    ZConfig zc=LinkSettings(c);
    return ZLink_ConfigValid(&zc);
}
static int DriveReady(unsigned i,const Feedback *f)
{
    if(f->m.ERR==ERR_Enable)return 1;
    return i==0 && SENTRY_FOLD_LEGACY_STATUS && f->m.ERR==ERR_Disable &&
           enable_sent[i] && f->counter!=enable_rx_count[i];
}
static int ConfigOK(void)
{
    if(!SENTRY_HARDWARE_READY) return 0;
    for(unsigned i=0;i<2;i++) {
        if(!esc[i]||esc[i]>=0x7ff || mst[i]>=0x7ff) return 0;
        if(esc[i]==mst[i] || esc[i]==esc[1-i] || mst[i]==mst[1-i] || esc[i]==mst[1-i]) return 0;
    }
    return SENTRY_P_MAX>0 && SENTRY_V_MAX>0 && SENTRY_T_MAX>0 && SENTRY_KP>0 && SENTRY_KD>0;
}
static int Fresh(const Feedback *f,uint32_t now)
{ return f->seen && now-f->tick<SENTRY_FB_MS && isfinite(f->m.Position) && isfinite(f->m.Velocity); }
static void Stop(uint32_t now)
{
#ifdef SENTRY_STM32H7
    Board_CanStop();
#endif
    enabling_since=0;ZLink_Reset(&linkage);
    for(unsigned i=0;i<2;i++) {
        axis[i].active=0; axis[i].v=0;neutral[i]=0;enable_sent[i]=0;tx_fail[i]=0; axis[i].settled=axis[i].blocked=axis[i].stalled=0;
        if(ConfigOK() && now-disable_tick[i]>=20U && DM4310_SetEnable(&DM4310_DEFAULT_BUS,esc[i],0)) disable_tick[i]=now;
    }
}
void Sentry_InvalidateFeedback(void)
{ feedback[0].seen=feedback[1].seen=0; }
static int LoadCalibration(void)
{
    int loaded=Sentry_Load(&cal) && CalValid(&cal);
#if SENTRY_USE_COMMISSIONED_LIMITS
    if(!loaded) {
        for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++) {
            cal.raw[i][j]=HardRaw(i,j);cal.end[i][j]=RawPosition(cal.raw[i][j]);
        }
        cal.mask=15;loaded=CalValid(&cal);
    }
#endif
    return loaded;
}
void Sentry_Init(void)
{
    valid=LoadCalibration();
    /* Valid limits permit remote mode, but boot requires S1 up/middle first. */
    calibration=!valid;
}
void Sentry_Rx(uint16_t id,const uint8_t *data,uint8_t n)
{
    if(n!=8) return;
    for(unsigned i=0;i<2;i++) if(id==mst[i] && (data[0]&15U)==(esc[i]&15U)) {
        DM4310_Get_Ranged(&feedback[i].m,data,&range);
        feedback[i].tick=HAL_GetTick(); feedback[i].counter++;feedback[i].seen=1;
    }
}
void Sentry_UsbRx(const uint8_t *data,uint32_t n)
{
    while(n--) {
        uint16_t next=(rx_write+1U)&255U;
        if(next==rx_read) { rx_overflow=1; return; }
        rx_ring[rx_write]=*data++; __DMB(); rx_write=next;
    }
}
static int Disabled(const Feedback *f,uint32_t now)
{
    return Fresh(f,now) && f->m.ERR==ERR_Disable && fabsf(f->m.Velocity)<0.03f &&
        now-mode_tick>=200U && (int32_t)(f->tick-mode_tick)>=0;
}
static void Command(const char *s,const Feedback f[2],uint32_t now)
{
    sequence++; result=0;
    if(!strcmp(s,"STATUS")) return;
    if(!SentryRemote_Disabled(HAL_GetTick())) { result=2; return; }
    if(!strcmp(s,"CAL BEGIN")) {
        calibration=1; armed=0; valid=0; memset(&cal,0,sizeof(cal)); mode_tick=now; Stop(now); return;
    }
    if(!strcmp(s,"CAL CANCEL")) {
        calibration=1; armed=0; memset(&cal,0,sizeof(cal));valid=LoadCalibration();mode_tick=now;return;
    }
    if(!strcmp(s,"RUN")) {
        if(!valid) { result=7;return; }
        calibration=0;armed=0;fault=0;return;
    }
    if(!calibration) { result=3;return; }
    const char *names[]={"CAL FOLD A","CAL FOLD B","CAL PITCH A","CAL PITCH B"};
    for(unsigned j=0;j<4;j++) if(!strcmp(s,names[j])) {
        unsigned i=j/2;
        if(!Disabled(&f[i],now)) { result=4;return; }
        cal.end[i][j%2]=f[i].m.Position;cal.raw[i][j%2]=(uint32_t)f[i].m.Pos_int;cal.mask|=1U<<j;valid=0;return;
    }
    if(!strcmp(s,"CAL SAVE")) {
        if(!Disabled(&f[0],now)||!Disabled(&f[1],now)) { result=4;return; }
        if(!CalValid(&cal)) {result=5;return;}
        Stop(now); valid=Sentry_Save(&cal);result=valid?0:6;
        /* Flash erase can stall execution: require new feedback and re-arm. */
        taskENTER_CRITICAL(); feedback[0].seen=feedback[1].seen=0; taskEXIT_CRITICAL();
        armed=0; mode_tick=HAL_GetTick();return;
    }
    result=1;
}
static void Commands(const Feedback f[2],uint32_t now)
{
    if(rx_overflow) {
        taskENTER_CRITICAL();rx_read=rx_write;rx_overflow=0;taskEXIT_CRITICAL();
        line_used=0;line_bad=1; result=8;sequence++;
    }
    /* Bound work per control tick; USB packets may split or combine lines. */
    for(unsigned n=0;n<64 && rx_read!=rx_write;n++) {
        uint8_t b=rx_ring[rx_read];rx_read=(rx_read+1U)&255U;
        if(b=='\r') continue;
        if(b=='\n') {
            if(line_used && !line_bad) {line[line_used]=0;Command(line,f,now);}
            line_used=0;line_bad=0;
            /* One command per tick prevents RUN/SAVE using pre-command snapshots. */
            return;
        }
        if(b<32 || b>126 || line_used>=sizeof(line)-1) {line_bad=1;result=8;}
        else if(!line_bad) line[line_used++]=(char)b;
    }
}
void Sentry_Control(float dt)
{
    Feedback f[2];uint32_t now=HAL_GetTick();
    taskENTER_CRITICAL();memcpy(f,feedback,sizeof(f));taskEXIT_CRITICAL();
    Commands(f,now);
    SentryRemote_Poll(now);
    int rc=SentryRemote_Fresh(now), master=SentryRemote_Enabled(now);
    float stick[2]={sentry_remote.s2==2?-1.0f:sentry_remote.s2==3?1.0f:0.0f,SentryRemote_Stick(1)};
    int timing=isfinite(dt)&&dt>0&&dt<=0.020f;
    if(!rc) {if(armed) fault=1;armed=0;}
    if(!timing) {if(armed) fault=2;armed=0;}
    if(!master)armed=0;
    if(SentryRemote_Disabled(now) && !calibration && valid && timing &&
       Fresh(&f[0],now) && Fresh(&f[1],now) && f[0].m.ERR==ERR_Disable && f[1].m.ERR==ERR_Disable) {armed=1;fault=0;}
    int run=ConfigOK() && !calibration && valid && armed && rc && master && timing;
    if(run && (!axis[0].active || !axis[1].active) && stick[1]!=0)fault=9;
    if(run) for(unsigned i=0;i<2;i++) {
        if(!Fresh(&f[i],now)) fault=3;
        else if(f[i].m.ERR>ERR_Enable || f[i].m.T_MOS>=SENTRY_TEMP_C || f[i].m.T_Rotor>=SENTRY_TEMP_C) fault=4;
        else if(axis[i].active && !DriveReady(i,&f[i])) fault=4;
    }
    if(fault) {armed=0;run=0;}
    if(!run) Stop(now);
    else {
        /* Enable both axes before issuing trajectories; no blind retries after a fault. */
        if(!enabling_since)enabling_since=now;
        int both=1;
        for(unsigned i=0;i<2;i++) if(!DriveReady(i,&f[i])) {
            both=0;
            if(now-enable_tick[i]>=50U) {
                enable_tick[i]=now;
                if(!neutral[i]) {
                    neutral[i]=DM4310_Send_Ranged(&DM4310_DEFAULT_BUS,esc[i],0,0,0,0,0,&range);
                    break;
                }
                taskENTER_CRITICAL();enable_rx_count[i]=feedback[i].counter;taskEXIT_CRITICAL();
                if(!DM4310_SetEnable(&DM4310_DEFAULT_BUS,esc[i],1)) {fault=6;armed=0;}
                else enable_sent[i]=1;
                break;
            }
        }
        if(!both && now-enabling_since>1000U)fault=4;
        if(both) {
            const float rad_per_count=(range.p_max-range.p_min)/65535.0f;
            ZConfig zc=LinkSettings(&cal);
            int error=ZLink_Step(&linkage,&zc,(float)f[0].m.Pos_int,(float)f[1].m.Pos_int,
                f[0].m.Velocity/rad_per_count,f[1].m.Velocity/rad_per_count,
                stick[0],stick[1],dt,now);
            if(error)fault=error==2?10:11;
            if(!fault)for(unsigned i=0;i<2;i++) {
                Axis *a=&axis[i];const ZTrajectory *t=i?&linkage.pitch:&linkage.fold;
                if(!a->active)a->blocked=now;
                /* Encoder range only: captured endpoints are sequence targets,
                 * no longer an absolute position clamp or an enable interlock. */
                uint16_t command=(uint16_t)Sentry_Clamp(roundf(t->p),0.0f,65535.0f);
                a->p=RawPosition(command);a->v=t->v*rad_per_count;
                a->goal=range.p_min+t->goal*rad_per_count;
                if(DM4310_Send_PositionRaw(&DM4310_DEFAULT_BUS,esc[i],command,a->v,0,SENTRY_KP,SENTRY_KD,&range)) {
                    a->active=1;tx_fail[i]=0;
                } else {if(!tx_fail[i])tx_fail[i]=now;if(now-tx_fail[i]>=12U)fault=6;}
                int reached=fabsf(f[i].m.Position-a->goal)<.015f && fabsf(f[i].m.Velocity)<.03f;
                if(reached) {if(!a->settled)a->settled=now;}else a->settled=0;
                if(fabsf(f[i].m.Position-a->goal)>.04f && fabsf(f[i].m.Velocity)<.02f) {
                    if(!a->stalled)a->stalled=now;
                    if(now-a->stalled>1500U)fault=7;
                }else a->stalled=0;
            }
        }
        if(fault) {armed=0;Stop(now);}
    }
    /* Query disabled drives so that manual calibration does not require motion.
     * Control/disable frames have priority over one round-robin query. */
    if(ConfigOK() && now-poll_tick>=25U) {
        static unsigned i;
        uint8_t q[8]={(uint8_t)esc[i],(uint8_t)(esc[i]>>8),0xcc,0};
        if(DM4310_SendFrame(&DM4310_DEFAULT_BUS,0x7ff,q)) {i^=1;poll_tick=now;}
    }
    status.mode=calibration;status.valid=valid;status.armed=armed;status.fault=fault;
    status.mask=cal.mask;status.seq=sequence;status.result=result;status.rc=rc;status.master=master;status.ready=ConfigOK();
    status.prep_pitch_target=0;status.parallel_motion=1;status.unfold_allowed=SENTRY_SEQ_UNFOLD_ALLOWED;
    status.fold_pitch_target=(unsigned)SENTRY_SEQ_FOLD_PITCH_RAW;status.open_pitch_target=(unsigned)SENTRY_SEQ_OPEN_PITCH_RAW;
    status.z_mode=linkage.mode;status.level_raw=(int)lroundf(SENTRY_SEQ_OPEN_PITCH_RAW);
    status.level_error_raw=f[1].m.Pos_int-status.level_raw;
    status.state=!axis[0].active?0:axis[0].settled && now-axis[0].settled>=150U?2:1;
    status.s1=sentry_remote.s1;status.s2=sentry_remote.s2;status.stick[0]=(int)(stick[0]*1000);status.stick[1]=(int)(stick[1]*1000);
    for(unsigned i=0;i<2;i++) {
        status.pos[i]=(int)(f[i].m.Position*1000);status.raw[i]=f[i].m.Pos_int;
        status.age[i]=f[i].seen?now-f[i].tick:0xffffffffU;
        status.err[i]=f[i].m.ERR;status.temp[i]=f[i].m.T_MOS>f[i].m.T_Rotor?f[i].m.T_MOS:f[i].m.T_Rotor;
        status.goal[i]=(int)(axis[i].goal*1000);
        for(unsigned j=0;j<2;j++) {status.end[2*i+j]=(int)(cal.end[i][j]*1000);status.endraw[2*i+j]=(int)cal.raw[i][j];}
    }
}
void Sentry_Background(void)
{
    static uint32_t last;static char output[768];Status s;
    uint32_t now=HAL_GetTick();
    if(now-last<100U || !CDC_TransmitReady_FS())return;
    taskENTER_CRITICAL();s=status;taskEXIT_CRITICAL();
    int n=snprintf(output,sizeof(output),
      "SENTRY seq=%lu result=%u cal=%u valid=%u mask=%lu ready=%u rc=%u master=%u armed=%u fault=%u fold_state=%u "
      "fold_raw=%d fold_mrad=%d fold_goal=%d fold_age=%lu pitch_raw=%d pitch_mrad=%d pitch_goal=%d pitch_age=%lu ends=%d,%d,%d,%d end_raw=%d,%d,%d,%d err=%d,%d temp=%d,%d s1=%u sticks=%d,%d z_mode=%u level_raw=%d level_err=%d s2=%u fold_pitch_target=%u open_pitch_target=%u prep_pitch_target=%u unfold_allowed=%u parallel_motion=%u\r\n",
      (unsigned long)s.seq,s.result,s.mode,s.valid,(unsigned long)s.mask,s.ready,s.rc,s.master,s.armed,s.fault,s.state,
      s.raw[0],s.pos[0],s.goal[0],(unsigned long)s.age[0],s.raw[1],s.pos[1],s.goal[1],(unsigned long)s.age[1],
      s.end[0],s.end[1],s.end[2],s.end[3],s.endraw[0],s.endraw[1],s.endraw[2],s.endraw[3],s.err[0],s.err[1],s.temp[0],s.temp[1],s.s1,s.stick[0],s.stick[1],s.z_mode,s.level_raw,s.level_error_raw,s.s2,s.fold_pitch_target,s.open_pitch_target,s.prep_pitch_target,s.unfold_allowed,s.parallel_motion);
    if(n>0 && n<(int)sizeof(output) && CDC_Transmit_FS((uint8_t *)output,(uint16_t)n)==USBD_OK) last=now;
}
