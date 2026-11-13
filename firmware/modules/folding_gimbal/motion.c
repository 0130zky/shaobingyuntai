#include "motion.h"
#include <math.h>
float Sentry_Clamp(float x,float a,float b) { return fminf(fmaxf(x,a),b); }
int Sentry_CalValid(const SentryCalibration *c,float range,float travel,float margin)
{
    if(c->mask!=15U) return 0;
    for(int i=0;i<2;i++) {
        float a=c->end[i][0],b=c->end[i][1],d=fabsf(b-a);
        if(!isfinite(a)||!isfinite(b)||fabsf(a)>range||fabsf(b)>range||
           d<=2*margin+0.02f||d>travel) return 0;
    }
    return 1;
}
void Sentry_Move(float *p,float *v,float goal,float speed,float accel,float dt)
{
    float d=goal-*p;
    float desired=copysignf(fminf(speed,sqrtf(2*accel*fabsf(d))),d);
    *v+=Sentry_Clamp(desired-*v,-accel*dt,accel*dt);
    float step=*v*dt;
    if((d>=0 && step>=d)||(d<=0 && step<=d)) { *p=goal; *v=0; }
    else *p+=step;
}

/* Clip position lead without repeatedly treating the feedback window as an
 * endpoint. Keep the velocity that the bounded position actually advanced at. */
static void Follow(float *p,float *v,float measured,float low,float high,float error,float dt)
{
    float previous=*p;
    float proposed=*p+*v*dt;
    *p=Sentry_Clamp(proposed,fmaxf(low,measured-error),fminf(high,measured+error));
    float actual=(*p-previous)/dt;
    if(*p!=proposed && fabsf(actual)<fabsf(*v)) *v=actual;
}

void Sentry_Track(float *p,float *v,float goal,float measured,float speed,float accel,
                  float low,float high,float error,float dt)
{
    float distance=goal-*p;
    float maximum=fminf(speed,sqrtf(2*accel*error));
    float desired=copysignf(fminf(maximum,sqrtf(2*accel*fabsf(distance))),distance);
    *v+=Sentry_Clamp(desired-*v,-accel*dt,accel*dt);
    /* Stop at the real destination only; the measured +/- error window moves. */
    if(distance>=0 && *v>=0) high=fminf(high,goal);
    if(distance<=0 && *v<=0) low=fmaxf(low,goal);
    Follow(p,v,measured,low,high,error,dt);
    if(*p==goal) *v=0;
}

/* Manual velocity is proportional to stick amplitude. Avoid integrating a
 * distant goal and then chasing it at a fixed speed, which saturates small sticks. */
void Sentry_Jog(float *p,float *v,float *goal,float measured,float stick,float sign,
                float speed,float accel,float low,float high,float error,float slow,float dt)
{
    /* Full throw uses the existing following/braking envelope; intermediate
     * throws scale this same envelope linearly rather than saturating early. */
    float maximum=fminf(speed,sqrtf(2*accel*error));
    float desired=Sentry_Clamp(stick,-1,1)*sign*maximum;
    float distance=desired>0?high-measured:measured-low;
    desired*=Sentry_Clamp(distance/slow,0,1);
    float braking=sqrtf(2*accel*fmaxf(distance,0));
    desired=Sentry_Clamp(desired,-braking,braking);
    *v+=Sentry_Clamp(desired-*v,-accel*dt,accel*dt);
    Follow(p,v,measured,low,high,error,dt);
    *goal=*p; /* No queued motion is left behind on release or reversal. */
}
