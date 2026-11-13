#include "RC_task.h"
uint8_t RC_MasterEnabled(void)
{
    if (!RC_LinkFresh()) return BOOSTER_RC_ONLY ? 0 : 1;       /* 合体版失联停机；USB 台架版才允许回退。 */
    return (RC_Data.ET07_DATA.SB_CH6 > 1500) ? 1 : 0;
}

/**
 * @description  : CH8 旋钮 → 三路摩擦轮统一转速(rpm)
 *                 旋钮量程 -100..+100（中位 0）。中位 5750，顺时针到底 10000；
 *                 逆时针 -80 以下到 -100 是死区，直接停转。
 *                 CH6 总使能关 / 链路掉线时返回 0。
 * @return        {int16_t} rpm，0 = 停
 */
int16_t RC_FrictionWheelRpm(void)
{
    if (!RC_LinkFresh() || !RC_MasterEnabled()) return 0;

    int32_t n = ((int32_t)RC_Data.ET07_DATA.SD_CH8 - RC_CH8_RAW_MID) * 100
                / (RC_CH8_RAW_MAX - RC_CH8_RAW_MID);
    if (n >  100) n =  100;
    if (n < -100) n = -100;

    if (n <= RC_FRI_DEAD_NEG) return 0;                       /* -100..-80：停 */
    if (n < 0)                                                 /* -80..0：0 → 中位标定转速 */
        return (int16_t)(RC_FRI_RPM_MID * (n - RC_FRI_DEAD_NEG)
                         / (0 - RC_FRI_DEAD_NEG));
    return (int16_t)(RC_FRI_RPM_MID                            /* 0..100：中位标定转速 → 10000 */
                     + (RC_FRI_RPM_MAX - RC_FRI_RPM_MID) * n / 100);
}
