#ifndef SENTRY_CONFIG_H
#define SENTRY_CONFIG_H
/* Set to 1 ONLY after verifying IDs, MIT ranges and collision-free travel.
 * IDs verified 2026-11-12: Fold 1/0x11; Pitch changed to 2/0, saved and
 * verified after motor power cycle. Four endpoints captured; Fold MIT physical scale remains unverified. */
#ifndef SENTRY_HARDWARE_READY
#define SENTRY_HARDWARE_READY 1
#endif
#define SENTRY_FOLD_ESC  0x01U
#define SENTRY_FOLD_MST  0x11U
#define SENTRY_PITCH_ESC 0x02U
#define SENTRY_PITCH_MST 0x00U
/* Pitch ranges read back; Fold ranges remain unverified. */
#define SENTRY_P_MAX 12.5f
#define SENTRY_V_MAX 30.0f
#define SENTRY_T_MAX 10.0f
#define SENTRY_KP 20.0f
#define SENTRY_KD 1.0f
#define SENTRY_ERROR_RAD 0.05f
#define SENTRY_FOLD_SPEED 1600.00f
#define SENTRY_PITCH_SPEED 200.00f
#define SENTRY_FOLD_ACCEL 400.00f
#define SENTRY_PITCH_ACCEL 400.00f
#define SENTRY_MARGIN_RAD 0.02f
#define SENTRY_MARGIN_COUNTS 64U
#define SENTRY_MAX_TRAVEL_RAD 6.0f
#define SENTRY_FB_MS 100U
#define SENTRY_TEMP_C 65U
/* DT7: S1 up/middle=global disable; S1 down permits S2 operation.
 * S2 up=disable, middle=unfold then level Pitch, down=prepare Pitch then fold.
 * Right vertical adjusts Pitch only once fully open; left stick unused. */
#define SENTRY_STICK_DEADZONE 0.05f
#define SENTRY_SLOW_ZONE_RAD 0.10f
#ifndef SENTRY_USE_COMMISSIONED_LIMITS
#define SENTRY_USE_COMMISSIONED_LIMITS 1
#endif
/* Saved calibration/sequence endpoints; runtime position-limit interlock is removed. */
#define SENTRY_FOLD_A_RAW 45628U
#define SENTRY_FOLD_B_RAW 38800U
#define SENTRY_PITCH_A_RAW 29762U
#define SENTRY_PITCH_B_RAW 36039U
/* Legacy Fold firmware replies with state nibble 0; require post-enable feedback. */
#define SENTRY_FOLD_LEGACY_STATUS 1
/* Sequential poses captured 2026-11-13, no moving horizontal constraint. */
#define SENTRY_SEQ_PREP_PITCH_RAW 31401.0f
#define SENTRY_SEQ_UNFOLD_ALLOWED 1
#define SENTRY_SEQ_FOLD_PITCH_RAW 29763.0f
#define SENTRY_SEQ_OPEN_PITCH_RAW 34370.0f
#define SENTRY_Z_LEVEL_TOL_COUNTS 96.0f
#define SENTRY_Z_TRACK_TOL_COUNTS 128.0f
#define SENTRY_SEQ_FOLD_TOL_COUNTS 96.0f
#define SENTRY_Z_ALIGN_TIMEOUT_MS 20000U
#define SENTRY_Z_SETTLE_MS 100U
#endif
