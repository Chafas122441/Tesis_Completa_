/**
 * @file gripper_limits.h
 * @brief Limites PWM y contratos de los tres canales RC de la pala.
 */
#ifndef GRIPPER_LIMITS_H
#define GRIPPER_LIMITS_H
/* Calibration envelope ONLY. Not mechanically validated endpoints.
 * Both copies must match. Expanded calibration window, NOT safe endpoints.
 * Channels: 1 Z/PA15, 2 X/PA1, 3 pusher/PB8.
 * Values are microseconds; no universal pulse-to-angle conversion exists. */
#define GRIP_CENTER_US 1500u
#define GRIP_MAX_STEP_US 10u
#define GRIP_GROUP_MAX_STEP_US 50u
#define GRIP_AUTO_MAX_STEP_US 150u /* faster bounded AUTO ramp; explicit AUTO permit only */
#define GRIP_GROUP_JOG_STEP_US 20u /* live held-key MANUAL; no presets */
#define GRIP_JOG_INPUT_MAX_AGE_MS 150u
#define GRIP_GROUP_PERIOD_MS 25u
#define GRIP_GROUP_ACK_GUARD_MS (GRIP_GROUP_PERIOD_MS + 5u)
#define GRIP_PERIOD_MS 50u
#define GRIP_ACK_GUARD_MS (GRIP_PERIOD_MS + 5u)
#define GRIP_WATCHDOG_MS 300u
#define GRIP_RUNTIME_RENEW_MS 80u
#define GRIP_MAGIC 0x4752u
#define GRIP_STREAM_MAGIC 0x4753u /* explicit live MANUAL permit, same22bytes */
#define GRIP_AUTO_MAGIC 0x4754u /* AUTO group ramp; requires AUTOMATIC STREAM */

static const unsigned short GRIP_MIN_US[3] = {500,500,500};
static const unsigned short GRIP_MAX_US[3] = {2500,2500,2500};
#endif
