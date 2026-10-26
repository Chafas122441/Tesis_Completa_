/**
 * @file gripper_manual.h
 * @brief Interfaz del servicio de los tres actuadores RC.
 */
#ifndef GRIPPER_MANUAL_H
#define GRIPPER_MANUAL_H
#include "stm32f4xx_hal.h"
#include "delta_stream_protocol.h"
void GripperManual_Init(void);
void GripperManual_Stop(void);
void GripperManual_Tick(void);
int GripperManual_Active(void);
/* Read-only lease age; never renews or re-enables PWM. UINT32_MAX if inactive. */
uint32_t GripperManual_UpdateAgeMs(void);
unsigned GripperManual_Channel(void);
/* Last accepted commands, NOT measured angles. */
unsigned GripperManual_Pulse(unsigned channel);
DeltaResultCode GripperManual_Begin(unsigned channel, unsigned pulse);
DeltaResultCode GripperManual_Set(unsigned channel, unsigned pulse);
DeltaResultCode GripperManual_GroupBegin(const uint16_t us[3]);
DeltaResultCode GripperManual_GroupSet(const uint16_t us[3]);
/* Call only after explicit AUTO STREAM/BEGIN permission checks. */
DeltaResultCode GripperManual_AutoGroupSet(const uint16_t us[3]);
#endif
