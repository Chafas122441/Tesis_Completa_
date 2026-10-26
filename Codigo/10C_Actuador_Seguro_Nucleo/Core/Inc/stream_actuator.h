/**
 * @file stream_actuator.h
 * @brief Interfaz de la maquina de actuacion y sus servicios periodicos.
 */
#ifndef STREAM_ACTUATOR_H
#define STREAM_ACTUATOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void StreamActuator_Init(UART_HandleTypeDef *linkUart,
                         UART_HandleTypeDef *servo1Uart,
                         UART_HandleTypeDef *servo2Uart,
                         UART_HandleTypeDef *servo3Uart,
                         uint8_t dryRun);
void StreamActuator_Loop(void);
uint8_t StreamActuator_StreamActive(void);
/* Servicio ligero tras ACK/watchdogs y antes de lecturas HTS bloqueantes. */
void StreamActuator_SetEncoderService(void (*service)(void));
/* 1 sólo si se transmitio la trama completa; 0 conserva el envio pendiente. */
uint8_t StreamActuator_EncoderTelemetry(int32_t position, uint16_t speedTenths,
                                       uint16_t intervalMs);
void StreamActuator_OnRxComplete(UART_HandleTypeDef *huart);
void StreamActuator_OnUartError(UART_HandleTypeDef *huart);

#endif
