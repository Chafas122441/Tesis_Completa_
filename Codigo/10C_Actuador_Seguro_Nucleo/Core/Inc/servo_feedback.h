/**
 * @file servo_feedback.h
 * @brief Interfaz de escritura y retroalimentacion de los servos HTS-35H.
 */
#ifndef SERVO_FEEDBACK_H
#define SERVO_FEEDBACK_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_FEEDBACK_FRAME_CAPACITY 16u

typedef enum {
  SERVO_READ_OK = 0,
  SERVO_READ_TX_ERROR,
  SERVO_READ_RX_TIMEOUT,
  SERVO_READ_RX_ERROR,
  SERVO_READ_BAD_LENGTH,
  SERVO_READ_BAD_ID,
  SERVO_READ_BAD_COMMAND,
  SERVO_READ_BAD_CHECKSUM,
  SERVO_READ_BAD_POSITION
} ServoReadStatus;

typedef struct {
  ServoReadStatus status;
  int16_t position;
  uint8_t frame[SERVO_FEEDBACK_FRAME_CAPACITY];
  uint8_t frameLength;
  uint32_t elapsedMs;
} ServoPositionReading;

HAL_StatusTypeDef ServoSendMove(UART_HandleTypeDef *port,
                                uint8_t servoId,
                                uint16_t raw,
                                uint16_t moveTimeMs,
                                uint8_t frame[10]);


HAL_StatusTypeDef ServoSendMoveBounded(UART_HandleTypeDef *port,
                                       uint8_t servoId,
                                       uint16_t raw,
                                       uint16_t moveTimeMs,
                                       uint8_t frame[10],
                                       uint32_t transmitTimeoutMs);

/* Waits for quietMs without any RX activity (max maxWaitMs). HAL_TIMEOUT means
   the line stayed busy: DIRECT then does not transmit (fail closed).
   drainedBytes (nullable) counts bytes/flags discarded while waiting. */
HAL_StatusTypeDef ServoWaitBusIdle(UART_HandleTypeDef *port,
                                   uint32_t quietMs,
                                   uint32_t maxWaitMs,
                                   uint16_t *drainedBytes);

/* MOVE_STOP 0x0C: best effort, not confirmed on HTS-35H hardware. */
HAL_StatusTypeDef ServoStopMove(UART_HandleTypeDef *port,
                                uint8_t servoId,
                                uint8_t frame[6]);

ServoReadStatus ServoReadPosition(UART_HandleTypeDef *port,
                                  uint8_t servoId,
                                  ServoPositionReading *reading,
                                  uint32_t timeoutMs);

ServoReadStatus ServoReadPositionBounded(UART_HandleTypeDef *port,
                                         uint8_t servoId,
                                         ServoPositionReading *reading,
                                         uint32_t timeoutMs,
                                         uint32_t transmitTimeoutMs);

const char *ServoReadStatusName(ServoReadStatus status);

#ifdef __cplusplus
}
#endif

#endif
