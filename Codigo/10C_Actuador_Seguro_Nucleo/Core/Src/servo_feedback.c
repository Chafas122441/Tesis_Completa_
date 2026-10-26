/**
 * @file servo_feedback.c
 * @brief Control UART half-duplex y lectura de posicion de los tres HTS-35H.
 * Gestiona cambio TX/RX, tramas, validacion y tiempos de espera por canal.
 */
#include "servo_feedback.h"

#include <string.h>

#define SERVO_FRAME_HEADER 0x55u
#define SERVO_MOVE_TIME_WRITE 0x01u

#define SERVO_MOVE_STOP 0x0Cu
#define SERVO_POS_READ 0x1Cu
#define SERVO_POSITION_RESPONSE_LENGTH 0x05u
#define SERVO_RAW_PROTOCOL_MAX 1000
#define SERVO_COMMAND_TX_TIMEOUT_MS 10u

static uint8_t ServoChecksum(const uint8_t *frame, uint8_t lastDataIndex) {
  uint16_t sum = 0u;
  for (uint8_t index = 2u; index <= lastDataIndex; ++index) {
    sum = (uint16_t)(sum + frame[index]);
  }
  return (uint8_t)(~sum);
}

static HAL_StatusTypeDef DrainReceiver(UART_HandleTypeDef *port, uint32_t timeoutMs) {
  const uint32_t startedMs = HAL_GetTick();
  __HAL_UART_CLEAR_OREFLAG(port);
  while (__HAL_UART_GET_FLAG(port, UART_FLAG_RXNE) != RESET) {
    volatile uint32_t discarded = port->Instance->DR;
    (void)discarded;
    /* Noise can begin after ServoWaitBusIdle. Do not turn a bounded DIRECT/STOP
       transmit into an unbounded drain while RXNE is continuously asserted. */
    if (HAL_GetTick() - startedMs >= timeoutMs) return HAL_TIMEOUT;
  }
  return HAL_OK;
}

HAL_StatusTypeDef ServoSendMoveBounded(UART_HandleTypeDef *port,
                                uint8_t servoId,
                                uint16_t raw,
                                uint16_t moveTimeMs,
                                uint8_t frame[10],
                                uint32_t transmitTimeoutMs) {
  frame[0] = SERVO_FRAME_HEADER;
  frame[1] = SERVO_FRAME_HEADER;
  frame[2] = servoId;
  frame[3] = 0x07u;
  frame[4] = SERVO_MOVE_TIME_WRITE;
  frame[5] = (uint8_t)(raw & 0xFFu);
  frame[6] = (uint8_t)(raw >> 8);
  frame[7] = (uint8_t)(moveTimeMs & 0xFFu);
  frame[8] = (uint8_t)(moveTimeMs >> 8);
  frame[9] = ServoChecksum(frame, 8u);

  if (DrainReceiver(port, transmitTimeoutMs) != HAL_OK) return HAL_TIMEOUT;
  if (HAL_HalfDuplex_EnableTransmitter(port) != HAL_OK) return HAL_ERROR;
  const HAL_StatusTypeDef result =
      HAL_UART_Transmit(port, frame, 10u, transmitTimeoutMs);
  if (HAL_HalfDuplex_EnableReceiver(port) != HAL_OK) return HAL_ERROR;
  __HAL_UART_CLEAR_OREFLAG(port);
  return result;
}

HAL_StatusTypeDef ServoSendMove(UART_HandleTypeDef *port,
                                uint8_t servoId,
                                uint16_t raw,
                                uint16_t moveTimeMs,
                                uint8_t frame[10]) {
  return ServoSendMoveBounded(port, servoId, raw, moveTimeMs, frame, 100u);
}

/* Half-duplex: a late reply from a previous read can still be on the wire.
   Transmitting over it corrupts the frame and HTS writes have no ACK, so a lost
   MOVE_TIME_WRITE leaves that joint still. Wait for a short quiet gap first.
   Never call __HAL_UART_CLEAR_OREFLAG here: on F4 it reads SR+DR and would
   silently consume a byte. Any RX flag is activity; reading DR after SR clears
   RXNE and ORE/NE/FE together. */
HAL_StatusTypeDef ServoWaitBusIdle(UART_HandleTypeDef *port,
                                   uint32_t quietMs,
                                   uint32_t maxWaitMs,
                                   uint16_t *drainedBytes) {
  const uint32_t startedMs = HAL_GetTick();
  uint32_t lastActivityMs = startedMs;
  uint16_t drained = 0u;
  HAL_StatusTypeDef result = HAL_ERROR;
  if (HAL_HalfDuplex_EnableReceiver(port) == HAL_OK) {
    for (;;) {
      if (__HAL_UART_GET_FLAG(port, UART_FLAG_RXNE) != RESET ||
          __HAL_UART_GET_FLAG(port, UART_FLAG_ORE) != RESET ||
          __HAL_UART_GET_FLAG(port, UART_FLAG_NE) != RESET ||
          __HAL_UART_GET_FLAG(port, UART_FLAG_FE) != RESET) {
        volatile uint32_t discarded = port->Instance->DR;
        (void)discarded;
        if (drained < UINT16_MAX) ++drained;
        lastActivityMs = HAL_GetTick();
      }
      const uint32_t now = HAL_GetTick();
      if (now - lastActivityMs >= quietMs) { result = HAL_OK; break; }
      /* Bounded even when noise never stops. */
      if (now - startedMs >= maxWaitMs) { result = HAL_TIMEOUT; break; }
    }
  }
  if (drainedBytes != NULL) *drainedBytes = drained;
  return result;
}

static HAL_StatusTypeDef ServoSendCommand(UART_HandleTypeDef *port,
                                           uint8_t servoId,
                                           uint8_t command,
                                           uint8_t frame[6]) {
  frame[0] = SERVO_FRAME_HEADER;
  frame[1] = SERVO_FRAME_HEADER;
  frame[2] = servoId;
  frame[3] = 0x03u;
  frame[4] = command;
  frame[5] = ServoChecksum(frame, 4u);

  if (DrainReceiver(port, SERVO_COMMAND_TX_TIMEOUT_MS) != HAL_OK) return HAL_TIMEOUT;
  if (HAL_HalfDuplex_EnableTransmitter(port) != HAL_OK) return HAL_ERROR;
  const HAL_StatusTypeDef result = HAL_UART_Transmit(
      port, frame, 6u, SERVO_COMMAND_TX_TIMEOUT_MS);
  if (HAL_HalfDuplex_EnableReceiver(port) != HAL_OK) return HAL_ERROR;
  __HAL_UART_CLEAR_OREFLAG(port);
  return result;
}

HAL_StatusTypeDef ServoStopMove(UART_HandleTypeDef *port,
                                uint8_t servoId,
                                uint8_t frame[6]) {
  return ServoSendCommand(port, servoId, SERVO_MOVE_STOP, frame);
}

static HAL_StatusTypeDef ReceiveByteBefore(UART_HandleTypeDef *port,
                                           uint8_t *value,
                                           uint32_t startedMs,
                                           uint32_t timeoutMs) {
  const uint32_t elapsed = HAL_GetTick() - startedMs;
  if (elapsed >= timeoutMs) return HAL_TIMEOUT;
  uint32_t remaining = timeoutMs - elapsed;
  if (remaining == 0u) remaining = 1u;
  return HAL_UART_Receive(port, value, 1u, remaining);
}

ServoReadStatus ServoReadPositionBounded(UART_HandleTypeDef *port,
                                  uint8_t servoId,
                                  ServoPositionReading *reading,
                                  uint32_t timeoutMs,
                                  uint32_t transmitTimeoutMs) {
  uint8_t request[6] = {
    SERVO_FRAME_HEADER, SERVO_FRAME_HEADER, servoId, 0x03u,
    SERVO_POS_READ, 0u
  };
  uint8_t byte = 0u;
  uint8_t headerCount = 0u;
  uint8_t remaining = 0u;
  const uint32_t startedMs = HAL_GetTick();

  memset(reading, 0, sizeof(*reading));
  reading->status = SERVO_READ_RX_TIMEOUT;
  request[5] = ServoChecksum(request, 4u);

  if (DrainReceiver(port, transmitTimeoutMs) != HAL_OK) {
    reading->status = SERVO_READ_TX_ERROR;
    reading->elapsedMs = HAL_GetTick() - startedMs;
    return reading->status;
  }
  if (HAL_HalfDuplex_EnableTransmitter(port) != HAL_OK) {
    reading->status = SERVO_READ_TX_ERROR;
    reading->elapsedMs = HAL_GetTick() - startedMs;
    return reading->status;
  }
  const HAL_StatusTypeDef transmitResult =
      HAL_UART_Transmit(port, request, sizeof(request), transmitTimeoutMs);
  const HAL_StatusTypeDef receiverResult = HAL_HalfDuplex_EnableReceiver(port);
  if (transmitResult != HAL_OK || receiverResult != HAL_OK) {
    reading->status = SERVO_READ_TX_ERROR;
    reading->elapsedMs = HAL_GetTick() - startedMs;
    return reading->status;
  }

  while (headerCount < 2u) {
    const HAL_StatusTypeDef receiveResult =
        ReceiveByteBefore(port, &byte, startedMs, timeoutMs);
    if (receiveResult != HAL_OK) {
      reading->status = receiveResult == HAL_TIMEOUT
          ? SERVO_READ_RX_TIMEOUT : SERVO_READ_RX_ERROR;
      reading->elapsedMs = HAL_GetTick() - startedMs;
      return reading->status;
    }
    if (byte == SERVO_FRAME_HEADER) headerCount++;
    else headerCount = 0u;
  }

  reading->frame[0] = SERVO_FRAME_HEADER;
  reading->frame[1] = SERVO_FRAME_HEADER;
  reading->frameLength = 2u;
  for (uint8_t index = 2u; index < 4u; ++index) {
    const HAL_StatusTypeDef receiveResult = ReceiveByteBefore(
        port, &reading->frame[index], startedMs, timeoutMs);
    if (receiveResult != HAL_OK) {
      reading->status = receiveResult == HAL_TIMEOUT
          ? SERVO_READ_RX_TIMEOUT : SERVO_READ_RX_ERROR;
      reading->elapsedMs = HAL_GetTick() - startedMs;
      return reading->status;
    }
    reading->frameLength++;
  }

  const uint8_t protocolLength = reading->frame[3];
  if (protocolLength < 3u ||
      (uint16_t)protocolLength + 3u > SERVO_FEEDBACK_FRAME_CAPACITY) {
    reading->status = SERVO_READ_BAD_LENGTH;
    reading->elapsedMs = HAL_GetTick() - startedMs;
    return reading->status;
  }

  remaining = (uint8_t)(protocolLength - 1u);
  for (uint8_t index = 0u; index < remaining; ++index) {
    const HAL_StatusTypeDef receiveResult = ReceiveByteBefore(
        port, &reading->frame[4u + index], startedMs, timeoutMs);
    if (receiveResult != HAL_OK) {
      reading->status = receiveResult == HAL_TIMEOUT
          ? SERVO_READ_RX_TIMEOUT : SERVO_READ_RX_ERROR;
      reading->elapsedMs = HAL_GetTick() - startedMs;
      return reading->status;
    }
    reading->frameLength++;
  }

  if (reading->frame[2] != servoId) {
    reading->status = SERVO_READ_BAD_ID;
  } else if (protocolLength != SERVO_POSITION_RESPONSE_LENGTH) {
    reading->status = SERVO_READ_BAD_LENGTH;
  } else if (reading->frame[4] != SERVO_POS_READ) {
    reading->status = SERVO_READ_BAD_COMMAND;
  } else if (reading->frame[reading->frameLength - 1u] !=
             ServoChecksum(reading->frame,
                           (uint8_t)(reading->frameLength - 2u))) {
    reading->status = SERVO_READ_BAD_CHECKSUM;
  } else {
    reading->position = (int16_t)((uint16_t)reading->frame[5] |
                                  ((uint16_t)reading->frame[6] << 8));
    if (reading->position < 0 ||
        reading->position > SERVO_RAW_PROTOCOL_MAX) {
      reading->status = SERVO_READ_BAD_POSITION;
    } else {
      reading->status = SERVO_READ_OK;
    }
  }

  reading->elapsedMs = HAL_GetTick() - startedMs;
  return reading->status;
}

ServoReadStatus ServoReadPosition(UART_HandleTypeDef *port,
                                  uint8_t servoId,
                                  ServoPositionReading *reading,
                                  uint32_t timeoutMs) {
  return ServoReadPositionBounded(port, servoId, reading, timeoutMs, 30u);
}

const char *ServoReadStatusName(ServoReadStatus status) {
  switch (status) {
    case SERVO_READ_OK: return "OK";
    case SERVO_READ_TX_ERROR: return "TX_ERROR";
    case SERVO_READ_RX_TIMEOUT: return "RX_TIMEOUT";
    case SERVO_READ_RX_ERROR: return "RX_ERROR";
    case SERVO_READ_BAD_LENGTH: return "BAD_LENGTH";
    case SERVO_READ_BAD_ID: return "BAD_ID";
    case SERVO_READ_BAD_COMMAND: return "BAD_COMMAND";
    case SERVO_READ_BAD_CHECKSUM: return "BAD_CHECKSUM";
    case SERVO_READ_BAD_POSITION: return "BAD_POSITION";
    default: return "UNKNOWN";
  }
}
