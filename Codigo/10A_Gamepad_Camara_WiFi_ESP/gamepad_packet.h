/**
 * @file gamepad_packet.h
 * @brief Contrato binario del mando entre ESP32 y Portenta.
 * Transporta ejes, botones, conexion, sesion y secuencia.
 */
#ifndef GAMEPAD_PACKET_H
#define GAMEPAD_PACKET_H

#include <stddef.h>
#include <stdint.h>

#define GAMEPAD_MAGIC 0x4750u
#define GAMEPAD_VERSION 1u
#define GAMEPAD_PACKET_SIZE 32u
#define GAMEPAD_CRC_BYTES 30u
#define GAMEPAD_FLAG_CONNECTED 0x01u

#if defined(__GNUC__)
#define GAMEPAD_PACKED __attribute__((packed))
#else
#define GAMEPAD_PACKED
#pragma pack(push, 1)
#endif

typedef struct GAMEPAD_PACKED {
  uint16_t magic;
  uint8_t version;
  uint8_t flags;
  uint32_t session;
  uint32_t sequence;
  int16_t lx;
  int16_t ly;
  int16_t rx;
  int16_t ry;
  uint16_t buttons;
  uint16_t misc;
  uint8_t dpad;
  uint8_t reserved[5];
  uint16_t crc;
} GamepadPacket;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

#if defined(__cplusplus)
static_assert(sizeof(GamepadPacket) == GAMEPAD_PACKET_SIZE,
              "GamepadPacket debe medir 32 bytes");
#endif

static inline uint16_t gamepadCrc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFFu;
  for (size_t index = 0; index < length; ++index) {
    crc ^= (uint16_t)data[index] << 8;
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static inline void gamepadFinalize(GamepadPacket *packet) {
  packet->magic = GAMEPAD_MAGIC;
  packet->version = GAMEPAD_VERSION;
  packet->crc = 0u;
  packet->crc = gamepadCrc16((const uint8_t *)packet, GAMEPAD_CRC_BYTES);
}

static inline int gamepadPacketValid(const GamepadPacket *packet) {
  return packet->magic == GAMEPAD_MAGIC &&
         packet->version == GAMEPAD_VERSION &&
         packet->crc ==
           gamepadCrc16((const uint8_t *)packet, GAMEPAD_CRC_BYTES);
}

#endif
