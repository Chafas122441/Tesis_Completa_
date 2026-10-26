/**
 * @file camera_packet.h
 * @brief Formato binario UDP de control y observaciones de la camara.
 * Define identificadores, estados y comprobaciones de integridad compartidas.
 */
#ifndef CAMERA_PACKET_H
#define CAMERA_PACKET_H

#include <stddef.h>
#include <stdint.h>

#define CAMERA_MAGIC 0x434Du
#define CAMERA_VERSION 1u
#define CAMERA_PACKET_SIZE 40u
#define CAMERA_CRC_BYTES 38u

enum CameraPacketType : uint8_t {
  CAMERA_TYPE_STATUS = 1u,
  CAMERA_TYPE_FRAME = 2u,
  CAMERA_TYPE_COMMAND = 3u,
  CAMERA_TYPE_ACK = 4u,
  CAMERA_TYPE_FAULT = 5u
};

enum CameraCommand : uint16_t {
  CAMERA_COMMAND_OFF = 0u,
  CAMERA_COMMAND_TAG_MODE = 16u,
  CAMERA_COMMAND_PIECE_MODE = 128u,
  CAMERA_COMMAND_STATUS = 0x00FFu
};

enum CameraFlags : uint16_t {
  CAMERA_FLAG_UART_READY = 0x0001u,
  CAMERA_FLAG_MODE_READY = 0x0002u,
  CAMERA_FLAG_EMPTY_FRAME = 0x0004u,
  CAMERA_FLAG_SETTLING = 0x0008u
};

enum CameraFaultCode : uint16_t {
  CAMERA_FAULT_NONE = 0u,
  CAMERA_FAULT_UART = 1u,
  CAMERA_FAULT_MODE = 2u,
  CAMERA_FAULT_READ = 3u,
  CAMERA_FAULT_BAD_COMMAND = 4u
};

#if defined(__GNUC__)
#define CAMERA_PACKED __attribute__((packed))
#else
#define CAMERA_PACKED
#pragma pack(push, 1)
#endif

typedef struct CAMERA_PACKED {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint32_t session;
  uint32_t sequence;
  uint32_t frame;
  int16_t algorithm;
  int16_t learnedId;
  int16_t tagContent;
  int16_t xCenter;
  int16_t yCenter;
  int16_t width;
  int16_t height;
  uint8_t index;
  uint8_t count;
  uint16_t readMs;
  uint16_t flags;
  uint16_t value;
  uint16_t crc;
} CameraPacket;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

#if defined(__cplusplus)
static_assert(sizeof(CameraPacket) == CAMERA_PACKET_SIZE,
              "CameraPacket debe medir 40 bytes");
#endif

static inline uint16_t cameraCrc16(const uint8_t *data, size_t length) {
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

static inline void cameraFinalize(CameraPacket *packet) {
  packet->magic = CAMERA_MAGIC;
  packet->version = CAMERA_VERSION;
  packet->crc = 0u;
  packet->crc = cameraCrc16((const uint8_t *)packet, CAMERA_CRC_BYTES);
}

static inline int cameraPacketValid(const CameraPacket *packet) {
  return packet->magic == CAMERA_MAGIC &&
         packet->version == CAMERA_VERSION &&
         packet->crc == cameraCrc16((const uint8_t *)packet,
                                    CAMERA_CRC_BYTES);
}

#endif
