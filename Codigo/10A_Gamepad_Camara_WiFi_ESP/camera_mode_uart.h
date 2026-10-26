/**
 * @file camera_mode_uart.h
 * @brief Transacciones UART de seleccion de modelo y lectura de HUSKYLENS 2.
 * Valida cabecera, longitud, checksum, algoritmo y primer cuadro recibido.
 */
#ifndef CAMERA_MODE_UART_H
#define CAMERA_MODE_UART_H
#include <stddef.h>
#include <stdint.h>

// HUSKYLENS V2: mismo KNOCK/SET_ALGORITHM de ProtocolV2.cpp (DFRobot).
// El SDK fija wait() a 5000 ms y no comprueba el algoritmo real del resultado.
// Cambio y lecturas locales acotadas; no modificar el SDK global.
// Invocar exclusivamente desde el worker propietario del UART.
namespace CameraModeUart {
enum class Code : uint8_t {
  OK = 0u, TIMEOUT, NACK, TX_SHORT, WRONG_ALGORITHM, MALFORMED, TOO_MANY, DRAIN_BUSY
};
struct Reply {
  Code code = Code::TIMEOUT;
  uint32_t elapsedMs = 0u;
  uint16_t ignoredFrames = 0u;
  uint16_t badChecksums = 0u;
  bool hasReturnValue = false;
  int8_t returnValue = 0;
  uint8_t replyAlgorithm = 0u;
  uint8_t replyLength = 0u;
  uint8_t returnTextLength = 0u;
  uint8_t returnText[245]{}; // payload extra completo, incluido formato de strings.
};

template<class Port, class Clock, class Idle>
Reply transact(Port &port, uint8_t command, uint8_t argument,
               uint32_t timeoutMs, Clock now, Idle idle) {
  // Header 55 AA / command / ALGORITHM_ANY(0) / length(10), arg + 9 ceros.
  uint8_t tx[16] = {0x55u, 0xAAu, command, 0u, 10u, argument};
  for (size_t i = 0u; i < sizeof(tx) - 1u; ++i) tx[15] += tx[i];
  Reply reply{};
  const uint32_t began = now();
  if (port.write(tx, sizeof(tx)) != sizeof(tx)) {
    reply.code = Code::TX_SHORT;
    reply.elapsedMs = now() - began;
    return reply;
  }
  port.flush();
  // length es uint8_t: como maximo 255 de payload + 5 header + checksum.
  uint8_t rx[261]{};
  uint16_t used = 0u;
  uint16_t expected = 0u;
  while (now() - began < timeoutMs) {
    // Limite por vuelta + comprobacion del reloj: ruido no prolonga deadline.
    for (uint16_t n = 0u; n < 128u && port.available() > 0; ++n) {
      if (now() - began >= timeoutMs) break;
      const int byteRead = port.read();
      if (byteRead < 0) break;
      const uint8_t value = static_cast<uint8_t>(byteRead);
      if (used == 0u) {
        if (value == 0x55u) rx[used++] = value;
        continue;
      }
      if (used == 1u && value != 0xAAu) {
        used = value == 0x55u ? 1u : 0u;
        continue;
      }
      rx[used++] = value;
      if (used == 5u) expected = static_cast<uint16_t>(rx[4]) + 6u;
      if (used < 5u || used != expected) continue;
      uint8_t sum = 0u;
      for (uint16_t i = 0u; i < expected - 1u; ++i) sum += rx[i];
      if (sum != rx[expected - 1u]) {
        ++reply.badChecksums;
      } else if (rx[2] == 0x1Au && rx[4] >= 2u) {
        // PacketData_t: rfu0 en payload[0], retValue en payload[1].
        reply.hasReturnValue = true;
        reply.returnValue = static_cast<int8_t>(rx[6]);
        reply.replyAlgorithm = rx[3];
        reply.replyLength = rx[4];
        if (rx[4] > 10u) {
          reply.returnTextLength = static_cast<uint8_t>(rx[4] - 10u);
          for (uint8_t i = 0u; i < reply.returnTextLength; ++i)
            reply.returnText[i] = rx[15u + i];
        }
        reply.code = rx[6] == 0u ? Code::OK : Code::NACK;
        reply.elapsedMs = now() - began;
        return reply;
      } else {
        ++reply.ignoredFrames;
      }
      used = 0u;
      expected = 0u;
    }
    idle(); // cede CPU al RTOS; nunca espera en loop()/Bluepad32.
  }
  reply.elapsedMs = now() - began;
  return reply;
}

struct FrameReply {
  Code code = Code::TIMEOUT;
  uint32_t elapsedMs = 0u;
  uint16_t ignoredFrames = 0u;
  uint16_t badChecksums = 0u;
  uint8_t actualAlgorithm = 0u;
  uint8_t count = 0u;
};

// El ACK de SET prueba la seleccion del modelo; INFO vacio prueba enlace vivo,
// no deteccion. Un NACK nunca habilita vacios por si solo. OFF/cambio de
// generacion no pueden reutilizar autorizacion de otro modelo.
struct EmptyFramePolicy {
  bool acknowledged = false;
  bool observed = false;
  uint32_t generation = 0u;
  int16_t algorithm = -1;
  void select(uint32_t nextGeneration, int16_t nextAlgorithm, bool ack) {
    generation = nextGeneration;
    algorithm = nextAlgorithm;
    acknowledged = ack;
    observed = false;
  }
  bool allows(uint32_t queryGeneration, int16_t queryAlgorithm) const {
    return generation == queryGeneration && algorithm == queryAlgorithm &&
      (acknowledged || observed);
  }
  void observe(uint32_t queryGeneration, int16_t queryAlgorithm, bool valid) {
    if (generation == queryGeneration && algorithm == queryAlgorithm)
      observed = valid;
  }
};

inline int32_t signedLe16(const uint8_t *p) {
  const uint16_t value = static_cast<uint16_t>(p[0]) |
    static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8u);
  return value < 32768u ? value : static_cast<int32_t>(value) - 65536;
}

inline bool blockStringsValid(const uint8_t *payload, uint8_t length) {
  if (length == 10u) return true; // sin nombre/content.
  if (length < 12u) return false;
  uint16_t cursor = 10u;
  for (uint8_t field = 0u; field < 2u; ++field) {
    if (cursor >= length) return false;
    const uint8_t size = payload[cursor++];
    if (cursor + size > length) return false;
    // No permitir que un NUL embebido convierta un nombre ajeno en pieza_C.
    for (uint16_t i = 0u; i < size; ++i)
      if (payload[cursor + i] == 0u) return false;
    cursor += size;
  }
  // El protocolo permite datos privados del algoritmo despues de los strings.
  return true;
}

// Callback recibe cada BLOCK validado. El consumidor descarta TODOS los
// resultados si code != OK; un paquete parcial no representa imagen vacia.
template<class Port, class Clock, class Idle, class OnBlock>
FrameReply readFrame(Port &port, uint8_t algorithm, uint32_t timeoutMs,
                     Clock now, Idle idle, OnBlock onBlock,
                     bool allowGeneralEmpty = false) {
  FrameReply reply{};
  const uint32_t began = now();
  const auto finish = [&](Code code) {
    reply.code = code;
    reply.elapsedMs = now() - began;
    return reply;
  };
  // Sin ID de transaccion en el protocolo. Vaciar respuestas anteriores y
  // exigir RX tranquilo antes de consultar; no usar cache como evidencia.
  uint32_t lastRxMs = began;
  uint16_t drained = 0u;
  for (;;) {
    for (uint16_t n = 0u; n < 128u && port.available() > 0; ++n) {
      if (now() - began >= 50u || drained >= 4096u)
        return finish(Code::DRAIN_BUSY);
      port.read();
      ++drained;
      lastRxMs = now();
    }
    if (port.available() == 0 && now() - lastRxMs >= 5u) break;
    if (now() - began >= 50u || now() - began >= timeoutMs)
      return finish(Code::DRAIN_BUSY);
    idle();
  }
  if (now() - began >= timeoutMs) return finish(Code::TIMEOUT);
  uint8_t tx[6] = {0x55u, 0xAAu, 0x01u, algorithm, 0u, 0u};
  for (uint8_t i = 0u; i < 5u; ++i) tx[5] += tx[i];
  if (port.write(tx, sizeof(tx)) != sizeof(tx)) return finish(Code::TX_SHORT);
  port.flush();
  uint8_t rx[261]{};
  uint16_t used = 0u, expected = 0u;
  bool haveInfo = false;
  uint8_t total = 0u;
  while (now() - began < timeoutMs) {
    for (uint16_t n = 0u; n < 128u && port.available() > 0; ++n) {
      if (now() - began >= timeoutMs) break;
      const int byteRead = port.read();
      if (byteRead < 0) break;
      const uint8_t value = static_cast<uint8_t>(byteRead);
      if (used == 0u) {
        if (value == 0x55u) rx[used++] = value;
        continue;
      }
      if (used == 1u && value != 0xAAu) {
        used = value == 0x55u ? 1u : 0u;
        continue;
      }
      rx[used++] = value;
      if (used == 5u) expected = static_cast<uint16_t>(rx[4]) + 6u;
      if (used < 5u || used != expected) continue;
      uint8_t sum = 0u;
      for (uint16_t i = 0u; i < expected - 1u; ++i) sum += rx[i];
      if (sum != rx[expected - 1u]) {
        ++reply.badChecksums;
        if (haveInfo) return finish(Code::MALFORMED);
      } else if (rx[2] == 0x1Bu || rx[2] == 0x1Cu || rx[2] == 0x1Du) {
        reply.actualAlgorithm = rx[3];
        // Algunas camaras usan header ANY(0); BLOCK.payload[1] identifica
        // el modelo. Un header especifico contradictorio sigue rechazado.
        if (rx[3] != 0u && rx[3] != algorithm)
          return finish(Code::WRONG_ALGORITHM);
        if (rx[2] == 0x1Bu) {
          if (haveInfo || rx[4] != 10u) return finish(Code::MALFORMED);
          const int32_t results = signedLe16(rx + 7u);
          const int32_t learned = signedLe16(rx + 9u);
          const int32_t blocks = signedLe16(rx + 11u);
          const int32_t learnedBlocks = signedLe16(rx + 13u);
          if (results < 0 || learned < 0 || learned > results ||
              blocks != results || learnedBlocks < 0 || learnedBlocks > blocks)
            return finish(Code::MALFORMED);
          if (results > 10) return finish(Code::TOO_MANY);
          total = static_cast<uint8_t>(results);
          haveInfo = true;
          if (total == 0u) {
            if (rx[3] == 0u && !allowGeneralEmpty)
              return finish(Code::WRONG_ALGORITHM);
            reply.actualAlgorithm = algorithm;
            return finish(Code::OK);
          }
        } else {
          if (!haveInfo || rx[2] != 0x1Cu || reply.count >= total ||
              !blockStringsValid(rx + 5u, rx[4]))
            return finish(Code::MALFORMED);
          if (rx[6] != algorithm) {
            reply.actualAlgorithm = rx[6];
            return finish(Code::WRONG_ALGORITHM);
          }
          reply.actualAlgorithm = algorithm; // comprobado, no inferido del GET.
          onBlock(rx);
          ++reply.count;
          if (reply.count == total) return finish(Code::OK);
        }
      } else {
        ++reply.ignoredFrames;
      }
      used = 0u;
      expected = 0u;
    }
    idle();
  }
  return finish(Code::TIMEOUT);
}
}  // namespace CameraModeUart
#endif
