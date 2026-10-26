/**
 * @file calibration_usb.h
 * @brief Telemetria USB de calibracion para observacion y registro en PC.
 */
#ifndef CALIBRATION_USB_H
#define CALIBRATION_USB_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Passive USB logging only. A full queue or disconnected PC drops records;
// no wait, no retry loop, no motor command, and no control-state mutation.
namespace CalibrationUsb {
constexpr uint32_t PERIOD_MS = 50u;
constexpr uint32_t LEASE_MS = 15000u;
constexpr unsigned PACKET_BYTES = 64u;
struct Packet { uint8_t data[PACKET_BYTES]; uint8_t length; };
class Spool {
 public:
  void enable(uint32_t now) { enabled_ = true; renewed_ = now; }
  void disable() { enabled_ = false; head_ = count_ = 0u; }
  bool active(uint32_t now) {
    if (enabled_ && now - renewed_ > LEASE_MS) disable();
    return enabled_;
  }
  bool push(const char* line, int length, bool sample = false) {
    if (!enabled_ || length <= 0 || length >= (int)PACKET_BYTES ||
        count_ >= 8u || (sample && count_ >= 4u)) { ++dropped_; return false; }
    Packet& slot = packets_[(head_ + count_) % 8u];
    memcpy(slot.data, line, (unsigned)length);
    slot.length = (uint8_t)length;
    ++count_; return true;
  }
  const Packet* front() const { return count_ ? &packets_[head_] : nullptr; }
  void sent(uint32_t actual) {
    if (!count_ || actual == 0u) return;
    if (actual != packets_[head_].length) ++dropped_;
    head_ = (head_ + 1u) % 8u; --count_;
  }
  uint32_t dropped() const { return dropped_; }
 private:
  Packet packets_[8] = {};
  unsigned head_ = 0u, count_ = 0u;
  bool enabled_ = false;
  uint32_t renewed_ = 0u, dropped_ = 0u;
};
inline int sample(char* out, uint32_t epoch, uint32_t now, uint32_t encoderMs,
                  int32_t count, uint32_t trial, uint8_t stateFlags, uint16_t gate) {
  return snprintf(out, PACKET_BYTES, "@CT,%08lX,%08lX,%08lX,%08lX,%08lX,%02X,%04X\n",
    (unsigned long)epoch, (unsigned long)now, (unsigned long)encoderMs,
    (unsigned long)(uint32_t)count, (unsigned long)trial, (unsigned)stateFlags, (unsigned)gate);
}
inline int piece(char* out, uint32_t epoch, uint32_t now, uint32_t trial,
                 int32_t encoderAtFrame, uint8_t id, float distance, float y) {
  return snprintf(out, PACKET_BYTES, "@CP,%08lX,%08lX,%08lX,%08lX,%u,%ld,%ld\n",
    (unsigned long)epoch, (unsigned long)now, (unsigned long)trial,
    (unsigned long)(uint32_t)encoderAtFrame, (unsigned)id,
    (long)(distance * 100.0f), (long)(y * 100.0f));
}
inline int gate(char* out, uint32_t epoch, uint32_t now, uint32_t trial, char event) {
  return snprintf(out, PACKET_BYTES, "@CG,%08lX,%08lX,%08lX,%c\n",
    (unsigned long)epoch, (unsigned long)now, (unsigned long)trial, event);
}
} // namespace CalibrationUsb
#endif
