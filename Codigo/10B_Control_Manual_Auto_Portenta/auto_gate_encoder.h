/**
 * @file auto_gate_encoder.h
 * @brief Conversion del avance del encoder en disparo de compuerta.
 * Separa referencia geometrica, adelantos y compensaciones temporales.
 */
#ifndef AUTO_GATE_ENCODER_H
#define AUTO_GATE_ENCODER_H
#include <cmath>
#include <stdint.h>

// Pure encoder arithmetic, not authority to close or automatic calibration.
// Nucleo positions already contain quadrature x4; never multiply by four again.
namespace AutoGateEncoder {
constexpr double COUNTS_PER_MM = 32.61062542030935;


constexpr double CAPTURE_Y_MIN_MM = -18.54;
constexpr double CAPTURE_Y_MAX_MM = 61.85;
constexpr double CAPTURE_SPEED_MIN_MM_S = 54.38671852517475;
constexpr double CAPTURE_SPEED_MAX_MM_S = 63.16959498479721;
constexpr double LATEST_MARKED_ENTRY_ERROR_MM = -77.36558112678381;
constexpr double AFTER_MARK_MARGIN_S = 0.100;
constexpr double CAPTURE_WINDOW_MM = 10.0;
constexpr double EXPERIMENTAL_TOTAL_LEAD_MM = -LATEST_MARKED_ENTRY_ERROR_MM -
  CAPTURE_SPEED_MAX_MM_S * AFTER_MARK_MARGIN_S;
struct LeadSelection { double closeLeadMm; bool experimental; };

inline LeadSelection selectLead(bool supervisedTrial, uint8_t pieceId, double y,
                                double speed, double fallback,
                                double window = CAPTURE_WINDOW_MM) {
  LeadSelection result = {fallback,false};
  if (!std::isfinite(fallback) || fallback < 0.0 || fallback > 400.0) {
    result.closeLeadMm = NAN;
    return result;
  }
  if (!supervisedTrial || pieceId != 1u || !std::isfinite(y) ||
      y < CAPTURE_Y_MIN_MM || y > CAPTURE_Y_MAX_MM || !std::isfinite(speed) ||
      speed < CAPTURE_SPEED_MIN_MM_S || speed > CAPTURE_SPEED_MAX_MM_S ||
      !std::isfinite(window) || window != CAPTURE_WINDOW_MM) return result;
  result.closeLeadMm = EXPERIMENTAL_TOTAL_LEAD_MM - window;
  result.experimental = true;
  return result;
}

inline int64_t advanceCounts(int32_t atFrame, int32_t current) {
  const uint32_t bits = static_cast<uint32_t>(atFrame) - static_cast<uint32_t>(current);
  return bits <= 0x7FFFFFFFu ? static_cast<int64_t>(bits)
    : static_cast<int64_t>(bits) - 0x100000000LL;
}

inline double advanceMm(int32_t atFrame, int32_t current,
                        double countsPerMm = COUNTS_PER_MM) {
  if (!std::isfinite(countsPerMm) || countsPerMm <= 0.0) return NAN;
  return static_cast<double>(advanceCounts(atFrame,current)) / countsPerMm;
}

struct Target {
  bool valid = false;
  int32_t frameCount = 0;
  uint32_t requiredAdvanceCounts = 0u;
  uint32_t targetCountBits = 0u;
  double countsPerMm = COUNTS_PER_MM;
  double thresholdMm = NAN;

  // Caller still requires physical pickup PASS, valid session, fresh data,
  // valid direction/overshoot and WAIT_ARRIVAL. This predicate has no I/O.
  bool reached(int32_t current) const {
    return valid && advanceCounts(frameCount,current) >=
                    static_cast<int64_t>(requiredAdvanceCounts);
  }
  double remainingMm(int32_t current) const {
    return valid ? thresholdMm - advanceMm(frameCount,current,countsPerMm) : NAN;
  }
};

inline bool target(int32_t atFrame, double distanceMm, double closeLeadMm,
                   double arrivalWindowMm, Target& result,
                   double countsPerMm = COUNTS_PER_MM) {
  result = Target(); // A failed reconfiguration cannot retain an old target.
  if (!std::isfinite(distanceMm) || distanceMm <= 0.0 ||
      !std::isfinite(closeLeadMm) || closeLeadMm < 0.0 || closeLeadMm > 400.0 ||
      !std::isfinite(arrivalWindowMm) || arrivalWindowMm <= 0.0 || arrivalWindowMm > 15.0 ||
      !std::isfinite(countsPerMm) || countsPerMm <= 0.0) return false;
  const double thresholdMm = distanceMm - closeLeadMm - arrivalWindowMm;
  const double counts = std::ceil(thresholdMm * countsPerMm);
  // Half the int32 modular range is direction-ambiguous. Round UP so
  // quantization never fires earlier than the requested millimetre threshold.
  if (!std::isfinite(thresholdMm) || thresholdMm <= 0.0 ||
      !std::isfinite(counts) || counts < 1.0 || counts >= 2147483648.0) return false;
  result.valid = true;
  result.frameCount = atFrame;
  result.requiredAdvanceCounts = static_cast<uint32_t>(counts);
  result.targetCountBits = static_cast<uint32_t>(atFrame) - result.requiredAdvanceCounts;
  result.countsPerMm = countsPerMm;
  result.thresholdMm = thresholdMm;
  return true;
}
} // namespace AutoGateEncoder
#endif
