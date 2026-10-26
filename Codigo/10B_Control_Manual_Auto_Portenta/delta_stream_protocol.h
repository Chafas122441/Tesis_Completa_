/**
 * @file delta_stream_protocol.h
 * @brief Protocolo binario UART version 3 de Portenta y Nucleo.
 * Tramas de 22 bytes con sesion, secuencia y CRC16. Los permisos negociados
 * distinguen aproximacion, movimiento directo y verificacion de llegada.
 */
#ifndef DELTA_STREAM_PROTOCOL_H
#define DELTA_STREAM_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define DELTA_LINK_SOF_A 0xA5u
#define DELTA_LINK_SOF_B 0x5Au
#define DELTA_LINK_VERSION 3u
/* AUTO five identical writes, including bounded REC350; not a servo ACK. */
#define DELTA_DIRECT_REPEAT5_CONTRACT 1u
/* Paired AUTO approach-only STREAM5 and post-pickup DIRECT read300 contract. */
#define DELTA_APPROACH_STEP_RAW 5u
#define DELTA_APPROACH_PERIOD_MS 50u
#define DELTA_APPROACH_MOVE_MS 100u
#define DELTA_POST_PICKUP_READ_BUDGET_MS 300u
#define DELTA_POST_PICKUP_MIN_DURATION_MS 1000u
#define DELTA_LINK_FRAME_SIZE 22u
#define DELTA_LINK_CRC_BYTES 20u
/* Plazo finito compartido: lectura/reintentos, una correccion y verificacion.
   No cambia el formato v3 ni renueva las leases de STREAM/RC/enlace. */
#define DELTA_CHECKPOINT_TIMEOUT_MS 6000u
#define DELTA_AUTO_CHECKPOINT_TIMEOUT_MS 16000u
/* Explicit pickup-only CP5 recovery. Never acceptance20 or a global AUTO bound. */
#define DELTA_PICKUP_FINE_MAX_ERROR_RAW 20u
#define DELTA_PICKUP_FINE_MAX_COMPENSATION_RAW 20u
#define DELTA_PICKUP_FINE_MAX_CORRECTIONS 4u
#define DELTA_PICKUP_FINE_TIMEOUT_MS 20000u
#define DELTA_FAULT_RECOVERY_TIMEOUT_MS 4000u
#define DELTA_FAULT_RECOVERY_MAGIC 0x5243u
/* Request-only SETPOINT bit. BEGIN/END/CHECKPOINT still use the base option.
   A coarse frame needs XYZ, >=100 ms spacing and >=200 ms interpolation.
   This never changes the physical feedback tolerance or correction limit. */
#define DELTA_STREAM_SETPOINT_COARSE 0x0040u
/* Request-only CHECKPOINT bit: explicit AUTO box arrival, not a STREAM mode.
   Other stages remain +/-5; the frame layout/version and correction bounds stay. */
#define DELTA_CHECKPOINT_BOX8 0x0080u
#define DELTA_CHECKPOINT_PRECISION_RAW 5u
#define DELTA_CHECKPOINT_BOX_TOLERANCE_RAW 8u
/* Separate permission: older BOX8 firmware must not interpret BOX15 as BOX8. */
#define DELTA_CHECKPOINT_BOX15 0x0100u
#define DELTA_CHECKPOINT_BOX_MASK (DELTA_CHECKPOINT_BOX8 | DELTA_CHECKPOINT_BOX15)
#define DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW 15u
#define DELTA_CHECKPOINT_BOX15_MAX_ERROR_RAW 20u
/* Explicit transit acceptance. No inheritance into pickup, END/STOP or HOME.
   Old firmware rejects this unknown bit instead of treating it as BOX15. */
#define DELTA_CHECKPOINT_AUTO9 0x0400u
#define DELTA_CHECKPOINT_AUTO9_TOLERANCE_RAW 9u
#define DELTA_CHECKPOINT_POLICY_MASK (DELTA_CHECKPOINT_BOX_MASK | DELTA_CHECKPOINT_AUTO9)
/* Explicit temporary AUTO policy: previously-invalid BOX8|BOX15 combination.
   Negotiate with AUTO_STOP before PWM; old firmware rejects the combination.
   Acceptance stays15; 16..30 only permits bounded recovery, never a PASS. */
#define DELTA_CHECKPOINT_AUTO15_REC30 (DELTA_CHECKPOINT_BOX8 | DELTA_CHECKPOINT_BOX15)
#define DELTA_AUTO15_TOLERANCE_RAW 15u
#define DELTA_AUTO15_RECOVERABLE_RAW 30u
/* CHECKPOINT-only, previously invalid combination. Negotiate before PWM with
   AUTO_STOP: probe still requires precision5. Runtime observations require
   fresh/stable/in-range feedback, NOT target precision, and return OBSERVED.
   This permission never applies to HOME, manual, END/STOP or ordinary CP5. */
#define DELTA_CHECKPOINT_OBSERVE (DELTA_CHECKPOINT_AUTO9 | DELTA_CHECKPOINT_BOX8)
/* Negotiation ONLY: observation + advisory RX timeouts on explicit type25.
   Previously invalid combination; older C rejects before PWM. It never makes
   missing feedback a valid checkpoint or changes type23/manual/HOME/pickup. */
#define DELTA_DIRECT_FINAL_READ_CAPABILITY (DELTA_CHECKPOINT_OBSERVE | DELTA_CHECKPOINT_BOX15)
/* Negotiation-only combination rejected by older C before PWM. Includes the
   type25 final-read contract plus bounded, host-preflighted endpoint retries. */
#define DELTA_ENDPOINT_RETRY_CAPABILITY (DELTA_CHECKPOINT_AUTO9 | DELTA_CHECKPOINT_BOX15)
#define DELTA_ENDPOINT_RETRY_MAX_ATTEMPTS 2u
#define DELTA_ENDPOINT_RETRY_TOLERANCE_RAW 15u
#define DELTA_ENDPOINT_RETRY_LEGACY_MAX_ERROR_RAW 30u
#define DELTA_ENDPOINT_RETRY_MAX_ERROR_RAW 350u /* recovery only; acceptance remains15 */
#define DELTA_ENDPOINT_RETRY_WAIT_MS 100u
/* Centre return recovers to the existing CP5 correction envelope, not a PASS.
   A separate precision checkpoint MUST still finish within5 before advancing. */
#define DELTA_ENDPOINT_CENTER_BRIDGE_RAW 12u
#define DELTA_ENDPOINT_STEP_RAW 4u
#define DELTA_ENDPOINT_STEP_SLICE_MS 100u
#define DELTA_ENDPOINT_STEP_REPEATS 2u
/* Linear RAW path identical to the host-preflighted recovery chord. Feedback
   gates every <=4 RAW slice; never accumulate slices against an idle joint. */
static inline uint16_t deltaEndpointStepCount(const uint16_t start[3],const uint16_t goal[3]) {
  uint16_t largest=0u;
  for(uint8_t i=0u;i<3u;++i){
    const uint16_t d=start[i]>goal[i]?start[i]-goal[i]:goal[i]-start[i];
    if(d>largest)largest=d;
  }
  return largest; // one-RAW grid; runtime chooses up to4 grid units from fresh feedback
}
static inline uint16_t deltaEndpointStepBudget(const uint16_t start[3],const uint16_t goal[3]) {
  const uint16_t count=deltaEndpointStepCount(start,goal);
  return count ? (uint16_t)(((count+3u)/4u)*150u+300u) : 0u;
}
static inline void deltaEndpointStepTarget(const uint16_t start[3],const uint16_t goal[3],
    uint16_t index,uint16_t count,uint16_t out[3]) {
  for(uint8_t i=0u;i<3u;++i){
    const int32_t d=(int32_t)goal[i]-start[i];
    const int32_t magnitude=d<0?-d:d;
    const int32_t offset=count?((magnitude*index+count/2u)/count):0;
    out[i]=(uint16_t)((int32_t)start[i]+(d<0?-offset:offset));
  }
}
/* Explicit opt-in: CHECKPOINT with this bit alone probes AUTO STOP support
   before RC/motion. END may then add AUTO9 or BOX15; no implicit inheritance.
   Old receivers reject the probe instead of applying legacy STOP limits. */
#define DELTA_AUTO_STOP_POLICY 0x0800u
/* Probe on CHECKPOINT before any RC PWM. Unknown receivers reject it. */
#define DELTA_LOCAL_POINT_CAPABILITY 0x1000u
/* A DIRECT_START is one bounded HTS interpolation to the final RAW endpoint.
   It is intentionally separate from POINT_START so old receivers cannot
   silently turn a requested two-command route back into 50 ms slices. */
#define DELTA_DIRECT_ENDPOINT_CAPABILITY 0x2000u

#define DELTA_DIRECT_TRANSACTION_CAPABILITY 0x4000u
/* Position preflight was performed for this exact joint segment. Unlike legacy
   DIRECT21, PATH22 never starts a recovery from a different measured origin. */
#define DELTA_DIRECT_VALIDATED_PATH_CAPABILITY 0x0200u
/* Explicit experimental LIFT timing, negotiated before RC PWM. The sender
   validates the same vertical joint segment; only the first probe is deferred.
   It never authorizes skipping progress/STOP/final physical verification. */
#define DELTA_DIRECT_LIFT_DELAY_CAPABILITY 0x8000u
#define DELTA_DIRECT_LIFT_FIRST_PROBE_MS 200u
/* Request-only CHECKPOINT capability. Old receivers reject bit3 before PWM.
   In this negotiated session type23 uses two initial copies, first probe250
   and duration>=600; the previous 200ms contract remains unchanged otherwise. */
#define DELTA_DIRECT_PROFILE250_CAPABILITY 0x0008u
#define DELTA_DIRECT_PROFILE250_FIRST_PROBE_MS 250u
#define DELTA_DIRECT_PROFILE250_MIN_DURATION_MS 600u
/* CHECKPOINT-only bit6 (SETPOINT uses it as COARSE): one coordinated retry.
   Old C rejects it before PWM. No in-flight single-axis catch-up is permitted. */
#define DELTA_DIRECT_RETRY_CAPABILITY 0x0040u
#define DELTA_DIRECT_RETRY_TIMEOUT_MS 6000u
#define DELTA_DIRECT_RETRY_ORIGIN_MAX_AGE_MS 200u
#define DELTA_DIRECT_MIN_DURATION_MS 200u
#define DELTA_DIRECT_MAX_DURATION_MS 12000u
#define DELTA_DIRECT_RAW_PER_SECOND 120u /* 28.8 deg/s; validated AUTO segments */

/* Selected axes alone move. An initial miss is not PASS or an emergency STOP.
   Zero means either already reached or outside recovery permission; caller distinguishes it. */
static inline uint8_t deltaEndpointRetryPlanTolerance(const uint16_t measured[3],
    const uint16_t nominal[3], uint16_t effective[3], uint16_t recoveryLimit,
    uint16_t tolerance) {
  uint8_t mask = 0u;
  for (uint8_t i = 0u; i < 3u; ++i) {
    const uint16_t d = measured[i] > nominal[i] ? measured[i]-nominal[i] : nominal[i]-measured[i];
    if (d > recoveryLimit) return 0u;
    effective[i] = measured[i];
    if (d > tolerance) {
      mask |= (uint8_t)(1u << i);
      effective[i] = nominal[i];
    }
  }
  return mask;
}
static inline uint8_t deltaEndpointRetryPlanLimit(const uint16_t measured[3],
    const uint16_t nominal[3], uint16_t effective[3], uint16_t recoveryLimit) {
  return deltaEndpointRetryPlanTolerance(measured,nominal,effective,recoveryLimit,
    DELTA_ENDPOINT_RETRY_TOLERANCE_RAW);
}
static inline uint8_t deltaEndpointRetryPlan(const uint16_t measured[3],
    const uint16_t nominal[3], uint16_t effective[3]) {
  return deltaEndpointRetryPlanLimit(measured,nominal,effective,DELTA_ENDPOINT_RETRY_MAX_ERROR_RAW);
}

static inline uint16_t deltaDirectMinimumMs(const uint16_t start[3],
                                            const uint16_t goal[3]) {
  uint16_t largest = 0u;
  for (uint8_t i = 0u; i < 3u; ++i) {
    const uint16_t d = start[i] > goal[i] ? start[i] - goal[i] : goal[i] - start[i];
    if (d > largest) largest = d;
  }
  uint32_t ms = ((uint32_t)largest * 1000u + DELTA_DIRECT_RAW_PER_SECOND - 1u) /
                DELTA_DIRECT_RAW_PER_SECOND;
  if (ms < DELTA_DIRECT_MIN_DURATION_MS) ms = DELTA_DIRECT_MIN_DURATION_MS;
  ms = ((ms + 49u) / 50u) * 50u;
  return ms > DELTA_DIRECT_MAX_DURATION_MS ? 0u : (uint16_t)ms;
}

typedef enum {
  DELTA_MSG_HELLO = 1,
  DELTA_MSG_HEARTBEAT = 2,
  DELTA_MSG_ARM = 3,
  DELTA_MSG_DISARM = 4,
  DELTA_MSG_HOME = 5,
  DELTA_MSG_MOVE = 6,
  DELTA_MSG_STATUS_REQUEST = 7,
  DELTA_MSG_STREAM_BEGIN = 8,
  DELTA_MSG_STREAM_SETPOINT = 9,
  DELTA_MSG_STREAM_END = 10,
  DELTA_MSG_GRIP_BEGIN = 11,
  DELTA_MSG_GRIP_SET = 12,
  DELTA_MSG_GRIP_END = 13,
  DELTA_MSG_GRIP_GROUP_BEGIN = 14,
  DELTA_MSG_GRIP_GROUP_SET = 15,
  DELTA_MSG_GRIP_STATE_REQUEST = 16,
  DELTA_MSG_STREAM_CHECKPOINT = 17,
  DELTA_MSG_FAULT_RECOVER = 18,
  DELTA_MSG_POINT_START = 19,
  DELTA_MSG_POINT_KEEPALIVE = 20, /* raw[0]=START sequence, raw[1..2]=0 */
  DELTA_MSG_DIRECT_START = 21, /* one HTS endpoint interpolation; no host slices */
  DELTA_MSG_DIRECT_PATH_START = 22, /* prevalidated exact segment; no motion retry */
  DELTA_MSG_DIRECT_LIFT_START = 23, /* validated LIFT, bounded delayed first probe */
  DELTA_MSG_DIRECT_RETRY = 24, /* same endpoint after confirmed hold + new B preflight */
  DELTA_MSG_DIRECT_FINAL_READ_START = 25, /* validated AUTO transit: RX/slow-progress advisory, final reads mandatory */
  DELTA_MSG_ENDPOINT_RETRY = 26, /* same nominal endpoint, only missed axes; new physical origin + B preflight */
  DELTA_MSG_ENDPOINT_REC200_PROBE = 27, /* CP5 before PWM; grants REC200 + safe END after OBSERVED */
  DELTA_MSG_ENDPOINT_STEP_RETRY = 28, /* second recovery: <=4 RAW/100ms, feedback-gated slices */
  DELTA_MSG_ENDPOINT_STEP_PROBE = 29, /* CP5 before PWM; REC200 + wait100 + bounded step fallback */
  DELTA_MSG_CENTER_RECOVERY_PROBE = 30, /* CP5 before PWM; centre recovery then mandatory CP5 */
  DELTA_MSG_CENTER_OBSERVE = 31, /* AUTO return only: fresh physical pose, NOT precision PASS */
  DELTA_MSG_PICKUP_FINE20_PROBE = 32, /* before PWM: centre contract + explicit pickup20 support, CP5 */
  DELTA_MSG_PICKUP_FINE20_CHECKPOINT = 33, /* AUTO approach/descent only: recover20, acceptance5 */
  DELTA_MSG_DIRECT_REPEAT5_PROBE = 35, /* before PWM: classic reader + TX5/REC5 contract */
  DELTA_MSG_APPROACH_STEPS_PROBE = 36, /* before PWM: TX5 + scoped STREAM5 + DIRECT300 */
  DELTA_MSG_APPROACH_SETPOINT = 37, /* AUTO XYZ approach only; <=5 RAW/50ms, >=100ms interpolation */
  DELTA_MSG_ACK = 0x80,
  DELTA_MSG_NACK = 0x81,
  DELTA_MSG_STATUS = 0x82,
  DELTA_MSG_ENCODER = 0x83,
  DELTA_MSG_FEEDBACK = 0x84,
  DELTA_MSG_GRIP_STATE = 0x85,
  DELTA_MSG_CHECKPOINT_RESULT = 0x86,
  DELTA_MSG_FAULT_RECOVERY_RESULT = 0x87,
  DELTA_MSG_DIRECT_RETRY_STATE = 0x88 /* original START token; BUSY/0 or OK/7 */
} DeltaMessageType;

enum {
  DELTA_FLAG_ARMED = 1u << 0,
  DELTA_FLAG_HOME_VALID = 1u << 1,
  DELTA_FLAG_BUSY = 1u << 2,
  DELTA_FLAG_DRY_RUN = 1u << 3,
  DELTA_FLAG_FAULT = 1u << 4,
  DELTA_FLAG_LINK_STALE = 1u << 5,
  DELTA_FLAG_STREAM_ACTIVE = 1u << 6,
  DELTA_FLAG_FINE_REQUIRED = 1u << 7 // physical compensated bridge, no coarse
};

enum {
  DELTA_STREAM_OPTION_Z_ONLY = 1u << 0,
  DELTA_STREAM_OPTION_XYZ_MANUAL = 1u << 1,
  DELTA_STREAM_OPTION_AUTOMATIC = 1u << 2,
  DELTA_STREAM_OPTION_MODE_MASK = 0x0003u,
  DELTA_STREAM_OPTION_RANGE_R1 = 1u << 4,
  DELTA_STREAM_OPTION_RANGE_R2 = 2u << 4,
  DELTA_STREAM_OPTION_RANGE_R3 = 3u << 4,
  DELTA_STREAM_OPTION_RANGE_MASK = 0x0030u
};

typedef enum {
  DELTA_RESULT_OK = 0,
  DELTA_RESULT_BAD_FRAME = 1,
  DELTA_RESULT_BAD_VERSION = 2,
  DELTA_RESULT_BAD_SESSION = 3,
  DELTA_RESULT_BAD_SEQUENCE = 4,
  DELTA_RESULT_DISARMED = 5,
  DELTA_RESULT_HOME_REQUIRED = 6,
  DELTA_RESULT_BUSY = 7,
  DELTA_RESULT_RAW_RANGE = 8,
  DELTA_RESULT_RAW_STEP = 9,
  DELTA_RESULT_MOVE_TIME = 10,
  DELTA_RESULT_BAD_TYPE = 11,
  DELTA_RESULT_TX_FAILED = 12,
  DELTA_RESULT_FAULT_LATCHED = 13,
  DELTA_RESULT_BAD_HOME = 14,
  DELTA_RESULT_FEEDBACK_FAILED = 15,
  DELTA_RESULT_STREAM_REQUIRED = 16,
  DELTA_RESULT_STREAM_ACTIVE = 17,
  DELTA_RESULT_STREAM_RATE = 18,
  DELTA_RESULT_STREAM_Z_ONLY = 19,
  DELTA_RESULT_STREAM_DISABLED = 20,
  DELTA_RESULT_BAD_RANGE_PROFILE = 21,
  DELTA_RESULT_OBSERVED = 22 /* valid physical pose; NOT a precision PASS */
} DeltaResultCode;

#if defined(__GNUC__)
#define DELTA_PACKED __attribute__((packed))
#else
#define DELTA_PACKED
#pragma pack(push, 1)
#endif

typedef struct DELTA_PACKED {
  uint8_t sofA;
  uint8_t sofB;
  uint8_t version;
  uint8_t type;
  uint32_t session;
  uint16_t sequence;
  uint16_t raw[3];
  uint16_t moveTimeMs;
  uint16_t status;
  uint16_t crc;
} DeltaLinkFrame;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

#if defined(__cplusplus)
static_assert(sizeof(DeltaLinkFrame) == DELTA_LINK_FRAME_SIZE,
              "DeltaLinkFrame debe medir 22 bytes");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(DeltaLinkFrame) == DELTA_LINK_FRAME_SIZE,
               "DeltaLinkFrame debe medir 22 bytes");
#endif

static inline uint16_t deltaCrc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFFu;
  for (size_t index = 0; index < length; ++index) {
    crc ^= (uint16_t)data[index] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static inline void deltaPrepareFrame(DeltaLinkFrame *frame) {
  frame->sofA = DELTA_LINK_SOF_A;
  frame->sofB = DELTA_LINK_SOF_B;
  frame->version = DELTA_LINK_VERSION;
  frame->crc = 0u;
  frame->crc = deltaCrc16((const uint8_t *)frame, DELTA_LINK_CRC_BYTES);
}

static inline int deltaFrameValid(const DeltaLinkFrame *frame) {
  if (frame->sofA != DELTA_LINK_SOF_A || frame->sofB != DELTA_LINK_SOF_B ||
      frame->version != DELTA_LINK_VERSION) return 0;
  return frame->crc == deltaCrc16((const uint8_t *)frame,
                                  DELTA_LINK_CRC_BYTES);
}

static inline uint16_t deltaMakeStatus(uint8_t flags, DeltaResultCode result) {
  return (uint16_t)flags | ((uint16_t)result << 8);
}

static inline uint8_t deltaStatusFlags(uint16_t status) {
  return (uint8_t)(status & 0x00FFu);
}

static inline DeltaResultCode deltaStatusResult(uint16_t status) {
  return (DeltaResultCode)((status >> 8) & 0x00FFu);
}

#endif
