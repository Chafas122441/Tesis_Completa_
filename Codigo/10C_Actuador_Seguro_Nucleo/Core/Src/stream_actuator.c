/**
 * @file stream_actuator.c
 * @brief Actuacion segura del DELTA y enlace con la Portenta.
 * Valida sesion, CRC, permisos y limites. Coordina HOME, STREAM, DIRECT,
 * retroalimentacion, compensacion acotada y parada. Los tres HTS usan canales
 * independientes: PA9 USART1, PB10 USART3 y PA0 UART4.
 */
#include "stream_actuator.h"
#include "gripper_manual.h"
#include "gripper_limits.h"
#include "delta_stream_protocol.h"
#include "local_point_profile.h"
#include "servo_feedback.h"

#include <stdio.h>
#include <string.h>

#define LINK_TIMEOUT_MS 750u
#define STREAM_TIMEOUT_MS 250u
#define STATUS_PERIOD_MS 5000u
/* printf usa el VCP a 115200 y es bloqueante. El resumen espaciado no debe
   perturbar el STREAM fisico de 20 Hz. */
#define STREAM_ACCEPT_PRINT_INTERVAL 200u
#define RANGE_R1_RAW_MIN 400u
#define RANGE_R1_RAW_MAX 540u
#define RANGE_R2_RAW_MIN 390u
#define RANGE_R2_RAW_MAX 580u
#define RANGE_R3_RAW_MIN 205u
#define RANGE_R3_RAW_MAX 700u
/* HOME conserva una envolvente independiente. Los setpoints STREAM usan el
   perfil R1/R2/R3 negociado al comenzar la sesion y validado en cada trama. */
#define HOME_RECOVERY_RAW_MIN 195u
#define HOME_RECOVERY_RAW_MAX 700u
/* IK de HOME elevado (0,0,-287 mm), guardas y tramos sin cambios. */
#define RAW_HOME 410u
/* 4 RAW cada 50 ms conserva el mismo limite de 80 RAW/s que 2 RAW/25 ms. */
#define STREAM_RAW_MAX_STEP 4u
#define STREAM_COARSE_MAX_STEP 8u
#define STREAM_COARSE_PERIOD_MS 100u
#define STREAM_COARSE_TIME_MS 200u
#define STREAM_Z_SPREAD_MAX 2u
#define HOME_TIME_MS 2500u
#define HOME_CAPTURE_DELAY_MS 100u
#define HOME_SEGMENT_MAX_STEP_RAW 25
#define HOME_SEGMENT_TIME_MS 1100u
#define HOME_SEGMENT_SETTLE_MS 200u
#define HOME_SEGMENT_TOLERANCE_RAW 5
#define HOME_SEGMENT_MIN_PROGRESS_RAW 2
#define HOME_PROGRESS_REQUIRED_DELTA_RAW 8
#define HOME_SEGMENT_MAX_COUNT 80u
#define HOME_SEGMENT_MAX_RETRIES 4u
#define HOME_FINAL_TIME_MS 1200u
#define HOME_FINAL_RETRY_SETTLE_MS 200u
#define STREAM_TIME_MIN_MS 50u
#define STREAM_TIME_MAX_MS 250u
#define STREAM_PHYSICAL_DELTA_RAW 1u
#define STREAM_END_TARGET_TIME_MS 700u
#define STREAM_END_SETTLE_MS 300u
#define CHECKPOINT_REASSERT_TIME_MS 500u
/* START congela el ultimo destino, lo transmite con 700 ms y espera otros
   300 ms antes de leer. En XYZ se corrige desde 4 RAW. Un residuo estable de
   hasta 5 RAW puede aceptarse tras cuatro rondas; hasta 8 RAW puede entrar al
   lazo, pero errores mayores siguen indicando un problema fisico. */
#define XYZ_CORRECTION_TRIGGER_RAW 3
#define XYZ_CORRECTION_MAX_ERROR_RAW 8
#define XYZ_CORRECTION_TIME_MS 700u
#define XYZ_CORRECTION_SETTLE_MS 300u
#define HOME_CORRECTION_TIME_MS 700u
#define HOME_CORRECTION_SETTLE_MS 400u
/* Compensacion de posicion por canal. Reenviar el
   mismo RAW no mueve un servo que se quedo corto por banda muerta: el servo ya
   cree haber llegado. La orden transmitida debe desplazarse respecto del
   destino fisico. commandedRaw sigue siendo el destino; servoCommandRaw es la
   orden compensada. */
#define FEEDBACK_MAX_CORRECTION_STEP_RAW 5
#define FEEDBACK_MAX_TOTAL_COMPENSATION_RAW 8
#define FEEDBACK_LARGE_ERROR_RAW 8
#define FEEDBACK_MAX_CORRECTIONS 4u
#define CORRECTION_RAW_MIN 430u
#define CORRECTION_MAX_OVERSHOOT_RAW 3u
#define CORRECTION_MAX_MEASURED_SPREAD_RAW 2u
#define MEASURED_CORRECTION_TIME_MS 500u
#define MEASURED_CORRECTION_SETTLE_MS 500u
#define FEEDBACK_SAMPLES 3u
#define FEEDBACK_REQUIRED_SAMPLES 2u
/* El control manual Z prioriza continuidad sobre nivelacion fina. Se conserva
   la tolerancia individual de 3 RAW y se admite hasta 6 RAW entre canales al
   cerrar STREAM, igual que el criterio de nivel de HOME ya validado. */
#define FEEDBACK_TOLERANCE_RAW 3
#define POST_CORRECTION_TOLERANCE_RAW 3
#define XYZ_FEEDBACK_TOLERANCE_RAW 5
#define STREAM_FINAL_MAX_CHANNEL_SPREAD_RAW 6
#define HOME_FEEDBACK_TOLERANCE_RAW 4
/* HOME admite una diferencia limitada entre canales al verificar el nivel. */
#define HOME_MAX_CHANNEL_SPREAD_RAW 6
#define FEEDBACK_MAX_SAMPLE_SPREAD_RAW 2
#define STREAM_MIN_REQUESTED_DELTA_RAW 4
#define STREAM_MIN_MEASURED_DELTA_RAW 3
#define FEEDBACK_READ_TIMEOUT_MS 35u
#define FEEDBACK_READ_GAP_MS 5u
#define FEEDBACK_SETTLE_MARGIN_MS 100u
#define FEEDBACK_RETRY_DELAY_MS 250u
#define FEEDBACK_READ_MAX_RETRIES 2u
/* DIRECT uses MOVE_TIME_WRITE, never WAIT/EXECUTE. LIFT23 duplicates only its
   initial batch, with the SAME full time. No mid-flight per-joint retry.
   PATH22/LIFT23 stop on lost progress; only legacy21 may replan once. */
#define DIRECT_BUS_QUIET_MS 2u
#define DIRECT_BUS_IDLE_MAX_MS 8u
#define DIRECT_WRITE_BUDGET_MS 40u      /* three idle waits + three 10-byte frames */
#define DIRECT_TX_TIMEOUT_MS 10u
#define DIRECT_START_PROBE_MS 100u      /* early "did every joint start?" read */
#define DIRECT_LIFT_DELAY_MIN_DURATION_MS 600u
#define DIRECT_LIFT_PROBE_GAP_MS 100u
#define DIRECT_LIFT_INITIAL_COPIES 2u
#define DIRECT_AUTO_INITIAL_COPIES 5u
#define DIRECT_AUTO_RECOVERY_COPIES 5u
#define DIRECT_REDUNDANT_WRITE_BUDGET_MS 80u
#define DIRECT_RECOVERY_MAX_DEVIATION_RAW 12u /* = AUTO compensation bound */
#define DIRECT_HOLD_TIME_MS 100u
#define DIRECT_HOLD_CONFIRM_MS DIRECT_HOLD_TIME_MS
#define DIRECT_HOLD_STABLE_GAP_MS 100u
#define DIRECT_HOLD_STABLE_RAW 1
#define DIRECT_HOLD_CONFIRM_BUDGET_MS 4000u
#define DIRECT_HOLD_CONFIRM_RAW 3
#define DIRECT_HOLD_READ_ATTEMPTS 3u
#define DIRECT_HOLD_READ_RETRY_MS 10u
#define DIRECT_PROBE_START 0u
#define DIRECT_PROBE_MID 1u
#define DIRECT_PROBE_RECOVERY 2u
#define DIRECT_PROBE_ADAPTIVE 3u
#define DIRECT_STOP_MAX_ATTEMPTS 2u
#define DIRECT_PROGRESS_MIN_PROBE_MS 150u
#define DIRECT_PROGRESS_MAX_PROBE_MS 400u
#define DIRECT_PROGRESS_MIN_DISTANCE_RAW 12u
#define DIRECT_PROGRESS_MIN_EXPECTED_RAW 8u
#define DIRECT_PROGRESS_MIN_REQUIRED_RAW 3u
#define DIRECT_PROGRESS_REVERSE_RAW 3
#define DIRECT_PROGRESS_CORRIDOR_RAW 5
#define DIRECT_PROGRESS_READ_TIMEOUT_MS 20u
#define DIRECT_PROGRESS_RETRY_GAP_MS 10u
#define DIRECT_PROGRESS_READ_ATTEMPTS 5u
/* From the first serviced attempt of each START/MID probe, not per retry.
   One UART operation per loop; no MOVE resend, stale PASS or lease extension. */
#define DIRECT_PROGRESS_READ_BUDGET_MS 120u
/* New paired post-pickup profile only: short reads, leases still serviced.
   Both early windows must fit before the final physical checkpoint. */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(DELTA_DIRECT_PROFILE250_FIRST_PROBE_MS +
  2u*DELTA_POST_PICKUP_READ_BUDGET_MS < DELTA_POST_PICKUP_MIN_DURATION_MS,
  "Post-pickup read300 requires room for both nonblocking windows");
#endif
#define CHECKPOINT_TIMEOUT_MS DELTA_CHECKPOINT_TIMEOUT_MS
#define CHECKPOINT_MAX_CORRECTIONS 1u
#define AUTO_CORRECTION_MAX_ERROR_RAW 12
#define AUTO_CHECKPOINT_MAX_CORRECTIONS 3u
/* One lost command/late settling may look like zero progress. Reassert the
   SAME compensated target once, never add another offset without progress. */
#define AUTO_PROGRESS_REASSERT_MAX 1u
#define AUTO_PROGRESS_REASSERT_SETTLE_MS 600u
/* Cada ventana incluye el lote inicial y ambos reintentos de lectura.
   Se reserva el peor caso aunque los reintentos CHECKPOINT sean selectivos. */
#define CHECKPOINT_READ_WINDOW_BUDGET_MS ( \
  (1u + FEEDBACK_READ_MAX_RETRIES) * 3u * FEEDBACK_SAMPLES * \
  (FEEDBACK_READ_TIMEOUT_MS + FEEDBACK_READ_GAP_MS + 1u) + \
  FEEDBACK_READ_MAX_RETRIES * FEEDBACK_RETRY_DELAY_MS + \
  FEEDBACK_SETTLE_MARGIN_MS)
/* Peor TX de tres puertos, movimiento, asentamiento y ventana POST completa. */
#define CHECKPOINT_CORRECTION_BUDGET_MS (300u + XYZ_CORRECTION_TIME_MS + \
  XYZ_CORRECTION_SETTLE_MS + CHECKPOINT_READ_WINDOW_BUDGET_MS)
#define AUTO_PROGRESS_REASSERT_BUDGET_MS (300u + XYZ_CORRECTION_TIME_MS + \
  AUTO_PROGRESS_REASSERT_SETTLE_MS + CHECKPOINT_READ_WINDOW_BUDGET_MS)

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(DELTA_CHECKPOINT_AUTO9_TOLERANCE_RAW == 9u &&
               DELTA_CHECKPOINT_AUTO9_TOLERANCE_RAW < AUTO_CORRECTION_MAX_ERROR_RAW,
               "AUTO9 changes acceptance, not correction step5/offset12 or pickup5");
_Static_assert(DELTA_CHECKPOINT_PRECISION_RAW == XYZ_FEEDBACK_TOLERANCE_RAW &&
               DELTA_CHECKPOINT_BOX_TOLERANCE_RAW == 8u &&
               DELTA_CHECKPOINT_BOX_TOLERANCE_RAW <= AUTO_CORRECTION_MAX_ERROR_RAW,
               "Box permission changes acceptance only, not correction/physical bounds");
_Static_assert(DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW == 15u &&
               DELTA_CHECKPOINT_BOX15_MAX_ERROR_RAW == 20u &&
               DELTA_CHECKPOINT_BOX15_MAX_ERROR_RAW - DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW <= FEEDBACK_MAX_CORRECTION_STEP_RAW &&
               AUTO_CORRECTION_MAX_ERROR_RAW == 12,
               "BOX15 may observe20, but correction steps5/offset12 stay bounded");
_Static_assert(DELTA_AUTO15_TOLERANCE_RAW == 15u && DELTA_AUTO15_RECOVERABLE_RAW == 30u &&
               FEEDBACK_MAX_CORRECTION_STEP_RAW == 5 && AUTO_CORRECTION_MAX_ERROR_RAW == 12,
               "AUTO15_REC30 increases observed-error eligibility, not steps or compensation");
_Static_assert(CHECKPOINT_TIMEOUT_MS >= 300u + CHECKPOINT_REASSERT_TIME_MS +
                STREAM_END_SETTLE_MS + CHECKPOINT_READ_WINDOW_BUDGET_MS +
                CHECKPOINT_CORRECTION_BUDGET_MS,
               "CHECKPOINT debe reservar ambas lecturas y la correccion completa.");
_Static_assert(DELTA_AUTO_CHECKPOINT_TIMEOUT_MS >= 300u + CHECKPOINT_REASSERT_TIME_MS +
                STREAM_END_SETTLE_MS + CHECKPOINT_READ_WINDOW_BUDGET_MS +
                AUTO_CHECKPOINT_MAX_CORRECTIONS * CHECKPOINT_CORRECTION_BUDGET_MS +
                AUTO_PROGRESS_REASSERT_MAX * AUTO_PROGRESS_REASSERT_BUDGET_MS,
               "AUTO debe reservar correcciones y unica reafirmacion sin progreso.");
_Static_assert(DELTA_PICKUP_FINE_TIMEOUT_MS >= 300u + CHECKPOINT_REASSERT_TIME_MS +
                STREAM_END_SETTLE_MS + CHECKPOINT_READ_WINDOW_BUDGET_MS +
                DELTA_PICKUP_FINE_MAX_CORRECTIONS * CHECKPOINT_CORRECTION_BUDGET_MS +
                AUTO_PROGRESS_REASSERT_MAX * AUTO_PROGRESS_REASSERT_BUDGET_MS,
               "Pickup20 reserves four corrections and one same-target reassert.");
_Static_assert(DELTA_PICKUP_FINE_MAX_COMPENSATION_RAW ==
                 DELTA_PICKUP_FINE_MAX_CORRECTIONS * FEEDBACK_MAX_CORRECTION_STEP_RAW &&
               DELTA_PICKUP_FINE_MAX_ERROR_RAW == 20u && XYZ_FEEDBACK_TOLERANCE_RAW == 5,
               "Pickup20 changes only recoverability/compensation, never precision or step5.");
_Static_assert(HOME_MAX_CHANNEL_SPREAD_RAW >= HOME_FEEDBACK_TOLERANCE_RAW,
               "El desnivel HOME no puede ser menor que la tolerancia de un canal.");
_Static_assert(STREAM_FINAL_MAX_CHANNEL_SPREAD_RAW >=
                 POST_CORRECTION_TOLERANCE_RAW,
               "El desnivel Z final debe contener la tolerancia individual.");
_Static_assert(STREAM_FINAL_MAX_CHANNEL_SPREAD_RAW <=
                 HOME_MAX_CHANNEL_SPREAD_RAW,
               "El STOP Z no debe ser mas permisivo que HOME.");
_Static_assert(FEEDBACK_MAX_TOTAL_COMPENSATION_RAW >=
                 FEEDBACK_MAX_CORRECTION_STEP_RAW,
               "La compensacion acumulada debe permitir al menos un paso.");
_Static_assert(FEEDBACK_MAX_TOTAL_COMPENSATION_RAW >=
                 XYZ_CORRECTION_MAX_ERROR_RAW,
               "Todo error XYZ admitido debe caber en la compensacion total.");
_Static_assert(FEEDBACK_LARGE_ERROR_RAW >= XYZ_CORRECTION_MAX_ERROR_RAW,
               "XYZ admitido debe usar OFFSET_COMP y no RESEND_DESIRED.");
_Static_assert(FEEDBACK_MAX_CORRECTIONS == 4u,
               "La secuencia STOP validada requiere cuatro rondas maximas.");
_Static_assert(HOME_RECOVERY_RAW_MIN <= RANGE_R3_RAW_MIN &&
                 HOME_RECOVERY_RAW_MAX >= RANGE_R3_RAW_MAX,
               "La envolvente HOME debe contener toda la guardia STREAM.");
_Static_assert(RAW_HOME >= HOME_RECOVERY_RAW_MIN &&
                 RAW_HOME <= HOME_RECOVERY_RAW_MAX,
               "RAW_HOME debe pertenecer a la envolvente de recuperacion.");
_Static_assert(HOME_SEGMENT_MAX_RETRIES <= FEEDBACK_MAX_CORRECTIONS,
               "HOME no debe reintentar mas que el lazo cerrado validado.");
_Static_assert(3u * (DIRECT_BUS_IDLE_MAX_MS + 2u) <= DIRECT_WRITE_BUDGET_MS &&
                 DIRECT_WRITE_BUDGET_MS + DIRECT_PROGRESS_READ_TIMEOUT_MS < STREAM_TIMEOUT_MS &&
                 DIRECT_START_PROBE_MS < DIRECT_PROGRESS_MIN_PROBE_MS,
               "DIRECT: escrituras acotadas dentro de la lease STREAM.");
/* Up to five: the shared deadline takes priority over the attempt count. */
_Static_assert(DIRECT_PROGRESS_READ_ATTEMPTS == 5u &&
                 DIRECT_BUS_IDLE_MAX_MS + DIRECT_PROGRESS_READ_TIMEOUT_MS +
                   DIRECT_PROGRESS_RETRY_GAP_MS < DIRECT_PROGRESS_READ_BUDGET_MS &&
                 DIRECT_PROGRESS_READ_BUDGET_MS < STREAM_TIMEOUT_MS,
               "DIRECT: hasta cinco intentos, sin ampliar ventana ni watchdog.");
_Static_assert(DELTA_DIRECT_LIFT_FIRST_PROBE_MS == 200u &&
                 DELTA_DIRECT_LIFT_FIRST_PROBE_MS < STREAM_TIMEOUT_MS &&
                 DELTA_DIRECT_LIFT_FIRST_PROBE_MS + DIRECT_LIFT_PROBE_GAP_MS +
                   2u * DIRECT_PROGRESS_READ_BUDGET_MS < DIRECT_LIFT_DELAY_MIN_DURATION_MS,
               "LIFT: retraso acotado, dos sondas antes del fin, mismas leases.");
_Static_assert(DIRECT_LIFT_INITIAL_COPIES == 2u &&
                 3u * DIRECT_LIFT_INITIAL_COPIES * (DIRECT_BUS_QUIET_MS + 2u) <= DIRECT_WRITE_BUDGET_MS,
               "LIFT: solo una copia inicial extra, dentro del presupuesto existente.");
_Static_assert(DIRECT_AUTO_INITIAL_COPIES == 5u && DIRECT_AUTO_RECOVERY_COPIES == 5u &&
                 3u * DIRECT_AUTO_INITIAL_COPIES * (DIRECT_BUS_QUIET_MS + 2u) <=
                   DIRECT_REDUNDANT_WRITE_BUDGET_MS &&
                 DIRECT_REDUNDANT_WRITE_BUDGET_MS + DIRECT_PROGRESS_READ_TIMEOUT_MS < STREAM_TIMEOUT_MS,
               "TX5/REC5: finite batch, unchanged leases and speeds.");
_Static_assert(DELTA_DIRECT_PROFILE250_FIRST_PROBE_MS == 250u &&
                 DELTA_DIRECT_PROFILE250_FIRST_PROBE_MS <= STREAM_TIMEOUT_MS &&
                 DELTA_DIRECT_PROFILE250_FIRST_PROBE_MS + DIRECT_LIFT_PROBE_GAP_MS +
                   2u * DIRECT_PROGRESS_READ_BUDGET_MS < DELTA_DIRECT_PROFILE250_MIN_DURATION_MS,
               "Perfil250: dos sondas acotadas, sin ampliar leases ni tiempo de lectura.");
#endif

typedef enum {
  VERIFY_NONE = 0u,
  VERIFY_HOME_CAPTURE,
  VERIFY_HOME_SEGMENT,
  VERIFY_HOME,
  VERIFY_STREAM_END,
  VERIFY_STREAM_CHECKPOINT,
  VERIFY_FAULT_RECOVERY
} VerificationPurpose;

static UART_HandleTypeDef *linkPort;
static UART_HandleTypeDef *servoPorts[3];
static const uint8_t servoIds[3] = {1u, 2u, 3u};
static uint8_t dryRunMode;

static uint8_t linkRxByte;
static uint8_t rxBytes[DELTA_LINK_FRAME_SIZE];
static volatile uint8_t rxIndex;
static volatile uint8_t frameReady;
static DeltaLinkFrame pendingFrame;

static uint32_t activeSession;
static uint16_t lastSequence;
static uint8_t sequenceKnown;
static uint32_t lastValidFrameMs;
static uint32_t lastStreamSetpointMs;
static uint32_t lastStreamMotionMs;
static uint8_t checkpointReassertNeeded;
static uint32_t lastStatusMs;
static uint32_t busyUntilMs;
static uint8_t linkStale = 1u;
static uint8_t armed;
static uint8_t homeValid;
static uint8_t homePending;
static uint8_t busy;
static uint8_t streamActive;
static uint16_t activeStreamOption;
static uint8_t faultLatched;
static uint8_t faultRecoverable;
static uint8_t recoveryPending;
static uint16_t recoverySequence;
static uint32_t recoveryStartedMs;
static int16_t checkpointPreviousError[3];
static int16_t checkpointPreviousSignedError[3];
static uint8_t manualGripRuntime; // separate permit, never inherited from calibration
static uint8_t automaticGripRuntime; // latched only by explicit AUTO GROUP_BEGIN
static uint8_t rawKnown;
static uint8_t verifiedSnapshotKnown;
static uint16_t commandedRaw[3];
static uint16_t servoCommandRaw[3];
static DeltaPointProfile localPoint;
static uint8_t localPointCapable, directPointCapable, directTransactionCapable;
static uint8_t directValidatedPathCapable, directValidatedPathActive;
static uint8_t directLiftDelayCapable, directLiftProbeActive, directProfile250Capable;
static uint8_t directRepeat5Capable;
static uint8_t approachStep5Capable;
static uint8_t directRetryCapable, directRetryUsed, directRetryPhase;
/* 1=confirm hold, 2/3=fresh stable pose rounds, 4=await B path validation. */
static uint16_t directRetryOrigin[3], directRetryFirst[3];
static uint32_t directRetryStartedMs, directRetryOriginMs;
static uint8_t localPointActive, directPointActive, localPointKnown;
static uint16_t localPointSequence, localPointGrip[3];
static uint32_t localPointTick;
static uint16_t directPointDuration;
static uint8_t directStartedMask;
static uint16_t directStartRaw[3], directGoalRaw[3];
static int16_t directProbeRaw[3];
static uint32_t directProbeElapsedMs[3];
static uint32_t directProbeDueMs, directProbeNextReadMs;
static uint8_t directProbePending, directProbeAxis, directProbeRequiredMask;
static uint8_t directProbeKnownMask, directProbeReadAttempts[3];
static uint8_t directFinalReadCapable, directFinalReadActive, directReadDeferred;
static uint8_t endpointRetryCapable, endpointRetryEligible, endpointRetryCount;
static uint8_t directEndpointAwaitingRecovery;
static uint16_t endpointRetryLimitRaw;
static uint16_t endpointRetryToleranceRaw=DELTA_ENDPOINT_RETRY_TOLERANCE_RAW;
static uint8_t endpointCenterCapable;
static uint8_t pickupFine20Capable, checkpointPickupFine20;
static uint8_t centerPrecisionRequired;
static uint8_t endpointStepCapable,endpointStepActive,endpointStepPhase;
static uint8_t endpointStepAxis,endpointStepReadAttempts,endpointStepRepeats,endpointStepTxMask;
static uint16_t endpointStepIndex,endpointStepCount;
static uint16_t endpointStepStart[3],endpointStepGoal[3],endpointStepTarget[3];
static int16_t endpointStepSample[3],endpointStepPrevious[3];
static uint32_t endpointStepDueMs,endpointStepReadStartedMs;
static uint32_t endpointRetryWaitStartMs;
static uint8_t endpointRetryWaitKnown;
static uint8_t directWriteMask = 0x07u;
static uint16_t endpointRetryNominal[3], endpointRetryDurationFloor;
static uint8_t directProbeTimeoutMask, directProbeMalformed;
static uint32_t directReadWarnings, directProgressWarnings;
static uint8_t directProgressCheckedMask;
static uint8_t directProbeWindowKnown;
static uint32_t directProbeWindowStartedMs;
static const char *directTransactionFaultReason;
/* Current common segment: the original endpoint or its single coordinated
   recovery (all three joints, one time). */
static uint16_t directSegStartRaw[3], directSegDurationMs;
static uint32_t directSegStartMs, directEndMs;
static uint8_t directRecovered, directBusBusyMask, directProbeStage;
static uint8_t directWriteAttempted;
/* Stop/hold: envelope captured at DIRECT start, hold poses awaiting re-read. */
static uint16_t directHoldMin, directHoldMax, directHoldRaw[3];
static uint8_t directHoldPendingMask, directHoldRetryMask, directAbortActive;
static uint8_t directHoldStableMask;
static int16_t directHoldCandidateRaw[3];
static uint32_t directHoldSampleMs[3];
static uint32_t directHoldStableGapMs[3], directHoldStartedMs;
static uint32_t directHoldDueMs;
static uint8_t directHoldReadAttempts[3];
static uint32_t directHoldReadDueMs[3];
static int16_t measuredRaw[3];
static uint8_t measuredKnown[3];
static int16_t streamBaselineRaw[3];
static uint8_t streamBaselineKnown;
static uint8_t streamCorrectionAttempted;
static uint8_t xyzCorrectionRounds;
/* Extremos del intervalo por canal: ultima orden cuya medicion quedo por
   debajo del destino y ultima que quedo por encima. Con ambos conocidos se
   bisecta hasta el ultimo RAW entero disponible. Se reinician cada vez que
   cambia el destino, porque pertenecen a un objetivo concreto. */
static uint16_t belowCommand[3];
static uint8_t belowKnown[3];
static uint16_t aboveCommand[3];
static uint8_t aboveKnown[3];
static uint8_t verificationReadRetryAttempted;
static uint8_t homeSegmentRetryCount;
static uint8_t homeCorrectionRounds;
static uint8_t homeSegmentCount;
static uint8_t homeSegmentTotal;
static int16_t homePathOriginRaw[3];
static int16_t homeSegmentStartRaw[3];
static int16_t feedbackSamples[3][FEEDBACK_SAMPLES];
static uint8_t feedbackValidCount[3];
static uint8_t feedbackSampleIndex;
static uint8_t feedbackChannel;
static uint8_t verificationPending;
static uint8_t verificationSampling;
static uint8_t verificationReadChannelMask;
static VerificationPurpose verificationPurpose;
static uint32_t verificationStartMs;
static uint32_t nextFeedbackReadMs;
static uint8_t checkpointPending;
static uint8_t checkpointBoxToleranceRaw; // 0/8/9/15, scoped to the current checkpoint only
static uint8_t automaticStopCapable; // explicit probe within this STREAM
static uint8_t streamEndToleranceRaw; // 0=legacy, otherwise explicit AUTO5/9/15
static uint8_t auto15Rec30Capable, checkpointRecover30, streamEndRecover30;
static uint8_t observeCapable, checkpointObserve;
static uint32_t automaticStopStartedMs;
static int16_t stopPreviousError[3];
static uint8_t checkpointCorrectionRounds;
static uint8_t checkpointProgressReasserts;
static uint8_t checkpointInvalidMask;
static uint8_t checkpointCompensationHeld;
static uint8_t checkpointTargetConverging;
/* Origen fisico, no evidencia PASS ni permiso. STOP sin TX no elimina el
   offset de la ultima orden; el siguiente STREAM conserva el puente <=4. */
static uint8_t checkpointOutputOffsetKnown;
static uint16_t checkpointSequence;
static uint32_t checkpointStartedMs;
static uint16_t checkpointGripUs[3];

static DeltaLinkFrame lastResponse;
static uint8_t lastResponseValid;

static uint32_t framesReceived;
static uint32_t framesValid;
static uint32_t crcErrors;
static uint32_t versionErrors;
static uint32_t sessionErrors;
static uint32_t sequenceErrors;
static uint32_t acceptedSetpoints;
static uint32_t rejectedFrames;
static uint32_t duplicateFrames;
static uint32_t streamTimeouts;
static uint32_t uartErrors;
static uint32_t servoTxBatches;
static uint32_t changedSetpoints;
static uint32_t skippedSameTargets;
static uint32_t coalescedSetpoints;
static uint32_t physicalStreamBatches;
static uint32_t finalTargetFlushes;
static uint32_t feedbackCorrections;
static uint32_t feedbackReadRetries;
static uint32_t homeSegmentRetries;
static uint32_t feedbackReads;
static uint32_t feedbackOk;
static uint32_t feedbackTimeouts;
static uint32_t feedbackInvalid;
static uint32_t verificationPassed;
static uint32_t verificationFaults;
static uint32_t directWrites;
static uint32_t directResends;
static uint32_t directWriteFailures;
static uint32_t directBusBusy;
static uint32_t directBusDrainedBytes;
static uint32_t directStops;
static uint32_t directStopFailures;
static uint32_t directProgressPasses;
static uint32_t directProgressFaults;
static uint32_t directReadAttempts, directReadRetries, directReadRecovered;
static uint32_t directReadExhausted, directReadBudgetFaults;
static uint32_t directReadBusBusy, directReadBusDrainedBytes;

static uint8_t ExecuteTargets(const uint16_t raw[3], uint16_t moveTimeMs);
static uint32_t DirectTransactionBudgetMs(uint8_t copies) {
  return copies > DIRECT_LIFT_INITIAL_COPIES ? DIRECT_REDUNDANT_WRITE_BUDGET_MS :
    DIRECT_WRITE_BUDGET_MS;
}

static uint8_t ExecuteDirectTransaction(const uint16_t raw[3],
                                        uint16_t moveTimeMs,
                                        uint8_t copies,
                                        uint8_t *failedMask);
static uint8_t CheckpointCorrectionGuards(void);
static uint8_t LocalPointGuards(void);
static void ClearDirectProbeState(void);
static void AbortDirectMotion(void);
static int16_t AbsI16(int16_t value);
static uint16_t AbsDiff(uint16_t a, uint16_t b);
static void LatchVerificationFault(const char *reason, uint8_t failedMask);

/* A distinct BEGIN permission, never inferred from XYZ/manual or RC activity. */
static uint8_t AutomaticCheckpoint(void) {
  return verificationPurpose == VERIFY_STREAM_CHECKPOINT &&
    (activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC) != 0u;
}
static uint8_t AutomaticStop(void) {
  return verificationPurpose == VERIFY_STREAM_END && streamEndToleranceRaw != 0u &&
    (activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC) != 0u;
}
static int16_t StopCorrectibleError(void) {
  if (AutomaticStop() && streamEndRecover30) return DELTA_AUTO15_RECOVERABLE_RAW;
  return AutomaticStop() ? (streamEndToleranceRaw == DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW
    ? DELTA_CHECKPOINT_BOX15_MAX_ERROR_RAW : AUTO_CORRECTION_MAX_ERROR_RAW)
    : XYZ_CORRECTION_MAX_ERROR_RAW;
}
static uint32_t CheckpointTimeout(void) {
  if (AutomaticCheckpoint() && checkpointPending && checkpointPickupFine20)
    return DELTA_PICKUP_FINE_TIMEOUT_MS;
  return (activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC)
    ? DELTA_AUTO_CHECKPOINT_TIMEOUT_MS : CHECKPOINT_TIMEOUT_MS;
}
static int16_t CheckpointPositionTolerance(void) {
  return checkpointPending && checkpointBoxToleranceRaw &&
    verificationPurpose == VERIFY_STREAM_CHECKPOINT &&
    (activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC)
      ? (int16_t)checkpointBoxToleranceRaw : XYZ_FEEDBACK_TOLERANCE_RAW;
}
static int16_t CheckpointCorrectibleError(void) {
  if (AutomaticCheckpoint() && checkpointPending && checkpointPickupFine20)
    return DELTA_PICKUP_FINE_MAX_ERROR_RAW;
  if (AutomaticCheckpoint() && checkpointPending && checkpointRecover30)
    return DELTA_AUTO15_RECOVERABLE_RAW;
  return AutomaticCheckpoint() && checkpointPending &&
      checkpointBoxToleranceRaw == DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW
    ? (int16_t)DELTA_CHECKPOINT_BOX15_MAX_ERROR_RAW
    : (AutomaticCheckpoint() ? AUTO_CORRECTION_MAX_ERROR_RAW : XYZ_CORRECTION_MAX_ERROR_RAW);
}
static int16_t CompensationLimit(void) {
  if (AutomaticCheckpoint() && checkpointPending && checkpointPickupFine20)
    return DELTA_PICKUP_FINE_MAX_COMPENSATION_RAW;
  return (AutomaticCheckpoint() || AutomaticStop()) ? AUTO_CORRECTION_MAX_ERROR_RAW
                               : FEEDBACK_MAX_TOTAL_COMPENSATION_RAW;
}

static const char *StreamModeName(uint16_t option) {
  const uint16_t mode = option & DELTA_STREAM_OPTION_MODE_MASK;
  if (mode == DELTA_STREAM_OPTION_Z_ONLY) return "Z_ONLY";
  if (mode == DELTA_STREAM_OPTION_XYZ_MANUAL) return "XYZ_MANUAL";
  return "NONE";
}

static const char *RangeProfileName(uint16_t option) {
  switch (option & DELTA_STREAM_OPTION_RANGE_MASK) {
    case DELTA_STREAM_OPTION_RANGE_R1: return "R1_400_540";
    case DELTA_STREAM_OPTION_RANGE_R2: return "R2_390_580";
    case DELTA_STREAM_OPTION_RANGE_R3: return "R3_EXP_205_700";
    default: return "NONE";
  }
}

static uint16_t ProfileRawMin(uint16_t option) {
  switch (option & DELTA_STREAM_OPTION_RANGE_MASK) {
    case DELTA_STREAM_OPTION_RANGE_R2: return RANGE_R2_RAW_MIN;
    case DELTA_STREAM_OPTION_RANGE_R3: return RANGE_R3_RAW_MIN;
    default: return RANGE_R1_RAW_MIN;
  }
}

static uint16_t ProfileRawMax(uint16_t option) {
  switch (option & DELTA_STREAM_OPTION_RANGE_MASK) {
    case DELTA_STREAM_OPTION_RANGE_R2: return RANGE_R2_RAW_MAX;
    case DELTA_STREAM_OPTION_RANGE_R3: return RANGE_R3_RAW_MAX;
    default: return RANGE_R1_RAW_MAX;
  }
}

static void ClearDirectProbeState(void) {
  directProbePending = 0u;
  directProbeAxis = 0u;
  directProbeRequiredMask = 0u;
  directProbeKnownMask = 0u;
  directProbeTimeoutMask = directProbeMalformed = 0u;
  memset(directProbeReadAttempts, 0, sizeof(directProbeReadAttempts));
  directProbeWindowKnown = 0u;
  directProbeWindowStartedMs = 0u;
  directProbeDueMs = directProbeNextReadMs = 0u;
  memset(directProbeRaw, 0, sizeof(directProbeRaw));
  memset(directProbeElapsedMs, 0, sizeof(directProbeElapsedMs));
}

static uint8_t DirectHoldRawValid(uint8_t index, int16_t raw) {
  /* A hold is a motion command too: only inside the validated DIRECT corridor
     and the RAW range active when that DIRECT started. */
  const int16_t start = (int16_t)directStartRaw[index];
  const int16_t goal = (int16_t)directGoalRaw[index];
  const int16_t low = (int16_t)((start < goal ? start : goal) - DIRECT_PROGRESS_CORRIDOR_RAW);
  const int16_t high = (int16_t)((start > goal ? start : goal) + DIRECT_PROGRESS_CORRIDOR_RAW);
  return raw >= low && raw <= high &&
         raw >= (int16_t)directHoldMin && raw <= (int16_t)directHoldMax;
}

static uint32_t DirectHoldStableGap(uint8_t index, uint16_t holdRaw) {
  if (AbsDiff(directGoalRaw[index], holdRaw) <= DIRECT_HOLD_CONFIRM_RAW)
    return DIRECT_HOLD_STABLE_GAP_MS;
  /* With an unconfirmed STOP the old interpolator may still be running. A
     fixed 100 ms test can mistake slow, quantized motion for a successful hold.
     Observe long enough for either old command to advance four RAW: this is
     deliberately wider than the allowed one-RAW spread between fresh samples.
     No finite software observation proves a servo has accepted its new goal;
     if this conservative window cannot fit the budget, retain uncertainty. */
  const uint16_t initialDistance = AbsDiff(directStartRaw[index], directGoalRaw[index]);
  const uint16_t segmentDistance = AbsDiff(directSegStartRaw[index], directGoalRaw[index]);
  if (!initialDistance || !segmentDistance) return DIRECT_HOLD_CONFIRM_BUDGET_MS + 1u;
  uint32_t gap = ((uint32_t)directPointDuration * 4u + initialDistance - 1u) /
                 initialDistance;
  const uint32_t segmentGap = ((uint32_t)directSegDurationMs * 4u + segmentDistance - 1u) /
                              segmentDistance;
  if (gap < segmentGap) gap = segmentGap;
  return gap < DIRECT_HOLD_STABLE_GAP_MS ? DIRECT_HOLD_STABLE_GAP_MS : gap;
}

/* Phase 1: MOVE_STOP 0x0C (unconfirmed on HTS-35H), one short frame per joint. */
static void SendDirectStopFrames(void) {
  if (!directStartedMask || dryRunMode) return;
  for (uint8_t index = 0u; index < 3u; ++index) {
    if (!(directStartedMask & (uint8_t)(1u << index))) continue;
    uint8_t stopFrame[6];
    HAL_StatusTypeDef stopResult = HAL_ERROR;
    for (uint8_t attempt = 0u;
         attempt < DIRECT_STOP_MAX_ATTEMPTS && stopResult != HAL_OK; ++attempt)
      stopResult = ServoStopMove(servoPorts[index], servoIds[index], stopFrame);
  }
}

/* Phase 2: proven commands only. Read the real pose, reject anything outside
   the envelope, hold it with 0x01. HAL_OK is NOT a confirmation: the joint
   stays in directStartedMask until ServiceDirectHoldConfirm re-reads it. */
static void AbortDirectMotion(void) {
  /* Repeated DISARM/FAULT/watchdog processing must not restart the confirmation
     budget or turn its single re-hold into unlimited unacknowledged retries. */
  if (!directStartedMask || directAbortActive) return;
  const uint8_t startedMask = directStartedMask;
  directHoldStartedMs = HAL_GetTick();
  uint8_t heldMask = 0u;
  if (!dryRunMode) {
    for (uint8_t index = 0u; index < 3u; ++index) {
      const uint8_t bit = (uint8_t)(1u << index);
      if (!(startedMask & bit)) continue;
      ServoPositionReading reading;
      uint8_t holdFrame[10];
      if (ServoReadPositionBounded(servoPorts[index], servoIds[index], &reading,
                            DIRECT_PROGRESS_READ_TIMEOUT_MS, DIRECT_TX_TIMEOUT_MS) == SERVO_READ_OK &&
          DirectHoldRawValid(index, reading.position) &&
          ServoSendMoveBounded(servoPorts[index], servoIds[index], (uint16_t)reading.position,
                        DIRECT_HOLD_TIME_MS, holdFrame, DIRECT_TX_TIMEOUT_MS) == HAL_OK) {
        heldMask |= bit;
        directHoldRaw[index] = (uint16_t)reading.position;
        directHoldStableGapMs[index] = DirectHoldStableGap(index, directHoldRaw[index]);
        servoCommandRaw[index] = directHoldRaw[index];
        commandedRaw[index] = directHoldRaw[index];
      }
    }
  }
  const uint8_t uncertainMask = (uint8_t)(startedMask & (uint8_t)~heldMask);
  directStartedMask = dryRunMode ? 0u : startedMask;
  directHoldPendingMask = dryRunMode ? 0u : heldMask;
  directHoldRetryMask = 0u;
  directHoldStableMask = 0u;
  memset(directHoldSampleMs, 0, sizeof(directHoldSampleMs));
  memset(directHoldReadAttempts, 0, sizeof(directHoldReadAttempts));
  memset(directHoldReadDueMs, 0, sizeof(directHoldReadDueMs));
  directAbortActive = directStartedMask != 0u;
  directHoldDueMs = HAL_GetTick() + DIRECT_HOLD_CONFIRM_MS;
  verifiedSnapshotKnown = 0u;
  checkpointOutputOffsetKnown = 0u;
  if (!directRetryPhase) directFinalReadActive=0u;
  directReadDeferred=0u;
  ++directStops;
  if (uncertainMask) ++directStopFailures;
  printf("[DIRECT STOP] requested_mask=0x%02X hold_pending=0x%02X uncertain_mask=0x%02X hold=%u,%u,%u; STOP0x0C+POS_READ+MOVE_TIME_WRITE, confirm in %u ms.\r\n",
         startedMask, heldMask, uncertainMask, directHoldRaw[0], directHoldRaw[1],
         directHoldRaw[2], (unsigned)DIRECT_HOLD_CONFIRM_MS);
}

/* Runs after DISARM/FAULT as well. Wait until the entire hold interpolation
   elapsed, then require two fresh stable samples separated by a speed-dependent
   window. A nearby/quantized sample is never by itself a successful stop. */
static void ServiceDirectHoldConfirm(void) {
  if (directAbortActive && !directHoldPendingMask) {
    if (directStartedMask && !faultLatched) {
      LatchVerificationFault("DIRECT_STOP_UNCERTAIN", directStartedMask);
    } else if (!directStartedMask) {
      directAbortActive = 0u;
    }
    return;
  }
  if (!directHoldPendingMask || (int32_t)(HAL_GetTick() - directHoldDueMs) < 0) return;
  uint8_t confirmedMask = 0u, uncertainMask = 0u;
  int16_t sample[3] = {-1, -1, -1};
  for (uint8_t index = 0u; index < 3u; ++index) {
    const uint8_t bit = (uint8_t)(1u << index);
    if (!(directHoldPendingMask & bit)) continue;
    const uint32_t now = HAL_GetTick();
    const uint32_t budgetElapsed = now - directHoldStartedMs;
    const uint32_t observed = (directHoldStableMask & bit)
      ? now - directHoldSampleMs[index] : 0u;
    const uint32_t gapRemaining = observed < directHoldStableGapMs[index]
      ? directHoldStableGapMs[index] - observed : 0u;
    if (budgetElapsed >= DIRECT_HOLD_CONFIRM_BUDGET_MS ||
        gapRemaining > DIRECT_HOLD_CONFIRM_BUDGET_MS - budgetElapsed) {
      printf("[DIRECT STOP UNCERTAIN] id=%u reason=DEADLINE elapsed_ms=%lu gap_remaining_ms=%lu.\r\n",
             servoIds[index], (unsigned long)budgetElapsed, (unsigned long)gapRemaining);
      uncertainMask |= bit;
      continue;
    }
    if ((directHoldStableMask & bit) &&
        now - directHoldSampleMs[index] < directHoldStableGapMs[index]) continue;
    if (directHoldReadAttempts[index] &&
        (int32_t)(now - directHoldReadDueMs[index]) < 0) continue;
    ServoPositionReading reading;
    ++directHoldReadAttempts[index];
    const uint32_t remaining = DIRECT_HOLD_CONFIRM_BUDGET_MS - budgetElapsed;
    const uint32_t readMs = remaining < DIRECT_PROGRESS_READ_TIMEOUT_MS ? remaining :
      DIRECT_PROGRESS_READ_TIMEOUT_MS;
    const ServoReadStatus status = ServoReadPositionBounded(
      servoPorts[index], servoIds[index], &reading, readMs,
      DIRECT_TX_TIMEOUT_MS);
    if (HAL_GetTick() - directHoldStartedMs >= DIRECT_HOLD_CONFIRM_BUDGET_MS) {
      printf("[DIRECT STOP UNCERTAIN] id=%u reason=READ_DEADLINE.\r\n", servoIds[index]);
      uncertainMask |= bit;
      continue;
    }
    if (status != SERVO_READ_OK) {
      if (directHoldReadAttempts[index] < DIRECT_HOLD_READ_ATTEMPTS &&
          HAL_GetTick() - directHoldStartedMs + DIRECT_HOLD_READ_RETRY_MS <
            DIRECT_HOLD_CONFIRM_BUDGET_MS) {
        directHoldReadDueMs[index] = HAL_GetTick() + DIRECT_HOLD_READ_RETRY_MS;
        printf("[DIRECT STOP READ RETRY] id=%u attempt=%u/%u status=%s; no MOVE, deadline unchanged.\r\n",
               servoIds[index], directHoldReadAttempts[index],
               (unsigned)DIRECT_HOLD_READ_ATTEMPTS, ServoReadStatusName(status));
        continue;
      }
      printf("[DIRECT STOP UNCERTAIN] id=%u reason=READ_EXHAUSTED attempts=%u status=%s.\r\n",
             servoIds[index], directHoldReadAttempts[index], ServoReadStatusName(status));
      uncertainMask |= bit;
      continue;
    }
    directHoldReadAttempts[index] = 0u;
    if (!DirectHoldRawValid(index, reading.position)) {
      printf("[DIRECT STOP UNCERTAIN] id=%u reason=RAW_CORRIDOR raw=%d.\r\n",
             servoIds[index], reading.position);
      uncertainMask |= bit;
      continue;
    }
    sample[index] = reading.position;
    const uint32_t sampleMs = HAL_GetTick();
    const uint8_t nearHold = AbsI16((int16_t)(reading.position -
      (int16_t)directHoldRaw[index])) <= DIRECT_HOLD_CONFIRM_RAW;
    if (nearHold) {
      if (!(directHoldStableMask & bit)) {
        directHoldStableMask |= bit;
        directHoldCandidateRaw[index] = reading.position;
        directHoldSampleMs[index] = sampleMs;
        continue;
      }
      if (sampleMs - directHoldSampleMs[index] < directHoldStableGapMs[index]) continue;
      if (AbsI16((int16_t)(reading.position - directHoldCandidateRaw[index])) <=
          DIRECT_HOLD_STABLE_RAW) {
        confirmedMask |= bit;
        measuredRaw[index] = reading.position;
        measuredKnown[index] = 1u;
        continue;
      }
    }
    uint8_t holdFrame[10];
    if (!(directHoldRetryMask & bit) &&
        ServoSendMoveBounded(servoPorts[index], servoIds[index], (uint16_t)reading.position,
                      DIRECT_HOLD_TIME_MS, holdFrame, DIRECT_TX_TIMEOUT_MS) == HAL_OK) {
      directHoldRetryMask |= bit; // still moving: the hold was probably lost
      directHoldRaw[index] = (uint16_t)reading.position;
      directHoldStableGapMs[index] = DirectHoldStableGap(index, directHoldRaw[index]);
      servoCommandRaw[index] = directHoldRaw[index];
      commandedRaw[index] = directHoldRaw[index];
      directHoldStableMask &= (uint8_t)~bit;
      continue;
    }
    printf("[DIRECT STOP UNCERTAIN] id=%u reason=UNSTABLE_OR_REHOLD_FAILED raw=%d.\r\n",
           servoIds[index], reading.position);
    uncertainMask |= bit;
  }
  directHoldPendingMask &= (uint8_t)~(confirmedMask | uncertainMask);
  directHoldStableMask &= directHoldPendingMask;
  directStartedMask &= (uint8_t)~confirmedMask;
  if (uncertainMask) ++directStopFailures;
  if (directHoldPendingMask) {
    uint32_t waitMs = DIRECT_HOLD_STABLE_GAP_MS;
    for (uint8_t i=0u; i<3u; ++i)
      if ((directHoldPendingMask & (uint8_t)(1u << i)) && directHoldReadAttempts[i])
        waitMs = DIRECT_HOLD_READ_RETRY_MS;
    directHoldDueMs = HAL_GetTick() + waitMs;
  }
  printf("[DIRECT STOP CONFIRM] confirmed=0x%02X rehold_pending=0x%02X uncertain=0x%02X sample=%d,%d,%d hold=%u,%u,%u\r\n",
         confirmedMask, directHoldPendingMask, uncertainMask, sample[0], sample[1],
         sample[2], directHoldRaw[0], directHoldRaw[1], directHoldRaw[2]);
}

static void StopGripperOutputs(void) {
  endpointStepActive=0u;
  directEndpointAwaitingRecovery=0u; // masks still drive real STOP/hold below
  endpointRetryEligible = 0u; // STOP/FAULT/watchdog never retries the old destination
  directRetryPhase = 0u; // STOP/FAULT/menu/watchdogs can NEVER resume a retry
  /* STOP frames are ~1 ms; RC PWM off is immediate. Both before the slower
     POS_READ/hold of every started DIRECT joint. */
  SendDirectStopFrames();
  manualGripRuntime = 0u;
  automaticGripRuntime = 0u;
  GripperManual_Stop();
  AbortDirectMotion();
  ClearDirectProbeState();
  localPointActive = directPointActive = localPointKnown = 0u;
  directLiftProbeActive = 0u;
}

static uint16_t ActiveRawMin(void) { return ProfileRawMin(activeStreamOption); }
static uint16_t ActiveRawMax(void) { return ProfileRawMax(activeStreamOption); }

static uint8_t RangeProfileValid(uint16_t option) {
  const uint16_t range = option & DELTA_STREAM_OPTION_RANGE_MASK;
  return range == DELTA_STREAM_OPTION_RANGE_R1 ||
         range == DELTA_STREAM_OPTION_RANGE_R2 ||
         range == DELTA_STREAM_OPTION_RANGE_R3;
}

static uint8_t StreamOptionValid(uint16_t option) {
  const uint16_t mode = option & DELTA_STREAM_OPTION_MODE_MASK;
  const uint16_t known = DELTA_STREAM_OPTION_MODE_MASK |
                         DELTA_STREAM_OPTION_RANGE_MASK | DELTA_STREAM_OPTION_AUTOMATIC;
  return (option & (uint16_t)~known) == 0u &&
         (!(option & DELTA_STREAM_OPTION_AUTOMATIC) || mode == DELTA_STREAM_OPTION_XYZ_MANUAL) &&
         (mode == DELTA_STREAM_OPTION_Z_ONLY ||
          mode == DELTA_STREAM_OPTION_XYZ_MANUAL) &&
         RangeProfileValid(option);
}

static uint8_t CurrentFlags(void) {
  uint8_t flags = 0u;
  if (armed) flags |= DELTA_FLAG_ARMED;
  if (homeValid) flags |= DELTA_FLAG_HOME_VALID;
  if (busy || localPointActive || directPointActive) flags |= DELTA_FLAG_BUSY;
  if (dryRunMode) flags |= DELTA_FLAG_DRY_RUN;
  if (faultLatched) flags |= DELTA_FLAG_FAULT;
  if (linkStale) flags |= DELTA_FLAG_LINK_STALE;
  if (streamActive) flags |= DELTA_FLAG_STREAM_ACTIVE;
  if (checkpointTargetConverging || checkpointCompensationHeld ||
      checkpointOutputOffsetKnown)
    flags |= DELTA_FLAG_FINE_REQUIRED;
  return flags;
}

static void PrintStatus(void) {
  printf("[STREAM STATUS] physical=%u session=%08lX stale=%u home=%u armed=%u stream=%u mode=%s range=%s busy=%u verify=%u stream_corr=%u xyz_corr=%u home_corr=%u read_retry=%u fault=%u desired=%u,%u,%u tx_raw=%u,%u,%u measured=%d,%d,%d known=%u%u%u baseline=%d,%d,%d baseline_known=%u rx/valid/crc/ver/session/seq/dup/reject=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu setpoints/change/same/coalesce=%lu/%lu/%lu/%lu stream_tx/end_target/correction/read_retry=%lu/%lu/%lu/%lu stream_timeout=%lu uart_err=%lu tx_batch=%lu fb_read/ok/timeout/invalid/pass/fault=%lu/%lu/%lu/%lu/%lu/%lu\r\n",
         dryRunMode ? 0u : 1u, (unsigned long)activeSession, linkStale, homeValid,
         armed, streamActive, StreamModeName(activeStreamOption),
         RangeProfileName(activeStreamOption), busy,
         verificationPending,
         streamCorrectionAttempted, xyzCorrectionRounds, homeCorrectionRounds,
         verificationReadRetryAttempted,
         faultLatched,
         commandedRaw[0], commandedRaw[1], commandedRaw[2],
         servoCommandRaw[0], servoCommandRaw[1], servoCommandRaw[2],
         measuredRaw[0], measuredRaw[1], measuredRaw[2],
         measuredKnown[0], measuredKnown[1], measuredKnown[2],
         streamBaselineRaw[0], streamBaselineRaw[1], streamBaselineRaw[2],
         streamBaselineKnown,
         (unsigned long)framesReceived, (unsigned long)framesValid,
         (unsigned long)crcErrors, (unsigned long)versionErrors,
         (unsigned long)sessionErrors, (unsigned long)sequenceErrors,
         (unsigned long)duplicateFrames, (unsigned long)rejectedFrames,
         (unsigned long)acceptedSetpoints, (unsigned long)changedSetpoints,
         (unsigned long)skippedSameTargets, (unsigned long)coalescedSetpoints,
         (unsigned long)physicalStreamBatches,
         (unsigned long)finalTargetFlushes,
         (unsigned long)feedbackCorrections,
         (unsigned long)feedbackReadRetries,
         (unsigned long)streamTimeouts,
         (unsigned long)uartErrors, (unsigned long)servoTxBatches,
         (unsigned long)feedbackReads, (unsigned long)feedbackOk,
         (unsigned long)feedbackTimeouts, (unsigned long)feedbackInvalid,
         (unsigned long)verificationPassed, (unsigned long)verificationFaults);
  printf("[GRIP STATUS] pwm_active=%u channel=%u us_ACK_Z_X_PALO=%u,%u,%u; COMMANDS, no RC feedback.\r\n",
         (unsigned)GripperManual_Active(), GripperManual_Channel(),
         GripperManual_Pulse(1), GripperManual_Pulse(2), GripperManual_Pulse(3));
  printf("[DIRECT STATUS] txn_cap=%u active=%u started_mask=0x%02X probe=%u required=0x%02X hold_pending=0x%02X write/recover/write_fail/bus_busy/bus_drained/stop/stop_fail/progress_ok/fault=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
         directTransactionCapable, directPointActive, directStartedMask,
         directProbePending, directProbeRequiredMask, directHoldPendingMask,
         (unsigned long)directWrites,
         (unsigned long)directResends,
         (unsigned long)directWriteFailures,
         (unsigned long)directBusBusy,
         (unsigned long)directBusDrainedBytes,
         (unsigned long)directStops,
         (unsigned long)directStopFailures,
         (unsigned long)directProgressPasses,
         (unsigned long)directProgressFaults);
  printf("[DIRECT READ STATUS] attempts/retry/recovered/exhausted/budget/busy/drained=%lu/%lu/%lu/%lu/%lu/%lu/%lu max_attempts=%u legacy_window_ms=%u post_pickup_window_ms=%u steps5_cap=%u\r\n",
         (unsigned long)directReadAttempts, (unsigned long)directReadRetries,
         (unsigned long)directReadRecovered, (unsigned long)directReadExhausted,
         (unsigned long)directReadBudgetFaults, (unsigned long)directReadBusBusy,
         (unsigned long)directReadBusDrainedBytes,
         (unsigned)DIRECT_PROGRESS_READ_ATTEMPTS, (unsigned)DIRECT_PROGRESS_READ_BUDGET_MS,
         (unsigned)DELTA_POST_PICKUP_READ_BUDGET_MS,approachStep5Capable);
  printf("[DIRECT WARNING STATUS] slow/rx=%lu/%lu final_read_active=%u rx_deferred=%u; warnings are NOT PASS.\r\n",
         (unsigned long)directProgressWarnings, (unsigned long)directReadWarnings,
         directFinalReadActive, directReadDeferred);
}

static void ConfigureServoPins(void) {
  GPIO_InitTypeDef gpio = {0};
  /* Cada UART half-duplex usa salida open-drain. Los pull-ups externos de
     1 kOhm fijan 3.3 V en Source y 5 V en Drain a traves del BS170. */
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio.Pin = GPIO_PIN_9;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_10;
  gpio.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_0;
  gpio.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &gpio);

  for (uint8_t index = 0u; index < 3u; ++index) {
    HAL_HalfDuplex_EnableReceiver(servoPorts[index]);
    __HAL_UART_CLEAR_OREFLAG(servoPorts[index]);
  }
}

static void FillResponse(DeltaLinkFrame *response, uint8_t type,
                         const DeltaLinkFrame *request,
                         DeltaResultCode result) {
  memset(response, 0, sizeof(*response));
  response->type = type;
  response->session = request != NULL ? request->session : activeSession;
  response->sequence = request != NULL ? request->sequence : lastSequence;
  memcpy(response->raw, commandedRaw, sizeof(response->raw));
  response->status = deltaMakeStatus(CurrentFlags(), result);
  deltaPrepareFrame(response);
}

static void TransmitLinkFrame(const DeltaLinkFrame *frame) {
  HAL_UART_Transmit(linkPort, (uint8_t *)frame, sizeof(*frame), 100u);
}

static void SendResponse(uint8_t type, const DeltaLinkFrame *request,
                         DeltaResultCode result, uint8_t cache) {
  DeltaLinkFrame response;
  FillResponse(&response, type, request, result);
  TransmitLinkFrame(&response);
  if (cache) {
    lastResponse = response;
    lastResponseValid = 1u;
  }
}

static void SendAsyncStatus(DeltaResultCode result) {
  if (activeSession == 0u || linkStale) return;
  DeltaLinkFrame response;
  FillResponse(&response, DELTA_MSG_STATUS, NULL, result);
  TransmitLinkFrame(&response);
}

static void SendDirectRetryState(DeltaResultCode result) {
  DeltaLinkFrame response = {0};
  response.type = DELTA_MSG_DIRECT_RETRY_STATE;
  response.session = activeSession;
  response.sequence = localPointSequence; // original motion token, not an ACK
  response.moveTimeMs = result == DELTA_RESULT_OK ? 7u : 0u;
  if (result == DELTA_RESULT_OK) memcpy(response.raw, directRetryOrigin, sizeof(response.raw));
  response.status = deltaMakeStatus(CurrentFlags(), result);
  deltaPrepareFrame(&response);
  TransmitLinkFrame(&response);
}

/* The request is ACKed immediately; this separate result cannot consume an
   ACK for SETPOINT/GRIP lease renewals while physical reads are in progress.
   raw[] are fresh measured medians, moveTimeMs is their valid-channel mask. */
static void SendCheckpointResult(DeltaResultCode result) {
  if (!checkpointPending || !activeSession || linkStale) return;
  DeltaLinkFrame response = {0};
  response.type = DELTA_MSG_CHECKPOINT_RESULT;
  response.session = activeSession;
  response.sequence = checkpointSequence;
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (measuredKnown[i]) {
      response.raw[i] = (uint16_t)measuredRaw[i];
      response.moveTimeMs |= (uint16_t)(1u << i);
    }
  }
  response.status = deltaMakeStatus(CurrentFlags(), result);
  deltaPrepareFrame(&response);
  TransmitLinkFrame(&response);
  checkpointPending = 0u;
  if(result==DELTA_RESULT_OK && !checkpointObserve && checkpointBoxToleranceRaw==0u)
    centerPrecisionRequired=0u; // only the actual precise checkpoint releases the centre
  if ((result == DELTA_RESULT_OK || result == DELTA_RESULT_OBSERVED) && !directPointActive &&
      !directAbortActive && !directHoldPendingMask) {
    /* OBSERVED is not arrival. Keep STOP provenance while any joint is still
       outside15: END/DISARM/watchdogs must hold its real pose, never flush the
        rejected endpoint again at the legacy700ms duration. Centre uses the
        CP5 correction envelope12 here; it still requires a separate CP5. */
    directEndpointAwaitingRecovery=0u;
    if(result==DELTA_RESULT_OBSERVED && endpointRetryEligible && directFinalReadActive){
      endpointRetryWaitStartMs=HAL_GetTick();endpointRetryWaitKnown=1u;
    }
    if (result == DELTA_RESULT_OBSERVED && directStartedMask)
      for (uint8_t i=0u;i<3u;++i)
        if (AbsDiff((uint16_t)measuredRaw[i],commandedRaw[i]) > endpointRetryToleranceRaw)
          directEndpointAwaitingRecovery=1u;
    if (!directEndpointAwaitingRecovery) directStartedMask = 0u;
    else printf("[DIRECT ENDPOINT PENDING] mask=0x%02X; recovery requires new B preflight, STOP retains measured pose.\r\n",
      directStartedMask);
    directFinalReadActive = directReadDeferred = 0u;
    ClearDirectProbeState();
  }
  if ((result==DELTA_RESULT_OK || result==DELTA_RESULT_OBSERVED) && !localPointActive && !directPointActive)
    localPointKnown=0u;
  checkpointBoxToleranceRaw = 0u;
  checkpointObserve = 0u;
  checkpointPickupFine20 = 0u;
}

static void SendRecoveryResult(DeltaResultCode result) {
  if (!recoveryPending) return;
  DeltaLinkFrame response = {0};
  response.type = DELTA_MSG_FAULT_RECOVERY_RESULT;
  response.session = activeSession;
  response.sequence = recoverySequence;
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (measuredKnown[i]) {
      response.raw[i] = (uint16_t)measuredRaw[i];
      response.moveTimeMs |= (uint16_t)(1u << i);
    }
  }
  response.status = deltaMakeStatus(CurrentFlags(), result);
  deltaPrepareFrame(&response);
  if (activeSession && !linkStale) TransmitLinkFrame(&response);
  recoveryPending = 0u;
}

/* Read-only recovery snapshot: raw[] are retained RC commands, NOT DELTA RAW
   and NOT measured servo angles. No PWM or reference is changed here. */
static void SendGripperState(const DeltaLinkFrame *request) {
  DeltaLinkFrame response;
  FillResponse(&response, DELTA_MSG_GRIP_STATE, request, DELTA_RESULT_OK);
  for (unsigned i = 0u; i < 3u; ++i)
    response.raw[i] = (uint16_t)GripperManual_Pulse(i + 1u);
  response.moveTimeMs = (uint16_t)GripperManual_Channel(); // 0: PWM off; 1..4 active
  deltaPrepareFrame(&response);
  TransmitLinkFrame(&response);
  lastResponse = response; // Retries receive the same snapshot, without side effects.
  lastResponseValid = 1u;
}

/* Independent telemetry never changes command ACKs or DELTA raw destinations. */
static uint16_t verifiedRawSnapshot[3];
/* A physical PASS belongs to both the nominal destination and the exact HTS
   command that produced it. A stable bounded compensation may remain applied;
   it is not a reason to move back to nominal before the next DIRECT. */
static uint16_t verifiedNominalSnapshot[3];
static uint16_t verifiedCommandSnapshot[3];
static uint32_t verifiedSnapshotMs;
static uint16_t encoderTelemetrySequence;
static void (*encoderService)(void);

void StreamActuator_SetEncoderService(void (*service)(void)) {
  encoderService = service;
}

static void SendFeedbackSnapshot(void) {
  if (!activeSession || linkStale) return;
  DeltaLinkFrame f = {0};
  f.type = DELTA_MSG_FEEDBACK;
  f.session = activeSession;
  f.sequence = lastSequence;
  memcpy(f.raw, verifiedRawSnapshot, sizeof(f.raw));
  uint32_t age = HAL_GetTick() - verifiedSnapshotMs;
  f.moveTimeMs = age > 65535u ? 65535u : (uint16_t)age;
  f.status = verifiedSnapshotKnown ? 7u : 0u;
  deltaPrepareFrame(&f);
  TransmitLinkFrame(&f);
}

uint8_t StreamActuator_EncoderTelemetry(int32_t position, uint16_t speedTenths,
                                       uint16_t intervalMs) {
  if (!activeSession || linkStale || frameReady ||
      linkPort->gState != HAL_UART_STATE_READY || intervalMs == 0u) return 0u;
  DeltaLinkFrame f = {0};
  f.type = DELTA_MSG_ENCODER;
  f.session = activeSession;
  f.sequence = (uint16_t)(encoderTelemetrySequence + 1u);
  f.raw[0] = (uint16_t)(uint32_t)position;
  f.raw[1] = (uint16_t)((uint32_t)position >> 16);
  f.raw[2] = speedTenths;
  f.moveTimeMs = intervalMs;
  f.status = 1u; /* calibrated estimate, no encoder health certification */
  deltaPrepareFrame(&f);
  /* 22 bytes a 115200 requieren ~2 ms. Acotar sólo telemetria; no cambiar
     los plazos de ACK. Un error no inventa una muestra enviada/fresca. */
  if (HAL_UART_Transmit(linkPort, (uint8_t *)&f, sizeof(f), 5u) != HAL_OK)
    return 0u;
  encoderTelemetrySequence = f.sequence;
  return 1u;
}

static uint8_t SequenceIsNewer(uint16_t candidate, uint16_t reference) {
  return (int16_t)(candidate - reference) > 0;
}

static uint16_t AbsDiff(uint16_t a, uint16_t b) {
  return a > b ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint8_t DirectSnapshotMatchesCommands(void) {
  if (!verifiedSnapshotKnown ||
      memcmp(commandedRaw, verifiedNominalSnapshot, sizeof(commandedRaw)) != 0 ||
      memcmp(servoCommandRaw, verifiedCommandSnapshot, sizeof(servoCommandRaw)) != 0)
    return 0u;
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (commandedRaw[i] < ActiveRawMin() || commandedRaw[i] > ActiveRawMax() ||
        servoCommandRaw[i] < ActiveRawMin() || servoCommandRaw[i] > ActiveRawMax() ||
        verifiedRawSnapshot[i] < ActiveRawMin() || verifiedRawSnapshot[i] > ActiveRawMax() ||
        AbsDiff(servoCommandRaw[i], commandedRaw[i]) > AUTO_CORRECTION_MAX_ERROR_RAW)
      return 0u;
  }
  return 1u;
}

static int16_t AbsI16(int16_t value) {
  return value < 0 ? (int16_t)-value : value;
}

static uint8_t HomeRecoveryRawInRange(int16_t raw) {
  return raw >= (int16_t)HOME_RECOVERY_RAW_MIN &&
         raw <= (int16_t)HOME_RECOVERY_RAW_MAX;
}

static void SortI16(int16_t *values, uint8_t count) {
  for (uint8_t i = 1u; i < count; ++i) {
    const int16_t value = values[i];
    uint8_t j = i;
    while (j > 0u && values[j - 1u] > value) {
      values[j] = values[j - 1u];
      --j;
    }
    values[j] = value;
  }
}

/* Los extremos del intervalo valen para un destino concreto. Al cambiar el
   destino hay que olvidarlos o se bisectaria entre ordenes de objetivos
   distintos. */
static void ResetCompensation(void) {
  memset(belowKnown, 0, sizeof(belowKnown));
  memset(aboveKnown, 0, sizeof(aboveKnown));
  memset(belowCommand, 0, sizeof(belowCommand));
  memset(aboveCommand, 0, sizeof(aboveCommand));
}

static void ClearCheckpointCorrection(void) {
  checkpointPickupFine20 = 0u;
  checkpointObserve = 0u;
  streamEndToleranceRaw = 0u;
  checkpointRecover30 = streamEndRecover30 = 0u;
  checkpointBoxToleranceRaw = 0u;
  checkpointCorrectionRounds = checkpointInvalidMask = 0u;
  checkpointProgressReasserts = 0u;
  checkpointCompensationHeld = checkpointTargetConverging = 0u;
}

static void CancelVerification(void) {
  verificationPending = 0u;
  verificationSampling = 0u;
  verificationPurpose = VERIFY_NONE;
  feedbackSampleIndex = 0u;
  feedbackChannel = 0u;
  verificationReadChannelMask = 0x07u;
}

static void FailRecovery(void) {
  /* Retain the original position fault. A failed passive read is NOT a
     permission, nor an automatic retry. A new double-X may start fresh reads;
     communication/TX faults were never eligible in the first place. */
  StopGripperOutputs();
  CancelVerification();
  busy = armed = streamActive = homeValid = homePending = rawKnown = 0u;
  faultLatched = 1u;
  verifiedSnapshotKnown = 0u;
  printf("[FAULT RECOVERY FAILED] bloqueo original conservado; nueva confirmacion solo repite lecturas, no mueve.\r\n");
  SendRecoveryResult(DELTA_RESULT_FEEDBACK_FAILED);
}

static const char *VerificationPurposeName(VerificationPurpose purpose) {
  if (purpose == VERIFY_HOME_CAPTURE) return "HOME_CAPTURE";
  if (purpose == VERIFY_HOME_SEGMENT) return "HOME_SEGMENT";
  if (purpose == VERIFY_HOME) return "HOME_FINAL";
  if (purpose == VERIFY_STREAM_END) return "STREAM_END";
  if (purpose == VERIFY_STREAM_CHECKPOINT) return "STREAM_CHECKPOINT";
  if (purpose == VERIFY_FAULT_RECOVERY) return "FAULT_RECOVERY_READ_ONLY";
  return "NONE";
}

static uint8_t IsHomeVerification(VerificationPurpose purpose) {
  return purpose == VERIFY_HOME_CAPTURE || purpose == VERIFY_HOME_SEGMENT ||
         purpose == VERIFY_HOME;
}

static void ScheduleVerification(VerificationPurpose purpose,
                                 uint32_t delayMs) {
  CancelVerification();
  memset(feedbackSamples, 0, sizeof(feedbackSamples));
  memset(feedbackValidCount, 0, sizeof(feedbackValidCount));
  memset(measuredKnown, 0, sizeof(measuredKnown));
  verificationPurpose = purpose;
  verificationPending = 1u;
  verificationStartMs = HAL_GetTick() + delayMs;
  busy = 1u;
  printf("[VERIFY SCHEDULED] purpose=%s delay_ms=%lu desired=%u,%u,%u\r\n",
         VerificationPurposeName(purpose),
         (unsigned long)delayMs, commandedRaw[0], commandedRaw[1],
           commandedRaw[2]);
}

/* Un reintento de lectura no representa un movimiento nuevo. Conservar las
   muestras validas evita que una perdida aislada del ID1 en una ronda y del
   ID3 en la siguiente se convierta artificialmente en NO_RESPONSE. Los
   canales inestables se limpian para exigir un lote nuevo coherente. */
static void ScheduleVerificationReadRetry(VerificationPurpose purpose,
                                          uint32_t delayMs,
                                          uint8_t insufficientMask,
                                          uint8_t unstableMask) {
  CancelVerification();
  if (purpose == VERIFY_STREAM_CHECKPOINT)
    verificationReadChannelMask = (uint8_t)(insufficientMask | unstableMask);
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    if ((unstableMask & (uint8_t)(1u << channel)) != 0u) {
      memset(feedbackSamples[channel], 0, sizeof(feedbackSamples[channel]));
      feedbackValidCount[channel] = 0u;
    }
  }
  memset(measuredKnown, 0, sizeof(measuredKnown));
  verificationPurpose = purpose;
  verificationPending = 1u;
  verificationStartMs = HAL_GetTick() + delayMs;
  busy = 1u;
  printf("[VERIFY RETRY SCHEDULED] purpose=%s delay_ms=%lu accumulated=%u,%u,%u read_mask=0x%02X\r\n",
         VerificationPurposeName(purpose), (unsigned long)delayMs,
         feedbackValidCount[0], feedbackValidCount[1], feedbackValidCount[2],
         verificationReadChannelMask);
}

static void LatchVerificationFault(const char *reason, uint8_t failedMask) {
  faultRecoverable = strcmp(reason, "OUT_OF_TOLERANCE") == 0;
  verifiedSnapshotKnown = 0u;
  StopGripperOutputs();
  ++verificationFaults;
  CancelVerification();
  busy = 0u;
  homePending = 0u;
  homeValid = 0u;
  armed = 0u;
  streamActive = 0u;
  activeStreamOption = 0u;
  rawKnown = 0u;
  faultLatched = 1u;
  SendRecoveryResult(DELTA_RESULT_FEEDBACK_FAILED);
  SendCheckpointResult(DELTA_RESULT_FEEDBACK_FAILED);
  checkpointCorrectionRounds = checkpointInvalidMask = 0u;
  checkpointCompensationHeld = checkpointTargetConverging = 0u;
  printf("[VERIFY FAULT] reason=%s failed_mask=0x%02X recoverable=%u; recuperacion supervisada o reinicio.\r\n",
         reason, failedMask, faultRecoverable);
  SendAsyncStatus(DELTA_RESULT_FEEDBACK_FAILED);
}

static void StartNextHomeSegment(void) {
  uint16_t target[3];
  int16_t maxDistance = 0;

  CancelVerification();
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    if (!measuredKnown[channel] ||
        !HomeRecoveryRawInRange(measuredRaw[channel])) {
      LatchVerificationFault("HOME_CAPTURE_OUT_OF_RANGE",
                             (uint8_t)(1u << channel));
      return;
    }
    homeSegmentStartRaw[channel] = measuredRaw[channel];
    const int16_t distance = AbsI16((int16_t)RAW_HOME - measuredRaw[channel]);
    if (distance > maxDistance) maxDistance = distance;
  }

  if (maxDistance <= HOME_SEGMENT_TOLERANCE_RAW) {
    const uint16_t finalHomeRaw[3] = {RAW_HOME, RAW_HOME, RAW_HOME};
    homeCorrectionRounds = 0u;
    if (!ExecuteTargets(finalHomeRaw, HOME_FINAL_TIME_MS)) {
      LatchVerificationFault("HOME_FINAL_TX_FAILED", 0x07u);
      return;
    }
    memcpy(commandedRaw, finalHomeRaw, sizeof(commandedRaw));
    ResetCompensation();
    rawKnown = 1u;
    printf("[HOME FINAL] entrada segura measured=%d,%d,%d; target=%u,%u,%u time_ms=%u.\r\n",
           measuredRaw[0], measuredRaw[1], measuredRaw[2], finalHomeRaw[0],
           finalHomeRaw[1], finalHomeRaw[2], (unsigned)HOME_FINAL_TIME_MS);
    ScheduleVerification(VERIFY_HOME,
                         HOME_FINAL_TIME_MS + HOME_FINAL_RETRY_SETTLE_MS);
    return;
  }

  if (homeSegmentCount >= HOME_SEGMENT_MAX_COUNT) {
    LatchVerificationFault("HOME_SEGMENT_LIMIT", 0x07u);
    return;
  }
  const uint8_t nextStep = (uint8_t)(homeSegmentCount + 1u);
  const uint8_t estimatedRemaining = (uint8_t)(
    (maxDistance + HOME_SEGMENT_MAX_STEP_RAW - 1) /
    HOME_SEGMENT_MAX_STEP_RAW);
  homeSegmentTotal = (uint8_t)(homeSegmentCount + estimatedRemaining);
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    const int16_t remaining = (int16_t)RAW_HOME - measuredRaw[channel];
    const int16_t distance = AbsI16(remaining);
    int16_t stepMagnitude = (int16_t)(
      ((int32_t)distance * HOME_SEGMENT_MAX_STEP_RAW + maxDistance - 1) /
      maxDistance);
    if (stepMagnitude > distance) stepMagnitude = distance;
    target[channel] = (uint16_t)(measuredRaw[channel] +
      (remaining < 0 ? -stepMagnitude : stepMagnitude));
  }
  homeSegmentRetryCount = 0u;
  verificationReadRetryAttempted = 0u;
  homeSegmentCount = nextStep;
  if (!ExecuteTargets(target, HOME_SEGMENT_TIME_MS)) {
    LatchVerificationFault("HOME_SEGMENT_TX_FAILED", 0x07u);
    return;
  }
  memcpy(commandedRaw, target, sizeof(commandedRaw));
  ResetCompensation();
  rawKnown = 1u;
  printf("[HOME SEGMENT] step=%u/~%u start=%d,%d,%d target=%u,%u,%u measured_step_max=%d time_ms=%u.\r\n",
         homeSegmentCount, homeSegmentTotal, homeSegmentStartRaw[0], homeSegmentStartRaw[1],
         homeSegmentStartRaw[2], target[0], target[1], target[2],
         HOME_SEGMENT_MAX_STEP_RAW, (unsigned)HOME_SEGMENT_TIME_MS);
  ScheduleVerification(VERIFY_HOME_SEGMENT,
                       HOME_SEGMENT_TIME_MS + HOME_SEGMENT_SETTLE_MS);
  SendAsyncStatus(DELTA_RESULT_OK);
}

static void StartHomePlanFromCapture(void) {
  int16_t maxDistance = 0;
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    if (!measuredKnown[channel] ||
        !HomeRecoveryRawInRange(measuredRaw[channel])) {
      LatchVerificationFault("HOME_CAPTURE_OUT_OF_RANGE",
                             (uint8_t)(1u << channel));
      return;
    }
    homePathOriginRaw[channel] = measuredRaw[channel];
    const int16_t distance = AbsI16((int16_t)RAW_HOME - measuredRaw[channel]);
    if (distance > maxDistance) maxDistance = distance;
  }
  homeSegmentCount = 0u;
  homeSegmentTotal = maxDistance <= HOME_SEGMENT_TOLERANCE_RAW
    ? 0u : (uint8_t)((maxDistance + HOME_SEGMENT_MAX_STEP_RAW - 1) /
                     HOME_SEGMENT_MAX_STEP_RAW);
  printf("[HOME PLAN] origin=%d,%d,%d final=%u,%u,%u estimated_segments=%u measured_step_max=%d.\r\n",
         homePathOriginRaw[0], homePathOriginRaw[1], homePathOriginRaw[2],
         (unsigned)RAW_HOME, (unsigned)RAW_HOME, (unsigned)RAW_HOME,
         homeSegmentTotal, HOME_SEGMENT_MAX_STEP_RAW);
  StartNextHomeSegment();
}

/* Planificador de compensacion por canal. Portado de 03C: OFFSET_COMP desplaza
   la orden segun el error medido, BRACKET_MID bisecta cuando ya se conocen los
   dos lados del objetivo y RESEND_DESIRED evita sobrecorregir una lectura
   dudosa. Los canales fuera de failedMask conservan su ultima orden.
   Devuelve 1 si algun canal propone una orden distinta de la ya enviada. */
static uint8_t PlanCompensatedTargets(uint16_t plan[3], uint8_t failedMask) {
  uint8_t anyChange = 0u;

  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    const uint16_t desired = commandedRaw[channel];
    const uint16_t previous = servoCommandRaw[channel];
    const uint8_t bit = (uint8_t)(1u << channel);
    const char *reason;
    int16_t proposed;

    plan[channel] = previous;
    if (!measuredKnown[channel]) continue;

    const int16_t error =
        (int16_t)(measuredRaw[channel] - (int16_t)desired);
    if (error < 0) {
      belowCommand[channel] = previous;
      belowKnown[channel] = 1u;
    } else if (error > 0) {
      aboveCommand[channel] = previous;
      aboveKnown[channel] = 1u;
    }

    if ((failedMask & bit) == 0u) {
      proposed = (int16_t)previous;
      reason = "HOLD";
    } else if (belowKnown[channel] && aboveKnown[channel]) {
      proposed = (int16_t)(((uint32_t)belowCommand[channel] +
                            (uint32_t)aboveCommand[channel]) / 2u);
      reason = "BRACKET_MID";
    } else if (AbsI16(error) > (AutomaticCheckpoint()
                               ? CheckpointCorrectibleError() :
                               (AutomaticStop() ? StopCorrectibleError() : FEEDBACK_LARGE_ERROR_RAW))) {
      proposed = (int16_t)desired;
      reason = "RESEND_DESIRED";
    } else {
      int16_t correction = error;
      if (correction > FEEDBACK_MAX_CORRECTION_STEP_RAW) {
        correction = FEEDBACK_MAX_CORRECTION_STEP_RAW;
      } else if (correction < -FEEDBACK_MAX_CORRECTION_STEP_RAW) {
        correction = -FEEDBACK_MAX_CORRECTION_STEP_RAW;
      }
      proposed = (int16_t)((int16_t)previous - correction);
      reason = "OFFSET_COMP";
    }

    const int16_t minimum =
        (int16_t)desired - CompensationLimit();
    const int16_t maximum =
        (int16_t)desired + CompensationLimit();
    if (AutomaticStop()) {
      if (proposed > (int16_t)previous + FEEDBACK_MAX_CORRECTION_STEP_RAW)
        proposed = (int16_t)previous + FEEDBACK_MAX_CORRECTION_STEP_RAW;
      if (proposed < (int16_t)previous - FEEDBACK_MAX_CORRECTION_STEP_RAW)
        proposed = (int16_t)previous - FEEDBACK_MAX_CORRECTION_STEP_RAW;
    }
    if (proposed < minimum) proposed = minimum;
    if (proposed > maximum) proposed = maximum;
    if (proposed < (int16_t)ActiveRawMin()) proposed = (int16_t)ActiveRawMin();
    if (proposed > (int16_t)ActiveRawMax()) proposed = (int16_t)ActiveRawMax();

    plan[channel] = (uint16_t)proposed;
    if (plan[channel] != previous) anyChange = 1u;

    printf("[FEEDBACK PLAN] id=%u desired=%u measured=%d error=%d last_cmd=%u next_cmd=%u reason=%s compensation=%d limit=%d\r\n",
           servoIds[channel], desired, measuredRaw[channel], error, previous,
           plan[channel], reason, (int16_t)(plan[channel]-desired), CompensationLimit());
  }
  return anyChange;
}

static uint8_t AttemptHomeSegmentRetry(uint8_t failedMask) {
  if (verificationPurpose != VERIFY_HOME_SEGMENT || failedMask == 0u ||
      homeSegmentRetryCount >= HOME_SEGMENT_MAX_RETRIES || dryRunMode) return 0u;

  verificationReadRetryAttempted = 0u;
  uint16_t plan[3];
  memcpy(plan, servoCommandRaw, sizeof(plan));
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    const uint8_t bit = (uint8_t)(1u << channel);
    if ((failedMask & bit) == 0u || !measuredKnown[channel]) continue;

    const int16_t desired = (int16_t)commandedRaw[channel];
    const int16_t error = (int16_t)(measuredRaw[channel] - desired);
    int16_t correction = error;
    if (correction > FEEDBACK_MAX_CORRECTION_STEP_RAW) {
      correction = FEEDBACK_MAX_CORRECTION_STEP_RAW;
    } else if (correction < -FEEDBACK_MAX_CORRECTION_STEP_RAW) {
      correction = -FEEDBACK_MAX_CORRECTION_STEP_RAW;
    }

    int16_t proposed = (int16_t)servoCommandRaw[channel] - correction;
    const int16_t compensationMin =
      (int16_t)(desired - FEEDBACK_MAX_TOTAL_COMPENSATION_RAW);
    const int16_t compensationMax =
      (int16_t)(desired + FEEDBACK_MAX_TOTAL_COMPENSATION_RAW);
    const int16_t measuredMin =
      (int16_t)(measuredRaw[channel] - HOME_SEGMENT_MAX_STEP_RAW);
    const int16_t measuredMax =
      (int16_t)(measuredRaw[channel] + HOME_SEGMENT_MAX_STEP_RAW);
    if (proposed < compensationMin) proposed = compensationMin;
    if (proposed > compensationMax) proposed = compensationMax;
    if (proposed < measuredMin) proposed = measuredMin;
    if (proposed > measuredMax) proposed = measuredMax;
    if (proposed < (int16_t)HOME_RECOVERY_RAW_MIN) {
      proposed = (int16_t)HOME_RECOVERY_RAW_MIN;
    }
    if (proposed > (int16_t)HOME_RECOVERY_RAW_MAX) {
      proposed = (int16_t)HOME_RECOVERY_RAW_MAX;
    }
    plan[channel] = (uint16_t)proposed;
    printf("[HOME RETRY PLAN] id=%u desired=%d measured=%d error=%d last_cmd=%u next_cmd=%u\r\n",
           servoIds[channel], desired, measuredRaw[channel], error,
           servoCommandRaw[channel], plan[channel]);
  }
  if (!ExecuteTargets(plan, HOME_SEGMENT_TIME_MS)) {
    LatchVerificationFault("HOME_SEGMENT_RETRY_TX_FAILED", failedMask);
    return 1u;
  }
  ++homeSegmentRetryCount;
  ++homeSegmentRetries;
  printf("[HOME SEGMENT RETRY] step=%u failed_mask=0x%02X desired=%u,%u,%u command=%u,%u,%u attempt=%u/%u.\r\n",
         homeSegmentCount, failedMask, commandedRaw[0], commandedRaw[1],
         commandedRaw[2], plan[0], plan[1], plan[2],
         homeSegmentRetryCount, (unsigned)HOME_SEGMENT_MAX_RETRIES);
  ScheduleVerification(VERIFY_HOME_SEGMENT,
                       HOME_SEGMENT_TIME_MS + HOME_SEGMENT_SETTLE_MS);
  return 1u;
}

static uint8_t AttemptHomeFinalCorrection(uint8_t failedMask) {
  if (verificationPurpose != VERIFY_HOME || failedMask == 0u ||
      homeCorrectionRounds >= FEEDBACK_MAX_CORRECTIONS || dryRunMode) return 0u;

  uint16_t plan[3];
  if (!PlanCompensatedTargets(plan, failedMask)) {
    printf("[HOME CORRECTION EXHAUSTED] failed_mask=0x%02X; el punto medio ya no produce una orden nueva.\r\n",
           failedMask);
    return 0u;
  }

  ++homeCorrectionRounds;
  verificationReadRetryAttempted = 0u;
  if (!ExecuteTargets(plan, HOME_CORRECTION_TIME_MS)) {
    LatchVerificationFault("HOME_FINAL_CORRECTION_TX_FAILED", failedMask);
    return 1u;
  }
  ++feedbackCorrections;
  printf("[HOME CORRECTION] round=%u/%u failed_mask=0x%02X desired=%u,%u,%u command=%u,%u,%u max_comp=%d\r\n",
         homeCorrectionRounds, (unsigned)FEEDBACK_MAX_CORRECTIONS, failedMask,
         commandedRaw[0], commandedRaw[1], commandedRaw[2],
         plan[0], plan[1], plan[2], FEEDBACK_MAX_TOTAL_COMPENSATION_RAW);
  ScheduleVerification(VERIFY_HOME,
                       HOME_CORRECTION_TIME_MS + HOME_CORRECTION_SETTLE_MS);
  return 1u;
}

static void CompleteVerification(void) {
  const VerificationPurpose completedPurpose = verificationPurpose;
  const uint8_t observed = completedPurpose == VERIFY_STREAM_CHECKPOINT && checkpointObserve;

  /* The final bounded feedback read can cross the deadline after Loop's
     pre-read watchdog check. Never publish a late PASS or retain RC PWM. */
  if (completedPurpose == VERIFY_STREAM_CHECKPOINT && checkpointPending &&
      HAL_GetTick() - checkpointStartedMs > CheckpointTimeout()) {
    LatchVerificationFault("CHECKPOINT_TIMEOUT", 0x07u);
    return;
  }
  if (completedPurpose == VERIFY_STREAM_CHECKPOINT &&
      !CheckpointCorrectionGuards()) return;

  if (completedPurpose == VERIFY_HOME_CAPTURE) {
    StartHomePlanFromCapture();
    return;
  }
  if (completedPurpose == VERIFY_HOME_SEGMENT) {
    StartNextHomeSegment();
    return;
  }

  if (!observed) ++verificationPassed;
  CancelVerification();
  busy = 0u;
  homePending = 0u;
  if (completedPurpose == VERIFY_HOME && !linkStale) homeValid = 1u;
  /* A physically verified HOME may retain a compensated HTS command. Publish
     that stable offset before ARM/BEGIN, just like checkpoint compensation.
     Do not infer compensation from an unverified/coalesced nominal mismatch. */
  checkpointOutputOffsetKnown = !dryRunMode && rawKnown &&
    memcmp(servoCommandRaw, commandedRaw, sizeof(commandedRaw)) != 0;
  printf(observed ? "[AUTO OBSERVED] precision=NOT_VERIFIED purpose=%s desired=%u,%u,%u measured=%d,%d,%d home=%u\r\n" :
         "[VERIFY PASS] purpose=%s desired=%u,%u,%u measured=%d,%d,%d home=%u\r\n",
         VerificationPurposeName(completedPurpose),
         commandedRaw[0], commandedRaw[1], commandedRaw[2], measuredRaw[0],
         measuredRaw[1], measuredRaw[2], homeValid);
  if (completedPurpose == VERIFY_STREAM_END) activeStreamOption = 0u;
  memcpy(verifiedRawSnapshot, measuredRaw, sizeof(verifiedRawSnapshot));
  memcpy(verifiedNominalSnapshot, commandedRaw, sizeof(verifiedNominalSnapshot));
  memcpy(verifiedCommandSnapshot, servoCommandRaw, sizeof(verifiedCommandSnapshot));
  verifiedSnapshotMs = HAL_GetTick();
  verifiedSnapshotKnown = !dryRunMode;
  if (completedPurpose == VERIFY_STREAM_CHECKPOINT) {
    checkpointReassertNeeded = 0u; // already measured after any bounded bridge
    SendCheckpointResult(observed ? DELTA_RESULT_OBSERVED : DELTA_RESULT_OK);
    return; // Keep ARM, STREAM and RC PWM; no implicit STOP or next target.
  }
  SendAsyncStatus(DELTA_RESULT_OK);
  SendFeedbackSnapshot();
}

static uint8_t AttemptMeasuredCorrection(void) {
  if (verificationPurpose != VERIFY_STREAM_END ||
      (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) !=
        DELTA_STREAM_OPTION_Z_ONLY ||
      streamCorrectionAttempted || dryRunMode) return 0u;

  int16_t measuredMin = measuredRaw[0];
  int16_t measuredMax = measuredRaw[0];
  for (uint8_t channel = 1u; channel < 3u; ++channel) {
    if (measuredRaw[channel] < measuredMin) measuredMin = measuredRaw[channel];
    if (measuredRaw[channel] > measuredMax) measuredMax = measuredRaw[channel];
  }
  if ((uint16_t)(measuredMax - measuredMin) >
      CORRECTION_MAX_MEASURED_SPREAD_RAW) {
    printf("[VERIFY CORRECTION REFUSED] reason=PLATFORM_NOT_LEVEL spread=%d max=%u.\r\n",
           measuredMax - measuredMin,
           (unsigned)CORRECTION_MAX_MEASURED_SPREAD_RAW);
    return 0u;
  }

  uint16_t correctionRaw[3];
  const uint16_t correctionFloor = ActiveRawMin() > CORRECTION_RAW_MIN
    ? ActiveRawMin() : CORRECTION_RAW_MIN;
  uint8_t anyCorrection = 0u;
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    const int16_t error = (int16_t)(measuredRaw[channel] -
                                    (int16_t)commandedRaw[channel]);
    correctionRaw[channel] = commandedRaw[channel];
    if (error < -FEEDBACK_TOLERANCE_RAW) {
      printf("[VERIFY CORRECTION REFUSED] reason=UNSAFE_DIRECTION id=%u error=%d.\r\n",
             servoIds[channel], error);
      return 0u;
    }
    if (error > FEEDBACK_TOLERANCE_RAW) {
      // Por debajo del piso ya validado, un clamp a 430 invertia la
      // correccion (p.ej. destino 420 -> 430). Rechazarla sin transmitir.
      if (commandedRaw[channel] <= correctionFloor) {
        printf("[VERIFY CORRECTION REFUSED] reason=CORRECTION_FLOOR id=%u desired=%u floor=%u.\r\n",
               servoIds[channel], commandedRaw[channel], correctionFloor);
        return 0u;
      }
      uint16_t overshoot = (uint16_t)error;
      if (overshoot > CORRECTION_MAX_OVERSHOOT_RAW) {
        overshoot = CORRECTION_MAX_OVERSHOOT_RAW;
      }
      correctionRaw[channel] = commandedRaw[channel] > overshoot
        ? (uint16_t)(commandedRaw[channel] - overshoot)
        : correctionFloor;
      if (correctionRaw[channel] < correctionFloor) {
        correctionRaw[channel] = correctionFloor;
      }
      anyCorrection = 1u;
    }
    printf("[VERIFY CORRECTION PLAN] id=%u desired=%u measured=%d error=%d correction_target=%u\r\n",
           servoIds[channel], commandedRaw[channel], measuredRaw[channel],
           error, correctionRaw[channel]);
  }
  if (!anyCorrection) {
    printf("[VERIFY CORRECTION REFUSED] reason=NO_SAFE_ADJUSTMENT.\r\n");
    return 0u;
  }

  streamCorrectionAttempted = 1u;
  if (!ExecuteTargets(correctionRaw, MEASURED_CORRECTION_TIME_MS)) {
    LatchVerificationFault("CORRECTION_TX_FAILED", 0x07u);
    return 1u;
  }
  ++feedbackCorrections;
  printf("[VERIFY CORRECTION] attempt=1/1 target=%u,%u,%u reference=%u,%u,%u time_ms=%u raw_min=%u\r\n",
         correctionRaw[0], correctionRaw[1], correctionRaw[2],
         commandedRaw[0], commandedRaw[1], commandedRaw[2],
         (unsigned)MEASURED_CORRECTION_TIME_MS,
         (unsigned)correctionFloor);
  ScheduleVerification(VERIFY_STREAM_END,
                       MEASURED_CORRECTION_TIME_MS +
                       MEASURED_CORRECTION_SETTLE_MS);
  return 1u;
}

static uint8_t ExtendedZFeedback(void) {
  if ((activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) != DELTA_STREAM_OPTION_Z_ONLY ||
      (activeStreamOption & DELTA_STREAM_OPTION_RANGE_MASK) != DELTA_STREAM_OPTION_RANGE_R3)
    return 0u;
  for (uint8_t i=0;i<3u;++i)
    if (commandedRaw[i] < CORRECTION_RAW_MIN) return 1u;
  return 0u;
}

static uint8_t AttemptXyzCorrection(uint8_t failedMask) {
  if (verificationPurpose != VERIFY_STREAM_END ||
      ((activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) !=
        DELTA_STREAM_OPTION_XYZ_MANUAL && !ExtendedZFeedback()) ||
      failedMask == 0u ||
      xyzCorrectionRounds >= FEEDBACK_MAX_CORRECTIONS || dryRunMode) return 0u;

  if (AutomaticStop() && (linkStale || !activeSession ||
      HAL_GetTick() - lastValidFrameMs > LINK_TIMEOUT_MS ||
      HAL_GetTick() - automaticStopStartedMs > DELTA_AUTO_CHECKPOINT_TIMEOUT_MS)) return 0u;
  /* Un error mayor que este no es banda muerta: es un servo trabado o una
     lectura dudosa. Compensar ahi seria sobrecorregir a ciegas. */
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    const int16_t error = (int16_t)(measuredRaw[channel] -
                                    (int16_t)commandedRaw[channel]);
    if (AbsI16(error) > StopCorrectibleError()) {
      printf("[VERIFY XYZ CORRECTION REFUSED] id=%u desired=%u measured=%d error=%d max=%d.\r\n",
             servoIds[channel], commandedRaw[channel], measuredRaw[channel],
             error, StopCorrectibleError());
      return 0u;
    }
    if (AutomaticStop() && (failedMask & (1u << channel)) && xyzCorrectionRounds &&
        AbsI16(error) >= stopPreviousError[channel]) {
      printf("[VERIFY AUTO STOP NO PROGRESS] id=%u error=%d previous=%d\r\n",
             servoIds[channel], error, stopPreviousError[channel]);
      return 0u;
    }
    stopPreviousError[channel] = AbsI16(error);
  }

  uint16_t plan[3];
  if (!PlanCompensatedTargets(plan, failedMask)) {
    printf("[VERIFY XYZ CORRECTION EXHAUSTED] failed_mask=0x%02X; sin orden nueva que probar.\r\n",
           failedMask);
    return 0u;
  }

  ++xyzCorrectionRounds;
  if (ExtendedZFeedback()) streamCorrectionAttempted=1u;
  verificationReadRetryAttempted = 0u;
  if (!ExecuteTargets(plan, XYZ_CORRECTION_TIME_MS)) {
    LatchVerificationFault("XYZ_CORRECTION_TX_FAILED", 0x07u);
    return 1u;
  }
  ++feedbackCorrections;
  printf("[VERIFY JOINT CORRECTION] round=%u/%u desired=%u,%u,%u command=%u,%u,%u time_ms=%u settle_ms=%u\r\n",
         xyzCorrectionRounds, (unsigned)FEEDBACK_MAX_CORRECTIONS,
         commandedRaw[0], commandedRaw[1], commandedRaw[2],
         plan[0], plan[1], plan[2],
         (unsigned)XYZ_CORRECTION_TIME_MS,
         (unsigned)XYZ_CORRECTION_SETTLE_MS);
  ScheduleVerification(VERIFY_STREAM_END,
                       XYZ_CORRECTION_TIME_MS + XYZ_CORRECTION_SETTLE_MS);
  return 1u;
}

/* No renueva setpoints, PWM, sesion ni plazo. SysTick vigila el PWM incluso
   durante TX; las otras leases se revisan entre cada puerto y al terminar. */
static uint8_t CheckpointCorrectionGuards(void) {
  const uint32_t now = HAL_GetTick();
  if (!checkpointPending || !streamActive || !armed || !homeValid ||
      faultLatched || linkStale || dryRunMode) return 0u;
  if (now - checkpointStartedMs > CheckpointTimeout()) {
    LatchVerificationFault("CHECKPOINT_TIMEOUT", 0x07u);
    return 0u;
  }
  if (lastStreamSetpointMs == 0u ||
      now - lastStreamSetpointMs > STREAM_TIMEOUT_MS) {
    LatchVerificationFault("CHECKPOINT_STREAM_WATCHDOG", 0x07u);
    return 0u;
  }
  if (activeSession == 0u || now - lastValidFrameMs > LINK_TIMEOUT_MS) {
    LatchVerificationFault("CHECKPOINT_LINK_WATCHDOG", 0x07u);
    return 0u;
  }
  if (manualGripRuntime && !GripperManual_Active()) {
    LatchVerificationFault("PALA_WATCHDOG", 0x07u);
    return 0u;
  }
  for (uint8_t i = 0u; i < 3u; ++i) {
    if ((uint16_t)GripperManual_Pulse(i + 1u) != checkpointGripUs[i]) {
      LatchVerificationFault("CHECKPOINT_GRIP_COMMAND", 0x07u);
      return 0u;
    }
  }
  return 1u;
}

static void ReassertUnchangedAutoCorrection(uint8_t failedMask) {
  const uint32_t elapsed = HAL_GetTick() - checkpointStartedMs;
  if (elapsed > CheckpointTimeout() ||
      CheckpointTimeout() - elapsed < AUTO_PROGRESS_REASSERT_BUDGET_MS) {
    LatchVerificationFault("AUTO_PROGRESS_REASSERT_BUDGET", failedMask);
    return;
  }
  uint16_t sameTarget[3];
  memcpy(sameTarget, servoCommandRaw, sizeof(sameTarget));
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (sameTarget[i] < ActiveRawMin() || sameTarget[i] > ActiveRawMax() ||
        AbsDiff(sameTarget[i], commandedRaw[i]) > (uint16_t)CompensationLimit()) {
      LatchVerificationFault("AUTO_PROGRESS_REASSERT_GUARD", (uint8_t)(1u << i));
      return;
    }
  }
  ++checkpointProgressReasserts;
  verificationReadRetryAttempted = 0u;
  if (!ExecuteTargets(sameTarget, XYZ_CORRECTION_TIME_MS)) {
    if (!faultLatched) LatchVerificationFault("AUTO_PROGRESS_REASSERT_TX", 0x07u);
    return;
  }
  printf("[AUTO PROGRESS REASSERT] attempt=%u/%u mask=0x%02X desired=%u,%u,%u command=%u,%u,%u move_ms=%u settle_ms=%u; same command, fresh full reads.\r\n",
         checkpointProgressReasserts, (unsigned)AUTO_PROGRESS_REASSERT_MAX, failedMask,
         commandedRaw[0], commandedRaw[1], commandedRaw[2],
         sameTarget[0], sameTarget[1], sameTarget[2],
         (unsigned)XYZ_CORRECTION_TIME_MS, (unsigned)AUTO_PROGRESS_REASSERT_SETTLE_MS);
  ScheduleVerification(VERIFY_STREAM_CHECKPOINT,
                       XYZ_CORRECTION_TIME_MS + AUTO_PROGRESS_REASSERT_SETTLE_MS);
}

static uint8_t AttemptCheckpointCorrection(uint8_t failedMask) {
  const uint8_t automatic = AutomaticCheckpoint();
  const int16_t maxError = CheckpointCorrectibleError();
  const uint8_t maxRounds = automatic ? (checkpointPickupFine20 ?
    DELTA_PICKUP_FINE_MAX_CORRECTIONS : AUTO_CHECKPOINT_MAX_CORRECTIONS) : CHECKPOINT_MAX_CORRECTIONS;
  if (verificationPurpose != VERIFY_STREAM_CHECKPOINT || !checkpointPending ||
      (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) !=
        DELTA_STREAM_OPTION_XYZ_MANUAL || failedMask == 0u || dryRunMode) return 0u;
  if (!CheckpointCorrectionGuards()) return 1u;
  uint8_t unchangedMask = 0u, worseMask = 0u;
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    const int16_t error = (int16_t)(measuredRaw[channel] -
                                    (int16_t)commandedRaw[channel]);
    if (!measuredKnown[channel] ||
        feedbackValidCount[channel] < FEEDBACK_REQUIRED_SAMPLES ||
        AbsI16(error) > maxError) {
      printf("[VERIFY CHECKPOINT CORRECTION REFUSED] id=%u error=%d max=%d known=%u.\r\n",
             servoIds[channel], error, maxError,
             measuredKnown[channel]);
      return 0u;
    }
    if (automatic && checkpointCorrectionRounds &&
        AbsI16(error) > CheckpointPositionTolerance() &&
        AbsI16(error) >= checkpointPreviousError[channel]) {
      if (AbsI16(error) > checkpointPreviousError[channel] ||
          error != checkpointPreviousSignedError[channel])
        worseMask |= (uint8_t)(1u << channel);
      else unchangedMask |= (uint8_t)(1u << channel);
    }
  }
  if (worseMask != 0u || (unchangedMask != 0u &&
      checkpointProgressReasserts >= AUTO_PROGRESS_REASSERT_MAX)) {
    LatchVerificationFault("AUTO_CORRECTION_NO_PROGRESS", (uint8_t)(worseMask | unchangedMask));
    return 1u;
  }
  if (unchangedMask != 0u) {
    ReassertUnchangedAutoCorrection(unchangedMask);
    return 1u;
  }
  if (checkpointCorrectionRounds >= maxRounds) return 0u;
  const uint32_t elapsed = HAL_GetTick() - checkpointStartedMs;
  if (elapsed > CheckpointTimeout() ||
      CheckpointTimeout() - elapsed < CHECKPOINT_CORRECTION_BUDGET_MS) {
    printf("[VERIFY CHECKPOINT BUDGET] elapsed_ms=%lu remaining_ms=%lu required_ms=%lu deadline_ms=%u\r\n",
           (unsigned long)elapsed,
           (unsigned long)(elapsed <= CheckpointTimeout()
             ? CheckpointTimeout() - elapsed : 0u),
           (unsigned long)CHECKPOINT_CORRECTION_BUDGET_MS,
           (unsigned)CheckpointTimeout());
    LatchVerificationFault("CHECKPOINT_CORRECTION_BUDGET", failedMask);
    return 1u;
  }
  uint16_t plan[3];
  if (!PlanCompensatedTargets(plan, failedMask)) return 0u;
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (plan[i] < ActiveRawMin() || plan[i] > ActiveRawMax() ||
        AbsDiff(plan[i], servoCommandRaw[i]) > FEEDBACK_MAX_CORRECTION_STEP_RAW ||
        AbsDiff(plan[i], commandedRaw[i]) > (uint16_t)CompensationLimit()) {
      LatchVerificationFault("CHECKPOINT_CORRECTION_GUARD", (uint8_t)(1u << i));
      return 1u;
    }
  }
  for (uint8_t i = 0u; i < 3u; ++i) {
    checkpointPreviousSignedError[i] = (int16_t)(measuredRaw[i] - commandedRaw[i]);
    checkpointPreviousError[i] = AbsI16(checkpointPreviousSignedError[i]);
  }
  ++checkpointCorrectionRounds;
  verificationReadRetryAttempted = 0u;
  if (!ExecuteTargets(plan, XYZ_CORRECTION_TIME_MS)) {
    if (!faultLatched)
      LatchVerificationFault("CHECKPOINT_CORRECTION_TX_FAILED", 0x07u);
    return 1u;
  }
  checkpointCompensationHeld = 1u;
  checkpointOutputOffsetKnown = 1u;
  ++feedbackCorrections;
  printf("[VERIFY CHECKPOINT CORRECTION] round=%u/%u desired=%u,%u,%u command=%u,%u,%u time_ms=%u settle_ms=%u elapsed_ms=%lu deadline_ms=%u\r\n",
         checkpointCorrectionRounds, (unsigned)maxRounds,
         commandedRaw[0], commandedRaw[1], commandedRaw[2],
         plan[0], plan[1], plan[2], (unsigned)XYZ_CORRECTION_TIME_MS,
         (unsigned)XYZ_CORRECTION_SETTLE_MS,
         (unsigned long)(HAL_GetTick() - checkpointStartedMs),
         (unsigned)CheckpointTimeout());
  ScheduleVerification(VERIFY_STREAM_CHECKPOINT,
                       XYZ_CORRECTION_TIME_MS + XYZ_CORRECTION_SETTLE_MS);
  return 1u;
}

static void EvaluateVerification(void) {
  if (verificationPurpose == VERIFY_FAULT_RECOVERY) {
    uint8_t failed = checkpointInvalidMask;
    for (uint8_t i = 0u; i < 3u; ++i) {
      if (feedbackValidCount[i] != FEEDBACK_SAMPLES) {
        failed |= (uint8_t)(1u << i);
        continue;
      }
      SortI16(feedbackSamples[i], FEEDBACK_SAMPLES);
      measuredRaw[i] = feedbackSamples[i][1];
      measuredKnown[i] = 1u;
      if (feedbackSamples[i][2] - feedbackSamples[i][0] > FEEDBACK_MAX_SAMPLE_SPREAD_RAW ||
          !HomeRecoveryRawInRange(measuredRaw[i]) ||
          AbsI16((int16_t)(measuredRaw[i] - commandedRaw[i])) > AUTO_CORRECTION_MAX_ERROR_RAW)
        failed |= (uint8_t)(1u << i);
    }
    if (failed || !faultLatched || !faultRecoverable || armed || streamActive ||
        GripperManual_Active() || manualGripRuntime || linkStale || dryRunMode ||
        HAL_GetTick() - lastValidFrameMs > LINK_TIMEOUT_MS ||
        HAL_GetTick() - recoveryStartedMs > DELTA_FAULT_RECOVERY_TIMEOUT_MS) {
      FailRecovery();
      return;
    }
    /* Only unlock. Never adopt a new target, move, HOME, ARM or resume AUTO. */
    CancelVerification();
    busy = 0u;
    faultLatched = faultRecoverable = 0u;
    homeValid = homePending = rawKnown = streamBaselineKnown = 0u;
    verifiedSnapshotKnown = checkpointOutputOffsetKnown = 0u;
    /* Three stable fresh reads per joint just proved no DIRECT is still moving. */
    directStartedMask = directHoldPendingMask = directHoldRetryMask = directAbortActive = 0u;
    directHoldStableMask = 0u;
    ResetCompensation();
    printf("[FAULT RECOVERY PASS] measured=%d,%d,%d; PWM OFF, DISARMED, HOME REQUIRED.\r\n",
           measuredRaw[0], measuredRaw[1], measuredRaw[2]);
    SendRecoveryResult(DELTA_RESULT_OK);
    SendAsyncStatus(DELTA_RESULT_OK);
    return;
  }
  uint8_t failedMask = 0u;
  uint8_t xyzCorrectionMask = 0u;
  uint8_t insufficientMask = 0u;
  uint8_t unstableMask = 0u;
  uint8_t baselineMissingMask = 0u;
  uint8_t noProgressMask = 0u;
  uint8_t homeNotLevelMask = 0u;
  uint8_t homeNoProgressMask = 0u;
  uint8_t homeCaptureRangeMask = 0u;
  uint8_t streamNotLevelMask = 0u;
  uint8_t boxRangeMask = 0u;
  const VerificationPurpose purpose = verificationPurpose;
  const uint8_t postCorrection =
    purpose == VERIFY_STREAM_END && streamCorrectionAttempted;
  const int16_t positionTolerance = AutomaticStop() ? streamEndToleranceRaw : purpose == VERIFY_STREAM_CHECKPOINT
    ? CheckpointPositionTolerance()
    : (purpose == VERIFY_HOME_SEGMENT
    ? HOME_SEGMENT_TOLERANCE_RAW
    : (purpose == VERIFY_HOME
        ? HOME_FEEDBACK_TOLERANCE_RAW
        : ((activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) ==
             DELTA_STREAM_OPTION_XYZ_MANUAL
            ? XYZ_FEEDBACK_TOLERANCE_RAW
            : (postCorrection
                ? POST_CORRECTION_TOLERANCE_RAW
                : FEEDBACK_TOLERANCE_RAW))));
  for (uint8_t channel = 0u; channel < 3u; ++channel) {
    const uint8_t count = feedbackValidCount[channel];
    if (count < FEEDBACK_REQUIRED_SAMPLES) {
      insufficientMask |= (uint8_t)(1u << channel);
      printf("[VERIFY SUMMARY] id=%u valid=%u/%u result=NO_RESPONSE\r\n",
             servoIds[channel], count, (unsigned)FEEDBACK_SAMPLES);
      continue;
    }
    SortI16(feedbackSamples[channel], count);
    const int16_t sampleSpread = (int16_t)(
      feedbackSamples[channel][count - 1u] - feedbackSamples[channel][0]);
    if (sampleSpread > FEEDBACK_MAX_SAMPLE_SPREAD_RAW) {
      unstableMask |= (uint8_t)(1u << channel);
    }
    measuredRaw[channel] = feedbackSamples[channel][count / 2u];
    measuredKnown[channel] = 1u;
    if (((purpose == VERIFY_STREAM_CHECKPOINT && checkpointBoxToleranceRaw) || AutomaticStop()) &&
        (measuredRaw[channel] < (int16_t)ActiveRawMin() ||
         measuredRaw[channel] > (int16_t)ActiveRawMax()))
      boxRangeMask |= (uint8_t)(1u << channel);
    const int16_t error = (int16_t)(measuredRaw[channel] -
                                    (int16_t)commandedRaw[channel]);
    if (purpose == VERIFY_HOME_CAPTURE) {
      if (!HomeRecoveryRawInRange(measuredRaw[channel])) {
        homeCaptureRangeMask |= (uint8_t)(1u << channel);
      }
    } else if (AbsI16(error) > positionTolerance) {
      failedMask |= (uint8_t)(1u << channel);
    }
    if (purpose == VERIFY_STREAM_END &&
      ((activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) ==
        DELTA_STREAM_OPTION_XYZ_MANUAL || ExtendedZFeedback()) &&
        AbsI16(error) > (AutomaticStop() ? positionTolerance : XYZ_CORRECTION_TRIGGER_RAW)) {
      xyzCorrectionMask |= (uint8_t)(1u << channel);
    }
    const char *positionResult;
    if (sampleSpread > FEEDBACK_MAX_SAMPLE_SPREAD_RAW) {
      positionResult = "UNSTABLE";
    } else if (purpose == VERIFY_HOME_CAPTURE) {
      positionResult = HomeRecoveryRawInRange(measuredRaw[channel])
        ? "CAPTURED" : "OUT_OF_RANGE";
    } else {
      positionResult = AbsI16(error) <= positionTolerance
        ? "REACHED" : "OUT_OF_TOLERANCE";
    }
    printf("[VERIFY SUMMARY] id=%u valid=%u/%u median=%d spread=%d desired=%u error=%d result=%s\r\n",
           servoIds[channel], count, (unsigned)FEEDBACK_SAMPLES,
           measuredRaw[channel], sampleSpread, commandedRaw[channel], error,
           positionResult);
    if (purpose == VERIFY_STREAM_END) {
      const int16_t requestedDelta = (int16_t)((int16_t)commandedRaw[channel] -
                                               streamBaselineRaw[channel]);
      const int16_t measuredDelta = (int16_t)(measuredRaw[channel] -
                                              streamBaselineRaw[channel]);
      const uint8_t directionMatches =
        (requestedDelta < 0 && measuredDelta < 0) ||
        (requestedDelta > 0 && measuredDelta > 0);
      const uint8_t smallNetTarget =
        AbsI16(requestedDelta) < STREAM_MIN_REQUESTED_DELTA_RAW;
      const uint8_t endpointReached =
        AbsI16(error) <= positionTolerance;
      if (!streamBaselineKnown) {
        baselineMissingMask |= (uint8_t)(1u << channel);
      } else if (!endpointReached && !smallNetTarget && (!directionMatches ||
                 AbsI16(measuredDelta) < STREAM_MIN_MEASURED_DELTA_RAW)) {
        noProgressMask |= (uint8_t)(1u << channel);
      }
      const char *progressResult = "PROGRESS_OK";
      if (!streamBaselineKnown) progressResult = "BASELINE_UNKNOWN";
      else if (endpointReached) progressResult = "ENDPOINT_REACHED";
      else if (smallNetTarget) progressResult = "RETURN_OR_SMALL_TARGET";
      else if (!directionMatches ||
               AbsI16(measuredDelta) < STREAM_MIN_MEASURED_DELTA_RAW) {
        progressResult = "NO_PROGRESS";
      }
      printf("[VERIFY PROGRESS] id=%u start=%d desired=%u measured=%d requested_delta=%d measured_delta=%d result=%s\r\n",
             servoIds[channel], streamBaselineRaw[channel],
             commandedRaw[channel], measuredRaw[channel], requestedDelta,
             measuredDelta, progressResult);
    }
    if (purpose == VERIFY_HOME_SEGMENT) {
      const int16_t requestedDelta = (int16_t)(
        (int16_t)commandedRaw[channel] - homeSegmentStartRaw[channel]);
      const int16_t measuredDelta = (int16_t)(
        measuredRaw[channel] - homeSegmentStartRaw[channel]);
      const uint8_t directionMatches = requestedDelta == 0 ||
        (requestedDelta < 0 && measuredDelta < 0) ||
        (requestedDelta > 0 && measuredDelta > 0);
      const uint8_t smallTarget =
        AbsI16(requestedDelta) < HOME_PROGRESS_REQUIRED_DELTA_RAW;
      if (!smallTarget && (!directionMatches ||
          AbsI16(measuredDelta) < HOME_SEGMENT_MIN_PROGRESS_RAW)) {
        homeNoProgressMask |= (uint8_t)(1u << channel);
      }
      printf("[VERIFY HOME PROGRESS] step=%u id=%u start=%d desired=%u measured=%d requested_delta=%d measured_delta=%d result=%s\r\n",
             homeSegmentCount, servoIds[channel], homeSegmentStartRaw[channel],
             commandedRaw[channel], measuredRaw[channel], requestedDelta,
             measuredDelta,
             requestedDelta == 0 ? "HOLD" :
               (smallTarget ? "SMALL_TARGET" :
                 ((directionMatches &&
                   AbsI16(measuredDelta) >= HOME_SEGMENT_MIN_PROGRESS_RAW)
                    ? "PROGRESS_OK" : "NO_PROGRESS")));
    }
  }
  if (purpose == VERIFY_HOME && insufficientMask == 0u) {
    int16_t homeMin = measuredRaw[0];
    int16_t homeMax = measuredRaw[0];
    for (uint8_t channel = 1u; channel < 3u; ++channel) {
      if (measuredRaw[channel] < homeMin) homeMin = measuredRaw[channel];
      if (measuredRaw[channel] > homeMax) homeMax = measuredRaw[channel];
    }
    const int16_t homeSpread = (int16_t)(homeMax - homeMin);
    if (homeSpread > HOME_MAX_CHANNEL_SPREAD_RAW) homeNotLevelMask = 0x07u;
    printf("[VERIFY HOME LEVEL] stage=%s min=%d max=%d spread=%d limit=%d result=%s\r\n",
           VerificationPurposeName(purpose), homeMin, homeMax, homeSpread,
           HOME_MAX_CHANNEL_SPREAD_RAW,
           homeNotLevelMask == 0u ? "LEVEL" : "NOT_LEVEL");
  }
  if (postCorrection &&
      (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) ==
        DELTA_STREAM_OPTION_Z_ONLY &&
      insufficientMask == 0u) {
    int16_t streamMin = measuredRaw[0];
    int16_t streamMax = measuredRaw[0];
    for (uint8_t channel = 1u; channel < 3u; ++channel) {
      if (measuredRaw[channel] < streamMin) streamMin = measuredRaw[channel];
      if (measuredRaw[channel] > streamMax) streamMax = measuredRaw[channel];
    }
    const int16_t streamSpread = (int16_t)(streamMax - streamMin);
    if (streamSpread > STREAM_FINAL_MAX_CHANNEL_SPREAD_RAW) {
      streamNotLevelMask = 0x07u;
    }
    printf("[VERIFY STREAM LEVEL] stage=POST_CORRECTION min=%d max=%d spread=%d limit=%d result=%s\r\n",
           streamMin, streamMax, streamSpread,
           STREAM_FINAL_MAX_CHANNEL_SPREAD_RAW,
           streamNotLevelMask == 0u ? "LEVEL" : "NOT_LEVEL");
  }
  if ((purpose == VERIFY_STREAM_CHECKPOINT || AutomaticStop()) && checkpointInvalidMask != 0u) {
    LatchVerificationFault("CHECKPOINT_INVALID_FEEDBACK", checkpointInvalidMask);
    return;
  }
  if (boxRangeMask != 0u) {
    LatchVerificationFault("BOX_FEEDBACK_RAW_RANGE", boxRangeMask);
    return;
  }
  if ((insufficientMask != 0u || unstableMask != 0u) &&
      verificationReadRetryAttempted < FEEDBACK_READ_MAX_RETRIES) {
    ++verificationReadRetryAttempted;
    ++feedbackReadRetries;
    printf("[VERIFY RETRY] reason=%s failed_mask=0x%02X attempt=%u/%u delay_ms=%u mode=ACCUMULATIVE; no se retransmite movimiento.\r\n",
           insufficientMask != 0u ? "NO_RESPONSE" : "UNSTABLE",
           insufficientMask != 0u ? insufficientMask : unstableMask,
           verificationReadRetryAttempted,
           (unsigned)FEEDBACK_READ_MAX_RETRIES,
           (unsigned)FEEDBACK_RETRY_DELAY_MS);
    ScheduleVerificationReadRetry(purpose, FEEDBACK_RETRY_DELAY_MS,
                                  insufficientMask, unstableMask);
    return;
  } else if (insufficientMask != 0u) {
    LatchVerificationFault("NO_RESPONSE", insufficientMask);
  } else if (unstableMask != 0u) {
    LatchVerificationFault("NOT_SETTLED", unstableMask);
  } else if (homeCaptureRangeMask != 0u) {
    LatchVerificationFault("HOME_CAPTURE_OUT_OF_RANGE", homeCaptureRangeMask);
  } else if (streamNotLevelMask != 0u) {
    LatchVerificationFault("STREAM_NOT_LEVEL", streamNotLevelMask);
  } else if (purpose == VERIFY_HOME &&
             (homeNotLevelMask != 0u || failedMask != 0u) &&
             AttemptHomeFinalCorrection((uint8_t)(homeNotLevelMask | failedMask))) {
    return;
  } else if (homeNotLevelMask != 0u) {
    LatchVerificationFault("HOME_FINAL_NOT_LEVEL", homeNotLevelMask);
  } else if (baselineMissingMask != 0u) {
    LatchVerificationFault("BASELINE_UNKNOWN", baselineMissingMask);
  } else if (purpose == VERIFY_HOME_SEGMENT &&
             (failedMask != 0u || homeNoProgressMask != 0u) &&
             AttemptHomeSegmentRetry(
               (uint8_t)(failedMask | homeNoProgressMask))) {
    return;
  } else if (purpose == VERIFY_STREAM_CHECKPOINT && checkpointObserve) {
    /* Only target precision is advisory. All read/stability/range/lease guards
       above remain mandatory, and DIRECT progress/STOP guards are unchanged. */
    if (failedMask) printf("[AUTO POSITION AVISO] mask=0x%02X error_raw=%d,%d,%d; sin afinado, siguiente ruta desde pose medida.\r\n",
      failedMask, measuredRaw[0]-(int16_t)commandedRaw[0],
      measuredRaw[1]-(int16_t)commandedRaw[1], measuredRaw[2]-(int16_t)commandedRaw[2]);
    CompleteVerification();
    return;
  } else if (purpose == VERIFY_STREAM_CHECKPOINT && failedMask != 0u &&
             AttemptCheckpointCorrection(failedMask)) {
    return;
  } else if (xyzCorrectionMask != 0u &&
             AttemptXyzCorrection(xyzCorrectionMask)) {
    return;
  } else if ((failedMask != 0u || noProgressMask != 0u) &&
              AttemptMeasuredCorrection()) {
    return;
  } else if (failedMask != 0u) {
    LatchVerificationFault(
      purpose == VERIFY_HOME_SEGMENT
        ? "HOME_SEGMENT_OUT_OF_TOLERANCE"
        : (purpose == VERIFY_HOME
            ? "HOME_FINAL_OUT_OF_TOLERANCE" : "OUT_OF_TOLERANCE"),
      failedMask);
  } else if (homeNoProgressMask != 0u) {
    LatchVerificationFault("HOME_SEGMENT_NO_PROGRESS", homeNoProgressMask);
  } else if (noProgressMask != 0u) {
    LatchVerificationFault("NO_MEASURABLE_PROGRESS", noProgressMask);
  } else {
    CompleteVerification();
  }
}

static void ServiceVerification(void) {
  if (!verificationPending) return;
  if (verificationPurpose == VERIFY_STREAM_END &&
      (directAbortActive || directHoldPendingMask || directStartedMask)) return;
  /* CHECKPOINT puede llegar durante el puente desde la orden compensada.
     Sus renewals completan ese puente; no medir una pose anterior. */
  if (verificationPurpose == VERIFY_STREAM_CHECKPOINT &&
      checkpointTargetConverging) return;
  const uint32_t now = HAL_GetTick();
  if ((int32_t)(now - verificationStartMs) < 0) return;

  if (dryRunMode) {
    for (uint8_t channel = 0u; channel < 3u; ++channel) {
      measuredRaw[channel] = verificationPurpose == VERIFY_HOME_CAPTURE
        ? (int16_t)RAW_HOME : (int16_t)commandedRaw[channel];
      measuredKnown[channel] = 1u;
    }
    CompleteVerification();
    return;
  }

  if (!verificationSampling) {
    verificationSampling = 1u;
    feedbackSampleIndex = 0u;
    feedbackChannel = 0u;
    nextFeedbackReadMs = now;
    printf("[VERIFY START] stage=%s samples=%u timeout_ms=%u.\r\n",
           IsHomeVerification(verificationPurpose)
             ? VerificationPurposeName(verificationPurpose)
             : (streamCorrectionAttempted ? "POST_CORRECTION" : "INITIAL"),
           (unsigned)FEEDBACK_SAMPLES, (unsigned)FEEDBACK_READ_TIMEOUT_MS);
  }
  if ((int32_t)(now - nextFeedbackReadMs) < 0) return;

  /* La pose permanece congelada. No consumir UART/plazo en canales ya
     completos; los inestables se limpiaron al programar el reintento.
     HOME y STREAM_END conservan su secuencia de lectura anterior. */
  while (verificationPurpose == VERIFY_STREAM_CHECKPOINT &&
         (((verificationReadChannelMask & (uint8_t)(1u << feedbackChannel)) == 0u) ||
          feedbackValidCount[feedbackChannel] >= FEEDBACK_SAMPLES)) {
    ++feedbackChannel;
    if (feedbackChannel >= 3u) {
      feedbackChannel = 0u;
      ++feedbackSampleIndex;
    }
    if (feedbackSampleIndex >= FEEDBACK_SAMPLES) {
      EvaluateVerification();
      return;
    }
  }

  ServoPositionReading reading;
  ++feedbackReads;
  const ServoReadStatus status = ServoReadPosition(
      servoPorts[feedbackChannel], servoIds[feedbackChannel], &reading,
      FEEDBACK_READ_TIMEOUT_MS);
  if (status == SERVO_READ_OK) {
    const uint8_t count = feedbackValidCount[feedbackChannel];
    if (count < FEEDBACK_SAMPLES) {
      feedbackSamples[feedbackChannel][count] = reading.position;
      feedbackValidCount[feedbackChannel] = (uint8_t)(count + 1u);
    }
    ++feedbackOk;
  } else if (status == SERVO_READ_RX_TIMEOUT) {
    ++feedbackTimeouts;
  } else {
    ++feedbackInvalid;
    if (verificationPurpose == VERIFY_STREAM_CHECKPOINT || verificationPurpose == VERIFY_FAULT_RECOVERY || AutomaticStop())
      checkpointInvalidMask |= (uint8_t)(1u << feedbackChannel);
  }
  printf("[VERIFY READ] sample=%u id=%u status=%s raw=%d elapsed_ms=%lu\r\n",
         (unsigned)(feedbackSampleIndex + 1u), servoIds[feedbackChannel],
         ServoReadStatusName(status),
         status == SERVO_READ_OK ? reading.position : -1,
         (unsigned long)reading.elapsedMs);

  ++feedbackChannel;
  if (feedbackChannel >= 3u) {
    feedbackChannel = 0u;
    ++feedbackSampleIndex;
  }
  if (feedbackSampleIndex >= FEEDBACK_SAMPLES) {
    EvaluateVerification();
  } else {
    nextFeedbackReadMs = HAL_GetTick() + FEEDBACK_READ_GAP_MS;
  }
}

static uint8_t RawInRange(const DeltaLinkFrame *frame) {
  const uint16_t minimum = ActiveRawMin();
  const uint16_t maximum = ActiveRawMax();
  for (uint8_t index = 0u; index < 3u; ++index) {
    if (frame->raw[index] < minimum || frame->raw[index] > maximum) return 0u;
  }
  return 1u;
}

static uint8_t RawStepAllowed(const DeltaLinkFrame *frame) {
  if (!rawKnown) return 0u;
  const uint16_t maxStep = frame->type==DELTA_MSG_APPROACH_SETPOINT && approachStep5Capable
    ? DELTA_APPROACH_STEP_RAW : (frame->status & DELTA_STREAM_SETPOINT_COARSE)
      ? STREAM_COARSE_MAX_STEP : STREAM_RAW_MAX_STEP;
  for (uint8_t index = 0u; index < 3u; ++index) {
    if (AbsDiff(frame->raw[index], commandedRaw[index]) >
        maxStep) return 0u;
  }
  return 1u;
}

static uint8_t ZOnlyAllowed(const DeltaLinkFrame *frame) {
  return AbsDiff(frame->raw[0], frame->raw[1]) <= STREAM_Z_SPREAD_MAX &&
         AbsDiff(frame->raw[1], frame->raw[2]) <= STREAM_Z_SPREAD_MAX &&
         AbsDiff(frame->raw[0], frame->raw[2]) <= STREAM_Z_SPREAD_MAX;
}

static uint8_t PhysicalDeltaReached(const DeltaLinkFrame *frame) {
  for (uint8_t index = 0u; index < 3u; ++index) {
    if (AbsDiff(frame->raw[index], servoCommandRaw[index]) >=
        STREAM_PHYSICAL_DELTA_RAW) return 1u;
  }
  return 0u;
}

static uint8_t ExecuteTargets(const uint16_t raw[3], uint16_t moveTimeMs) {
  /* Cualquier consigna HTS posterior invalida la ultima pose medida. DIRECT
     sólo puede arrancar desde un checkpoint fisico nuevo. */
  verifiedSnapshotKnown = 0u;
  if (dryRunMode) {
    memcpy(servoCommandRaw, raw, sizeof(servoCommandRaw));
    return 1u;
  }
  HAL_StatusTypeDef status[3];
  uint8_t frames[3][10];
  for (uint8_t index = 0u; index < 3u; ++index) {
    if ((localPointActive || directPointActive) && !LocalPointGuards()) return 0u;
    if (verificationPurpose == VERIFY_STREAM_CHECKPOINT && checkpointPending &&
        !CheckpointCorrectionGuards()) return 0u;
    status[index] = ServoSendMove(servoPorts[index], servoIds[index],
                                  raw[index], moveTimeMs, frames[index]);
    if ((localPointActive || directPointActive) &&
        (status[index] != HAL_OK || !LocalPointGuards())) return 0u;
    if (verificationPurpose == VERIFY_STREAM_CHECKPOINT && checkpointPending &&
        (status[index] != HAL_OK || !CheckpointCorrectionGuards())) return 0u;
  }
  for (uint8_t index = 0u; index < 3u; ++index) {
    if (status[index] != HAL_OK) return 0u;
  }
  memcpy(servoCommandRaw, raw, sizeof(servoCommandRaw));
  if (memcmp(servoCommandRaw, commandedRaw, sizeof(commandedRaw)) == 0)
    checkpointOutputOffsetKnown = 0u;
  ++servoTxBatches;
  return 1u;
}

static HAL_StatusTypeDef DirectWriteBudgeted(uint8_t index, uint16_t raw,
                                             uint16_t moveTimeMs, uint32_t budgetMs) {
  uint8_t frame[10];
  directWriteAttempted = 0u;
  if (budgetMs < DIRECT_BUS_QUIET_MS + 2u) {
    directTransactionFaultReason = "DIRECT_WRITE_BUDGET";
    return HAL_TIMEOUT;
  }
  const uint32_t startedMs = HAL_GetTick();
  const uint32_t idleBudget = budgetMs - 2u < DIRECT_BUS_IDLE_MAX_MS
    ? budgetMs - 2u : DIRECT_BUS_IDLE_MAX_MS;
  uint16_t drained = 0u;
  const HAL_StatusTypeDef idle = ServoWaitBusIdle(
    servoPorts[index], DIRECT_BUS_QUIET_MS, idleBudget, &drained);
  directBusDrainedBytes += drained;
  if (idle != HAL_OK) {
    /* Fail closed: a frame sent over an active reply can be lost unseen. */
    directBusBusyMask |= (uint8_t)(1u << index);
    ++directBusBusy;
    return HAL_BUSY;
  }
  if (!LocalPointGuards()) {
    directTransactionFaultReason = "DIRECT_WRITE_PERMIT_LOST";
    return HAL_ERROR;
  }
  const uint32_t elapsed = HAL_GetTick() - startedMs;
  if (elapsed >= budgetMs - 1u) {
    directTransactionFaultReason = "DIRECT_WRITE_BUDGET";
    return HAL_TIMEOUT;
  }
  // The driver may drain for txTimeout and transmit for txTimeout. Both must
  // fit the REMAINING common transaction budget, even for the second batch.
  const uint32_t remaining = budgetMs - elapsed;
  const uint32_t txTimeout = remaining / 2u < DIRECT_TX_TIMEOUT_MS
    ? remaining / 2u : DIRECT_TX_TIMEOUT_MS;
  directWriteAttempted = 1u;
  ++directWrites;
  return ServoSendMoveBounded(servoPorts[index], servoIds[index], raw, moveTimeMs,
                              frame, txTimeout);
}

static HAL_StatusTypeDef DirectWrite(uint8_t index, uint16_t raw, uint16_t moveTimeMs) {
  return DirectWriteBudgeted(index, raw, moveTimeMs, DIRECT_WRITE_BUDGET_MS);
}

/* Plain MOVE_TIME_WRITE (0x01), the same command the STREAM uses. HTS writes
   have no ACK; the progress probes are the physical confirmation. On failure
   the caller latches FAULT, whose StopGripperOutputs holds started joints. */
static uint8_t ExecuteDirectTransaction(const uint16_t raw[3],
                                        uint16_t moveTimeMs,
                                        uint8_t copies,
                                        uint8_t *failedMask) {
  *failedMask = 0u;
  directTransactionFaultReason = "DIRECT_WRITE_FAILED";
  directStartedMask = directBusBusyMask = 0u;
  if (dryRunMode || (copies != 1u && copies != DIRECT_LIFT_INITIAL_COPIES &&
      !(directRepeat5Capable && (copies == DIRECT_AUTO_INITIAL_COPIES ||
                               copies == DIRECT_AUTO_RECOVERY_COPIES)))) return 0u;
  const uint32_t txBudgetMs=DirectTransactionBudgetMs(copies);
  const uint32_t startedMs = HAL_GetTick();
  for (uint8_t copy = 0u; copy < copies; ++copy) {
    for (uint8_t index = 0u; index < 3u; ++index) {
      const uint8_t bit = (uint8_t)(1u << index);
      if (!(directWriteMask & bit)) continue;
      if (!LocalPointGuards()) {
        directTransactionFaultReason = "DIRECT_WRITE_PERMIT_LOST";
        *failedMask = 0x07u;
        return 0u;
      }
      const uint32_t elapsed = HAL_GetTick() - startedMs;
      if (elapsed >= txBudgetMs) {
        directTransactionFaultReason = "DIRECT_WRITE_BUDGET";
        *failedMask = 0x07u;
        return 0u;
      }
      const HAL_StatusTypeDef result = DirectWriteBudgeted(
        index, raw[index], moveTimeMs, txBudgetMs - elapsed);
      if (result == HAL_BUSY) {
        *failedMask |= bit; // no frame on THIS attempt; prior copies may have started it
        directTransactionFaultReason = "DIRECT_BUS_BUSY";
        return 0u;
      }
      if (result != HAL_OK) {
        if (!directWriteAttempted) {
          *failedMask |= bit;
          return 0u;
        }
        ++directWriteFailures;
        *failedMask |= bit;
        directStartedMask |= bit; // a HAL error does not prove no byte was sent
        directTransactionFaultReason = "DIRECT_WRITE_TX_FAILED";
        return 0u;
      }
      directStartedMask |= bit;
      if (HAL_GetTick() - startedMs > txBudgetMs) {
        directTransactionFaultReason = "DIRECT_WRITE_BUDGET";
        *failedMask = 0x07u;
        return 0u;
      }
    }
  }
  /* Leases are re-checked after the last frame too, never extended by TX. */
  if (!LocalPointGuards()) {
    directTransactionFaultReason = "DIRECT_WRITE_PERMIT_LOST";
    *failedMask = 0x07u;
    return 0u;
  }
  for (uint8_t i = 0u; i < 3u; ++i)
    if (directWriteMask & (uint8_t)(1u << i)) servoCommandRaw[i] = raw[i];
  verifiedSnapshotKnown = 0u;
  ++servoTxBatches;
  return 1u;
}

static uint32_t DirectFirstProbeOffsetMs(void) {
  if (!directLiftProbeActive) return DIRECT_START_PROBE_MS;
  return directProfile250Capable ? DELTA_DIRECT_PROFILE250_FIRST_PROBE_MS :
    DELTA_DIRECT_LIFT_FIRST_PROBE_MS;
}

static uint32_t DirectMidProbeOffsetMs(void) {
  uint32_t offsetMs = (uint32_t)directSegDurationMs / 4u;
  if (offsetMs < DIRECT_PROGRESS_MIN_PROBE_MS) offsetMs = DIRECT_PROGRESS_MIN_PROBE_MS;
  if (offsetMs > DIRECT_PROGRESS_MAX_PROBE_MS) offsetMs = DIRECT_PROGRESS_MAX_PROBE_MS;
  if (directLiftProbeActive &&
      offsetMs < DirectFirstProbeOffsetMs() + DIRECT_LIFT_PROBE_GAP_MS)
    offsetMs = DirectFirstProbeOffsetMs() + DIRECT_LIFT_PROBE_GAP_MS;
  return offsetMs;
}

/* All three joints are always read (a coordinated recovery needs the full
   pose); only joints expected to move clearly by offsetMs are stall-checked. */
static uint8_t ScheduleDirectProbe(uint8_t stage, uint32_t offsetMs) {
  uint8_t requiredMask = 0u;
  for (uint8_t index = 0u; index < 3u; ++index) {
    const uint16_t distance = AbsDiff(directSegStartRaw[index], directGoalRaw[index]);
    const uint32_t expected = (uint32_t)distance * offsetMs / (uint32_t)directSegDurationMs;
    if (distance >= DIRECT_PROGRESS_MIN_DISTANCE_RAW &&
        expected >= DIRECT_PROGRESS_MIN_EXPECTED_RAW)
      requiredMask |= (uint8_t)(1u << index);
  }
  ClearDirectProbeState();
  if (!requiredMask) return 0u;
  directProbeStage = stage;
  directProbeRequiredMask = requiredMask;
  directProbePending = 1u;
  directProbeDueMs = directProbeNextReadMs = directSegStartMs + offsetMs;
  return 1u;
}

/* Short-distance joints can be invisible in both early windows. Check them
   once >=8 RAW is expected instead of lowering the noise threshold. Every
   PASS removes an axis; at most three extra windows, still no MOVE resends. */
static uint32_t DirectReadBudgetMs(void) {
  return approachStep5Capable && directFinalReadActive
    ? DELTA_POST_PICKUP_READ_BUDGET_MS : DIRECT_PROGRESS_READ_BUDGET_MS;
}

static uint8_t ScheduleAdaptiveDirectProbe(void) {
  if (!directProfile250Capable) return 0u;
  const uint32_t elapsed = HAL_GetTick() - directSegStartMs;
  uint32_t next = UINT32_MAX;
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (directProgressCheckedMask & (uint8_t)(1u << i)) continue;
    const uint16_t distance = AbsDiff(directSegStartRaw[i], directGoalRaw[i]);
    if (distance < DIRECT_PROGRESS_MIN_DISTANCE_RAW) continue;
    uint32_t offset = ((uint32_t)DIRECT_PROGRESS_MIN_EXPECTED_RAW * directSegDurationMs +
                        distance - 1u) / distance;
    if (offset < DirectFirstProbeOffsetMs()) offset = DirectFirstProbeOffsetMs();
    if (offset <= elapsed) offset = elapsed + 1u;
    if (offset + DirectReadBudgetMs() < directSegDurationMs && offset < next)
      next = offset;
  }
  if (next == UINT32_MAX || !ScheduleDirectProbe(DIRECT_PROBE_ADAPTIVE, next)) return 0u;
  printf("[DIRECT ADAPTIVE PROBE] offset_ms=%lu required=0x%02X checked=0x%02X.\r\n",
         (unsigned long)next, directProbeRequiredMask, directProgressCheckedMask);
  return 1u;
}

static void BeginDirectProgressProbe(uint32_t startedMs) {
  directRecovered = 0u;
  directReadDeferred = 0u;
  directProgressCheckedMask = 0u;
  memcpy(directSegStartRaw, directStartRaw, sizeof(directSegStartRaw));
  directSegStartMs = startedMs;
  directSegDurationMs = directPointDuration;
  directEndMs = startedMs + directPointDuration;
  const uint32_t firstMs = DirectFirstProbeOffsetMs();
  if (!ScheduleDirectProbe(DIRECT_PROBE_START, firstMs) &&
      !ScheduleDirectProbe(DIRECT_PROBE_MID, DirectMidProbeOffsetMs()))
    (void)ScheduleAdaptiveDirectProbe();
  printf("[DIRECT PROBE PLAN] lift_delay=%u first_ms=%lu mid_ms=%lu read_budget_ms=%u; final_PASS_required.\r\n",
         directLiftProbeActive, (unsigned long)firstMs,
         (unsigned long)DirectMidProbeOffsetMs(), (unsigned)DirectReadBudgetMs());
}

/* Chebyshev distance, in RAW, from a joint pose to the validated straight
   joint path directStartRaw -> directGoalRaw. Convex in f: ternary search. */
static float DirectLineDistance(const int16_t pose[3], float f) {
  float worst = 0.0f;
  for (uint8_t index = 0u; index < 3u; ++index) {
    const float start = (float)directStartRaw[index];
    float d = (float)pose[index] - (start + f * ((float)directGoalRaw[index] - start));
    if (d < 0.0f) d = -d;
    if (d > worst) worst = d;
  }
  return worst;
}

static uint16_t DirectPathDeviationRaw(const int16_t pose[3]) {
  float low = 0.0f, high = 1.0f;
  for (uint8_t iteration = 0u; iteration < 40u; ++iteration) {
    const float m1 = low + (high - low) / 3.0f;
    const float m2 = high - (high - low) / 3.0f;
    if (DirectLineDistance(pose, m1) <= DirectLineDistance(pose, m2)) high = m2;
    else low = m1;
  }
  return (uint16_t)(DirectLineDistance(pose, 0.5f * (low + high)) + 0.999f);
}

/* One joint did not start (lost/garbled frame). Re-send the SAME endpoint to
   ALL three joints from the measured pose with one common time, so they arrive
   together again. The new segment ends on the validated endpoint, so it never
   strays farther from the validated joint path than the arm already is; that
   deviation must be <= DIRECT_RECOVERY_MAX_DEVIATION_RAW. Only once. */
static void RecoverStalledDirect(uint8_t stalledMask) {
  if (directValidatedPathActive) {
    if (directRetryCapable && directProfile250Capable && !directRetryUsed &&
        LocalPointGuards()) {
      directRetryUsed = 1u;
      directRetryPhase = 1u;
      directRetryStartedMs = HAL_GetTick();
      SendDirectRetryState(DELTA_RESULT_BUSY);
      /* HTS only: keep the closed RC gate under its unchanged lease. Explicit
         STOP/FAULT still withdraws PWM via StopGripperOutputs and cancels this. */
      SendDirectStopFrames();
      AbortDirectMotion();
      ClearDirectProbeState();
      printf("[DIRECT RETRY HOLD] seq=%u stalled=0x%02X; stop all, require stable pose and new B preflight.\r\n",
             localPointSequence, stalledMask);
      return;
    }
    ++directProgressFaults;
    LatchVerificationFault("DIRECT_NO_PROGRESS", stalledMask);
    return;
  }
  if (!LocalPointGuards()) {
    LatchVerificationFault("DIRECT_PROGRESS_PERMIT_LOST", 0x07u);
    return;
  }
  const uint16_t deviation = DirectPathDeviationRaw(directProbeRaw);
  if (deviation > DIRECT_RECOVERY_MAX_DEVIATION_RAW) {
    ++directProgressFaults;
    printf("[DIRECT RECOVERY REFUSED] stalled=0x%02X path_dev_raw=%u max=%u sample=%d,%d,%d.\r\n",
           stalledMask, deviation, (unsigned)DIRECT_RECOVERY_MAX_DEVIATION_RAW,
           directProbeRaw[0], directProbeRaw[1], directProbeRaw[2]);
    LatchVerificationFault("DIRECT_RECOVERY_DEVIATION", stalledMask);
    return;
  }
  const uint32_t elapsed = HAL_GetTick() - localPointTick;
  uint32_t timeMs = directPointDuration > elapsed ? directPointDuration - elapsed : 0u;
  for (uint8_t index = 0u; index < 3u; ++index) {
    const uint32_t minimum =
      ((uint32_t)AbsDiff((uint16_t)directProbeRaw[index], directGoalRaw[index]) * 1000u +
       DELTA_DIRECT_RAW_PER_SECOND - 1u) / DELTA_DIRECT_RAW_PER_SECOND;
    if (timeMs < minimum) timeMs = minimum;
  }
  if (timeMs < DIRECT_HOLD_TIME_MS) timeMs = DIRECT_HOLD_TIME_MS;
  if (timeMs > DELTA_DIRECT_MAX_DURATION_MS) {
    LatchVerificationFault("DIRECT_RECOVERY_TIME", stalledMask);
    return;
  }
  const uint32_t writesStartedMs = HAL_GetTick();
  /* Keep the slowest old/new command metadata valid even if this recovery's
     transaction only reaches some axes before it has to abort. */
  for (uint8_t index = 0u; index < 3u; ++index)
    directSegStartRaw[index] = (uint16_t)directProbeRaw[index];
  directSegDurationMs = (uint16_t)timeMs;
  for (uint8_t index = 0u; index < 3u; ++index) {
    const uint8_t bit = (uint8_t)(1u << index);
    if (!LocalPointGuards()) {
      LatchVerificationFault("DIRECT_PROGRESS_PERMIT_LOST", 0x07u);
      return;
    }
    const HAL_StatusTypeDef result = DirectWrite(index, directGoalRaw[index], (uint16_t)timeMs);
    if (result != HAL_OK) {
      if (result != HAL_BUSY) ++directWriteFailures;
      LatchVerificationFault(result == HAL_BUSY ? "DIRECT_RECOVERY_BUS_BUSY"
                                                : "DIRECT_RECOVERY_TX", bit);
      return;
    }
    if (HAL_GetTick() - writesStartedMs > DIRECT_WRITE_BUDGET_MS) {
      LatchVerificationFault("DIRECT_WRITE_BUDGET", 0x07u);
      return;
    }
  }
  for (uint8_t index = 0u; index < 3u; ++index)
    directSegStartRaw[index] = (uint16_t)directProbeRaw[index];
  directSegStartMs = HAL_GetTick();
  directSegDurationMs = (uint16_t)timeMs;
  directEndMs = directSegStartMs + timeMs;
  busyUntilMs = directEndMs;
  directRecovered = 1u;
  ++directResends;
  printf("[DIRECT RECOVERY] stalled=0x%02X from=%d,%d,%d goal=%u,%u,%u time_ms=%lu path_dev_raw=%u; 3 joints, same endpoint, once.\r\n",
         stalledMask, directProbeRaw[0], directProbeRaw[1], directProbeRaw[2],
         directGoalRaw[0], directGoalRaw[1], directGoalRaw[2],
         (unsigned long)timeMs, deviation);
  (void)ScheduleDirectProbe(DIRECT_PROBE_RECOVERY, DIRECT_START_PROBE_MS);
}

/* Only the explicitly negotiated AUTO transit (type25) can postpone a lack
   of positive progress to its final physical checkpoint. Never apply while
   confirming STOP/acquiring a recovery origin, or to legacy/fine moves. */
static uint8_t DirectProgressAdvisoryAllowed(void) {
  return directFinalReadActive && directFinalReadCapable && directPointActive &&
         directValidatedPathActive && !directRetryPhase && LocalPointGuards();
}

static void WarnDirectProgress(uint8_t stalledMask) {
  ++directProgressWarnings;
  printf("[DIRECT PROGRESS WARNING] seq=%u stage=%u slow=0x%02X known=0x%02X origin=%u,%u,%u goal=%u,%u,%u sample=%d,%d,%d elapsed_ms=%lu,%lu,%lu; mismo destino, sin STOP/reenvio, lectura FINAL obligatoria; NO_PASS.\r\n",
         localPointSequence, directProbeStage, stalledMask, directProbeKnownMask,
         directSegStartRaw[0], directSegStartRaw[1], directSegStartRaw[2],
         directGoalRaw[0], directGoalRaw[1], directGoalRaw[2],
         directProbeRaw[0], directProbeRaw[1], directProbeRaw[2],
         (unsigned long)directProbeElapsedMs[0],
         (unsigned long)directProbeElapsedMs[1],
         (unsigned long)directProbeElapsedMs[2]);
}

static void EvaluateDirectProgress(void) {
  if ((directProbeKnownMask & 0x07u) != 0x07u) {
    ++directProgressFaults;
    LatchVerificationFault("DIRECT_PROGRESS_INCOMPLETE",
                           (uint8_t)(0x07u & (uint8_t)~directProbeKnownMask));
    return;
  }
  if (directRetryPhase == 2u || directRetryPhase == 3u) {
    for (uint8_t i = 0u; i < 3u; ++i) {
      if (!DirectHoldRawValid(i, directProbeRaw[i]) ||
          (directRetryPhase == 3u && AbsI16((int16_t)(directProbeRaw[i] -
             (int16_t)directRetryFirst[i])) > DIRECT_HOLD_STABLE_RAW)) {
        LatchVerificationFault("DIRECT_RETRY_UNSTABLE", (uint8_t)(1u << i));
        return;
      }
    }
    if (directRetryPhase == 2u) {
      for (uint8_t i = 0u; i < 3u; ++i) directRetryFirst[i] = (uint16_t)directProbeRaw[i];
      directRetryPhase = 3u;
      ClearDirectProbeState();
      directProbePending = 1u;
      directProbeDueMs = directProbeNextReadMs = HAL_GetTick() + DIRECT_HOLD_STABLE_GAP_MS;
    } else {
      for (uint8_t i = 0u; i < 3u; ++i) directRetryOrigin[i] = (uint16_t)directProbeRaw[i];
      directRetryOriginMs = HAL_GetTick();
      directRetryPhase = 4u;
      ClearDirectProbeState();
      SendDirectRetryState(DELTA_RESULT_OK);
      printf("[DIRECT RETRY READY] seq=%u origin=%u,%u,%u; waiting validated path <=200ms.\r\n",
             localPointSequence, directRetryOrigin[0], directRetryOrigin[1], directRetryOrigin[2]);
    }
    return;
  }
  uint8_t corridorMask = 0u;
  uint8_t reverseMask = 0u;
  uint8_t stalledMask = 0u;
  for (uint8_t index = 0u; index < 3u; ++index) {
    const uint8_t bit = (uint8_t)(1u << index);
    const int32_t start = directSegStartRaw[index];
    const int32_t goal = directGoalRaw[index];
    const int32_t sample = directProbeRaw[index];
    const int32_t low = (start < goal ? start : goal) - DIRECT_PROGRESS_CORRIDOR_RAW;
    const int32_t high = (start > goal ? start : goal) + DIRECT_PROGRESS_CORRIDOR_RAW;
    if (sample < low || sample > high) {
      corridorMask |= bit; // every joint, also those too slow to stall-check
      continue;
    }
    if (!(directProbeRequiredMask & bit)) continue;
    const int32_t signedProgress = goal >= start ? sample - start : start - sample;
    const uint32_t distance = (uint32_t)(goal >= start ? goal - start : start - goal);
    uint32_t elapsed = directProbeElapsedMs[index];
    if (elapsed > directSegDurationMs) elapsed = directSegDurationMs;
    const uint32_t expected = distance * elapsed / directSegDurationMs;
    uint32_t required = expected / 4u;
    if (required < DIRECT_PROGRESS_MIN_REQUIRED_RAW)
      required = DIRECT_PROGRESS_MIN_REQUIRED_RAW;
    if (required > distance) required = distance;
    if (signedProgress < -DIRECT_PROGRESS_REVERSE_RAW) reverseMask |= bit;
    else if ((uint32_t)(signedProgress < 0 ? 0 : signedProgress) < required)
      stalledMask |= bit;
  }
  if (stalledMask && !corridorMask && !reverseMask && DirectProgressAdvisoryAllowed()) {
    WarnDirectProgress(stalledMask);
    /* This is not a progress PASS. Mark this window's axes as inspected only
       to avoid repeatedly scheduling the same adaptive probe every loop. */
    directProgressCheckedMask |= directProbeRequiredMask;
    const uint8_t stage = directProbeStage;
    if (stage == DIRECT_PROBE_START &&
        ScheduleDirectProbe(DIRECT_PROBE_MID, DirectMidProbeOffsetMs())) return;
    if (!ScheduleAdaptiveDirectProbe()) ClearDirectProbeState();
    return;
  }
  if (corridorMask || reverseMask ||
      (stalledMask && (directRecovered ||
        (directValidatedPathActive && (!directRetryCapable || directRetryUsed))))) {
    ++directProgressFaults;
    const uint8_t failed = (uint8_t)(corridorMask | reverseMask | stalledMask);
    printf("[DIRECT PROGRESS FAIL] stage=%u corridor=0x%02X reverse=0x%02X stalled=0x%02X recovered=%u sample=%d,%d,%d.\r\n",
           directProbeStage, corridorMask, reverseMask, stalledMask, directRecovered,
           directProbeRaw[0], directProbeRaw[1], directProbeRaw[2]);
    LatchVerificationFault(corridorMask ? "DIRECT_PROGRESS_CORRIDOR" :
                           (reverseMask ? "DIRECT_PROGRESS_REVERSE" :
                                          "DIRECT_NO_PROGRESS"), failed);
    return;
  }
  if (stalledMask) {
    printf("[DIRECT PROGRESS STALL] stage=%u stalled=0x%02X sample=%d,%d,%d; coordinated recovery.\r\n",
           directProbeStage, stalledMask, directProbeRaw[0], directProbeRaw[1],
           directProbeRaw[2]);
    RecoverStalledDirect(stalledMask);
    return;
  }
  ++directProgressPasses;
  directProgressCheckedMask |= directProbeRequiredMask;
  const uint8_t stage = directProbeStage;
  printf("[DIRECT PROGRESS PASS] stage=%u required_mask=0x%02X recovered=%u sample=%d,%d,%d elapsed_ms=%lu,%lu,%lu.\r\n",
         stage, directProbeRequiredMask, directRecovered, directProbeRaw[0],
         directProbeRaw[1], directProbeRaw[2],
         (unsigned long)directProbeElapsedMs[0],
         (unsigned long)directProbeElapsedMs[1],
         (unsigned long)directProbeElapsedMs[2]);
  if (stage == DIRECT_PROBE_START &&
      ScheduleDirectProbe(DIRECT_PROBE_MID, DirectMidProbeOffsetMs())) return;
  if (!ScheduleAdaptiveDirectProbe()) ClearDirectProbeState();
}

/* Explicit type25 can finish its already validated interpolation after a
   missing reply. This is NOT a successful probe or a physical snapshot. All
   final reads remain mandatory before retiring the endpoint/starting another.
   Never use this while acquiring/confirming a recovery STOP or stable origin. */
static uint8_t DeferDirectMissingRead(const char *reason, uint8_t missingMask) {
  if (!DirectProgressAdvisoryAllowed() || !directProbeTimeoutMask) return 0u;
  if (directProbeMalformed) {
    printf("[DIRECT WARNING BLOCKED] seq=%u stage=%u reason=INVALID_REPLY known=0x%02X timeout=0x%02X.\r\n",
           localPointSequence,directProbeStage,directProbeKnownMask,directProbeTimeoutMask);
    return 0u;
  }
  uint8_t stalledMask=0u;
  /* Lack of positive progress is advisory too, including when a different
     axis times out. Genuine reversal/range/corridor violations still stop. */
  for (uint8_t i=0u;i<3u;++i) {
    if (!(directProbeKnownMask & (uint8_t)(1u<<i))) continue;
    const int32_t start=directSegStartRaw[i], goal=directGoalRaw[i], sample=directProbeRaw[i];
    if (!DirectHoldRawValid(i,directProbeRaw[i])) {
      printf("[DIRECT WARNING BLOCKED] seq=%u stage=%u reason=RANGE_CORRIDOR id=%u raw=%ld.\r\n",
             localPointSequence,directProbeStage,servoIds[i],(long)sample);
      return 0u;
    }
    if (directProbeRequiredMask & (uint8_t)(1u<<i)) {
      const int32_t progress=goal>=start ? sample-start : start-sample;
      if (progress < -DIRECT_PROGRESS_REVERSE_RAW) {
        printf("[DIRECT WARNING BLOCKED] seq=%u stage=%u reason=REVERSE id=%u origin=%ld raw=%ld goal=%ld.\r\n",
               localPointSequence,directProbeStage,servoIds[i],(long)start,(long)sample,(long)goal);
        return 0u;
      }
      const uint32_t distance=(uint32_t)(goal>=start ? goal-start : start-goal);
      const uint32_t elapsed=directProbeElapsedMs[i]>directSegDurationMs ?
        directSegDurationMs : directProbeElapsedMs[i];
      uint32_t required=(distance*elapsed/directSegDurationMs)/4u;
      if (required<DIRECT_PROGRESS_MIN_REQUIRED_RAW) required=DIRECT_PROGRESS_MIN_REQUIRED_RAW;
      if (required>distance) required=distance;
      if ((uint32_t)(progress<0 ? 0 : progress)<required) stalledMask|=(uint8_t)(1u<<i);
    }
  }
  if (stalledMask) WarnDirectProgress(stalledMask);
  ++directReadWarnings;
  printf("[DIRECT READ WARNING] seq=%u stage=%u reason=%s timeout_mask=0x%02X unobserved=0x%02X; continua MISMO destino, lectura FINAL obligatoria; NO_PASS.\r\n",
    localPointSequence,directProbeStage,reason,directProbeTimeoutMask,missingMask);
  directReadDeferred=1u;
  ClearDirectProbeState();
  return 1u;
}

/* A retry cannot start a new budget, renew permits or reuse a previous pose. */
static uint8_t DirectProbeReadBudget(uint32_t now, uint32_t *remaining) {
  if (!directProbeWindowKnown) {
    directProbeWindowKnown = 1u;
    directProbeWindowStartedMs = now;
  }
  const uint32_t elapsed = now - directProbeWindowStartedMs;
  if (elapsed >= DirectReadBudgetMs() - 1u) {
    if (DeferDirectMissingRead("RX_TIMEOUT_BUDGET",
        (uint8_t)(0x07u & (uint8_t)~directProbeKnownMask))) return 0u;
    ++directReadBudgetFaults;
    ++directProgressFaults;
    printf("[DIRECT READ BUDGET] stage=%u age_ms=%lu known=0x%02X; STOP.\r\n",
           directProbeStage, (unsigned long)elapsed, directProbeKnownMask);
    LatchVerificationFault("DIRECT_PROGRESS_READ_BUDGET",
                          (uint8_t)(0x07u & (uint8_t)~directProbeKnownMask));
    return 0u;
  }
  *remaining = DirectReadBudgetMs() - elapsed;
  return 1u;
}

static void ServiceDirectProgressProbe(void) {
  if (!directProbePending) return;
  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - directProbeDueMs) < 0) return;
  if (directProbeAxis >= 3u) {
    EvaluateDirectProgress();
    return;
  }
  uint32_t remaining;
  /* Enforce the same deadline during the retry gap, not only when querying. */
  if (directProbeWindowKnown && !DirectProbeReadBudget(now, &remaining)) return;
  if ((int32_t)(now - directProbeNextReadMs) < 0) return;
  if (!LocalPointGuards()) {
    LatchVerificationFault("DIRECT_PROGRESS_PERMIT_LOST", 0x07u);
    return;
  }
  const uint8_t bit = (uint8_t)(1u << directProbeAxis);
  if (!DirectProbeReadBudget(now, &remaining)) return;
  ++directProbeReadAttempts[directProbeAxis];
  ++directReadAttempts;
  /* Drain late/partial responses and require silence before sending POS_READ.
     Idle failure consumes an attempt, sends no query and publishes no pose. */
  uint16_t drained = 0u;
  const uint32_t idleBudget = remaining < DIRECT_BUS_IDLE_MAX_MS ? remaining : DIRECT_BUS_IDLE_MAX_MS;
  const HAL_StatusTypeDef idle = ServoWaitBusIdle(
    servoPorts[directProbeAxis], DIRECT_BUS_QUIET_MS, idleBudget, &drained);
  directReadBusDrainedBytes += drained;
  if (idle != HAL_OK) ++directReadBusBusy;
  now = HAL_GetTick();
  if (!LocalPointGuards()) {
    LatchVerificationFault("DIRECT_PROGRESS_PERMIT_LOST", 0x07u);
    return;
  }
  if (!DirectProbeReadBudget(now, &remaining)) return;
  ServoPositionReading reading;
  memset(&reading, 0, sizeof(reading));
  ServoReadStatus status = SERVO_READ_TX_ERROR;
  if (idle == HAL_OK) {
    const uint32_t readBudget = remaining < DIRECT_PROGRESS_READ_TIMEOUT_MS ?
      remaining : DIRECT_PROGRESS_READ_TIMEOUT_MS;
    /* Driver drain and TX each use txBudget, inside the overall readBudget. */
    const uint32_t txBudget = readBudget / 2u < DIRECT_TX_TIMEOUT_MS ?
      readBudget / 2u : DIRECT_TX_TIMEOUT_MS;
    status = ServoReadPositionBounded(
      servoPorts[directProbeAxis], servoIds[directProbeAxis], &reading,
      readBudget, txBudget);
  }
  if (status == SERVO_READ_RX_TIMEOUT) directProbeTimeoutMask |= bit;
  else if (status != SERVO_READ_OK) directProbeMalformed=1u;
  now = HAL_GetTick();
  if (!LocalPointGuards()) {
    LatchVerificationFault("DIRECT_PROGRESS_PERMIT_LOST", 0x07u);
    return;
  }
  if (!DirectProbeReadBudget(now, &remaining)) return;
  if (status != SERVO_READ_OK) {
    if (directProbeReadAttempts[directProbeAxis] < DIRECT_PROGRESS_READ_ATTEMPTS) {
      ++directReadRetries;
      printf("[DIRECT READ RETRY] stage=%u id=%u attempt=%u/%u status=%s idle=%u read_ms=%lu\r\n",
             directProbeStage, servoIds[directProbeAxis], directProbeReadAttempts[directProbeAxis],
             (unsigned)DIRECT_PROGRESS_READ_ATTEMPTS, ServoReadStatusName(status),
             (unsigned)idle, (unsigned long)reading.elapsedMs);
      directProbeNextReadMs = now + DIRECT_PROGRESS_RETRY_GAP_MS;
      return;
    }
    if (DeferDirectMissingRead("RX_TIMEOUT_ATTEMPTS",bit)) return;
    ++directReadExhausted;
    ++directProgressFaults;
    printf("[DIRECT PROGRESS READ FAIL] id=%u status=%s attempts=%u; STOP.\r\n",
           servoIds[directProbeAxis], ServoReadStatusName(status), directProbeReadAttempts[directProbeAxis]);
    LatchVerificationFault("DIRECT_PROGRESS_READ", bit);
    return;
  }
  if (directProbeReadAttempts[directProbeAxis] > 1u) {
    ++directReadRecovered;
    printf("[DIRECT READ RECOVERED] stage=%u id=%u attempts=%u raw=%d; sin reenviar MOVE.\r\n",
           directProbeStage, servoIds[directProbeAxis], directProbeReadAttempts[directProbeAxis],
           reading.position);
  }
  directProbeRaw[directProbeAxis] = reading.position;
  directProbeElapsedMs[directProbeAxis] = now - directSegStartMs;
  directProbeKnownMask |= bit;
  ++directProbeAxis;
  directProbeNextReadMs = now;
}

static uint8_t LocalPointGuards(void) {
  const uint32_t now=HAL_GetTick();
  if (!armed || !homeValid || !rawKnown || !streamActive || faultLatched || linkStale || dryRunMode ||
      verificationPending || checkpointPending ||
      now-lastValidFrameMs>LINK_TIMEOUT_MS || now-lastStreamSetpointMs>STREAM_TIMEOUT_MS ||
      !manualGripRuntime || !automaticGripRuntime || !GripperManual_Active() ||
      GripperManual_UpdateAgeMs() > GRIP_WATCHDOG_MS) return 0u; // RC lease, not only Tick
  for (uint8_t i=0u;i<3u;++i)
    if ((uint16_t)GripperManual_Pulse(i+1u)!=localPointGrip[i]) return 0u;
  return 1u;
}

static void ServiceLocalPoint(void) {
  if (!localPointActive) return;
  if (!LocalPointGuards()) {
    LatchVerificationFault("LOCAL_POINT_PERMIT_LOST",0x07u); return;
  }
  const uint32_t now=HAL_GetTick(), elapsed=now-localPointTick;
  if (localPoint.elapsedMs==localPoint.durationMs) {
    if (elapsed>=DELTA_POINT_SLICE_MS) {
      localPointActive=0u;busy=0u;
      /* Model end only. The host must still request the physical checkpoint. */
      printf("[LOCAL POINT END] seq=%u raw=%u,%u,%u; pending physical PASS.\r\n",
        localPointSequence,commandedRaw[0],commandedRaw[1],commandedRaw[2]);
    }
    return;
  }
  if (elapsed<DELTA_POINT_PERIOD_MS) return;
  if (elapsed>DELTA_POINT_SLICE_MS) {
    LatchVerificationFault("LOCAL_POINT_SCHEDULE",0x07u); return; // no burst catch-up
  }
  const uint16_t next=(uint16_t)(localPoint.elapsedMs+DELTA_POINT_PERIOD_MS);
  uint16_t raw[3];
  for (uint8_t i=0u;i<3u;++i) {
    raw[i]=deltaPointRaw(&localPoint,i,next);
    if (raw[i]<ActiveRawMin() || raw[i]>ActiveRawMax() ||
        AbsDiff(raw[i],commandedRaw[i])>DELTA_POINT_MAX_STEP_RAW) {
      LatchVerificationFault("LOCAL_POINT_RAW_STEP",0x07u); return;
    }
  }
  if (!ExecuteTargets(raw,DELTA_POINT_SLICE_MS)) {
    LatchVerificationFault("LOCAL_POINT_TX",0x07u); return;
  }
  memcpy(commandedRaw,raw,sizeof(commandedRaw)); // last short slice, NEVER distant endpoint
  localPoint.elapsedMs=next;
  localPointTick=HAL_GetTick();busy=1u;busyUntilMs=localPointTick+DELTA_POINT_SLICE_MS;
  lastStreamMotionMs=localPointTick;checkpointReassertNeeded=1u;
}

static void FinishEndpointStep(const char *reason) {
  endpointStepActive=directPointActive=busy=0u;
  printf("[ENDPOINT STEP END] reason=%s progress=%u/%u; final physical checkpoint REQUIRED, not PASS.\r\n",
    reason,endpointStepIndex,endpointStepCount);
}

static void ServiceEndpointStep(void) {
  if(!LocalPointGuards()){LatchVerificationFault("ENDPOINT_STEP_PERMIT",0x07u);return;}
  uint32_t now=HAL_GetTick();
  if((int32_t)(now-directEndMs)>=0){FinishEndpointStep("BUDGET_EXHAUSTED");return;}
  if((int32_t)(now-endpointStepDueMs)<0)return;
  if(endpointStepPhase==0u){
    if(now-endpointStepReadStartedMs>DELTA_DIRECT_RETRY_ORIGIN_MAX_AGE_MS){
      FinishEndpointStep("STALE_ORIGIN");return;
    }
    /* No accumulated error: select the furthest preflighted chord point no
       more than4 RAW from EVERY fresh measured joint. Never skip ahead when
       one joint stays behind. Existing leases still gate each individual TX. */
    if(!endpointStepRepeats){
      uint16_t next=(uint16_t)(endpointStepIndex+DELTA_ENDPOINT_STEP_RAW);
      if(next>endpointStepCount)next=endpointStepCount;
      uint8_t fits=0u;
      for(;next>endpointStepIndex;--next){
        deltaEndpointStepTarget(endpointStepStart,endpointStepGoal,next,endpointStepCount,endpointStepTarget);
        fits=1u;
        for(uint8_t i=0u;i<3u;++i)
          if(AbsDiff(endpointStepTarget[i],(uint16_t)endpointStepSample[i])>DELTA_ENDPOINT_STEP_RAW)fits=0u;
        if(fits)break;
      }
      if(!fits){FinishEndpointStep("NO_SAFE_NEXT_SLICE");return;}
      endpointStepIndex=next;
      endpointStepTxMask=directWriteMask;
    }
    const uint32_t txStartedMs=HAL_GetTick();
    const uint8_t copies=directRepeat5Capable ? DIRECT_AUTO_RECOVERY_COPIES : 1u;
    const uint32_t txBudgetMs=DirectTransactionBudgetMs(copies);
    for(uint8_t copy=0u;copy<copies;++copy){
     for(uint8_t i=0u;i<3u;++i){
      endpointStepPrevious[i]=endpointStepSample[i];
      const uint8_t bit=(uint8_t)(1u<<i);
      if(!(endpointStepTxMask&bit))continue;
      if(!LocalPointGuards() || !DirectHoldRawValid(i,(int16_t)endpointStepTarget[i]) ||
         AbsDiff(endpointStepTarget[i],(uint16_t)endpointStepSample[i])>DELTA_ENDPOINT_STEP_RAW){
        LatchVerificationFault("ENDPOINT_STEP_ORIGIN",bit);return;
      }
      const uint32_t txAge=HAL_GetTick()-txStartedMs;
      if(txAge>=txBudgetMs){LatchVerificationFault("ENDPOINT_STEP_TX_BUDGET",bit);return;}
      const HAL_StatusTypeDef sent=DirectWriteBudgeted(i,endpointStepTarget[i],DELTA_ENDPOINT_STEP_SLICE_MS,txBudgetMs-txAge);
      if(directWriteAttempted)directStartedMask|=bit;
      if(sent!=HAL_OK){LatchVerificationFault("ENDPOINT_STEP_TX",bit);return;}
      directStartedMask|=bit;servoCommandRaw[i]=endpointStepTarget[i];
      if(HAL_GetTick()-txStartedMs>txBudgetMs){LatchVerificationFault("ENDPOINT_STEP_TX_BUDGET",bit);return;}
     }
    }
    verifiedSnapshotKnown=0u;
    if(!LocalPointGuards()){LatchVerificationFault("ENDPOINT_STEP_PERMIT",0x07u);return;}
    endpointStepPhase=1u;endpointStepAxis=endpointStepReadAttempts=0u;
    endpointStepDueMs=HAL_GetTick()+DELTA_ENDPOINT_STEP_SLICE_MS;
    endpointStepReadStartedMs=endpointStepDueMs;
    return;
  }
  /* One channel per loop; fine-step RX keeps the original120ms window. Neither a
     timeout nor an invalid frame permits another slice. */
  const uint8_t i=endpointStepAxis,bit=(uint8_t)(1u<<i);
  const uint32_t age=now-endpointStepReadStartedMs;
  if(age>=DIRECT_PROGRESS_READ_BUDGET_MS){LatchVerificationFault("ENDPOINT_STEP_READ_BUDGET",bit);return;}
  const uint32_t remaining=DIRECT_PROGRESS_READ_BUDGET_MS-age;
  uint16_t drained=0u;
  const uint32_t idleBudget=remaining<DIRECT_BUS_IDLE_MAX_MS?remaining:DIRECT_BUS_IDLE_MAX_MS;
  const HAL_StatusTypeDef idle=ServoWaitBusIdle(servoPorts[i],DIRECT_BUS_QUIET_MS,idleBudget,&drained);
  if(!LocalPointGuards()){LatchVerificationFault("ENDPOINT_STEP_PERMIT",bit);return;}
  now=HAL_GetTick();
  if(now-endpointStepReadStartedMs>=DIRECT_PROGRESS_READ_BUDGET_MS){LatchVerificationFault("ENDPOINT_STEP_READ_BUDGET",bit);return;}
  const uint32_t left=DIRECT_PROGRESS_READ_BUDGET_MS-(now-endpointStepReadStartedMs);
  const uint32_t timeout=left<DIRECT_PROGRESS_READ_TIMEOUT_MS?left:DIRECT_PROGRESS_READ_TIMEOUT_MS;
  ServoPositionReading reading;memset(&reading,0,sizeof(reading));
  const ServoReadStatus status=idle==HAL_OK?
    ServoReadPositionBounded(servoPorts[i],servoIds[i],&reading,timeout,
      timeout/2u<DIRECT_TX_TIMEOUT_MS?timeout/2u:DIRECT_TX_TIMEOUT_MS):SERVO_READ_RX_TIMEOUT;
  if(!LocalPointGuards()){LatchVerificationFault("ENDPOINT_STEP_PERMIT",bit);return;}
  if(HAL_GetTick()-endpointStepReadStartedMs>DIRECT_PROGRESS_READ_BUDGET_MS){LatchVerificationFault("ENDPOINT_STEP_READ_BUDGET",bit);return;}
  ++endpointStepReadAttempts;
  if(status!=SERVO_READ_OK){
    if(status==SERVO_READ_RX_TIMEOUT && endpointStepReadAttempts<DIRECT_PROGRESS_READ_ATTEMPTS){
      endpointStepDueMs=HAL_GetTick()+DIRECT_PROGRESS_RETRY_GAP_MS;return;
    }
    LatchVerificationFault("ENDPOINT_STEP_READ",bit);return;
  }
  const int16_t direction=endpointStepGoal[i]>=endpointStepStart[i]?1:-1;
  if(!DirectHoldRawValid(i,reading.position) ||
     direction*(reading.position-endpointStepPrevious[i]) < -(int16_t)DIRECT_PROGRESS_REVERSE_RAW){
    LatchVerificationFault("ENDPOINT_STEP_CORRIDOR_REVERSE",bit);return;
  }
  endpointStepSample[i]=reading.position;
  if(++endpointStepAxis<3u){endpointStepReadAttempts=0u;endpointStepDueMs=HAL_GetTick();return;}
  if(endpointStepPhase==2u){ // initial fresh scan, before the FIRST short MOVE
    endpointStepPhase=0u;endpointStepDueMs=HAL_GetTick();return;
  }
  uint8_t inside=1u,stalled=0u;
  for(uint8_t j=0u;j<3u;++j){
    const uint16_t error=AbsDiff((uint16_t)endpointStepSample[j],endpointRetryNominal[j]);
    if(error>endpointRetryToleranceRaw ||
       AbsDiff(servoCommandRaw[j],endpointRetryNominal[j])>AUTO_CORRECTION_MAX_ERROR_RAW)inside=0u;
    const uint16_t requested=AbsDiff(endpointStepTarget[j],(uint16_t)endpointStepPrevious[j]);
    if((directWriteMask&(1u<<j)) && error>endpointRetryToleranceRaw &&
       (AbsDiff(endpointStepTarget[j],(uint16_t)endpointStepSample[j])>5u ||
        (requested>=3u && (endpointStepGoal[j]>=endpointStepStart[j]?1:-1)*
          (endpointStepSample[j]-endpointStepPrevious[j])<1)))
      stalled|=(uint8_t)(1u<<j);
  }
  if(inside){FinishEndpointStep(endpointRetryToleranceRaw==DELTA_ENDPOINT_CENTER_BRIDGE_RAW ?
    "WITHIN12_REQUIRES_CP5":"WITHIN15_REQUIRES_CP");return;}
  if(stalled){
    if(endpointStepRepeats>=DELTA_ENDPOINT_STEP_REPEATS){FinishEndpointStep("NO_PROGRESS");return;}
    ++endpointStepRepeats;endpointStepTxMask=stalled;
    printf("[ENDPOINT STEP REPEAT] mask=0x%02X attempt=%u/2; same short target, no advance.\r\n",stalled,endpointStepRepeats);
  }else endpointStepRepeats=0u;
  endpointStepPhase=0u;endpointStepDueMs=HAL_GetTick();
}

static void ServiceDirectPoint(void) {
  if (!directPointActive) return;
  if(endpointStepActive){ServiceEndpointStep();return;}
  if (!LocalPointGuards()) {
    LatchVerificationFault("DIRECT_MOVE_PERMIT_LOST",0x07u); return;
  }
  if (directRetryPhase) {
    if (HAL_GetTick() - directRetryStartedMs > DELTA_DIRECT_RETRY_TIMEOUT_MS ||
        (directRetryPhase == 4u &&
         HAL_GetTick() - directRetryOriginMs > DELTA_DIRECT_RETRY_ORIGIN_MAX_AGE_MS)) {
      LatchVerificationFault("DIRECT_RETRY_TIMEOUT", 0x07u); return;
    }
    if (directRetryPhase == 1u) {
      if (directAbortActive || directStartedMask || directHoldPendingMask) return;
      directRetryPhase = 2u;
      ClearDirectProbeState();
      directProbePending = 1u;
      directProbeDueMs = directProbeNextReadMs = HAL_GetTick();
    }
    if (directRetryPhase == 2u || directRetryPhase == 3u) ServiceDirectProgressProbe();
    return; // never mistake an interrupted endpoint's timer for completion
  }
  ServiceDirectProgressProbe();
  if (!directPointActive || faultLatched || directProbePending || directRetryPhase) return;
  if ((int32_t)(HAL_GetTick()-directEndMs) < 0) return;
  directPointActive=0u;busy=0u;
  printf("[DIRECT MOVE END] seq=%u raw=%u,%u,%u; pending physical PASS.\r\n",
    localPointSequence,commandedRaw[0],commandedRaw[1],commandedRaw[2]);
}

static uint8_t ExecuteFrameTargets(const DeltaLinkFrame *frame) {
  /* DeltaLinkFrame esta packed; copiar evita accesos uint16_t desalineados. */
  uint16_t raw[3];
  memcpy(raw, frame->raw, sizeof(raw));
  return ExecuteTargets(raw, frame->moveTimeMs);
}

static void Reject(const DeltaLinkFrame *frame, DeltaResultCode result) {
  ++rejectedFrames;
  SendResponse(DELTA_MSG_NACK, frame, result, 1u);
  printf("[STREAM NACK] type=%u seq=%u result=%u\r\n",
         frame->type, frame->sequence, (unsigned)result);
}

static void ResetForSession(const DeltaLinkFrame *frame) {
  directRepeat5Capable=0u;
  approachStep5Capable=0u;
  localPointCapable=directPointCapable=directTransactionCapable=0u;
  directValidatedPathCapable=directValidatedPathActive=0u;
  directLiftDelayCapable=directLiftProbeActive=directProfile250Capable=0u;
  directRetryCapable=directRetryUsed=directRetryPhase=0u;
  automaticStopCapable = 0u;
  auto15Rec30Capable = 0u;
  observeCapable = 0u;
  directFinalReadCapable = directFinalReadActive = directReadDeferred = 0u;
  endpointRetryCapable=endpointRetryEligible=endpointRetryCount=0u;directWriteMask=0x07u;
  endpointRetryLimitRaw=0u;directEndpointAwaitingRecovery=0u;
  endpointCenterCapable=centerPrecisionRequired=pickupFine20Capable=0u;endpointRetryToleranceRaw=DELTA_ENDPOINT_RETRY_TOLERANCE_RAW;
  endpointStepCapable=endpointStepActive=0u;
  recoveryPending = 0u; // HELLO cannot clear the latch or inherit recovery.
  checkpointPending = 0u;
  ClearCheckpointCorrection();
  checkpointOutputOffsetKnown = 0u;
  verifiedSnapshotKnown = 0u;
  encoderTelemetrySequence = 0u;
  StopGripperOutputs();
  CancelVerification();
  activeSession = frame->session;
  lastSequence = frame->sequence;
  sequenceKnown = 1u;
  lastValidFrameMs = HAL_GetTick();
  lastStreamSetpointMs = 0u;
  linkStale = 0u;
  armed = 0u;
  homeValid = 0u;
  homePending = 0u;
  busy = 0u;
  streamActive = 0u;
  activeStreamOption = 0u;
  rawKnown = 0u;
  streamBaselineKnown = 0u;
  streamCorrectionAttempted = 0u;
  xyzCorrectionRounds = 0u;
  ResetCompensation();
  verificationReadRetryAttempted = 0u;
  memset(streamBaselineRaw, 0, sizeof(streamBaselineRaw));
  memset(measuredKnown, 0, sizeof(measuredKnown));
  lastResponseValid = 0u;
}

static void ProcessFrame(const DeltaLinkFrame *frame) {
  ++framesReceived;
  if (frame->sofA != DELTA_LINK_SOF_A || frame->sofB != DELTA_LINK_SOF_B)
    return;
  if (frame->version != DELTA_LINK_VERSION) {
    ++versionErrors;
    SendResponse(DELTA_MSG_NACK, frame, DELTA_RESULT_BAD_VERSION, 0u);
    return;
  }
  if (!deltaFrameValid(frame)) { ++crcErrors; return; }
  ++framesValid;

  if (frame->type == DELTA_MSG_HELLO) {
    if (frame->session == 0u) {
      ++sessionErrors;
      SendResponse(DELTA_MSG_NACK, frame, DELTA_RESULT_BAD_SESSION, 0u);
      return;
    }
    ResetForSession(frame);
    SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
    printf("[STREAM LINK] HELLO v3 session=%08lX; seguro y desarmado.\r\n",
           (unsigned long)activeSession);
    return;
  }

  if (activeSession == 0u || frame->session != activeSession) {
    ++sessionErrors;
    SendResponse(DELTA_MSG_NACK, frame, DELTA_RESULT_BAD_SESSION, 0u);
    return;
  }
  if (sequenceKnown && frame->sequence == lastSequence) {
    ++duplicateFrames;
    if (lastResponseValid) TransmitLinkFrame(&lastResponse);
    return;
  }
  if (sequenceKnown && !SequenceIsNewer(frame->sequence, lastSequence)) {
    ++sequenceErrors;
    SendResponse(DELTA_MSG_NACK, frame, DELTA_RESULT_BAD_SEQUENCE, 0u);
    return;
  }

  lastSequence = frame->sequence;
  sequenceKnown = 1u;
  lastValidFrameMs = HAL_GetTick();
  linkStale = 0u;

  if (recoveryPending && frame->type != DELTA_MSG_HEARTBEAT &&
      frame->type != DELTA_MSG_STATUS_REQUEST && frame->type != DELTA_MSG_DISARM &&
      frame->type != DELTA_MSG_GRIP_STATE_REQUEST) {
    Reject(frame, DELTA_RESULT_BUSY);
    return;
  }

  /* Calibration remains exclusive. A runtime permit belongs to this STREAM only. */
  if (GripperManual_Active() &&
      (frame->type == DELTA_MSG_HOME || frame->type == DELTA_MSG_MOVE ||
       (!manualGripRuntime && (frame->type == DELTA_MSG_ARM ||
                               frame->type == DELTA_MSG_STREAM_BEGIN)))) {
    Reject(frame, DELTA_RESULT_BUSY);
    return;
  }
  switch (frame->type) {
    case DELTA_MSG_POINT_START: {
      uint16_t goal[3];memcpy(goal,frame->raw,sizeof(goal));
      if (faultLatched) Reject(frame,DELTA_RESULT_FAULT_LATCHED);
      else if (!localPointCapable || frame->status!=activeStreamOption ||
          !(activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC) ||
          (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK)!=DELTA_STREAM_OPTION_XYZ_MANUAL)
        Reject(frame,DELTA_RESULT_BAD_FRAME);
      else if (localPointActive || directPointActive || localPointKnown || centerPrecisionRequired ||
          verificationPending || checkpointPending ||
          checkpointTargetConverging || checkpointCompensationHeld || checkpointOutputOffsetKnown ||
          memcmp(servoCommandRaw,commandedRaw,sizeof(commandedRaw))!=0)
        Reject(frame,DELTA_RESULT_BUSY);
      else {
        for (uint8_t i=0u;i<3u;++i) localPointGrip[i]=(uint16_t)GripperManual_Pulse(i+1u);
        if (!LocalPointGuards()) {Reject(frame,DELTA_RESULT_DISARMED);break;}
        uint8_t valid=1u;
        for (uint8_t i=0u;i<3u;++i)
          if (goal[i]<ActiveRawMin() || goal[i]>ActiveRawMax()) valid=0u;
        if (!valid) {Reject(frame,DELTA_RESULT_RAW_RANGE);break;}
        if (!deltaPointInit(&localPoint,commandedRaw,goal,frame->moveTimeMs)) {
          Reject(frame,DELTA_RESULT_MOVE_TIME);break;
        }
        localPointKnown=localPointActive=1u;localPointSequence=frame->sequence;
        localPointTick=lastStreamSetpointMs=HAL_GetTick();
        SendResponse(DELTA_MSG_ACK,frame,DELTA_RESULT_OK,1u);
        printf("[LOCAL POINT START] seq=%u goal=%u,%u,%u duration_ms=%u slice_ms=100 step_max=4\r\n",
          localPointSequence,goal[0],goal[1],goal[2],localPoint.durationMs);
      }
      break;
    }
    case DELTA_MSG_ENDPOINT_STEP_RETRY:
    case DELTA_MSG_ENDPOINT_RETRY:
    case DELTA_MSG_DIRECT_FINAL_READ_START:
    case DELTA_MSG_DIRECT_LIFT_START:
    case DELTA_MSG_DIRECT_PATH_START:
    case DELTA_MSG_DIRECT_START: {
      const uint8_t stepRetry=frame->type==DELTA_MSG_ENDPOINT_STEP_RETRY;
      const uint8_t endpointRetry = frame->type == DELTA_MSG_ENDPOINT_RETRY || stepRetry;
      uint16_t goal[3], nominal[3];memcpy(nominal,frame->raw,sizeof(nominal));
      memcpy(goal,nominal,sizeof(goal));
      uint8_t retryMask = 0x07u;
      if (endpointRetry) retryMask = deltaEndpointRetryPlanTolerance(verifiedRawSnapshot,nominal,goal,
        endpointRetryLimitRaw,endpointRetryToleranceRaw);
      uint8_t retryOriginValid=1u;
      if (endpointRetry)
        for(uint8_t i=0u;i<3u;++i)
          if(!DirectHoldRawValid(i,(int16_t)verifiedRawSnapshot[i])) retryOriginValid=0u;
      if (faultLatched) Reject(frame,DELTA_RESULT_FAULT_LATCHED);
      else if(centerPrecisionRequired && !endpointRetry)Reject(frame,DELTA_RESULT_BUSY);
      else if (!directPointCapable || !directTransactionCapable ||
          ((frame->type == DELTA_MSG_DIRECT_PATH_START || frame->type == DELTA_MSG_DIRECT_LIFT_START ||
            frame->type == DELTA_MSG_DIRECT_FINAL_READ_START || endpointRetry) &&
           !directValidatedPathCapable) ||
          ((frame->type == DELTA_MSG_DIRECT_LIFT_START || frame->type == DELTA_MSG_DIRECT_FINAL_READ_START || endpointRetry) &&
           !directLiftDelayCapable) ||
          ((frame->type == DELTA_MSG_DIRECT_FINAL_READ_START || endpointRetry) &&
           (!directFinalReadCapable || !directProfile250Capable || !observeCapable)) ||
          (directProfile250Capable && frame->type != DELTA_MSG_DIRECT_LIFT_START &&
           frame->type != DELTA_MSG_DIRECT_FINAL_READ_START && !endpointRetry) ||
          frame->status!=activeStreamOption ||
          !(activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC) ||
          (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK)!=DELTA_STREAM_OPTION_XYZ_MANUAL)
        Reject(frame,DELTA_RESULT_BAD_FRAME);
      else if (!DirectSnapshotMatchesCommands())
        Reject(frame,DELTA_RESULT_FEEDBACK_FAILED);
      else if (endpointRetry && (!endpointRetryCapable || !endpointRetryEligible ||
          endpointRetryCount >= DELTA_ENDPOINT_RETRY_MAX_ATTEMPTS || !retryMask || !retryOriginValid ||
          memcmp(nominal,endpointRetryNominal,sizeof(nominal)) ||
          memcmp(nominal,commandedRaw,sizeof(nominal)) ||
          HAL_GetTick()-verifiedSnapshotMs > DELTA_DIRECT_RETRY_ORIGIN_MAX_AGE_MS ||
          frame->moveTimeMs < endpointRetryDurationFloor))
        Reject(frame,DELTA_RESULT_FEEDBACK_FAILED);
      else if(endpointRetry && endpointStepCapable && (!endpointRetryWaitKnown ||
          HAL_GetTick()-endpointRetryWaitStartMs<DELTA_ENDPOINT_RETRY_WAIT_MS))
        Reject(frame,DELTA_RESULT_BUSY);
      else if(endpointRetry && endpointStepCapable && endpointRetryCount==1u && !stepRetry)
        Reject(frame,DELTA_RESULT_BAD_FRAME); // new contract: second recovery is feedback-gated
      else if(stepRetry && (!endpointStepCapable || endpointRetryCount!=1u ||
          frame->moveTimeMs<deltaEndpointStepBudget(verifiedRawSnapshot,goal)))
        Reject(frame,DELTA_RESULT_BAD_FRAME);
      else if (localPointActive || directPointActive || localPointKnown ||
          verificationPending || checkpointPending || checkpointTargetConverging ||
          directAbortActive || directHoldPendingMask ||
          (directStartedMask && !(endpointRetry && directEndpointAwaitingRecovery)))
        Reject(frame,DELTA_RESULT_BUSY); // also while a previous stop is unconfirmed
      else {
        uint8_t valid=1u;
        for (uint8_t i=0u;i<3u;++i) {
          localPointGrip[i]=(uint16_t)GripperManual_Pulse(i+1u);
          if (goal[i]<ActiveRawMin() || goal[i]>ActiveRawMax()) valid=0u;
        }
        /* La velocidad y la sonda parten de feedback fisico, no del destino
           nominal que pudo aceptarse con una tolerancia distinta de cero. */
        const uint16_t minimum=deltaDirectMinimumMs(verifiedRawSnapshot,goal);
        if (!valid) {Reject(frame,DELTA_RESULT_RAW_RANGE);break;}
        if (!minimum || frame->moveTimeMs<minimum ||
            (directProfile250Capable && frame->moveTimeMs<DELTA_DIRECT_PROFILE250_MIN_DURATION_MS) ||
            (approachStep5Capable && !stepRetry &&
             (frame->type==DELTA_MSG_DIRECT_FINAL_READ_START || endpointRetry) &&
             frame->moveTimeMs<DELTA_POST_PICKUP_MIN_DURATION_MS) ||
            frame->moveTimeMs>DELTA_DIRECT_MAX_DURATION_MS || frame->moveTimeMs%50u) {
          Reject(frame,DELTA_RESULT_MOVE_TIME);break;
        }
        directRetryUsed=directRetryPhase=0u;
        endpointRetryWaitKnown=0u;
        directEndpointAwaitingRecovery=0u;
        directFinalReadActive = frame->type == DELTA_MSG_DIRECT_FINAL_READ_START || endpointRetry;
        directReadDeferred=0u;
        localPointSequence=frame->sequence;directPointDuration=frame->moveTimeMs;
        const uint8_t initialCopies = endpointRetry ?
          (directRepeat5Capable ? DIRECT_AUTO_RECOVERY_COPIES : 1u) :
          (frame->type == DELTA_MSG_DIRECT_LIFT_START ||
          frame->type == DELTA_MSG_DIRECT_FINAL_READ_START)
          ? (directRepeat5Capable ? DIRECT_AUTO_INITIAL_COPIES : DIRECT_LIFT_INITIAL_COPIES) : 1u;
        directWriteMask = endpointRetry ? retryMask : 0x07u;
        if (endpointRetry) ++endpointRetryCount;
        else {
          endpointRetryCount=0u;
          endpointRetryToleranceRaw=DELTA_ENDPOINT_RETRY_TOLERANCE_RAW;
          endpointRetryEligible=frame->type == DELTA_MSG_DIRECT_FINAL_READ_START;
          memcpy(endpointRetryNominal,nominal,sizeof(nominal));
          endpointRetryDurationFloor=frame->moveTimeMs;
        }
        memcpy(directStartRaw,verifiedRawSnapshot,sizeof(directStartRaw));
        memcpy(directGoalRaw,goal,sizeof(directGoalRaw));
        memcpy(directSegStartRaw,directStartRaw,sizeof(directSegStartRaw));
        directSegDurationMs=directPointDuration;
        directHoldMin=ActiveRawMin();directHoldMax=ActiveRawMax();
        if(stepRetry){
          ClearCheckpointCorrection();checkpointOutputOffsetKnown=0u;
          ResetCompensation();checkpointReassertNeeded=0u;
          for(uint8_t i=0u;i<3u;++i){
            endpointStepStart[i]=verifiedRawSnapshot[i];endpointStepGoal[i]=goal[i];
            endpointStepSample[i]=endpointStepPrevious[i]=(int16_t)verifiedRawSnapshot[i];
          }
          endpointStepCount=deltaEndpointStepCount(endpointStepStart,endpointStepGoal);
          endpointStepIndex=0u;endpointStepRepeats=0u;endpointStepPhase=2u;
          endpointStepAxis=endpointStepReadAttempts=0u;
          endpointStepDueMs=HAL_GetTick();endpointStepActive=directPointActive=localPointKnown=1u;
          endpointStepReadStartedMs=endpointStepDueMs;
          directFinalReadActive=1u;ClearDirectProbeState();
          directEndMs=HAL_GetTick()+directPointDuration;busy=1u;busyUntilMs=directEndMs;
          lastStreamSetpointMs=HAL_GetTick();verifiedSnapshotKnown=0u;
          memcpy(commandedRaw,nominal,sizeof(commandedRaw));
          SendResponse(DELTA_MSG_ACK,frame,DELTA_RESULT_OK,1u);
          printf("[ENDPOINT STEP START] seq=%u mask=0x%02X budget_ms=%u max_step=4 slice_ms=100; real feedback before EVERY next slice.\r\n",
            localPointSequence,retryMask,directPointDuration);
          break;
        }
        directValidatedPathActive = frame->type != DELTA_MSG_DIRECT_START;
        // Only the explicitly negotiated LIFT23 may postpone its first probe.
        // Short moves retain 100 ms; never postpone both probes until the end.
        directLiftProbeActive = (frame->type == DELTA_MSG_DIRECT_LIFT_START ||
          frame->type == DELTA_MSG_DIRECT_FINAL_READ_START || endpointRetry) &&
          directPointDuration >= DIRECT_LIFT_DELAY_MIN_DURATION_MS;
        lastStreamSetpointMs=HAL_GetTick();
        uint8_t failedMask=0u;
        /* Si el permiso cae antes de entrar a la transaccion, no reutilizar el
           motivo que haya dejado un DIRECT anterior. */
        directTransactionFaultReason="DIRECT_MOVE_PERMIT_LOST";
        const uint32_t transactionStartedMs = HAL_GetTick();
        if (!LocalPointGuards() ||
            !ExecuteDirectTransaction(goal,directPointDuration,initialCopies,&failedMask)) {
          if (!failedMask) failedMask=0x07u;
          const char *failureReason = directTransactionFaultReason
            ? directTransactionFaultReason : "DIRECT_TRANSACTION_FAILED";
          const uint32_t failureElapsedMs = HAL_GetTick() - transactionStartedMs;
          const uint8_t failureStartedMask = directStartedMask;
          const uint8_t failurePwmActive = (uint8_t)GripperManual_Active();
          /* Stop motion/PWM before diagnostic output can consume UART time.
             Capture the original state above because fault latching clears it. */
          if (!faultLatched) LatchVerificationFault(failureReason, failedMask);
          else StopGripperOutputs();
          printf("[DIRECT TRANSACTION REJECT] seq=%u reason=%s failed_mask=0x%02X elapsed_ms=%lu started_mask=0x%02X pwm_active=%u.\r\n",
            frame->sequence, failureReason, failedMask,
            (unsigned long)failureElapsedMs, failureStartedMask,
            (unsigned)failurePwmActive);
          Reject(frame,DELTA_RESULT_TX_FAILED);break;
        }
        /* The write replaced the verified compensated hold on every channel. The
           old offset must not leak into the new endpoint's final checkpoint. */
        ClearCheckpointCorrection();
        checkpointOutputOffsetKnown = 0u;
        ResetCompensation();
        directPointActive=localPointKnown=1u;
        memcpy(commandedRaw,nominal,sizeof(commandedRaw));rawKnown=1u;
        localPointTick=HAL_GetTick(); /* STREAM lease stays at frame reception */
        const uint32_t initialSpanMs = localPointTick - transactionStartedMs;
        busy=1u;busyUntilMs=localPointTick+directPointDuration;
        lastStreamMotionMs=localPointTick;
        BeginDirectProgressProbe(localPointTick);
        if (endpointRetry) printf("[ENDPOINT RETRY] attempt=%u/2 mask=0x%02X nominal=%u,%u,%u effective=%u,%u,%u duration_ms=%u; same speed ceiling, final read REQUIRED.\r\n",
          endpointRetryCount,retryMask,nominal[0],nominal[1],nominal[2],
          goal[0],goal[1],goal[2],directPointDuration);
        if (directFinalReadActive) printf("[DIRECT FINAL READ POLICY] seq=%u intermediate_RX=WARNING slow_progress=WARNING final_feedback=REQUIRED; STOP/range/reverse/leases retained.\r\n",localPointSequence);
        /* The long endpoint command itself is the final assertion. A normal
           checkpoint only reads; it does not send a third duplicate motion. */
        checkpointReassertNeeded=0u;
        SendResponse(DELTA_MSG_ACK,frame,DELTA_RESULT_OK,1u);
        printf("[DIRECT MOVE START] seq=%u goal=%u,%u,%u duration_ms=%u write=MOVE_TIME_WRITE axis_mask=0x%02X copies_per_axis=%u initial_span_ms=%lu bus_busy_mask=0x%02X progress_mask=0x%02X max_raw_s=%u validated_path=%u\r\n",
          localPointSequence,goal[0],goal[1],goal[2],directPointDuration,
          directWriteMask,initialCopies,(unsigned long)initialSpanMs,directBusBusyMask,directProbeRequiredMask,
          (unsigned)DELTA_DIRECT_RAW_PER_SECOND, directValidatedPathActive);
      }
      break;
    }
    case DELTA_MSG_DIRECT_RETRY: {
      directWriteMask = 0x07u; // legacy coordinated recovery moves all joints
      /* This is not a new start: same session, original keepalive token, same
         endpoint and RC pose. Only B can approve the newly measured joint path. */
      if (!directRetryCapable || directRetryPhase != 4u || !directRetryUsed ||
          !directPointActive || !localPointKnown || !LocalPointGuards() ||
          directAbortActive || directStartedMask || directHoldPendingMask ||
          frame->status != activeStreamOption ||
          memcmp(frame->raw, directGoalRaw, sizeof(directGoalRaw)) != 0 ||
          HAL_GetTick()-directRetryOriginMs > DELTA_DIRECT_RETRY_ORIGIN_MAX_AGE_MS) {
        LatchVerificationFault("DIRECT_RETRY_INVALID", 0x07u);
        Reject(frame, DELTA_RESULT_FEEDBACK_FAILED); break;
      }
      const uint16_t minimum = deltaDirectMinimumMs(directRetryOrigin, directGoalRaw);
      if (!minimum || frame->moveTimeMs < minimum ||
          frame->moveTimeMs < directPointDuration ||
          frame->moveTimeMs < DELTA_DIRECT_PROFILE250_MIN_DURATION_MS ||
          frame->moveTimeMs > DELTA_DIRECT_MAX_DURATION_MS || frame->moveTimeMs%50u) {
        LatchVerificationFault("DIRECT_RETRY_TIME", 0x07u);
        Reject(frame, DELTA_RESULT_MOVE_TIME); break;
      }
      memcpy(directStartRaw, directRetryOrigin, sizeof(directStartRaw));
      memcpy(directSegStartRaw, directStartRaw, sizeof(directSegStartRaw));
      directPointDuration = directSegDurationMs = frame->moveTimeMs;
      uint8_t failedMask = 0u;
      if (!ExecuteDirectTransaction(directGoalRaw, directPointDuration,
              directRepeat5Capable ? DIRECT_AUTO_RECOVERY_COPIES : DIRECT_LIFT_INITIAL_COPIES, &failedMask)) {
        LatchVerificationFault(directTransactionFaultReason, failedMask ? failedMask : 0x07u);
        Reject(frame, DELTA_RESULT_TX_FAILED); break;
      }
      directRetryPhase = 0u; // directRetryUsed stays true: never a second recovery
      memcpy(commandedRaw, directGoalRaw, sizeof(commandedRaw));
      localPointTick = HAL_GetTick();
      busy=1u; busyUntilMs=localPointTick+directPointDuration;
      lastStreamMotionMs=localPointTick;
      BeginDirectProgressProbe(localPointTick);
      ++directResends;
      SendResponse(DELTA_MSG_ACK,frame,DELTA_RESULT_OK,1u);
      printf("[DIRECT RETRY START] seq=%u endpoint=%u,%u,%u duration_ms=%u copies=%u attempt=1; final checkpoint required.\r\n",
             localPointSequence, directGoalRaw[0], directGoalRaw[1], directGoalRaw[2], directPointDuration,
             directRepeat5Capable ? DIRECT_AUTO_RECOVERY_COPIES : DIRECT_LIFT_INITIAL_COPIES);
      break;
    }
    case DELTA_MSG_POINT_KEEPALIVE:
      if (!localPointKnown || frame->raw[0]!=localPointSequence || frame->raw[1] ||
          frame->raw[2] || frame->moveTimeMs || frame->status!=activeStreamOption)
        Reject(frame,DELTA_RESULT_BAD_FRAME);
      else if (!LocalPointGuards()) {
        LatchVerificationFault("LOCAL_POINT_KEEPALIVE_EXPIRED",0x07u);
        Reject(frame,DELTA_RESULT_DISARMED);
      } else {
        lastStreamSetpointMs=HAL_GetTick();
        SendResponse(DELTA_MSG_ACK,frame,DELTA_RESULT_OK,1u);
      }
      break;
    case DELTA_MSG_FAULT_RECOVER:
      if (frame->status != DELTA_FAULT_RECOVERY_MAGIC || frame->moveTimeMs ||
          frame->raw[0] || frame->raw[1] || frame->raw[2])
        Reject(frame, DELTA_RESULT_BAD_FRAME);
      else if (!faultLatched || !faultRecoverable || dryRunMode)
        Reject(frame, DELTA_RESULT_FAULT_LATCHED);
      else if (armed || streamActive || busy || verificationPending ||
               GripperManual_Active() || manualGripRuntime)
        Reject(frame, DELTA_RESULT_BUSY);
      else {
        recoveryPending = 1u;
        recoverySequence = frame->sequence;
        recoveryStartedMs = HAL_GetTick();
        checkpointInvalidMask = 0u;
        verifiedSnapshotKnown = 0u;
        StopGripperOutputs();
        ScheduleVerification(VERIFY_FAULT_RECOVERY, 1200u);
        SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
      }
      break;
    case DELTA_MSG_GRIP_STATE_REQUEST:
      if (frame->status != GRIP_MAGIC || frame->moveTimeMs != 0u ||
          frame->raw[0] || frame->raw[1] || frame->raw[2])
        Reject(frame, DELTA_RESULT_BAD_FRAME);
      else
        SendGripperState(frame);
      break;

    case DELTA_MSG_GRIP_GROUP_BEGIN:
    case DELTA_MSG_GRIP_GROUP_SET: {
      DeltaResultCode result;
      const uint16_t us[3]={frame->raw[0],frame->raw[1],frame->raw[2]};
      const uint8_t automaticRequest = frame->status == GRIP_AUTO_MAGIC;
      const uint8_t runtimeRequest = frame->status == GRIP_STREAM_MAGIC || automaticRequest;
      const uint8_t runtimeFailure = runtimeRequest || manualGripRuntime;
      const uint8_t checkpointRenewal = checkpointPending && runtimeRequest &&
        frame->type == DELTA_MSG_GRIP_GROUP_SET && manualGripRuntime &&
        memcmp(us, checkpointGripUs, sizeof(us)) == 0;
      if((frame->status!=GRIP_MAGIC && !runtimeRequest) || frame->moveTimeMs!=0u)
        result=DELTA_RESULT_RAW_RANGE;
      else if(faultLatched) result=DELTA_RESULT_FAULT_LATCHED;
      else if(automaticRequest && !(activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC))
        result=DELTA_RESULT_DISARMED;
      else if((localPointActive || directPointActive || directEndpointAwaitingRecovery || centerPrecisionRequired) &&
              (frame->type!=DELTA_MSG_GRIP_GROUP_SET ||
              !automaticRequest || memcmp(us,localPointGrip,sizeof(us))!=0))
        result=DELTA_RESULT_BUSY;
      else if(runtimeRequest && (!armed || !streamActive || !homeValid ||
                                  (verificationPending && !checkpointRenewal) || dryRunMode))
        result=DELTA_RESULT_DISARMED;
      else if(!runtimeRequest &&
              (armed || busy || streamActive || verificationPending || dryRunMode ||
               directAbortActive || directHoldPendingMask || directStartedMask))
        result=DELTA_RESULT_BUSY;
      else if(frame->type==DELTA_MSG_GRIP_GROUP_SET &&
              (runtimeRequest!=manualGripRuntime || automaticRequest!=automaticGripRuntime))
        result=DELTA_RESULT_DISARMED;
      else {
        result=frame->type==DELTA_MSG_GRIP_GROUP_BEGIN ?
          GripperManual_GroupBegin(us) : (automaticRequest ?
            GripperManual_AutoGroupSet(us) : GripperManual_GroupSet(us));
        if(result==DELTA_RESULT_OK && frame->type==DELTA_MSG_GRIP_GROUP_BEGIN) {
          manualGripRuntime=runtimeRequest;
          automaticGripRuntime=automaticRequest;
        }
      }
      if(result!=DELTA_RESULT_OK) {
        if(runtimeFailure) LatchVerificationFault("PALA_RUNTIME_REJECTED",0x07u);
        else StopGripperOutputs();
        Reject(frame,result);
      } else SendResponse(DELTA_MSG_ACK,frame,result,1u);
      if(frame->type==DELTA_MSG_GRIP_GROUP_BEGIN || result!=DELTA_RESULT_OK)
        printf("[GRIP GROUP] runtime=%u auto=%u type=%u us=%u,%u,%u result=%u PWM=%u\r\n",
          runtimeRequest,automaticRequest,frame->type,us[0],us[1],us[2],result,GripperManual_Active());
      break;
    }
    case DELTA_MSG_GRIP_BEGIN:
    case DELTA_MSG_GRIP_SET:
    case DELTA_MSG_GRIP_END: {
      DeltaResultCode result = DELTA_RESULT_OK;
      if (frame->type == DELTA_MSG_GRIP_END) {
        StopGripperOutputs();
        checkpointCorrectionRounds = checkpointInvalidMask = 0u;
        if (checkpointPending) {
          ClearCheckpointCorrection();
          checkpointPending = 0u;
          CancelVerification();
          busy = armed = streamActive = 0u;
          activeStreamOption = 0u;
          lastStreamSetpointMs = 0u;
        }
      } else if (checkpointPending) {
        /* Checkpoints permit only identical runtime GROUP_SET renewals.
           An individual RC command must not drop PWM yet leave a later
           DELTA PASS capable of advancing the pickup sequence. */
        LatchVerificationFault("CHECKPOINT_GRIP_COMMAND", 0x07u);
        result = DELTA_RESULT_DISARMED;
      } else if (frame->status != GRIP_MAGIC || frame->moveTimeMs != 0u ||
                 frame->raw[2] != 0u) {
        result = DELTA_RESULT_RAW_RANGE;
      } else if (faultLatched) {
        result = DELTA_RESULT_FAULT_LATCHED;
      } else if (armed || busy || streamActive || verificationPending || dryRunMode ||
                 directAbortActive || directHoldPendingMask || directStartedMask) {
        result = DELTA_RESULT_BUSY;
      } else {
        result = frame->type == DELTA_MSG_GRIP_BEGIN ?
          GripperManual_Begin(frame->raw[0], frame->raw[1]) :
          GripperManual_Set(frame->raw[0], frame->raw[1]);
      }
      if (result != DELTA_RESULT_OK) {
        StopGripperOutputs();
        Reject(frame, result);
      } else {
        SendResponse(DELTA_MSG_ACK, frame, result, 1u);
      }
      if (frame->type != DELTA_MSG_GRIP_SET || result != DELTA_RESULT_OK)
        printf("[GRIP] type=%u channel=%u us=%u result=%u PWM=%u; consigna, no feedback.\r\n",
               frame->type, frame->raw[0], frame->raw[1], result,
               GripperManual_Active());
      break;
    }
    case DELTA_MSG_HEARTBEAT:
      SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
      break;

    case DELTA_MSG_HOME:
      if (faultLatched) Reject(frame, DELTA_RESULT_FAULT_LATCHED);
      else if (streamActive) Reject(frame, DELTA_RESULT_STREAM_ACTIVE);
      else if (busy) Reject(frame, DELTA_RESULT_BUSY);
      else if (directAbortActive || directHoldPendingMask || directStartedMask)
        Reject(frame, DELTA_RESULT_BUSY);
      else if (frame->raw[0] != RAW_HOME || frame->raw[1] != RAW_HOME ||
               frame->raw[2] != RAW_HOME || frame->moveTimeMs != HOME_TIME_MS) {
        Reject(frame, DELTA_RESULT_BAD_HOME);
      } else {
        streamCorrectionAttempted = 0u;
        xyzCorrectionRounds = 0u;
        ClearCheckpointCorrection();
        checkpointOutputOffsetKnown = 0u;
        ResetCompensation();
        verificationReadRetryAttempted = 0u;
        homeSegmentRetryCount = 0u;
        homeCorrectionRounds = 0u;
        homeSegmentCount = 0u;
        homeSegmentTotal = 0u;
        armed = 0u;
        homeValid = 0u;
        homePending = 1u;
        ScheduleVerification(VERIFY_HOME_CAPTURE, HOME_CAPTURE_DELAY_MS);
        SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
        printf("[STREAM HOME] recuperacion cerrada aceptada: primero se mide sin mover; despues trayectoria sincronizada en segmentos <=%d RAW con feedback hasta %u,%u,%u.\r\n",
               HOME_SEGMENT_MAX_STEP_RAW, (unsigned)RAW_HOME,
               (unsigned)RAW_HOME, (unsigned)RAW_HOME);
      }
      break;

    case DELTA_MSG_ARM:
      if (faultLatched) Reject(frame, DELTA_RESULT_FAULT_LATCHED);
      else if (streamActive) Reject(frame, DELTA_RESULT_STREAM_ACTIVE);
      else if (busy) Reject(frame, DELTA_RESULT_BUSY);
      else if (directAbortActive || directHoldPendingMask || directStartedMask)
        Reject(frame, DELTA_RESULT_BUSY);
      else if (!homeValid) Reject(frame, DELTA_RESULT_HOME_REQUIRED);
      else {
        armed = 1u;
        SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
      }
      break;

    case DELTA_MSG_DISARM:
      if (recoveryPending) {
        CancelVerification(); busy = 0u;
        SendRecoveryResult(DELTA_RESULT_FEEDBACK_FAILED);
      }
      StopGripperOutputs();
      ClearCheckpointCorrection();
      if (checkpointPending) {
        checkpointPending = 0u;
        CancelVerification();
        busy = 0u;
      }
      if (IsHomeVerification(verificationPurpose)) {
        CancelVerification();
        busy = 0u;
        homePending = 0u;
        homeValid = 0u;
        rawKnown = 0u;
        printf("[STREAM HOME] cancelado; el segmento corto actual puede terminar, no se enviaran mas.\r\n");
      }
      armed = 0u;
      streamActive = 0u;
      activeStreamOption = 0u;
      lastStreamSetpointMs = 0u;
      SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
      printf("[STREAM SAFETY] DISARM; no se generan destinos nuevos.\r\n");
      break;

    case DELTA_MSG_STREAM_BEGIN:
      if (faultLatched) Reject(frame, DELTA_RESULT_FAULT_LATCHED);
      else if (!armed) Reject(frame, DELTA_RESULT_DISARMED);
      else if (streamActive &&
               (checkpointCompensationHeld || checkpointTargetConverging))
        Reject(frame, DELTA_RESULT_STREAM_ACTIVE);
      else if (!homeValid || !rawKnown || !measuredKnown[0] ||
               !measuredKnown[1] || !measuredKnown[2]) {
        Reject(frame, DELTA_RESULT_HOME_REQUIRED);
      }
      else if (busy) Reject(frame, DELTA_RESULT_BUSY);
      else if (directAbortActive || directHoldPendingMask || directStartedMask)
        Reject(frame, DELTA_RESULT_BUSY);
      else if (!RangeProfileValid(frame->status)) {
        Reject(frame, DELTA_RESULT_BAD_RANGE_PROFILE);
      }
      else if (!StreamOptionValid(frame->status)) {
        Reject(frame, DELTA_RESULT_BAD_FRAME);
      } else {
        uint8_t validStart = 1u;
        for (uint8_t i = 0u; i < 3u; ++i) {
          if (commandedRaw[i] < ProfileRawMin(frame->status) ||
              commandedRaw[i] > ProfileRawMax(frame->status) ||
              frame->raw[i] != commandedRaw[i]) validStart = 0u;
          if (checkpointOutputOffsetKnown &&
              (servoCommandRaw[i] < ProfileRawMin(frame->status) ||
               servoCommandRaw[i] > ProfileRawMax(frame->status))) validStart = 0u;
        }
        if (!validStart) {
          Reject(frame, DELTA_RESULT_RAW_RANGE);
          break;
        }
        if ((frame->status & DELTA_STREAM_OPTION_MODE_MASK) ==
              DELTA_STREAM_OPTION_Z_ONLY && !ZOnlyAllowed(frame)) {
          Reject(frame, DELTA_RESULT_STREAM_Z_ONLY);
          break;
        }
        memcpy(streamBaselineRaw, measuredRaw, sizeof(streamBaselineRaw));
        streamBaselineKnown = 1u;
        streamCorrectionAttempted = 0u;
        xyzCorrectionRounds = 0u;
        ClearCheckpointCorrection();
        ResetCompensation();
        verificationReadRetryAttempted = 0u;
        streamActive = 1u;
        automaticStopCapable = 0u;
        auto15Rec30Capable = 0u;
        observeCapable = 0u;
        directFinalReadCapable = directFinalReadActive = directReadDeferred = 0u;
        endpointRetryCapable=endpointRetryEligible=endpointRetryCount=0u;directWriteMask=0x07u;
        endpointRetryLimitRaw=0u;directEndpointAwaitingRecovery=0u;
        endpointCenterCapable=centerPrecisionRequired=pickupFine20Capable=0u;endpointRetryToleranceRaw=DELTA_ENDPOINT_RETRY_TOLERANCE_RAW;
        endpointStepCapable=endpointStepActive=0u;
        localPointCapable=directPointCapable=directTransactionCapable=0u;
        directValidatedPathCapable=directValidatedPathActive=0u;
        directLiftDelayCapable=directLiftProbeActive=directProfile250Capable=0u;
        directRepeat5Capable=0u;
        approachStep5Capable=0u;
        directRetryCapable=directRetryUsed=directRetryPhase=0u;
        activeStreamOption = frame->status;
        lastStreamSetpointMs = HAL_GetTick();
        lastStreamMotionMs = HAL_GetTick();
        checkpointReassertNeeded = 0u;
        SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
        printf("[STREAM BEGIN] mode=%s range=%s raw=%u..%u watchdog=%u ms, baseline=%d,%d,%d.\r\n",
               StreamModeName(activeStreamOption),
               RangeProfileName(activeStreamOption),
               (unsigned)ActiveRawMin(), (unsigned)ActiveRawMax(),
               (unsigned)STREAM_TIMEOUT_MS, streamBaselineRaw[0],
               streamBaselineRaw[1], streamBaselineRaw[2]);
      }
      break;

    case DELTA_MSG_ENDPOINT_STEP_PROBE:
    case DELTA_MSG_ENDPOINT_REC200_PROBE:
    case DELTA_MSG_CENTER_RECOVERY_PROBE:
    case DELTA_MSG_CENTER_OBSERVE:
    case DELTA_MSG_PICKUP_FINE20_PROBE:
    case DELTA_MSG_APPROACH_STEPS_PROBE:
    case DELTA_MSG_DIRECT_REPEAT5_PROBE:
    case DELTA_MSG_PICKUP_FINE20_CHECKPOINT:
    case DELTA_MSG_STREAM_CHECKPOINT: {
      const uint8_t approachProbe=frame->type==DELTA_MSG_APPROACH_STEPS_PROBE;
      const uint8_t repeat5Probe=frame->type==DELTA_MSG_DIRECT_REPEAT5_PROBE || approachProbe;
      const uint8_t fine20Probe=frame->type==DELTA_MSG_PICKUP_FINE20_PROBE || repeat5Probe;
      const uint8_t fine20Checkpoint=frame->type==DELTA_MSG_PICKUP_FINE20_CHECKPOINT;
      const uint8_t centerProbe=frame->type==DELTA_MSG_CENTER_RECOVERY_PROBE || fine20Probe;
      const uint8_t centerObservation=frame->type==DELTA_MSG_CENTER_OBSERVE;
      const uint8_t recovery200 = frame->type == DELTA_MSG_ENDPOINT_REC200_PROBE ||
        frame->type == DELTA_MSG_ENDPOINT_STEP_PROBE || centerProbe;
      const uint16_t policy = frame->status & DELTA_CHECKPOINT_POLICY_MASK;
      const uint8_t auto15 = policy == DELTA_CHECKPOINT_AUTO15_REC30;
      const uint8_t negotiate15 = auto15 && (frame->status & DELTA_AUTO_STOP_POLICY);
      const uint8_t endpointCapability = policy == DELTA_ENDPOINT_RETRY_CAPABILITY;
      const uint8_t finalRead = policy == DELTA_DIRECT_FINAL_READ_CAPABILITY || endpointCapability;
      const uint8_t observe = policy == DELTA_CHECKPOINT_OBSERVE || finalRead;
      const uint8_t negotiateObserve = observe && (frame->status & DELTA_AUTO_STOP_POLICY);
      if (faultLatched) Reject(frame, DELTA_RESULT_FAULT_LATCHED);
      else if (!armed || dryRunMode) Reject(frame, DELTA_RESULT_DISARMED);
      else if (!homeValid) Reject(frame, DELTA_RESULT_HOME_REQUIRED);
      else if (!streamActive) Reject(frame, DELTA_RESULT_STREAM_REQUIRED);
      else if (verificationPending || checkpointPending || localPointActive || directPointActive)
        Reject(frame, DELTA_RESULT_BUSY);
      else if(fine20Checkpoint && (!pickupFine20Capable || centerPrecisionRequired ||
          !(activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC) ||
          frame->status!=activeStreamOption))
        Reject(frame,DELTA_RESULT_BAD_FRAME);
      else if(centerObservation && (!endpointCenterCapable || !endpointRetryEligible ||
          policy!=DELTA_CHECKPOINT_OBSERVE || negotiateObserve ||
          frame->status!=(uint16_t)(activeStreamOption|DELTA_CHECKPOINT_OBSERVE)))
        Reject(frame,DELTA_RESULT_BAD_FRAME);
      else if (recovery200 && (!endpointCapability || !negotiateObserve ||
          (frame->status & (DELTA_DIRECT_ENDPOINT_CAPABILITY | DELTA_DIRECT_TRANSACTION_CAPABILITY |
            DELTA_DIRECT_VALIDATED_PATH_CAPABILITY | DELTA_DIRECT_LIFT_DELAY_CAPABILITY |
            DELTA_DIRECT_PROFILE250_CAPABILITY)) !=
          (DELTA_DIRECT_ENDPOINT_CAPABILITY | DELTA_DIRECT_TRANSACTION_CAPABILITY |
            DELTA_DIRECT_VALIDATED_PATH_CAPABILITY | DELTA_DIRECT_LIFT_DELAY_CAPABILITY |
            DELTA_DIRECT_PROFILE250_CAPABILITY)))
        Reject(frame, DELTA_RESULT_BAD_FRAME);
      else if (finalRead && (!negotiateObserve ||
                !(frame->status & DELTA_DIRECT_PROFILE250_CAPABILITY)))
        Reject(frame, DELTA_RESULT_BAD_FRAME);
      else if ((frame->status & ~(DELTA_CHECKPOINT_POLICY_MASK | DELTA_AUTO_STOP_POLICY |
                                  DELTA_LOCAL_POINT_CAPABILITY |
                                  DELTA_DIRECT_ENDPOINT_CAPABILITY |
                                  DELTA_DIRECT_TRANSACTION_CAPABILITY |
                                  DELTA_DIRECT_VALIDATED_PATH_CAPABILITY |
                                  DELTA_DIRECT_LIFT_DELAY_CAPABILITY |
                                  DELTA_DIRECT_PROFILE250_CAPABILITY |
                                  DELTA_DIRECT_RETRY_CAPABILITY)) != activeStreamOption ||
               ((frame->status & (DELTA_LOCAL_POINT_CAPABILITY |
                                   DELTA_DIRECT_ENDPOINT_CAPABILITY |
                                   DELTA_DIRECT_TRANSACTION_CAPABILITY |
                                   DELTA_DIRECT_VALIDATED_PATH_CAPABILITY |
                                   DELTA_DIRECT_LIFT_DELAY_CAPABILITY |
                                   DELTA_DIRECT_PROFILE250_CAPABILITY |
                                   DELTA_DIRECT_RETRY_CAPABILITY)) &&
                (!(frame->status & DELTA_AUTO_STOP_POLICY) || (policy && !negotiate15 && !negotiateObserve))) ||
               ((frame->status & DELTA_DIRECT_LIFT_DELAY_CAPABILITY) &&
                !(frame->status & DELTA_DIRECT_VALIDATED_PATH_CAPABILITY)) ||
               ((frame->status & DELTA_DIRECT_PROFILE250_CAPABILITY) &&
                !(frame->status & DELTA_DIRECT_LIFT_DELAY_CAPABILITY)) ||
               ((frame->status & DELTA_DIRECT_RETRY_CAPABILITY) &&
                !(frame->status & DELTA_DIRECT_PROFILE250_CAPABILITY)) ||
               ((frame->status & DELTA_DIRECT_VALIDATED_PATH_CAPABILITY) &&
                (GripperManual_Active() ||
                 (frame->status & (DELTA_DIRECT_ENDPOINT_CAPABILITY |
                                    DELTA_DIRECT_TRANSACTION_CAPABILITY)) !=
                   (DELTA_DIRECT_ENDPOINT_CAPABILITY | DELTA_DIRECT_TRANSACTION_CAPABILITY))) ||
               ((frame->status & (DELTA_DIRECT_ENDPOINT_CAPABILITY |
                                   DELTA_DIRECT_TRANSACTION_CAPABILITY)) &&
                (frame->status & (DELTA_DIRECT_ENDPOINT_CAPABILITY |
                                   DELTA_DIRECT_TRANSACTION_CAPABILITY)) !=
                  (DELTA_DIRECT_ENDPOINT_CAPABILITY |
                   DELTA_DIRECT_TRANSACTION_CAPABILITY)) ||
               ((frame->status & DELTA_AUTO_STOP_POLICY) &&
                ((policy && !negotiate15 && !negotiateObserve) ||
                 !(activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC))) ||
               ((frame->status & DELTA_CHECKPOINT_POLICY_MASK) &&
                ((frame->status & DELTA_CHECKPOINT_POLICY_MASK) &
                 ((frame->status & DELTA_CHECKPOINT_POLICY_MASK) - 1u)) && !auto15 && !observe) ||
               (auto15 && !negotiate15 && !auto15Rec30Capable) ||
               (negotiate15 && GripperManual_Active()) ||
               (observe && !negotiateObserve && !observeCapable) ||
               (negotiateObserve && GripperManual_Active()) ||
               ((frame->status & DELTA_CHECKPOINT_POLICY_MASK) &&
                !(activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC)) ||
               frame->moveTimeMs != 0u ||
               (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) !=
                 DELTA_STREAM_OPTION_XYZ_MANUAL ||
               memcmp(frame->raw, commandedRaw, sizeof(commandedRaw)) != 0)
        Reject(frame, DELTA_RESULT_BAD_FRAME);
      else {
        checkpointPending = 1u;
        if(repeat5Probe)directRepeat5Capable=1u;
        if(approachProbe)approachStep5Capable=1u;
        if (frame->status & DELTA_AUTO_STOP_POLICY) automaticStopCapable = 1u;
        if (frame->status & DELTA_LOCAL_POINT_CAPABILITY) localPointCapable = 1u;
        if (frame->status & DELTA_DIRECT_ENDPOINT_CAPABILITY) directPointCapable = 1u;
        if (frame->status & DELTA_DIRECT_TRANSACTION_CAPABILITY)
          directTransactionCapable = 1u;
        if (frame->status & DELTA_DIRECT_VALIDATED_PATH_CAPABILITY)
          directValidatedPathCapable = 1u;
        if (frame->status & DELTA_DIRECT_LIFT_DELAY_CAPABILITY)
          directLiftDelayCapable = 1u;
        if (frame->status & DELTA_DIRECT_PROFILE250_CAPABILITY)
          directProfile250Capable = 1u;
        if (frame->status & DELTA_DIRECT_RETRY_CAPABILITY) directRetryCapable = 1u;
        if (negotiate15) auto15Rec30Capable = 1u;
        /* The observation contract includes legacy AUTO15 STOP support. */
        if (negotiateObserve) observeCapable = auto15Rec30Capable = 1u;
        if (finalRead && negotiateObserve) directFinalReadCapable=1u;
        if (endpointCapability && negotiateObserve) {
          endpointRetryCapable=1u;
          endpointStepCapable=frame->type==DELTA_MSG_ENDPOINT_STEP_PROBE || centerProbe;
          endpointCenterCapable=centerProbe;
          pickupFine20Capable=fine20Probe;
          endpointRetryLimitRaw=recovery200 ? DELTA_ENDPOINT_RETRY_MAX_ERROR_RAW :
            DELTA_ENDPOINT_RETRY_LEGACY_MAX_ERROR_RAW;
        }
        checkpointObserve = observe && !negotiateObserve;
        if(centerObservation)centerPrecisionRequired=1u;
        if(checkpointObserve)endpointRetryToleranceRaw=centerObservation ?
          DELTA_ENDPOINT_CENTER_BRIDGE_RAW : DELTA_ENDPOINT_RETRY_TOLERANCE_RAW;
        checkpointRecover30 = auto15 && !negotiate15;
        checkpointPickupFine20 = fine20Checkpoint;
        checkpointBoxToleranceRaw = (negotiate15 || negotiateObserve) ? 0u :
          (observe ? DELTA_AUTO15_TOLERANCE_RAW : (auto15 ? DELTA_AUTO15_TOLERANCE_RAW :
          ((frame->status & DELTA_CHECKPOINT_AUTO9)
          ? DELTA_CHECKPOINT_AUTO9_TOLERANCE_RAW
          : ((frame->status & DELTA_CHECKPOINT_BOX15) ? DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW
          : ((frame->status & DELTA_CHECKPOINT_BOX8) ? DELTA_CHECKPOINT_BOX_TOLERANCE_RAW : 0u)))));
        checkpointSequence = frame->sequence;
        checkpointStartedMs = HAL_GetTick();
        checkpointCorrectionRounds = checkpointInvalidMask = 0u;
        checkpointProgressReasserts = 0u;
        ResetCompensation();
        for (uint8_t i = 0u; i < 3u; ++i)
          checkpointGripUs[i] = (uint16_t)GripperManual_Pulse(i + 1u);
        verificationReadRetryAttempted = 0u;
        uint32_t delayMs = STREAM_END_SETTLE_MS;
        const int32_t remainingMs = (int32_t)(busyUntilMs - HAL_GetTick());
        if (busy && remainingMs > 0) delayMs += (uint32_t)remainingMs;
        /* Reafirmar una sola vez la ultima orden real. Los limites de
           correccion dependen del permiso AUTO; el puente sigue <=4 RAW. */
        if (!checkpointObserve && checkpointReassertNeeded && !checkpointTargetConverging) {
          uint16_t endpointRaw[3];
          memcpy(endpointRaw, servoCommandRaw, sizeof(endpointRaw));
          verificationPurpose = VERIFY_STREAM_CHECKPOINT;
          if (!ExecuteTargets(endpointRaw, CHECKPOINT_REASSERT_TIME_MS)) {
            if (!faultLatched) LatchVerificationFault("CHECKPOINT_REASSERT_TX", 0x07u);
            Reject(frame, DELTA_RESULT_TX_FAILED);
            break;
          }
          busy = 1u;
          busyUntilMs = HAL_GetTick() + CHECKPOINT_REASSERT_TIME_MS;
          delayMs = CHECKPOINT_REASSERT_TIME_MS + STREAM_END_SETTLE_MS;
          checkpointReassertNeeded = 0u;
          printf("[CHECKPOINT REASSERT] tx_raw=%u,%u,%u move_ms=%u settle_ms=%u\r\n",
                 servoCommandRaw[0], servoCommandRaw[1], servoCommandRaw[2],
                 (unsigned)CHECKPOINT_REASSERT_TIME_MS, (unsigned)STREAM_END_SETTLE_MS);
        }
        printf("[CHECKPOINT POLICY] policy_raw=%u tolerance_raw=%u correctible_raw=%u compensation_raw=%u pickup20=%u deadline_ms=%u\r\n",
               (unsigned)checkpointBoxToleranceRaw, (unsigned)(checkpointBoxToleranceRaw
                 ? checkpointBoxToleranceRaw : DELTA_CHECKPOINT_PRECISION_RAW),
               (unsigned)(checkpointPickupFine20 ? DELTA_PICKUP_FINE_MAX_ERROR_RAW :
                checkpointRecover30 ? DELTA_AUTO15_RECOVERABLE_RAW :
                 (checkpointBoxToleranceRaw == DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW
                 ? DELTA_CHECKPOINT_BOX15_MAX_ERROR_RAW
                 : ((activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC)
                    ? AUTO_CORRECTION_MAX_ERROR_RAW : XYZ_CORRECTION_MAX_ERROR_RAW))),
               (unsigned)(checkpointPickupFine20 ? DELTA_PICKUP_FINE_MAX_COMPENSATION_RAW :
                 (activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC)
                 ? AUTO_CORRECTION_MAX_ERROR_RAW : FEEDBACK_MAX_TOTAL_COMPENSATION_RAW),
               (unsigned)checkpointPickupFine20,
               (unsigned)(checkpointPickupFine20 ? DELTA_PICKUP_FINE_TIMEOUT_MS :
                 (activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC)
                 ? DELTA_AUTO_CHECKPOINT_TIMEOUT_MS : CHECKPOINT_TIMEOUT_MS));
        /* Precision5, AUTO9 and legacy BOX8/15; offset12/step5 remain. */
        ScheduleVerification(VERIFY_STREAM_CHECKPOINT, delayMs);
        SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
      }
      break;

    }
    case DELTA_MSG_APPROACH_SETPOINT:
    case DELTA_MSG_STREAM_SETPOINT:
      if (faultLatched) Reject(frame, DELTA_RESULT_FAULT_LATCHED);
      else if (!armed) Reject(frame, DELTA_RESULT_DISARMED);
      else if (!homeValid) Reject(frame, DELTA_RESULT_HOME_REQUIRED);
      else if (!streamActive) Reject(frame, DELTA_RESULT_STREAM_REQUIRED);
      else if(frame->type==DELTA_MSG_APPROACH_SETPOINT &&
          (!approachStep5Capable || frame->status!=activeStreamOption ||
           !(activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC) ||
           (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK)!=DELTA_STREAM_OPTION_XYZ_MANUAL ||
           checkpointPending || checkpointTargetConverging ||
           checkpointCompensationHeld || checkpointOutputOffsetKnown))
        Reject(frame,DELTA_RESULT_BAD_FRAME);
      else if (localPointActive || directPointActive) Reject(frame, DELTA_RESULT_BUSY);
      else if (checkpointPending &&
               memcmp(frame->raw, commandedRaw, sizeof(commandedRaw)) != 0) {
        LatchVerificationFault("CHECKPOINT_TARGET_CHANGED", 0x07u);
        Reject(frame, DELTA_RESULT_BUSY);
      }
      else if ((localPointKnown || directEndpointAwaitingRecovery || centerPrecisionRequired) &&
               memcmp(frame->raw,commandedRaw,sizeof(commandedRaw))!=0)
        Reject(frame, DELTA_RESULT_BUSY); // endpoint must get physical PASS before a new target
      else if ((frame->status & ~DELTA_STREAM_SETPOINT_COARSE) != activeStreamOption ||
               ((frame->status & DELTA_STREAM_SETPOINT_COARSE) &&
                (checkpointPending || checkpointTargetConverging ||
                 checkpointCompensationHeld || checkpointOutputOffsetKnown ||
                 (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) !=
                    DELTA_STREAM_OPTION_XYZ_MANUAL))) {
        Reject(frame, DELTA_RESULT_BAD_FRAME);
      } else if (!RawInRange(frame)) Reject(frame, DELTA_RESULT_RAW_RANGE);
      else if (!RawStepAllowed(frame)) Reject(frame, DELTA_RESULT_RAW_STEP);
      else if ((activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) ==
                 DELTA_STREAM_OPTION_Z_ONLY &&
               !ZOnlyAllowed(frame)) {
        Reject(frame, DELTA_RESULT_STREAM_Z_ONLY);
      }
      else if (frame->moveTimeMs < STREAM_TIME_MIN_MS ||
               frame->moveTimeMs > STREAM_TIME_MAX_MS) {
        Reject(frame, DELTA_RESULT_MOVE_TIME);
      } else if(frame->type==DELTA_MSG_APPROACH_SETPOINT &&
                (frame->moveTimeMs<DELTA_APPROACH_MOVE_MS ||
                 (memcmp(frame->raw,commandedRaw,sizeof(commandedRaw))!=0 &&
                  HAL_GetTick()-lastStreamMotionMs<DELTA_APPROACH_PERIOD_MS))) {
        Reject(frame,DELTA_RESULT_STREAM_RATE);
      } else if ((frame->status & DELTA_STREAM_SETPOINT_COARSE) &&
                 (frame->moveTimeMs < STREAM_COARSE_TIME_MS ||
                  HAL_GetTick() - lastStreamMotionMs < STREAM_COARSE_PERIOD_MS)) {
        Reject(frame, DELTA_RESULT_STREAM_RATE);
      } else {
        const uint8_t targetChanged =
          memcmp(frame->raw, commandedRaw, sizeof(commandedRaw)) != 0;
        /* Even a coalesced nominal change without HTS TX changes the target
           authorized by PASS. An identical lease renewal keeps it valid. */
        if (targetChanged) verifiedSnapshotKnown = 0u;
        const uint8_t boundedTransition = checkpointTargetConverging ||
          (targetChanged &&
            (checkpointCompensationHeld || checkpointOutputOffsetKnown));
        const uint8_t physicalDeltaReached =
          (targetChanged && PhysicalDeltaReached(frame)) || boundedTransition;
        uint16_t transitionRaw[3];
        if (boundedTransition) {
          /* Al abandonar una pose compensada, nominal 4 + offset 5 podria
             saltar 9 RAW. Converger desde la ultima orden real en pasos <=4,
             sin alterar la consigna; renewals completan el ultimo tramo. */
          for (uint8_t i = 0u; i < 3u; ++i) {
            int16_t delta = (int16_t)((int16_t)frame->raw[i] -
                                      (int16_t)servoCommandRaw[i]);
            if (delta > (int16_t)STREAM_RAW_MAX_STEP)
              delta = (int16_t)STREAM_RAW_MAX_STEP;
            if (delta < -(int16_t)STREAM_RAW_MAX_STEP)
              delta = -(int16_t)STREAM_RAW_MAX_STEP;
            transitionRaw[i] = (uint16_t)((int16_t)servoCommandRaw[i] + delta);
          }
        }
        if (physicalDeltaReached &&
            !(boundedTransition
                ? ExecuteTargets(transitionRaw, frame->moveTimeMs)
                : ExecuteFrameTargets(frame))) {
          StopGripperOutputs();
          ClearCheckpointCorrection();
          faultLatched = 1u;
          faultRecoverable = 0u;
          armed = 0u;
          streamActive = 0u;
          homeValid = 0u;
          Reject(frame, DELTA_RESULT_TX_FAILED);
          break;
        }
        memcpy(commandedRaw, frame->raw, sizeof(commandedRaw));
        if (memcmp(servoCommandRaw, commandedRaw, sizeof(commandedRaw)) == 0)
          checkpointOutputOffsetKnown = 0u;
        if (!checkpointPending) ResetCompensation();
        if (targetChanged) checkpointCorrectionRounds = checkpointInvalidMask = 0u;
        if (boundedTransition) {
          checkpointTargetConverging =
            memcmp(servoCommandRaw, commandedRaw, sizeof(commandedRaw)) != 0;
          checkpointCompensationHeld = 0u;
          if (checkpointPending) {
            /* Cambia solo el momento de lectura, nunca checkpointStartedMs.
               Cada tramo debe terminar y asentarse antes de tomar muestras. */
            verificationStartMs = HAL_GetTick() + frame->moveTimeMs +
                                  STREAM_END_SETTLE_MS;
          }
        }
        rawKnown = 1u;
        if (targetChanged) {
          ++changedSetpoints;
        }
        if (physicalDeltaReached) {
          lastStreamMotionMs = HAL_GetTick();
          if (!checkpointPending) checkpointReassertNeeded = 1u;
          busy = 1u;
          busyUntilMs = HAL_GetTick() + frame->moveTimeMs;
          ++physicalStreamBatches;
        } else if (targetChanged) {
          ++coalescedSetpoints;
        } else {
          ++skippedSameTargets;
        }
        lastStreamSetpointMs = HAL_GetTick();
        ++acceptedSetpoints;
        SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
        if ((acceptedSetpoints % STREAM_ACCEPT_PRINT_INTERVAL) == 0u) {
          printf("[STREAM ACCEPT] count=%lu change=%lu same=%lu coalesce=%lu stream_tx=%lu tx_batch=%lu desired=%u,%u,%u tx_raw=%u,%u,%u\r\n",
                  (unsigned long)acceptedSetpoints,
                  (unsigned long)changedSetpoints,
                  (unsigned long)skippedSameTargets,
                  (unsigned long)coalescedSetpoints,
                  (unsigned long)physicalStreamBatches,
                  (unsigned long)servoTxBatches,
                  commandedRaw[0], commandedRaw[1], commandedRaw[2],
                  servoCommandRaw[0], servoCommandRaw[1],
                  servoCommandRaw[2]);
        }
      }
      break;

    case DELTA_MSG_STREAM_END:
      if (!streamActive) Reject(frame, DELTA_RESULT_STREAM_REQUIRED);
      else if ((frame->status & ~(DELTA_AUTO_STOP_POLICY | DELTA_CHECKPOINT_POLICY_MASK)) != activeStreamOption ||
          ((frame->status & DELTA_CHECKPOINT_POLICY_MASK) && !(frame->status & DELTA_AUTO_STOP_POLICY)) ||
          ((frame->status & DELTA_CHECKPOINT_POLICY_MASK) &&
           (frame->status & DELTA_CHECKPOINT_POLICY_MASK) != DELTA_CHECKPOINT_AUTO9 &&
           (frame->status & DELTA_CHECKPOINT_POLICY_MASK) != DELTA_CHECKPOINT_BOX15 &&
           (frame->status & DELTA_CHECKPOINT_POLICY_MASK) != DELTA_CHECKPOINT_AUTO15_REC30) ||
          ((frame->status & DELTA_CHECKPOINT_POLICY_MASK) == DELTA_CHECKPOINT_AUTO15_REC30 &&
           !auto15Rec30Capable) ||
          ((frame->status & DELTA_AUTO_STOP_POLICY) &&
           (!automaticStopCapable || !(activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC))) ||
          frame->moveTimeMs != 0u) {
        Reject(frame, DELTA_RESULT_BAD_FRAME);
      }
      else {
        uint16_t finalRaw[3];
        /* Capture before StopGripperOutputs: a stopped DIRECT must END on its
           measured hold pose, never re-send the old distant endpoint. */
        const uint8_t directMotion = directPointActive || directStartedMask != 0u;
        const uint8_t holdVerifiedCompensation = checkpointCompensationHeld &&
          !checkpointPending && !verificationPending && !checkpointTargetConverging;
        memcpy(finalRaw, holdVerifiedCompensation ? servoCommandRaw : commandedRaw,
               sizeof(finalRaw));
        checkpointPending = 0u; // STOP cancels any older asynchronous result.
        ClearCheckpointCorrection();
        streamEndToleranceRaw = !(frame->status & DELTA_AUTO_STOP_POLICY) ? 0u :
          ((frame->status & DELTA_CHECKPOINT_BOX15) ? DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW :
           ((frame->status & DELTA_CHECKPOINT_AUTO9) ? DELTA_CHECKPOINT_AUTO9_TOLERANCE_RAW : DELTA_CHECKPOINT_PRECISION_RAW));
        streamEndRecover30 = (frame->status & DELTA_CHECKPOINT_POLICY_MASK) ==
          DELTA_CHECKPOINT_AUTO15_REC30;
        automaticStopStartedMs = HAL_GetTick();
        xyzCorrectionRounds = 0u;
        memset(stopPreviousError,0,sizeof(stopPreviousError));
        StopGripperOutputs(); // STOP withdraws all RC PWM before final DELTA verification.
        if (directMotion) {
          const uint8_t uncertain = (uint8_t)(directStartedMask & (uint8_t)~directHoldPendingMask);
          if (uncertain) {
            directAbortActive = 0u;
            LatchVerificationFault("DIRECT_STOP_UNCERTAIN", uncertain);
            Reject(frame, DELTA_RESULT_TX_FAILED);
            printf("[STREAM END TARGET] DIRECT sin pose de retencion valida; sin destino nuevo.\r\n");
            break;
          }
          memcpy(finalRaw, servoCommandRaw, sizeof(finalRaw)); // measured hold pose
          memcpy(commandedRaw, finalRaw, sizeof(commandedRaw));
        }
        streamActive = 0u;
        armed = 0u;
        lastStreamSetpointMs = 0u;
        if (!ExecuteTargets(finalRaw, STREAM_END_TARGET_TIME_MS)) {
          homeValid = 0u;
          rawKnown = 0u;
          faultLatched = 1u;
          faultRecoverable = 0u;
          busy = 0u;
          Reject(frame, DELTA_RESULT_TX_FAILED);
          printf("[STREAM END TARGET] fallo TX; FAULT enclavado.\r\n");
          break;
        }
        ++finalTargetFlushes;
        verificationReadRetryAttempted = 0u;
        printf("[STREAM END TARGET] desired=%u,%u,%u time_ms=%u; midiendo antes de decidir correccion.\r\n",
               commandedRaw[0], commandedRaw[1], commandedRaw[2],
               (unsigned)STREAM_END_TARGET_TIME_MS);
        ScheduleVerification(VERIFY_STREAM_END,
                             STREAM_END_TARGET_TIME_MS +
                             STREAM_END_SETTLE_MS);
        SendResponse(DELTA_MSG_ACK, frame, DELTA_RESULT_OK, 1u);
        if (streamEndToleranceRaw)
          printf("[AUTO STOP POLICY] tolerance_raw=%u correctible_raw=%d compensation_raw=%d; no resume.\r\n",
                 streamEndToleranceRaw,
                 streamEndRecover30 ? DELTA_AUTO15_RECOVERABLE_RAW :
                   (streamEndToleranceRaw == DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW ?
                     DELTA_CHECKPOINT_BOX15_MAX_ERROR_RAW : AUTO_CORRECTION_MAX_ERROR_RAW),
                 AUTO_CORRECTION_MAX_ERROR_RAW);
        printf("[STREAM END] mode=%s desarmado; verificacion fisica pendiente%s.\r\n",
               StreamModeName(activeStreamOption),
               (activeStreamOption & DELTA_STREAM_OPTION_MODE_MASK) ==
                 DELTA_STREAM_OPTION_Z_ONLY
                 ? " con correccion Z segura"
                 : " con correccion XYZ acotada por canal");
      }
      break;

    case DELTA_MSG_STATUS_REQUEST:
      SendResponse(DELTA_MSG_STATUS, frame, DELTA_RESULT_OK, 1u);
      SendFeedbackSnapshot();
      break;

    case DELTA_MSG_MOVE:
      Reject(frame, DELTA_RESULT_STREAM_DISABLED);
      break;

    default:
      Reject(frame, DELTA_RESULT_BAD_TYPE);
      break;
  }
}

void StreamActuator_Init(UART_HandleTypeDef *linkUart,
                         UART_HandleTypeDef *servo1Uart,
                         UART_HandleTypeDef *servo2Uart,
                         UART_HandleTypeDef *servo3Uart,
                         uint8_t dryRun) {
  linkPort = linkUart;
  directRepeat5Capable=0u;
  approachStep5Capable=0u;
  localPointCapable=directPointCapable=directTransactionCapable=0u;
  directValidatedPathCapable=directValidatedPathActive=0u;
  directLiftDelayCapable=directLiftProbeActive=directProfile250Capable=0u;
  localPointActive=directPointActive=localPointKnown=0u;
  directRetryCapable=directRetryUsed=directRetryPhase=0u;
  directStartedMask = directHoldPendingMask = directHoldRetryMask = directAbortActive = 0u;
  directHoldStableMask = 0u;
  directHoldStartedMs = directHoldDueMs = 0u;
  memset(directHoldStableGapMs, 0, sizeof(directHoldStableGapMs));
  memset(directHoldSampleMs, 0, sizeof(directHoldSampleMs));
  directTransactionFaultReason = "DIRECT_TRANSACTION_FAILED";
  ClearDirectProbeState();
  automaticStopCapable = streamEndToleranceRaw = 0u;
  auto15Rec30Capable = checkpointRecover30 = streamEndRecover30 = 0u;
  observeCapable = checkpointObserve = 0u;
  directFinalReadCapable = directFinalReadActive = directReadDeferred = 0u;
  endpointRetryCapable=endpointRetryEligible=endpointRetryCount=0u;directWriteMask=0x07u;
  endpointRetryLimitRaw=0u;directEndpointAwaitingRecovery=0u;
  endpointCenterCapable=centerPrecisionRequired=pickupFine20Capable=0u;endpointRetryToleranceRaw=DELTA_ENDPOINT_RETRY_TOLERANCE_RAW;
  endpointStepCapable=endpointStepActive=0u;
  directReadWarnings=directProgressWarnings=0u;
  servoPorts[0] = servo1Uart;
  servoPorts[1] = servo2Uart;
  servoPorts[2] = servo3Uart;
  dryRunMode = dryRun ? 1u : 0u;
  checkpointReassertNeeded = 0u;
  lastStreamMotionMs = 0u;
  GripperManual_Init();
  ConfigureServoPins();
  HAL_UART_Receive_IT(linkPort, &linkRxByte, 1u);

  printf("\r\n================================================\r\n");
  printf("NUCLEO 10C SEGURO v2.51 - HOME25/1100 + APPROACH STEPS5 + POST DIRECT READ300 + UART CLASICA TX5 REC350x5 + FINO CP5 + STOP - %s\r\n",
         dryRunMode ? "DRY RUN: CERO TX A SERVOS" : "SALIDA FISICA ACTIVA");
  printf("20 Hz esperado, interpolacion 50..250 ms, watchdog 250 ms.\r\n");
  printf("Telemetria STREAM reducida: resumen cada %u setpoints; STATUS completo al detener.\r\n",
         (unsigned)STREAM_ACCEPT_PRINT_INTERVAL);
  printf("No mueve al arrancar. R1=400..540 R2=390..580 R3=340..620; max 4 RAW/trama.\r\n");
  printf("Cada STREAM fija modo y perfil; ambos se revalidan en todas las tramas.\r\n");
  printf("Feedback: 2/3 validas por servo y hasta %u reintentos acumulativos sin mover.\r\n",
         (unsigned)FEEDBACK_READ_MAX_RETRIES);
  printf("STREAM_END: espera %u ms, mide y corrige hasta %u rondas por canal.\r\n",
         (unsigned)(STREAM_END_TARGET_TIME_MS + STREAM_END_SETTLE_MS),
         (unsigned)FEEDBACK_MAX_CORRECTIONS);
  printf("HOME y XYZ compensan la banda muerta: la orden se desplaza del destino\r\n");
  printf("hasta %d RAW (paso %d RAW, %u rondas) con biseccion al pasarse.\r\n",
         FEEDBACK_MAX_TOTAL_COMPENSATION_RAW,
         FEEDBACK_MAX_CORRECTION_STEP_RAW,
         (unsigned)FEEDBACK_MAX_CORRECTIONS);
  printf("desired = destino fisico; command = orden compensada; measured = mediana.\r\n");
  printf("Z_ONLY conserva la correccion v2.8 validada en la etapa 06.\r\n");
  printf("STOP Z: tolerancia individual %d RAW y diferencia maxima %d RAW.\r\n",
         POST_CORRECTION_TOLERANCE_RAW,
         STREAM_FINAL_MAX_CHANNEL_SPREAD_RAW);
  printf("XYZ corrige desde %d RAW, rechaza correccion >%d y acepta final <=%d RAW.\r\n",
         XYZ_CORRECTION_TRIGGER_RAW + 1, XYZ_CORRECTION_MAX_ERROR_RAW,
         XYZ_FEEDBACK_TOLERANCE_RAW);
  printf("AUTO checkpoint: hasta12 RAW, pasos<=5, compensacion<=12, hasta3 rondas con progreso, final<=5; plazo16000ms.\r\n");
  printf("Recuperacion de posicion: lecturas3/3 estables, PWM OFF, desarmado, HOME posterior explicito; nunca reanuda AUTO.\r\n");
  printf("HOME: recupera desde RAW %u..%u por tramos proporcionales de maximo %d RAW medidos, movimiento %u ms y asentamiento %u ms.\r\n",
         (unsigned)HOME_RECOVERY_RAW_MIN,
         (unsigned)HOME_RECOVERY_RAW_MAX, HOME_SEGMENT_MAX_STEP_RAW,
         (unsigned)HOME_SEGMENT_TIME_MS, (unsigned)HOME_SEGMENT_SETTLE_MS);
  printf("HOME: tolerancia intermedia 5 RAW; cambios menores de 8 RAW se validan por posicion.\r\n");
  printf("HOME: hasta %u reintentos cortos por segmento antes de enclavar FAULT.\r\n",
         (unsigned)HOME_SEGMENT_MAX_RETRIES);
  printf("HOME final RAW %u: tolerancia individual %d RAW y diferencia maxima %d RAW.\r\n",
         (unsigned)RAW_HOME, HOME_FEEDBACK_TOLERANCE_RAW, HOME_MAX_CHANNEL_SPREAD_RAW);
  printf("================================================\r\n");
  PrintStatus();
}

void StreamActuator_Loop(void) {
  if (manualGripRuntime && !GripperManual_Active())
    LatchVerificationFault("PALA_WATCHDOG",0x07u);
  if (frameReady) {
    DeltaLinkFrame local;
    __disable_irq();
    local = pendingFrame;
    frameReady = 0u;
    __enable_irq();
    ProcessFrame(&local);
  }
  const uint32_t now = HAL_GetTick();
  if (checkpointPending && now - checkpointStartedMs > CheckpointTimeout())
    LatchVerificationFault("CHECKPOINT_TIMEOUT", 0x07u);
  if (recoveryPending && now - recoveryStartedMs > DELTA_FAULT_RECOVERY_TIMEOUT_MS)
    FailRecovery();

  if (busy && !verificationPending &&
      !localPointActive && !directPointActive && !directProbePending &&
      (int32_t)(now - busyUntilMs) >= 0) {
    busy = 0u;
  }

  if (streamActive && lastStreamSetpointMs != 0u &&
      now - lastStreamSetpointMs > STREAM_TIMEOUT_MS) {
    ++streamTimeouts;
    checkpointPending = 0u;
    ClearCheckpointCorrection();
    StopGripperOutputs();
    streamActive = 0u;
    activeStreamOption = 0u;
    armed = 0u;
    homeValid = 0u;
    homePending = 0u;
    rawKnown = 0u;
    streamBaselineKnown = 0u;
    streamCorrectionAttempted = 0u;
    xyzCorrectionRounds = 0u;
    ResetCompensation();
    verificationReadRetryAttempted = 0u;
    CancelVerification();
    busy = 0u;
    printf("[STREAM WATCHDOG] 250 ms sin setpoint: DISARM y HOME invalido.\r\n");
  }

  if (activeSession != 0u && !linkStale &&
      now - lastValidFrameMs > LINK_TIMEOUT_MS) {
    recoveryPending = 0u; // leave FAULT latched; stale reads cannot unlock
    StopGripperOutputs();
    checkpointPending = 0u;
    ClearCheckpointCorrection();
    linkStale = 1u;
    streamActive = 0u;
    activeStreamOption = 0u;
    armed = 0u;
    homeValid = 0u;
    homePending = 0u;
    rawKnown = 0u;
    streamBaselineKnown = 0u;
    streamCorrectionAttempted = 0u;
    xyzCorrectionRounds = 0u;
    ResetCompensation();
    verificationReadRetryAttempted = 0u;
    CancelVerification();
    busy = 0u;
    printf("[STREAM SAFETY] Timeout USART6: DISARM y HOME invalido.\r\n");
  }

  // Revisar primero el enlace: no emitir correcciones tras vencer watchdog.
  ServiceDirectHoldConfirm(); // confirms a DIRECT stop even after DISARM/FAULT
  ServiceLocalPoint(); // local slices only after watchdogs; no host segment ACK dependency
  ServiceDirectPoint(); // transactional endpoint plus bounded early progress probe
  /* El comando y su ACK tienen prioridad. Muestrear/enviar encoder antes de
     cada lectura HTS (hasta 35 ms) y sus printf, sin saltar verificaciones. */
  if (encoderService != NULL) encoderService();
  ServiceVerification();

  /* Durante movimiento, STREAM ACCEPT entrega un diagnostico compacto. El
     STATUS completo se difiere para que UART2 no bloquee el despacho a los
     tres servos dentro de un ciclo de control. */
  if (!streamActive && now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    PrintStatus();
  }
}

uint8_t StreamActuator_StreamActive(void) { return streamActive; }

void StreamActuator_OnRxComplete(UART_HandleTypeDef *huart) {
  if (huart != linkPort) return;
  const uint8_t value = linkRxByte;
  if (!frameReady) {
    if (rxIndex == 0u) {
      if (value == DELTA_LINK_SOF_A) rxBytes[rxIndex++] = value;
    } else if (rxIndex == 1u) {
      if (value == DELTA_LINK_SOF_B) rxBytes[rxIndex++] = value;
      else {
        rxIndex = value == DELTA_LINK_SOF_A ? 1u : 0u;
        if (rxIndex == 1u) rxBytes[0] = value;
      }
    } else {
      rxBytes[rxIndex++] = value;
      if (rxIndex == DELTA_LINK_FRAME_SIZE) {
        memcpy(&pendingFrame, rxBytes, sizeof(pendingFrame));
        rxIndex = 0u;
        frameReady = 1u;
      }
    }
  }
  HAL_UART_Receive_IT(linkPort, &linkRxByte, 1u);
}

void StreamActuator_OnUartError(UART_HandleTypeDef *huart) {
  if (huart != linkPort) return;
  ++uartErrors;
  __HAL_UART_CLEAR_OREFLAG(linkPort);
  rxIndex = 0u;
  HAL_UART_Receive_IT(linkPort, &linkRxByte, 1u);
}
