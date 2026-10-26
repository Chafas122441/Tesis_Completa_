/**
 * @file 10B_Control_Manual_Auto_Portenta.ino
 * @brief Coordinador del robot DELTA en el nucleo M7 de la Portenta H7.
 * Integra menu OLED, homografia, encoder, cinematica FK/IK/Jacobiano y
 * secuencias manual y automatica. Serial1 comunica con Nucleo a 115200:
 * D14 TX y D13 RX. Conserva armado explicito, limites y cancelacion segura.
 */
#include <Arduino.h>
#include <USB/PluggableUSBSerial.h> // public core USB object; UART wrapper lacks send_nb
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include "hal/trng_api.h"
#include "gamepad_packet.h"
#include "camera_packet.h"
#include "camera_alignment.h"
#include "oled_sh1106.h"
#include "nucleo_stream_link.h"
#include "gripper_auto_calibration.h"
#include "auto_box_calibration.h"  
#include "auto_pickup_config.h"
#include "auto_cartesian_arrival.h"
#include "auto_transport_policy.h"
#include "auto_joint_ptp.h"
#include "auto_direct_endpoint.h"
#include "auto_local_point.h"
#include "calibration_usb.h"

static_assert(AutoTransport::DISPATCH_PERIOD_MS == NucleoStreamLink::COARSE_ACK_GUARD_MS,
              "Controller and wire-level coarse ACK cadence must match");
static_assert(AutoTransport::MOVE_TIME_MS >= NucleoStreamLink::COARSE_MIN_MOVE_MS,
              "Coarse interpolation must satisfy receiver timing");

/*
  ETAPA 10B - CONTROL MANUAL, OLED Y AUTOMATICO DE RECOGIDA

  Gamepad -> Bluetooth -> ESP32 10A -> UDP/Wi-Fi -> Portenta

  Calcula velocidad cartesiana mediante Jacobiano DLS a 20 Hz y envia
  setpoints binarios v3 interpolados a la Nucleo 10C.
  HOME, modo, velocidad, armado, inicio y parada se manejan con el gamepad.
  El monitor USB es solo diagnostico y nunca bloquea el funcionamiento.

    - Cinematica directa (FK) desde angulos simulados.
    - Jacobiano numerico J = d(X,Y,Z) / d(theta1,theta2,theta3).
    - Resolucion por inversa, transpuesta y DLS (minimos cuadrados amortiguados).
    - Control de velocidad cartesiana mientras se sostiene el joystick.

  OPTIONS abre el menu. HOME y ARMAR E INICIAR exigen doble X. Durante STREAM,
  OPTIONS termina y verifica antes de reabrir el menu; CIRCULO cancela y desarma.
  Una reconexion nunca rearma ni reanuda.
*/

namespace Config {
constexpr uint32_t USB_BAUD = 115200u;
constexpr uint32_t NUCLEO_BAUD = 115200u;
constexpr char AP_SSID[] = "DELTA_PORTENTA";
constexpr char AP_PASSWORD[] = "PortentaDelta2026";
constexpr uint16_t UDP_PORT = 1234u;
constexpr uint16_t ESP_UDP_PORT = 1235u;
constexpr uint32_t AP_RETRY_MS = 5000u;

constexpr uint32_t GAMEPAD_TIMEOUT_MS = 600u;
/* HUSKYLENS bloquea temporalmente el loop del ESP32 al ejecutar begin() o
   switchAlgorithm(). Durante esa ventana no salen heartbeats del gamepad,
   aunque Bluepad32 siga conectado. La gracia se aplica unicamente mientras
   AUTO espera el cambio de algoritmo y el robot esta detenido/desarmado. */
constexpr uint32_t CAMERA_SWITCH_GAMEPAD_TIMEOUT_MS = 8000u;
static_assert(CAMERA_SWITCH_GAMEPAD_TIMEOUT_MS > GAMEPAD_TIMEOUT_MS,
              "La gracia de camara debe superar el timeout operativo");
/* El gamepad llega a 20 Hz. Ejecutar el lazo al doble producia miles de
   setpoints que redondeaban al mismo entero RAW. A 20 Hz cada destino aporta
   desplazamiento util al interpolador del HTS-35H sin cambiar la velocidad
   articular maxima por segundo. */
constexpr uint32_t CONTROL_PERIOD_MS = 50u;       // 20 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 500u;    // 2 Hz; evita rafagas USB durante STREAM
constexpr uint32_t STATUS_PERIOD_MS = 5000u;
constexpr uint32_t OLED_PERIOD_MS = 150u;
constexpr uint32_t OLED_RUN_PAGE_PERIOD_MS = 25u;
constexpr uint32_t OLED_IDLE_PAGE_PERIOD_MS = 2u;
constexpr uint32_t CAMERA_COMMAND_RETRY_MS = 500u;
constexpr uint32_t CAMERA_TIMEOUT_MS = 1500u;
constexpr uint32_t CAMERA_DIAGNOSTIC_PERIOD_MS = 1000u;
constexpr uint32_t CONFIRM_TIMEOUT_MS = 5000u;
constexpr uint32_t CENTER_TIME_MS = 500u;
constexpr int16_t CENTER_THRESHOLD = 70;
/* Histeresis del joystick: un eje empieza a mandar movimiento al superar
   ENGAGE y debe regresar por debajo de RELEASE para detenerse. */
constexpr int16_t STICK_ENGAGE_THRESHOLD = 70;
constexpr int16_t STICK_RELEASE_THRESHOLD = 50;
constexpr int16_t STICK_MAX = 512;
constexpr uint32_t CENTER_CALIBRATION_MS = 2500u;
constexpr uint16_t CENTER_CALIBRATION_MIN_SAMPLES = 8u;
constexpr int16_t CENTER_CALIBRATION_MAX_SPAN = 30;
constexpr int16_t CENTER_CALIBRATION_MAX_OFFSET = 140;
constexpr size_t USB_LINE_CAPACITY = 48u;

constexpr uint16_t BUTTON_A = 0x0001u;
constexpr uint16_t BUTTON_B = 0x0002u;
constexpr uint16_t MISC_START = 0x0004u;
constexpr uint8_t DPAD_UP = 0x01u;
constexpr uint8_t DPAD_DOWN = 0x02u;

constexpr float E_MM = 69.859f;
constexpr float F_MM = 79.204f;
constexpr float RF_MM = 200.0f;
constexpr float RE_MM = 350.0f;
constexpr float SQRT3 = 1.7320508075688772f;
constexpr float SIN120 = SQRT3 / 2.0f;
constexpr float COS120 = -0.5f;
constexpr float TAN30 = 1.0f / SQRT3;
constexpr float PI_VALUE = 3.14159265358979323846f;
constexpr float DEG_TO_RAD_VALUE = PI_VALUE / 180.0f;
constexpr float RAD_TO_DEG_VALUE = 180.0f / PI_VALUE;

constexpr float HOME_X_MM = 0.0f;
constexpr float HOME_Y_MM = 0.0f;
/* HOME elevado 50 mm para retirar la pala y verificar la entrada AUTO.
   El transito AUTO se configura por separado a la altura de descarga. */
constexpr float HOME_Z_MM = -287.0f;
constexpr uint16_t AUTO_HOME_RAW = 410u;
constexpr int AUTO_HOME_TOLERANCE_RAW = 4;
constexpr int AUTO_HOME_SPREAD_MAX_RAW = 6;
constexpr float AUTO_HOME_POSITION_TOLERANCE_MM = 1.0f;
/* Caja envolvente solamente para acotar el calculo. No define por si sola un
   punto seguro: IK/FK, RAW y Jacobiano se comprueban para cada setpoint. */
constexpr float X_MIN_MM = -270.0f;
constexpr float X_MAX_MM = 270.0f;
constexpr float Y_MIN_MM = -295.0f;
constexpr float Y_MAX_MM = 280.0f;

constexpr float GRIPPER_CONTACT_Z_MM = -365.5f;
constexpr float Z_MIN_MM = -367.0f;
/* R3 experimental: 70 mm mas arriba. Esta caja NO certifica colisiones.
   Cada paso necesita IK/FK/RAW/condicion; R1/R2 conservan sus rangos. */
constexpr float Z_MAX_MM = -180.0f;
constexpr float DIRECT_LIFT_MAX_XY_DRIFT_MM = 5.5f;
constexpr float HIGH_XY_ASSIST_Z_MM = -300.0f;
/* La referencia se mantiene dentro de los limites exactos. Este epsilon solo
   evita que el error submilimetrico de FK/DLS convierta -280.000 en una falsa
   salida del workspace. No amplia el recorrido comandado. */
constexpr float WORKSPACE_POSITION_EPSILON_MM = 0.10f;
/* Las maniobras automaticas de escape se limitan a una velocidad moderada,
   aunque el perfil manual seleccionado sea 40..80 mm/s. */
constexpr float WORKSPACE_ASSIST_MAX_SPEED_MM_S = 30.0f;
constexpr uint8_t WORKSPACE_ASSIST_SEARCH_STEPS = 12u;
constexpr float BAND_SAFE_X_MM = 0.0f;
constexpr float BAND_SAFE_Y_MM = 0.0f;
constexpr float BAND_SAFE_Z_MM = -367.0f;
constexpr float TCP_OFFSET_Z_MM = 0.0f;
constexpr float BAND_TARGET_TOLERANCE_MM = 0.20f;
static_assert(BAND_SAFE_Z_MM == Z_MIN_MM,
              "El objetivo de banda debe coincidir con el piso fisico");
static_assert(BAND_SAFE_Z_MM < GRIPPER_CONTACT_Z_MM,
              "La toma debe quedar debajo del primer contacto medido");
constexpr uint8_t WORKSPACE_BISECTION_STEPS = 10u;
constexpr uint32_t WORKSPACE_PRINT_PERIOD_MS = 1000u;

constexpr float PRECISION_SPEED_MM_S = 10.0f;
constexpr float NORMAL_SPEED_MM_S = 30.0f;
constexpr float SPEED_40_MM_S = 40.0f;
constexpr float SPEED_50_MM_S = 50.0f;
constexpr float SPEED_60_MM_S = 60.0f;
constexpr float SPEED_80_MM_S = 80.0f;
constexpr float ACCEL_MM_S2 = 120.0f;
constexpr float DECEL_MM_S2 = 200.0f;
/* Atenua el jitter de alta frecuencia del stick antes del limitador de
   aceleracion. Al soltarlo se fuerza cero y el frenado sigue gobernado por
   DECEL_MM_S2, por lo que no se prolonga una orden del operador. */
constexpr float STICK_FILTER_TAU_S = 0.030f;
/* Retiene variaciones pequenas de la orden manual. Cero, inicio, inversion
   y extremo pasan inmediatamente. No afecta AUTO ni PLANO SEGURO BANDA. */
constexpr float STICK_COMMAND_HYSTERESIS = 0.015f;
constexpr float POSITION_KP_S = 3.0f;
/* Limite legado del prototipo para comparaciones matematicas. El AUTO real
   usa perfiles por tramo en auto_pickup_config.h, conservando la rampa,
   DLS, 4 RAW/trama y VERIFY de la arquitectura actual. */
constexpr float AUTO_SPEED_MM_S = 100.0f;
constexpr float AUTO_TARGET_TOLERANCE_MM = 0.20f;
constexpr float DLS_LAMBDA_MM_RAD = 2.0f;
constexpr float JACOBIAN_STEP_RAD = 0.0005f;
/* 0.90 grados a 20 Hz equivalen a la misma cota de 36 grados/s que antes:
   0.45 grados a 40 Hz. El redondeo nunca debe superar 4 RAW por trama. */
constexpr float MAX_JOINT_STEP_RAD = 0.90f * DEG_TO_RAD_VALUE;
constexpr float MAX_CONDITION_F = 12.0f;
constexpr float MAX_NUMERIC_RESIDUAL_MM = 0.50f;
/* Dos ciclos de solape: cada destino de 50 ms se ejecuta en 100 ms. Esto deja
   al interpolador interno unir consignas de hasta 4 RAW sin pausas entre ellas. */
constexpr uint16_t STREAM_MOVE_TIME_MS = 100u;
constexpr int STREAM_RAW_MAX_STEP = 4;
constexpr int STREAM_Z_SPREAD_MAX = 2;
static_assert(STREAM_RAW_MAX_STEP * 1000u / CONTROL_PERIOD_MS == 80u,
              "La cadencia nueva no debe superar 80 RAW/s");
static_assert(STREAM_MOVE_TIME_MS >= 2u * CONTROL_PERIOD_MS,
              "La interpolacion debe solapar al menos dos setpoints");
constexpr int SERVO_HOME_RAW[3] = {500, 500, 500};
constexpr int SERVO_SIGN[3] = {1, 1, 1};
constexpr float THETA_HOME_DEG[3] = {21.99f, 21.99f, 21.99f};
constexpr float SERVO_UNITS_PER_DEG = 1000.0f / 240.0f;
/* Margen extra al medio RAW de redondeo, exclusivamente XYZ manual. */
constexpr float STREAM_RAW_HYSTERESIS_RAW = 0.15f;
static_assert(STICK_COMMAND_HYSTERESIS >= 0.0f &&
              STICK_COMMAND_HYSTERESIS <= 0.05f,
              "Ventana de mando fuera de rango");
static_assert(STREAM_RAW_HYSTERESIS_RAW >= 0.0f &&
              MAX_JOINT_STEP_RAD * RAD_TO_DEG_VALUE * SERVO_UNITS_PER_DEG +
                STREAM_RAW_HYSTERESIS_RAW < STREAM_RAW_MAX_STEP,
              "La histeresis debe conservar la cota de 4 RAW por trama");
}

namespace ManualAntiJitter {
float stableAxis(float requested, float previous, float tolerance) {
  if (fabsf(requested) < 0.0001f || requested * previous <= 0.0f ||
      fabsf(requested) >= 1.0f || fabsf(requested - previous) >= tolerance) {
    return requested;
  }
  return previous;
}

int stableRaw(float desired, int previous, float hysteresis) {
  if (hysteresis > 0.0f &&
      fabsf(desired - static_cast<float>(previous)) <= 0.5f + hysteresis) {
    return previous;
  }
  return static_cast<int>(roundf(desired));
}
} // namespace ManualAntiJitter

struct Vec3 {
  float v[3];
};

struct Mat3 {
  float m[3][3];
};

enum class ManualMode : uint8_t { Z_ONLY = 0u, XYZ_MANUAL };
enum class SpeedProfile : uint8_t {
  PRECISION = 0u, NORMAL, SPEED_40, SPEED_50, SPEED_60, SPEED_80, COUNT
};
enum class RangeProfile : uint8_t { R1 = 0u, R2, R3, COUNT };
enum class OperationMode : uint8_t { MANUAL = 0u, AUTO_PICKUP };
enum class MenuItem : uint8_t {
  HOME = 0u, BAND_SAFE, OPERATION, SPEED, CAMERA,
  ARM_START, STATUS, GRIP_Z, GRIP_X, GRIP_PUSH, GRIP_MANUAL, GRIP_RECOVER, FAULT_RECOVER, COUNT
};
enum class Confirmation : uint8_t {
  NONE = 0u, HOME, BAND_SAFE, CAMERA, ARM_START, GRIP, GRIP_RECOVER, FAULT_RECOVER
};
enum class StartPhase : uint8_t {
  IDLE = 0u, WAIT_ARM_ACK, WAIT_STREAM_ACK,
  WAIT_PALA_REF, WAIT_PALA_POSE, WAIT_PALA_END, WAIT_PALA_ARM, WAIT_AUTO_REF
};
enum class WorkspaceAssistMode : uint8_t {
  NONE = 0u, LOWER_FOR_XY, RETREAT_FOR_Z, COMPONENT_PROJECTED
};

WiFiUDP udp;
NucleoStreamLink nucleo(Serial1);
CameraAlignment cameraAlignment;
OledSh1106 oled;
GamepadPacket gamepad{};
IPAddress espAddress(10, 0, 0, 2);
bool apReady = false;
bool gamepadKnown = false;
bool gamepadFresh = false;
bool previousGamepadConnected = false;
bool runEnabled = false;
bool simulationValid = false;
bool stopRequested = false;
bool pendingSetpoint = false;
bool homeRequestPending = false;
bool homeRequestAcknowledged = false;
bool previousNucleoReady = false;
bool streamEndVerificationPending = false;
bool menuOpen = false;
bool reopenMenuAfterStop = false;
bool armAfterAckPending = false;
uint32_t armAfterAckRequestedMs = 0u;
constexpr ManualMode manualMode = ManualMode::XYZ_MANUAL;
OperationMode operationMode = OperationMode::MANUAL;
SpeedProfile speedProfile = SpeedProfile::PRECISION;
constexpr RangeProfile rangeProfile = RangeProfile::R3;
uint16_t activeStreamOption = DELTA_STREAM_OPTION_XYZ_MANUAL |
                              DELTA_STREAM_OPTION_RANGE_R3;
MenuItem selectedMenuItem = MenuItem::HOME;
Confirmation confirmation = Confirmation::NONE;
StartPhase startPhase = StartPhase::IDLE;
uint32_t confirmationDeadlineMs = 0u;
Vec3 pendingThetaRad{{0.0f, 0.0f, 0.0f}};
Vec3 pendingReferencePose{{Config::HOME_X_MM, Config::HOME_Y_MM,
                           Config::HOME_Z_MM}};
float pendingConditionF = NAN;
float pendingDeterminant = NAN;
float pendingErrorMm = NAN;
float pendingInverseResidualMm = NAN;
float pendingTransposeResidualMm = NAN;
float pendingDlsResidualMm = NAN;
float pendingExactResidualMm = NAN;
uint32_t streamCommandsSent = 0u;
uint32_t streamCommandsRejected = 0u;
uint32_t activeGamepadSession = 0u;
uint32_t lastGamepadSequence = 0u;
uint32_t lastGamepadMs = 0u;
uint32_t gamepadGapMaxMs = 0u;
uint32_t gamepadLossEvents = 0u;
uint32_t loopMaxUs = 0u;
uint32_t centeredSinceMs = 0u;
int16_t gamepadNeutral[4] = {0, 0, 0, 0};
bool centerCalibrated = false;
bool centerCalibrationActive = false;
bool axisDriveActive[3] = {false, false, false};
uint32_t centerCalibrationStartedMs = 0u;
uint16_t centerCalibrationSamples = 0u;
int32_t centerCalibrationSum[4] = {0, 0, 0, 0};
int16_t centerCalibrationMin[4] = {0, 0, 0, 0};
int16_t centerCalibrationMax[4] = {0, 0, 0, 0};
uint32_t lastApAttemptMs = 0u;
uint32_t lastControlMs = 0u;
uint32_t lastTelemetryMs = 0u;
uint32_t lastStatusMs = 0u;
uint32_t udpValid = 0u;
uint32_t udpLengthErrors = 0u;
uint32_t udpCrcErrors = 0u;
uint32_t udpSequenceErrors = 0u;
uint32_t controllerCycles = 0u;
uint32_t safetyStops = 0u;
uint32_t workspaceLimits = 0u;
uint32_t lastWorkspaceLimitPrintMs = 0u;
uint32_t lastWorkspaceAssistPrintMs = 0u;
WorkspaceAssistMode workspaceAssistMode = WorkspaceAssistMode::NONE;
bool automaticBandMove = false;
bool autoAuthorized = false;
uint32_t autoHomeWarningUntilMs = 0u;
bool autoTargetActive = false;
uint8_t autoPieceId = 0u;
Vec3 autoTargetPose{{Config::HOME_X_MM, Config::HOME_Y_MM,
                     Config::HOME_Z_MM}};
AutoPickup::Sequence autoPickup;
uint16_t pendingAutoStopFlags = 0u;
AutoJointPtp::Plan autoJointPtp;
enum class LocalPointPhase : uint8_t {IDLE,ESCAPE,READY,RUNNING,FINISHED,PRECHECK,WAIT_PRECHECK,RETRY_READY};
LocalPointPhase autoLocalPhase=LocalPointPhase::IDLE;
Vec3 autoLocalGoal{};
uint32_t autoLocalToken=0u,autoLocalStartedMs=0u;
bool autoDirectMove=false;
bool autoEndpointRetryPending=false;
bool autoCenterPrecisionPending=false;
bool autoEndpointStepPending=false;
bool autoEndpointOriginRefreshed=false;
uint32_t autoEndpointRetryQueuedMs=0u;
uint8_t autoEndpointRetryAttempts=0u;
uint16_t autoEndpointNominal[3]={0u,0u,0u},autoEndpointDurationFloor=0u;
bool autoDirectRcRenewalKnown=false;
uint32_t autoDirectRcRenewalMs=0u;
constexpr uint32_t DIRECT_RC_PREPARE_MAX_AGE_MS=60u;
uint32_t autoPtpToken = 0u;
uint32_t autoPtpPendingElapsedMs = 0u;
bool autoPtpSegmentPending = false;
bool autoRcActive = false;
bool autoRcPending = false;
bool autoRcBeginPending = false;
bool autoCapabilityPending = false;
bool autoCapabilityPassed = false;
uint32_t autoRuntimeStartMs = 0u;
uint16_t autoRcPendingUs[3] = {};
uint32_t autoRcPendingToken = 0u;
uint32_t autoRcLastAckMs = 0u;
uint32_t autoActionToken = 0u;
uint32_t autoGateTriggerMs = 0u;
uint32_t autoGateThresholdSeenMs = 0u;
bool autoGateThresholdSeen = false;
bool autoGateFirstAckLogged = false;
bool autoGateFinalAckLogged = false;
uint32_t autoCheckpointToken = 0u;
bool autoPointHold = false;
bool autoPointSettling = false;
uint32_t autoPointSettleStartedMs = 0u;
bool autoSnapshotReceived = false;
uint32_t autoStartGeneration = 0u;
uint32_t autoStartMs = 0u;
CameraPieceTarget autoPendingPiece;
bool autoPiecePending = false;
int32_t autoEncoderAtFrame = 0;
bool autoEncoderAssociated = false;
uint32_t cameraCommandSequence = 0u;
uint32_t cameraCommandSession = 0u; // independent Portenta boot session
CalibrationUsb::Spool calibrationUsb;
uint32_t calibrationUsbLastMs = 0u, calibrationUsbTrial = 0u;

uint32_t cameraPacketsValid = 0u;
uint32_t cameraLengthErrors = 0u;
uint32_t cameraCrcErrors = 0u;
uint32_t cameraSessionChanges = 0u;
uint32_t lastCameraSession = 0u;
uint32_t lastCameraCommandMs = 0u;
uint32_t lastCameraDiagnosticMs = 0u;
uint16_t lastCameraCommand = 0xFFFFu;
bool cameraSwitchGraceReported = false;
bool oledReady = false;
bool oledDirty = true;
uint32_t lastOledMs = 0u;
uint32_t lastOledPageMs = 0u;
uint8_t oledFlushPage = 8u;
bool oledFlushActive = false;
uint16_t previousButtons = 0u;
uint16_t previousMisc = 0u;
uint8_t previousDpad = 0u;
/* Individual calibration or three-RC manual operation, DELTA stopped. */
enum class GripPhase : uint8_t { OFF, BEGIN_PENDING, ACTIVE, STOPPING };
GripPhase gripPhase = GripPhase::OFF;
uint16_t gripUs[3] = {1500u, 1500u, 1500u};
uint16_t gripPendingUs = 1500u;
uint8_t gripChannel = 1u;
uint32_t gripLastAckMs = 0u;
uint32_t gripLastPrintMs = 0u;
bool gripEndSent = false;
bool gripReferenceUncertain = false;
bool gripStateQueryNeeded = true;
uint32_t gripSessionGeneration = 0u;
uint32_t gripLastStateQueryMs = 0u;
uint16_t gripRecoveryPreviewUs[3] = {};
uint32_t gripRecoveryPreviewGeneration = 0u;
uint32_t gripArmStartedMs = 0u;
uint32_t gripArmGeneration = 0u;
uint32_t gripArmAtTargetMs = 0u;
bool gripArmStateReceived = false;
constexpr uint32_t GRIP_ARM_TIMEOUT_MS = 8000u;
constexpr uint32_t GRIP_ARM_SETTLE_MS = 300u; // command dwell, NOT measured arrival
bool gripGroupMode = false;
bool gripRuntime = false; // explicit MANUAL STREAM permit, never AUTO/calibration
uint16_t gripTargetUs[3] = {1500u,1500u,1500u};
uint16_t gripPendingGroupUs[3] = {1500u,1500u,1500u};
bool showValueScreen = false;
bool beltDisplayKnown = false;
uint16_t beltDisplayTenths = 0u;
void printPositionMark();

bool preparingManualPala() {
  return startPhase == StartPhase::WAIT_PALA_REF ||
         startPhase == StartPhase::WAIT_PALA_POSE ||
         startPhase == StartPhase::WAIT_PALA_END ||
         startPhase == StartPhase::WAIT_PALA_ARM;
}

void serviceBeltDisplay() {
  const bool known = nucleo.encoderFresh();
  const uint16_t received = nucleo.beltSpeedTenths();
  const int difference = abs((int)received - (int)beltDisplayTenths);
  /* One-second estimate; retain stable display within 2 mm/s or 2 percent. */
  const int threshold = max(20, (int)beltDisplayTenths / 50);
  if (known != beltDisplayKnown || (known &&
      (difference >= threshold || ((received == 0u) != (beltDisplayTenths == 0u))))) {
    beltDisplayKnown = known;
    beltDisplayTenths = received;
    oledDirty = true;
    Serial.print(F("[BANDA] velocidad_estimada_mm_s="));
    if (known) Serial.print(beltDisplayTenths * 0.1f, 1);
    else Serial.print(F("SIN_DATO"));
    Serial.println(F(" D=49mm factor=32.610625 cuentas/mm"));
  }
}

void beltOledLine(char *line, size_t size) {
  if (beltDisplayKnown)
    snprintf(line, size, "BANDA:%u.%u mm/s", beltDisplayTenths / 10u,
             beltDisplayTenths % 10u);
  else
    snprintf(line, size, "BANDA: SIN DATO");
}

void stopGripper() {
  if (gripPhase != GripPhase::OFF) {
    gripPhase = GripPhase::STOPPING;
    oledDirty = true;
  }
}

bool gripStateStopped() {
  const uint8_t unsafe = DELTA_FLAG_ARMED | DELTA_FLAG_STREAM_ACTIVE |
      DELTA_FLAG_BUSY | DELTA_FLAG_DRY_RUN | DELTA_FLAG_FAULT | DELTA_FLAG_LINK_STALE;
  return nucleo.gripperStateFresh() && nucleo.gripperStateChannel() == 0u &&
      (nucleo.gripperStateFlags() & unsafe) == 0u;
}

void serviceGripReference() {
  // Read only while idle: no extra UART traffic during DELTA/RC movement.
  if (!nucleo.ready() || nucleo.waitingResponse() || nucleo.armed() ||
      nucleo.streamActive() || nucleo.busy() || nucleo.remoteFault() ||
      nucleo.dryRun() || gripPhase != GripPhase::OFF || runEnabled ||
      stopRequested || streamEndVerificationPending || homeRequestPending ||
      startPhase != StartPhase::IDLE) return;
  const bool previewing = menuOpen && selectedMenuItem == MenuItem::GRIP_RECOVER;
  if (!gripStateQueryNeeded && !previewing) return;
  const uint32_t now = millis();
  if (now - gripLastStateQueryMs < 500u) return;
  gripLastStateQueryMs = now;
  nucleo.sendGripperStateRequest();
}

void printGripper() {
  if (gripGroupMode) {
    Serial.print(F("[PALA MANUAL] us_ACK_Z_X_PALO="));
    for (unsigned i=0;i<3;++i) {
      if(i) Serial.print(','); Serial.print(gripUs[i]);
    }
    Serial.print(F(" target="));
    for (unsigned i=0;i<3;++i) {
      if(i) Serial.print(','); Serial.print(gripTargetUs[i]);
    }
    Serial.println(gripRuntime ? F("; CONSIGNAS, sin feedback. MANUAL DELTA+RC.")
                               : F("; CONSIGNAS, no feedback. DELTA detenido."));
    if (!gripRuntime) printPositionMark();
    return;
  }
  Serial.print(F("[GRIP MARK] canal=")); Serial.print(gripChannel);
  Serial.print(F(" us_ACK=")); Serial.print(gripUs[gripChannel - 1u]);
  Serial.print(F(" ack_age_ms=")); Serial.print(millis() - gripLastAckMs);
  Serial.print(F(" limites=")); Serial.print(GRIP_MIN_US[gripChannel - 1u]);
  Serial.print('/'); Serial.print(GRIP_MAX_US[gripChannel - 1u]);
  Serial.print(F(" botones=0x")); Serial.print(gamepad.buttons, HEX);
  Serial.print(F(" dpad=0x")); Serial.print(gamepad.dpad, HEX);
  Serial.println(F(" CONSIGNA, NO ANGULO MEDIDO; AUTO PALA BLOQUEADO."));
  printPositionMark();
}

void serviceGripper() {
  if (gripPhase == GripPhase::OFF) return;
  const uint32_t now = millis();
  if (!nucleo.ready()) {
    gripReferenceUncertain = true;
    stopGripper();
    return; // Nucleo watchdog removes PWM independently.
  }
  const bool unsafeGrip = !gamepadFresh || now - lastGamepadMs > 200u ||
      nucleo.remoteFault() || operationMode != OperationMode::MANUAL ||
      (!gripRuntime && (nucleo.armed() || nucleo.streamActive() || nucleo.busy())) ||
      (gripRuntime && (stopRequested || !nucleo.streamActive() ||
        (gripPhase == GripPhase::ACTIVE && !runEnabled)));
  if (unsafeGrip) {
    if (gripRuntime && !stopRequested)
      pauseSimulation(F("PALA/GP inseguro: detener DELTA y retirar PWM"));
    else stopGripper();
  }
  if (nucleo.waitingResponse()) return;
  if (gripPhase == GripPhase::STOPPING) {
    if (!gripEndSent)
      gripEndSent = nucleo.sendGripper(DELTA_MSG_GRIP_END, gripChannel, 0u);
    return;
  }
  if (gripRuntime && gripPhase == GripPhase::ACTIVE &&
      (now - lastControlMs >= 180u ||
       (now - lastControlMs >= Config::CONTROL_PERIOD_MS - 15u &&
        now - gripLastAckMs < GRIP_RUNTIME_RENEW_MS)))
    return; // DELTA deadline; allow an aged RC lease to renew in a bounded slot.
  if (gripGroupMode) {
    if (gripPhase != GripPhase::ACTIVE ||
        now - gripLastAckMs < GRIP_GROUP_ACK_GUARD_MS) return;
    if (startPhase == StartPhase::WAIT_PALA_POSE) {
      // Explicit ARM preparation only. Live JOG below remains independent.
      for (uint8_t i=0u;i<3u;++i) {
        const int delta = (int)gripTargetUs[i] - (int)gripUs[i];
        gripPendingGroupUs[i] = (uint16_t)((int)gripUs[i] +
          constrain(delta, -(int)GRIP_GROUP_MAX_STEP_US, (int)GRIP_GROUP_MAX_STEP_US));
      }
      nucleo.sendGripperGroup(DELTA_MSG_GRIP_GROUP_SET, gripPendingGroupUs, false);
      return; // Repeated target renews PWM during the bounded dwell.
    }
    // Live held keys only. No queued presets and no motion on another channel.
    const bool inputFresh = gamepadFresh &&
      now - lastGamepadMs <= GRIP_JOG_INPUT_MAX_AGE_MS;
    int direction[3] = {0,0,0};
    if (inputFresh) {
      direction[0] = ((gamepad.dpad & 0x04u) != 0u) -
                     ((gamepad.dpad & 0x08u) != 0u);
      direction[1] = ((gamepad.buttons & 0x20u) != 0u) -
                     ((gamepad.buttons & 0x10u) != 0u);
      direction[2] = ((gamepad.buttons & 0x80u) != 0u) -
                     ((gamepad.buttons & 0x40u) != 0u);
    }
    for (unsigned i=0;i<3;++i) {
      const int next = (int)gripUs[i] + direction[i] * (int)GRIP_GROUP_JOG_STEP_US;
      gripPendingGroupUs[i] = (uint16_t)constrain(next,
        (int)GRIP_MIN_US[i], (int)GRIP_MAX_US[i]);
    }

    for (unsigned i=0;i<3;++i) gripTargetUs[i]=gripPendingGroupUs[i];
    nucleo.sendGripperGroup(DELTA_MSG_GRIP_GROUP_SET, gripPendingGroupUs, gripRuntime);
    if(now-gripLastPrintMs>=1000u) { gripLastPrintMs=now;printGripper(); }
    return;
  }
  // Receiver timing starts on ACK, so retries cannot compress the 50 ms guard.
  if (gripPhase != GripPhase::ACTIVE || now - gripLastAckMs < GRIP_ACK_GUARD_MS) return;
  int direction = 0;
  if (gripChannel == 1u)
    direction = ((gamepad.dpad & 0x04u) != 0u) -
                ((gamepad.dpad & 0x08u) != 0u);
  else if (gripChannel == 2u)
    direction = ((gamepad.buttons & 0x20u) != 0u) -
                ((gamepad.buttons & 0x10u) != 0u);
  else
    direction = ((gamepad.buttons & 0x80u) != 0u) -
                ((gamepad.buttons & 0x40u) != 0u);
  if (now - lastGamepadMs > GRIP_JOG_INPUT_MAX_AGE_MS) direction = 0;
  const unsigned i = gripChannel - 1u;
  int next = (int)gripUs[i] + direction * (int)GRIP_MAX_STEP_US;
  if (next < GRIP_MIN_US[i]) next = GRIP_MIN_US[i];
  if (next > GRIP_MAX_US[i]) next = GRIP_MAX_US[i];
  if (nucleo.sendGripper(DELTA_MSG_GRIP_SET, gripChannel, (uint16_t)next)) {
    gripPendingUs = (uint16_t)next;
  }
  if (now - gripLastPrintMs >= 1000u) {
    gripLastPrintMs = now;
    printGripper();
  }
}
Vec3 referencePose{{Config::HOME_X_MM, Config::HOME_Y_MM, Config::HOME_Z_MM}};
Vec3 simulatedThetaRad{{0.0f, 0.0f, 0.0f}};
Vec3 commandedVelocity{{0.0f, 0.0f, 0.0f}};
Vec3 filteredManualCommand{{0.0f, 0.0f, 0.0f}};
Vec3 heldManualCommand{{0.0f, 0.0f, 0.0f}};
float lastConditionF = NAN;
float lastDeterminant = NAN;
float lastErrorMm = NAN;
float lastInverseResidualMm = NAN;
float lastTransposeResidualMm = NAN;
float lastDlsResidualMm = NAN;
float lastExactResidualMm = NAN;
char usbLine[Config::USB_LINE_CAPACITY]{};
size_t usbLineLength = 0u;

Vec3 add(const Vec3 &a, const Vec3 &b) {
  return Vec3{{a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2]}};
}

Vec3 subtract(const Vec3 &a, const Vec3 &b) {
  return Vec3{{a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2]}};
}

Vec3 scale(const Vec3 &a, float factor) {
  return Vec3{{a.v[0] * factor, a.v[1] * factor, a.v[2] * factor}};
}

float dot(const Vec3 &a, const Vec3 &b) {
  return a.v[0] * b.v[0] + a.v[1] * b.v[1] + a.v[2] * b.v[2];
}

float magnitude(const Vec3 &a) {
  return sqrtf(dot(a, a));
}

Vec3 limitMagnitude(const Vec3 &input, float maximum) {
  const float length = magnitude(input);
  return (length > maximum && length > 0.000001f)
    ? scale(input, maximum / length) : input;
}

Vec3 matVec(const Mat3 &matrix, const Vec3 &vector) {
  Vec3 result{};
  for (uint8_t row = 0u; row < 3u; ++row) {
    result.v[row] = matrix.m[row][0] * vector.v[0] +
                    matrix.m[row][1] * vector.v[1] +
                    matrix.m[row][2] * vector.v[2];
  }
  return result;
}

Vec3 transposeMatVec(const Mat3 &matrix, const Vec3 &vector) {
  Vec3 result{};
  for (uint8_t column = 0u; column < 3u; ++column) {
    result.v[column] = matrix.m[0][column] * vector.v[0] +
                       matrix.m[1][column] * vector.v[1] +
                       matrix.m[2][column] * vector.v[2];
  }
  return result;
}

float determinant(const Mat3 &a) {
  return a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) -
         a.m[0][1] * (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0]) +
         a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]);
}

bool invert(const Mat3 &a, Mat3 &inverse, float &det) {
  det = determinant(a);
  if (!isfinite(det) || fabsf(det) < 0.000001f) return false;
  const float invDet = 1.0f / det;
  inverse.m[0][0] =  (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) * invDet;
  inverse.m[0][1] = -(a.m[0][1] * a.m[2][2] - a.m[0][2] * a.m[2][1]) * invDet;
  inverse.m[0][2] =  (a.m[0][1] * a.m[1][2] - a.m[0][2] * a.m[1][1]) * invDet;
  inverse.m[1][0] = -(a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0]) * invDet;
  inverse.m[1][1] =  (a.m[0][0] * a.m[2][2] - a.m[0][2] * a.m[2][0]) * invDet;
  inverse.m[1][2] = -(a.m[0][0] * a.m[1][2] - a.m[0][2] * a.m[1][0]) * invDet;
  inverse.m[2][0] =  (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]) * invDet;
  inverse.m[2][1] = -(a.m[0][0] * a.m[2][1] - a.m[0][1] * a.m[2][0]) * invDet;
  inverse.m[2][2] =  (a.m[0][0] * a.m[1][1] - a.m[0][1] * a.m[1][0]) * invDet;
  return true;
}

float frobeniusNorm(const Mat3 &matrix) {
  float sum = 0.0f;
  for (uint8_t row = 0u; row < 3u; ++row) {
    for (uint8_t column = 0u; column < 3u; ++column) {
      sum += matrix.m[row][column] * matrix.m[row][column];
    }
  }
  return sqrtf(sum);
}

Mat3 multiplyByTranspose(const Mat3 &j) {
  Mat3 result{};
  for (uint8_t row = 0u; row < 3u; ++row) {
    for (uint8_t column = 0u; column < 3u; ++column) {
      for (uint8_t k = 0u; k < 3u; ++k) {
        result.m[row][column] += j.m[row][k] * j.m[column][k];
      }
    }
  }
  return result;
}

bool deltaForward(const Vec3 &thetaRad, Vec3 &pose) {
  const float t = (Config::F_MM - Config::E_MM) * Config::TAN30 / 2.0f;
  const float y1 = -(t + Config::RF_MM * cosf(thetaRad.v[0]));
  const float z1 = -Config::RF_MM * sinf(thetaRad.v[0]);
  const float y2 = (t + Config::RF_MM * cosf(thetaRad.v[1])) * 0.5f;
  const float x2 = y2 * Config::SQRT3;
  const float z2 = -Config::RF_MM * sinf(thetaRad.v[1]);
  const float y3 = (t + Config::RF_MM * cosf(thetaRad.v[2])) * 0.5f;
  const float x3 = -y3 * Config::SQRT3;
  const float z3 = -Config::RF_MM * sinf(thetaRad.v[2]);

  const float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
  if (fabsf(dnm) < 0.000001f) return false;
  const float w1 = y1 * y1 + z1 * z1;
  const float w2 = x2 * x2 + y2 * y2 + z2 * z2;
  const float w3 = x3 * x3 + y3 * y3 + z3 * z3;
  const float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  const float b1 = -0.5f * ((w2 - w1) * (y3 - y1) -
                            (w3 - w1) * (y2 - y1));
  const float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  const float b2 = 0.5f * ((w2 - w1) * x3 - (w3 - w1) * x2);
  const float a = a1 * a1 + a2 * a2 + dnm * dnm;
  const float b = 2.0f * (a1 * b1 + a2 * (b2 - y1 * dnm) -
                           z1 * dnm * dnm);
  const float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 +
                  dnm * dnm * (z1 * z1 - Config::RE_MM * Config::RE_MM);
  const float discriminant = b * b - 4.0f * a * c;
  if (!isfinite(discriminant) || discriminant < 0.0f || fabsf(a) < 0.000001f) {
    return false;
  }
  pose.v[2] = -0.5f * (b + sqrtf(discriminant)) / a;
  pose.v[0] = (a1 * pose.v[2] + b1) / dnm;
  pose.v[1] = (a2 * pose.v[2] + b2) / dnm;
  return isfinite(pose.v[0]) && isfinite(pose.v[1]) && isfinite(pose.v[2]);
}

int deltaCalcAngleYZ(float x0, float y0, float z0, float &thetaDeg) {
  const float y1 = -0.5f * Config::TAN30 * Config::F_MM;
  y0 -= 0.5f * Config::TAN30 * Config::E_MM;
  if (fabsf(z0) < 0.0001f) return -1;
  const float a = (x0 * x0 + y0 * y0 + z0 * z0 +
                   Config::RF_MM * Config::RF_MM -
                   Config::RE_MM * Config::RE_MM - y1 * y1) / (2.0f * z0);
  const float b = (y1 - y0) / z0;
  const float discriminant = -(a + b * y1) * (a + b * y1) +
    Config::RF_MM * (b * b * Config::RF_MM + Config::RF_MM);
  if (discriminant < 0.0f) return -1;
  const float yj = (y1 - a * b - sqrtf(discriminant)) / (b * b + 1.0f);
  const float zj = a + b * yj;
  thetaDeg = atan2f(-zj, y1 - yj) * Config::RAD_TO_DEG_VALUE;
  if (yj > y1) thetaDeg += 180.0f;
  return 0;
}

bool deltaInverse(const Vec3 &pose, Vec3 &thetaRad) {
  float angle[3]{};
  if (deltaCalcAngleYZ(pose.v[0], pose.v[1], pose.v[2], angle[0]) != 0) return false;
  if (deltaCalcAngleYZ(pose.v[0] * Config::COS120 + pose.v[1] * Config::SIN120,
                       pose.v[1] * Config::COS120 - pose.v[0] * Config::SIN120,
                       pose.v[2], angle[1]) != 0) return false;
  if (deltaCalcAngleYZ(pose.v[0] * Config::COS120 - pose.v[1] * Config::SIN120,
                       pose.v[1] * Config::COS120 + pose.v[0] * Config::SIN120,
                       pose.v[2], angle[2]) != 0) return false;
  for (uint8_t index = 0u; index < 3u; ++index) {
    thetaRad.v[index] = angle[index] * Config::DEG_TO_RAD_VALUE;
  }
  return true;
}

bool numericalJacobian(const Vec3 &thetaRad, Mat3 &jacobian) {
  for (uint8_t column = 0u; column < 3u; ++column) {
    Vec3 plus = thetaRad;
    Vec3 minus = thetaRad;
    plus.v[column] += Config::JACOBIAN_STEP_RAD;
    minus.v[column] -= Config::JACOBIAN_STEP_RAD;
    Vec3 posePlus{};
    Vec3 poseMinus{};
    if (!deltaForward(plus, posePlus) || !deltaForward(minus, poseMinus)) return false;
    for (uint8_t row = 0u; row < 3u; ++row) {
      jacobian.m[row][column] =
        (posePlus.v[row] - poseMinus.v[row]) /
        (2.0f * Config::JACOBIAN_STEP_RAD);
    }
  }
  return true;
}

bool dlsStep(const Mat3 &jacobian, const Vec3 &cartesianStep, Vec3 &jointStep) {
  Mat3 jjT = multiplyByTranspose(jacobian);
  const float lambdaSquared = Config::DLS_LAMBDA_MM_RAD *
                              Config::DLS_LAMBDA_MM_RAD;
  for (uint8_t diagonal = 0u; diagonal < 3u; ++diagonal) {
    jjT.m[diagonal][diagonal] += lambdaSquared;
  }
  Mat3 inverseTerm{};
  float ignoredDeterminant = 0.0f;
  if (!invert(jjT, inverseTerm, ignoredDeterminant)) return false;
  jointStep = transposeMatVec(jacobian, matVec(inverseTerm, cartesianStep));
  return true;
}

bool transposeStep(const Mat3 &jacobian, const Vec3 &cartesianStep,
                   Vec3 &jointStep) {
  // Un joystick centrado produce un paso cartesiano exactamente cero. Eso es
  // una orden valida de reposo, no una singularidad del Jacobiano.
  if (magnitude(cartesianStep) < 0.000001f) {
    jointStep = Vec3{{0.0f, 0.0f, 0.0f}};
    return true;
  }
  const Vec3 gradient = transposeMatVec(jacobian, cartesianStep);
  const Vec3 projected = matVec(jacobian, gradient);
  const float denominator = dot(projected, projected);
  if (denominator < 0.0000001f) return false;
  const float alpha = dot(cartesianStep, projected) / denominator;
  jointStep = scale(gradient, alpha);
  return isfinite(alpha);
}

float predictedResidual(const Vec3 &theta, const Vec3 &jointStep,
                        const Vec3 &targetPose) {
  Vec3 predictedPose{};
  if (!deltaForward(add(theta, jointStep), predictedPose)) return NAN;
  return magnitude(subtract(targetPose, predictedPose));
}

float thetaToRawValue(float thetaRad, uint8_t index) {
  const float thetaDeg = thetaRad * Config::RAD_TO_DEG_VALUE;
  const float corrected = thetaDeg - Config::THETA_HOME_DEG[index];
  return Config::SERVO_HOME_RAW[index] +
    Config::SERVO_SIGN[index] * corrected * Config::SERVO_UNITS_PER_DEG;
}

int thetaToRaw(float thetaRad, uint8_t index) {
  return static_cast<int>(roundf(thetaToRawValue(thetaRad, index)));
}

int rawMinForRange(RangeProfile profile) {
  switch (profile) {
    case RangeProfile::R2: return 390;
    case RangeProfile::R3: return 215; // experimental; guardia 10C=205
    default: return 400;
  }
}

int rawMaxForRange(RangeProfile profile) {
  switch (profile) {
    case RangeProfile::R2: return 580;
    case RangeProfile::R3: return 690; // margen sobre guardia 10C=700
    default: return 540;
  }
}

int selectedRawMin() { return rawMinForRange(rangeProfile); }

int selectedRawMax() { return rawMaxForRange(rangeProfile); }

bool currentReferenceFitsRange(RangeProfile profile) {
  const uint16_t *raw = nucleo.remoteRaw();
  const int minimum = rawMinForRange(profile);
  const int maximum = rawMaxForRange(profile);
  for (uint8_t index = 0u; index < 3u; ++index) {
    if (raw[index] < minimum || raw[index] > maximum) return false;
  }
  return true;
}

bool thetaRawSafe(const Vec3 &theta) {
  for (uint8_t index = 0u; index < 3u; ++index) {
    if (!isfinite(theta.v[index])) return false;
    const int raw = thetaToRaw(theta.v[index], index);
    if (raw < selectedRawMin() || raw > selectedRawMax()) return false;
  }
  return true;
}

bool thetaRawSafeForPose(const Vec3 &theta, const Vec3 &pose) {
  (void)pose;
  return thetaRawSafe(theta);
}

bool sequenceIsNewer(uint32_t candidate, uint32_t reference) {
  return static_cast<int32_t>(candidate - reference) > 0;
}

int16_t gamepadAxis(uint8_t index) {
  switch (index) {
    case 0u: return gamepad.lx;
    case 1u: return gamepad.ly;
    case 2u: return gamepad.rx;
    default: return gamepad.ry;
  }
}

bool axesCentered() {
  for (uint8_t index = 0u; index < 4u; ++index) {
    const int32_t relative = static_cast<int32_t>(gamepadAxis(index)) -
                             static_cast<int32_t>(gamepadNeutral[index]);
    if (abs(relative) > Config::CENTER_THRESHOLD) return false;
  }
  return true;
}

bool axesCenteredLongEnough() {
  return centeredSinceMs != 0u &&
         millis() - centeredSinceMs >= Config::CENTER_TIME_MS;
}

void resetAxisHysteresis() {
  axisDriveActive[0] = false;
  axisDriveActive[1] = false;
  axisDriveActive[2] = false;
  filteredManualCommand = Vec3{{0.0f, 0.0f, 0.0f}};
  heldManualCommand = Vec3{{0.0f, 0.0f, 0.0f}};
}

float normalizedAxis(int16_t raw, int16_t neutral, bool &driveActive) {
  int32_t relative = static_cast<int32_t>(raw) - static_cast<int32_t>(neutral);
  if (relative > Config::STICK_MAX) relative = Config::STICK_MAX;
  if (relative < -Config::STICK_MAX) relative = -Config::STICK_MAX;
  const int32_t magnitudeRaw = abs(relative);

  if (!driveActive) {
    if (magnitudeRaw <= Config::STICK_ENGAGE_THRESHOLD) return 0.0f;
    driveActive = true;
  } else if (magnitudeRaw <= Config::STICK_RELEASE_THRESHOLD) {
    driveActive = false;
    return 0.0f;
  }

  const float normalized =
    static_cast<float>(magnitudeRaw - Config::STICK_RELEASE_THRESHOLD) /
    static_cast<float>(Config::STICK_MAX - Config::STICK_RELEASE_THRESHOLD);
  const float limited = fminf(normalized, 1.0f);
  const float shaped = 0.35f * limited + 0.65f * limited * limited;
  return (relative < 0 ? -1.0f : 1.0f) * shaped;
}

void resetCenterCalibration() {
  centerCalibrationActive = false;
  centerCalibrated = false;
  centerCalibrationStartedMs = 0u;
  centerCalibrationSamples = 0u;
  centeredSinceMs = 0u;
  for (uint8_t index = 0u; index < 4u; ++index) {
    gamepadNeutral[index] = 0;
    centerCalibrationSum[index] = 0;
    centerCalibrationMin[index] = 0;
    centerCalibrationMax[index] = 0;
  }
}

void printCenterDiagnostic(const __FlashStringHelper *prefix) {
  Serial.print(prefix);
  Serial.print(F(" axes="));
  Serial.print(gamepad.lx); Serial.print(',');
  Serial.print(gamepad.ly); Serial.print(',');
  Serial.print(gamepad.rx); Serial.print(',');
  Serial.print(gamepad.ry);
  Serial.print(F(" neutral="));
  Serial.print(gamepadNeutral[0]); Serial.print(',');
  Serial.print(gamepadNeutral[1]); Serial.print(',');
  Serial.print(gamepadNeutral[2]); Serial.print(',');
  Serial.print(gamepadNeutral[3]);
  Serial.print(F(" delta="));
  for (uint8_t index = 0u; index < 4u; ++index) {
    if (index != 0u) Serial.print(',');
    Serial.print(static_cast<int32_t>(gamepadAxis(index)) - gamepadNeutral[index]);
  }
  Serial.print(F(" centered="));
  Serial.println(axesCentered() ? 1 : 0);
}

void startCenterCalibration() {
  centerCalibrationActive = true;
  centerCalibrated = false;
  centerCalibrationStartedMs = millis();
  centerCalibrationSamples = 0u;
  centeredSinceMs = 0u;
  for (uint8_t index = 0u; index < 4u; ++index) {
    gamepadNeutral[index] = 0;
    centerCalibrationSum[index] = 0;
    centerCalibrationMin[index] = 32767;
    centerCalibrationMax[index] = -32768;
  }
  Serial.println(F("[CENTER CAL START] Suelta ambos joysticks durante 2.5 s; no habra movimiento."));
}

void finishCenterCalibration() {
  centerCalibrationActive = false;
  bool valid = centerCalibrationSamples >= Config::CENTER_CALIBRATION_MIN_SAMPLES;
  int16_t candidateNeutral[4] = {0, 0, 0, 0};
  int16_t span[4] = {0, 0, 0, 0};
  if (centerCalibrationSamples > 0u) {
    for (uint8_t index = 0u; index < 4u; ++index) {
      candidateNeutral[index] = static_cast<int16_t>(
        centerCalibrationSum[index] / static_cast<int32_t>(centerCalibrationSamples));
      span[index] = centerCalibrationMax[index] - centerCalibrationMin[index];
      if (span[index] > Config::CENTER_CALIBRATION_MAX_SPAN ||
          abs(static_cast<int32_t>(candidateNeutral[index])) >
            Config::CENTER_CALIBRATION_MAX_OFFSET) {
        valid = false;
      }
    }
  }

  Serial.print(valid ? F("[CENTER CAL OK]") : F("[CENTER CAL FAILED]"));
  Serial.print(F(" neutral="));
  for (uint8_t index = 0u; index < 4u; ++index) {
    if (index != 0u) Serial.print(',');
    Serial.print(candidateNeutral[index]);
  }
  Serial.print(F(" span="));
  for (uint8_t index = 0u; index < 4u; ++index) {
    if (index != 0u) Serial.print(',');
    Serial.print(span[index]);
  }
  Serial.print(F(" samples="));
  Serial.println(centerCalibrationSamples);

  if (valid) {
    memcpy(gamepadNeutral, candidateNeutral, sizeof(gamepadNeutral));
    centerCalibrated = true;
    if (axesCentered()) centeredSinceMs = millis();
  } else {
    for (uint8_t index = 0u; index < 4u; ++index) gamepadNeutral[index] = 0;
    centerCalibrated = false;
    centeredSinceMs = 0u;
    Serial.println(F("[CENTER CAL HELP] No muevas el control; requiere >=8 muestras, span<=30 y offset<=140."));
  }
}

void sampleCenterCalibration() {
  if (!centerCalibrationActive) return;
  for (uint8_t index = 0u; index < 4u; ++index) {
    const int16_t value = gamepadAxis(index);
    centerCalibrationSum[index] += value;
    if (value < centerCalibrationMin[index]) centerCalibrationMin[index] = value;
    if (value > centerCalibrationMax[index]) centerCalibrationMax[index] = value;
  }
  ++centerCalibrationSamples;
  if (millis() - centerCalibrationStartedMs >= Config::CENTER_CALIBRATION_MS) {
    finishCenterCalibration();
  }
}

float approach(float current, float target, float maximumChange) {
  if (current < target) return fminf(current + maximumChange, target);
  if (current > target) return fmaxf(current - maximumChange, target);
  return current;
}

const char *modeName() {
  return manualMode == ManualMode::Z_ONLY ? "Z" : "XYZ";
}

const char *operationName() {
  return operationMode == OperationMode::MANUAL ? "MANUAL" : "AUTOMATICO";
}

bool automaticOperation() { return operationMode != OperationMode::MANUAL; }

const char *workspaceAssistName() {
  switch (workspaceAssistMode) {
    case WorkspaceAssistMode::LOWER_FOR_XY: return "BAJANDO PARA XY";
    case WorkspaceAssistMode::RETREAT_FOR_Z: return "RETIRANDO PARA Z";
    case WorkspaceAssistMode::COMPONENT_PROJECTED: return "LIMITE PROYECTADO";
    default: return "";
  }
}

void setWorkspaceAssistMode(WorkspaceAssistMode mode) {
  if (workspaceAssistMode == mode) return;
  workspaceAssistMode = mode;
  oledDirty = true;
  /* No imprimir la transicion a NONE ni alternancias rapidas entre la ayuda
     radial y la proyeccion. Esas rafagas de Serial y OLED perturbaban el lazo
     precisamente al tocar una frontera. */
  if (mode == WorkspaceAssistMode::NONE) return;
  const uint32_t now = millis();
  if (lastWorkspaceAssistPrintMs != 0u &&
      now - lastWorkspaceAssistPrintMs < Config::WORKSPACE_PRINT_PERIOD_MS) {
    return;
  }
  lastWorkspaceAssistPrintMs = now;
  if (mode == WorkspaceAssistMode::LOWER_FOR_XY) {
    Serial.println(F("[WORKSPACE ASSIST] XY solicitado en techo; bajando primero hasta Z=-300."));
  } else if (mode == WorkspaceAssistMode::RETREAT_FOR_Z) {
    Serial.println(F("[WORKSPACE ASSIST] Z bloqueado por limite RAW lateral; retirada minima hacia el centro."));
  } else if (mode == WorkspaceAssistMode::COMPONENT_PROJECTED) {
    Serial.println(F("[WORKSPACE ASSIST] Componente exterior bloqueada; se conservan los ejes tangenciales validos."));
  }
}

const char *speedName() {
  switch (speedProfile) {
    case SpeedProfile::NORMAL: return "NORMAL 30 mm/s";
    case SpeedProfile::SPEED_40: return "PRUEBA 40 mm/s";
    case SpeedProfile::SPEED_50: return "PRUEBA 50 mm/s";
    case SpeedProfile::SPEED_60: return "PRUEBA 60 mm/s";
    case SpeedProfile::SPEED_80: return "PRUEBA 80 mm/s";
    default: return "PRECISION 10 mm/s";
  }
}

float selectedSpeedMmS() {
  switch (speedProfile) {
    case SpeedProfile::NORMAL: return Config::NORMAL_SPEED_MM_S;
    case SpeedProfile::SPEED_40: return Config::SPEED_40_MM_S;
    case SpeedProfile::SPEED_50: return Config::SPEED_50_MM_S;
    case SpeedProfile::SPEED_60: return Config::SPEED_60_MM_S;
    case SpeedProfile::SPEED_80: return Config::SPEED_80_MM_S;
    default: return Config::PRECISION_SPEED_MM_S;
  }
}

const char *rangeName() {
  switch (rangeProfile) {
    case RangeProfile::R2: return "R2 390..580";
    case RangeProfile::R3: return "R3 EXPERIMENTAL 215..690";
    default: return "R1 400..540";
  }
}

uint16_t rangeOption() {
  switch (rangeProfile) {
    case RangeProfile::R2: return DELTA_STREAM_OPTION_RANGE_R2;
    case RangeProfile::R3: return DELTA_STREAM_OPTION_RANGE_R3;
    default: return DELTA_STREAM_OPTION_RANGE_R1;
  }
}

uint16_t streamOption() {

  return DELTA_STREAM_OPTION_XYZ_MANUAL | DELTA_STREAM_OPTION_RANGE_R3 |
    ((operationMode == OperationMode::AUTO_PICKUP && autoAuthorized && !automaticBandMove)
       ? DELTA_STREAM_OPTION_AUTOMATIC : 0u);
}

const char *menuItemName(MenuItem item) {
  switch (item) {
    case MenuItem::HOME: return "HOME";
    case MenuItem::BAND_SAFE: return "PLANO SEGURO BANDA";
    case MenuItem::OPERATION: return "OPERACION";
    case MenuItem::SPEED: return "VELOCIDAD";
    case MenuItem::CAMERA: return "CALIBRAR CAMARA";
    case MenuItem::ARM_START: return "ARMAR E INICIAR";
    case MenuItem::STATUS: return "STATUS";
    case MenuItem::GRIP_Z: return "PALA: GIRO Z";
    case MenuItem::GRIP_X: return "PALA: INCLINAR X";
    case MenuItem::GRIP_PUSH: return "PALA: PALO";
    case MenuItem::GRIP_MANUAL: return "PALA MANUAL (3 RC)";
    case MenuItem::GRIP_RECOVER: return "RECUPERAR PALA";
    case MenuItem::FAULT_RECOVER: return "RECUPERAR FAULT";
    default: return "?";
  }
}

void printStatus();
bool workspacePointValid(const Vec3 &pose);

void printMenu() {
  Serial.println();
  Serial.println(F("========== MENU GAMEPAD ETAPA 10 =========="));
  for (uint8_t index = 0u; index < static_cast<uint8_t>(MenuItem::COUNT);
       ++index) {
    const MenuItem item = static_cast<MenuItem>(index);
    Serial.print(item == selectedMenuItem ? F(" > ") : F("   "));
    Serial.print(index + 1u);
    Serial.print(F(". "));
    Serial.print(menuItemName(item));
    if (item == MenuItem::OPERATION) {
      Serial.print(F(" [")); Serial.print(operationName()); Serial.print(']');
    } else if (item == MenuItem::SPEED) {
      Serial.print(F(" [")); Serial.print(speedName()); Serial.print(']');
    } else if (item == MenuItem::CAMERA) {
      Serial.print(F(" [")); Serial.print(cameraAlignment.stateName()); Serial.print(']');
    }
    Serial.println();
  }
  Serial.println(F("D-pad: navegar | X: elegir/confirmar | CIRCULO: cancelar/desarmar"));
  Serial.println(F("MANUAL: tras VERIFY PASS puedes rearmar desde la pose actual. AUTO: iniciar desde HOME verificado."));
  Serial.println(F("XYZ + R3 EXP 215..690 FIJOS. HOME, PLANO, CAMARA y ARMAR: doble X en 5 s."));
  Serial.println(F("PALA CAL: doble X, un RC. PALA MANUAL: tres RC; DELTA desarmado."));
  Serial.println(F("12. RECUPERAR PALA: leer consignas Nucleo y doble X tras revisar referencia/V_GRIP apagado; NO activa PWM."));
  Serial.println(F("13. RECUPERAR FAULT: doble X, solo lecturas HTS estables; PWM OFF, luego HOME/ARM aparte. No borra fallos de comunicacion."));
  Serial.println(F("PALA MANUAL JOG: cruz izq/der Z; L1/R1 X; L2/R2 palo. Soltar detiene; X/TRI no mueven."));
  Serial.println(F("TRIANGULO: ver/marcar XYZ, RAW DELTA y us de los tres RC; no mueve."));
  Serial.println(F("============================================"));
  oledDirty = true;
}

void renderOled() {
  if (!oledReady) return;
  oled.clear();
  char line[22];
  if (nucleo.remoteFault()) {
    oled.line(0u, "FAULT NUCLEO", true);
    oled.line(2u, "REINICIA 10C");
    oled.line(4u, "MOTORES OFF");
  } else if (autoHomeWarningUntilMs != 0u &&
             static_cast<int32_t>(autoHomeWarningUntilMs - millis()) > 0) {
    oled.line(0u, "AUTO: REQUIERE HOME", true);
    oled.line(2u, "HOME + DOBLE X");
    oled.line(4u, "ESPERA VERIFY PASS");
    oled.line(6u, "Z=-287 SIN DESCENSO");
  } else if (preparingManualPala()) {
    oled.line(0u, "PREPARANDO PALA", true);
    oled.line(1u, startPhase == StartPhase::WAIT_PALA_REF ? "SINCRONIZANDO..." :
                 startPhase == StartPhase::WAIT_PALA_POSE ? "MOVIENDO RC..." : "INICIANDO MANUAL");
    snprintf(line, sizeof(line), "Z:%u ->1730 us", gripUs[0]); oled.line(2u, line);
    snprintf(line, sizeof(line), "X:%u ->1390 us", gripUs[1]); oled.line(3u, line);
    snprintf(line, sizeof(line), "PALO:%u ->1360 us", gripUs[2]); oled.line(4u, line);
    oled.line(5u, "DELTA DESARMADO");
    oled.line(6u, "OPTIONS/CIRC: CANCEL");
    oled.line(7u, "CONSIGNA, NO FEEDBACK");
  } else if (menuOpen && selectedMenuItem == MenuItem::GRIP_RECOVER &&
             confirmation == Confirmation::GRIP_RECOVER) {
    oled.line(0u, "RECUPERAR SOLO REF", true);
    oled.line(1u, "V_GRIP APAGADO");
    oled.line(2u, "REVISAR REF FISICA");
    snprintf(line, sizeof(line), "Z: %u us", gripRecoveryPreviewUs[0]); oled.line(3u, line);
    snprintf(line, sizeof(line), "X: %u us", gripRecoveryPreviewUs[1]); oled.line(4u, line);
    snprintf(line, sizeof(line), "PALO: %u us", gripRecoveryPreviewUs[2]); oled.line(5u, line);
    oled.line(6u, "X:CONF CIRC:CANCEL");
    oled.line(7u, "NO PWM / NO MOVIMIENTO");
  } else if (showValueScreen) {
    oled.line(0u, "VALORES / TRI: SALIR", true);
    snprintf(line, sizeof(line), "X:%5.1f Y:%5.1f", referencePose.v[0], referencePose.v[1]);
    oled.line(1u, line);
    snprintf(line, sizeof(line), "Z:%6.1f REF:%s", referencePose.v[2],
             nucleo.homeValid() ? "OK" : "NO");
    oled.line(2u, line);
    const uint16_t *raw = nucleo.remoteRaw();
    snprintf(line, sizeof(line), "CMD %u %u %u", raw[0], raw[1], raw[2]);
    oled.line(3u, line);
    raw = nucleo.measuredRawSnapshot();
    if (nucleo.feedbackSnapshotKnown())
      snprintf(line, sizeof(line), "ULT M %u %u %u", raw[0], raw[1], raw[2]);
    else snprintf(line, sizeof(line), "ULT M: SIN VERIFICAR");
    oled.line(4u, line);
    snprintf(line, sizeof(line), "RC Z:%u X:%u us", gripUs[0], gripUs[1]);
    oled.line(5u, line);
    snprintf(line, sizeof(line), "PALO:%u us CAL:%u", gripUs[2],
             gripPhase == GripPhase::OFF ? 0u : gripChannel);
    oled.line(6u, line);
    beltOledLine(line, sizeof(line));
    oled.line(7u, line);
  } else if (gripPhase != GripPhase::OFF && gripGroupMode && !gripRuntime && !menuOpen) {
    oled.line(0u,"PALA MANUAL / 3 RC",true);
    snprintf(line,sizeof(line),"Z:%u X:%u us",gripUs[0],gripUs[1]);oled.line(1u,line);
    snprintf(line,sizeof(line),"PALO:%u us",gripUs[2]);oled.line(2u,line);
    oled.line(3u,"CRUZ IZQ-/DER+ Z");
    oled.line(4u,"L2-/R2+ PALO");
    oled.line(5u,"L1-/R1+ X; SOLTAR=0");
    oled.line(6u,"OPTIONS/CIRC: PWM OFF");
    oled.line(7u,"LIBRE: VIGILAR TOPES");
  } else if (gripPhase != GripPhase::OFF && !gripRuntime && !menuOpen) {
    oled.line(0u, "PALA CALIBRACION", true);
    snprintf(line, sizeof(line), "SERVO %u: %s", gripChannel,
             gripChannel == 1u ? "GIRO Z" : gripChannel == 2u ? "INCLINA X" : "PALO");
    oled.line(1u, line);
    snprintf(line, sizeof(line), "CONSIGNA %u us", gripUs[gripChannel - 1u]);
    oled.line(2u, line);
    snprintf(line, sizeof(line), "RANGO %u..%u us", GRIP_MIN_US[gripChannel - 1u],
             GRIP_MAX_US[gripChannel - 1u]);
    oled.line(3u, line);
    oled.line(4u, gripChannel == 1u ? "CRUZ IZQ - / DER +" :
                    gripChannel == 2u ? "L1 - / R1 +" : "L2 - / R2 +");
    oled.line(5u, "X: MARCAR EN SERIAL");
    oled.line(6u, "OPTIONS/CIRC: PWM OFF");
    oled.line(7u, gripPhase == GripPhase::ACTIVE ? "NO ES FEEDBACK" : "ESPERA ACK...");
  } else if (automaticOperation() &&
             cameraAlignment.state() != CameraAlignment::State::READY &&
             cameraAlignment.state() != CameraAlignment::State::OFF) {
    /* Durante la calibracion la OLED se convierte en un panel de diagnostico.
       Asi se puede comprobar UART, objetos crudos y tags aceptados sin PC. */
    oled.line(0u, "AUTO CAM DIAG", true);
    snprintf(line, sizeof(line), "EST:%s", cameraAlignment.stateName());
    oled.line(1u, line);
    snprintf(line, sizeof(line), "T0:%u/5 T1:%u/5",
             cameraAlignment.calibrationSamples(0u),
             cameraAlignment.calibrationSamples(1u));
    oled.line(2u, line);
    snprintf(line, sizeof(line), "T2:%u/5 T3:%u/5",
             cameraAlignment.calibrationSamples(2u),
             cameraAlignment.calibrationSamples(3u));
    oled.line(3u, line);
    snprintf(line, sizeof(line), "RAW:%u ID:%d TAG:%d",
             cameraAlignment.lastRawTagCount(),
             cameraAlignment.lastRawLearnedId(),
             cameraAlignment.lastRawTagContent());
    oled.line(4u, line);
    snprintf(line, sizeof(line), "PX:%d PY:%d",
             cameraAlignment.lastRawTagX(), cameraAlignment.lastRawTagY());
    oled.line(5u, line);
    snprintf(line, sizeof(line), "F:%lu O:%lu BAD:%lu",
             static_cast<unsigned long>(cameraAlignment.tagFramesSeen()),
             static_cast<unsigned long>(cameraAlignment.tagObjectsSeen()),
             static_cast<unsigned long>(cameraAlignment.invalidTagContents()));
    oled.line(6u, line);
    oled.line(7u,
      cameraAlignment.state() == CameraAlignment::State::FAULT
        ? "FAULT: RECALIBRAR" : "LEYENDO TAGS...");
  } else if (menuOpen) {
    if (!gamepadFresh)
      snprintf(line, sizeof(line), "MENU / MANDO OFF");
    else if (!nucleo.ready())
      snprintf(line, sizeof(line), "MENU / ENLACE NUC OFF");
    else if (operationMode == OperationMode::MANUAL &&
             (gripReferenceUncertain || gripStateQueryNeeded))
      snprintf(line, sizeof(line), "MENU: ARM PREPARA PALA");
    else if (!simulationValid || !nucleo.homeValid())
      snprintf(line, sizeof(line), "MENU: HOME PENDIENTE");
    else if (automaticOperation() &&
        (cameraAlignment.observedPieceId() == 1u ||
         cameraAlignment.observedPieceId() == 2u))
      snprintf(line, sizeof(line), "AUTO %s %u/3",
               cameraPieceClassName(cameraAlignment.observedPieceId()),
               cameraAlignment.observationSamples());
    else if (beltDisplayKnown)
      snprintf(line, sizeof(line), "MENU XYZ R3 B:%u", beltDisplayTenths / 10u);
    else
      snprintf(line, sizeof(line), "MENU XYZ R3 B:--");
    oled.line(0u, line, true);
    const uint8_t count = static_cast<uint8_t>(MenuItem::COUNT);
    uint8_t first = static_cast<uint8_t>(selectedMenuItem);
    if (first > 3u) first -= 3u; else first = 0u;
    if (first + 7u > count) first = count > 7u ? count - 7u : 0u;
    for (uint8_t row = 1u; row < 8u && first + row - 1u < count; ++row) {
      const MenuItem item = static_cast<MenuItem>(first + row - 1u);
      const char marker = item == selectedMenuItem ? '>' : ' ';
      const char *label = menuItemName(item);
      if (item == MenuItem::OPERATION)
        snprintf(line, sizeof(line), "%c OP:%s", marker, operationName());
      else if (item == MenuItem::SPEED)
        snprintf(line, sizeof(line), "%c VEL:%.0f MM/S", marker, selectedSpeedMmS());
      else if (item == MenuItem::CAMERA) {
        if (cameraAlignment.state() == CameraAlignment::State::COLLECTING_TAGS)
          snprintf(line, sizeof(line), "%c CAM %u/%u/%u/%u", marker,
                   cameraAlignment.calibrationSamples(0u),
                   cameraAlignment.calibrationSamples(1u),
                   cameraAlignment.calibrationSamples(2u),
                   cameraAlignment.calibrationSamples(3u));
        else if (cameraAlignment.state() == CameraAlignment::State::READY &&
                 (cameraAlignment.observedPieceId() == 1u ||
                  cameraAlignment.observedPieceId() == 2u))
          snprintf(line, sizeof(line), "%c %s %u/3", marker,
                   cameraPieceClassName(cameraAlignment.observedPieceId()),
                   cameraAlignment.observationSamples());
        else
          snprintf(line, sizeof(line), "%c CAM:%s", marker,
                   cameraAlignment.stateName());
      }
      else if (item == MenuItem::GRIP_RECOVER) {
        snprintf(line, sizeof(line), "%c REC PALA %s", marker,
          confirmation == Confirmation::GRIP_RECOVER ? "DOBLE X" :
          gripReferenceUncertain ? "PEND" : "OK");
      }
      else if (item == MenuItem::FAULT_RECOVER) {
        snprintf(line, sizeof(line), "%c REC FAULT %s", marker,
          nucleo.recoveryActive() ? "LEE" :
          confirmation == Confirmation::FAULT_RECOVER ? "2X" : "");
      }
      else
        snprintf(line, sizeof(line), "%c %s", marker, label);
      oled.line(row, line, item == selectedMenuItem);
    }
  } else {
    if (!gamepadFresh) snprintf(line, sizeof(line), "MANDO DESCONECTADO");
    else if (!nucleo.ready()) snprintf(line, sizeof(line), "SIN ENLACE NUCLEO");
    else if (stopRequested || streamEndVerificationPending)
      snprintf(line, sizeof(line), "PARANDO / VERIFICAR");
    else snprintf(line, sizeof(line), "%s %s", operationName(),
             runEnabled ? "RUN" : "PARADO");
    oled.line(0u, line, runEnabled);
    snprintf(line, sizeof(line), "GP:%s NUC:%s", gamepadFresh ? "OK" : "OFF",
             nucleo.ready() ? "OK" : "OFF");
    oled.line(1u, line);
    snprintf(line, sizeof(line), "X:%6.1f Y:%6.1f", referencePose.v[0], referencePose.v[1]);
    oled.line(2u, line);
    snprintf(line, sizeof(line), "Z:%6.1f V:%3.0f", referencePose.v[2],
             automaticOperation()
               ? AutoSpeedProfile::cruiseLimit(autoPickup.state(),
                   AutoPickupSetup::speedProfileConfig()) : selectedSpeedMmS());
    oled.line(3u, line);
    const uint16_t *raw = nucleo.remoteRaw();
    snprintf(line, sizeof(line), "RAW %u %u %u", raw[0], raw[1], raw[2]);
    oled.line(4u, line);
    beltOledLine(line, sizeof(line));
    oled.line(5u, line);
    if (operationMode == OperationMode::AUTO_PICKUP) {
      if (cameraAlignment.state() == CameraAlignment::State::COLLECTING_TAGS ||
          cameraAlignment.state() == CameraAlignment::State::WAIT_TAG_MODE ||
          cameraAlignment.state() == CameraAlignment::State::SOLVING) {
        snprintf(line, sizeof(line), "T0:%u T1:%u",
                 cameraAlignment.calibrationSamples(0u),
                 cameraAlignment.calibrationSamples(1u));
        oled.line(6u, line);
        snprintf(line, sizeof(line), "T2:%u T3:%u",
                 cameraAlignment.calibrationSamples(2u),
                 cameraAlignment.calibrationSamples(3u));
        oled.line(7u, line);
      } else {
        snprintf(line,sizeof(line),"%s",AutoPickup::Sequence::stateName(autoPickup.state()));
        oled.line(6u,line);
        oled.line(7u,autoCapabilityPending ? "COMPROBANDO 10C..." : "OPTIONS/CIRC: STOP");
      }
    } else {
      if (gripRuntime)
        snprintf(line, sizeof(line), "RC:%u %u %u", gripUs[0], gripUs[1], gripUs[2]);
      else
        snprintf(line, sizeof(line), "%s R%u", modeName(),
                 static_cast<unsigned>(rangeProfile) + 1u);
      oled.line(6u, line);
      oled.line(7u, streamEndVerificationPending
        ? "VERIFICANDO"
        : workspaceAssistMode != WorkspaceAssistMode::NONE
          ? workspaceAssistName() : "OPTIONS: MENU");
    }
  }
  oledDirty = false;
}

void serviceOled() {
  const uint32_t now = millis();
  if (!oledReady) return;

  if (!oledFlushActive &&
      (oledDirty || now - lastOledMs >= Config::OLED_PERIOD_MS)) {
    renderOled();
    lastOledMs = now;
    oledFlushPage = 0u;
    oledFlushActive = true;
  }
  if (!oledFlushActive) return;

  /* Un ACK pendiente no debe congelar la interfaz (en particular sin UART).
     Una sola pagina ~3 ms a 400 kHz; UART sigue recibiendo en su buffer.
     Durante RUN conservamos margen antes del siguiente ciclo DELTA. */
  if (runEnabled &&
      (pendingSetpoint || now - lastControlMs >= Config::CONTROL_PERIOD_MS - 8u))
    return;
  const uint32_t pagePeriod = runEnabled
    ? Config::OLED_RUN_PAGE_PERIOD_MS : Config::OLED_IDLE_PAGE_PERIOD_MS;
  if (lastOledPageMs != 0u && now - lastOledPageMs < pagePeriod) return;
  lastOledPageMs = now;
  if (!oled.flushPage(oledFlushPage)) {
    oledReady = false;
    oledFlushActive = false;
    Serial.println(F("[OLED WARN] Error I2C; control continua por Serial."));
    return;
  }
  ++oledFlushPage;
  if (oledFlushPage >= 8u) oledFlushActive = false;
}

bool initializeCameraCommandSession() {
  trng_t rng;
  trng_init(&rng);
  bool ok = false;
  for (uint8_t attempt = 0u; attempt < 3u; ++attempt) {
    uint32_t candidate = 0u;
    size_t produced = 0u;
    const int rc = trng_get_bytes(&rng, reinterpret_cast<uint8_t *>(&candidate),
                                  sizeof(candidate), &produced);
    if (rc == 0 && produced == sizeof(candidate) && candidate != 0u) {
      cameraCommandSession = candidate;
      ok = true;
      break;
    }
  }
  trng_free(&rng);
  return ok;
}

bool sendCameraCommand(uint16_t command) {
  if (!apReady || cameraCommandSession == 0u) return false;
  CameraPacket packet{};
  packet.type = CAMERA_TYPE_COMMAND;
  packet.session = cameraCommandSession;
  packet.sequence = ++cameraCommandSequence;
  packet.value = command;
  cameraFinalize(&packet);
  if (!udp.beginPacket(espAddress, Config::ESP_UDP_PORT)) return false;
  udp.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  if (!udp.endPacket()) return false;
  lastCameraCommand = command;
  lastCameraCommandMs = millis();
  Serial.print(F("[CAM CMD] "));
  Serial.println(command);
  return true;
}

bool autoHomePositionVerified() {
  /* homeValid significa referencia conservada, NO que este en HOME.
     Exigir tambien destino HOME, modelo local y medicion del ultimo PASS.
     El snapshot es historico, no un sensor continuo contra movimiento manual. */
  if (!simulationValid || !nucleo.ready() || !nucleo.homeValid() ||
      nucleo.dryRun() || nucleo.remoteFault() || nucleo.busy() ||
      nucleo.armed() || nucleo.streamActive() || runEnabled ||
      !nucleo.feedbackSnapshotKnown()) return false;
  const Vec3 home{{Config::HOME_X_MM, Config::HOME_Y_MM, Config::HOME_Z_MM}};
  if (!isfinite(referencePose.v[0]) || !isfinite(referencePose.v[1]) ||
      !isfinite(referencePose.v[2]) ||
      magnitude(subtract(referencePose, home)) >
        Config::AUTO_HOME_POSITION_TOLERANCE_MM) return false;
  const uint16_t *command = nucleo.remoteRaw();
  const uint16_t *measured = nucleo.measuredRawSnapshot();
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (command[i] != Config::AUTO_HOME_RAW ||
        abs(static_cast<int>(measured[i]) - Config::AUTO_HOME_RAW) >
          Config::AUTO_HOME_TOLERANCE_RAW) return false;
  }
  const int spread = max(measured[0], max(measured[1], measured[2])) -
                     min(measured[0], min(measured[1], measured[2]));
  return spread <= Config::AUTO_HOME_SPREAD_MAX_RAW;
}

void showAutoHomeRequired() {
  Serial.println(F("[AUTO HOME REQUIRED] Ejecuta HOME (doble X), espera VERIFY PASS y vuelve a AUTOMATICO. Inicio seguro en 0,0,-287; recogida requiere validacion fisica."));
  Serial.print(F("[AUTO START POSE] current_xyz="));
  for (uint8_t i=0u;i<3u;++i) {
    if (i) Serial.print(',');
    Serial.print(referencePose.v[i],2);
  }
  Serial.print(F(" desired_raw="));
  for (uint8_t i=0u;i<3u;++i) {
    if (i) Serial.print(',');
    Serial.print(nucleo.remoteRaw()[i]);
  }
  Serial.println(F("; WAIT_PIECE no es HOME. Tras STOP no rearmar solo ni saltar HOME."));
  // Existing paged renderer shows the warning without delay()/blocking I2C.
  autoHomeWarningUntilMs = millis() + 2500u;
  oledDirty = true;
}

void startCameraCalibration() {
  // Nunca renovar una calibracion ni autorizar rebind durante movimiento/STOP.
  if (runEnabled || nucleo.armed() || nucleo.streamActive() ||
      startPhase != StartPhase::IDLE || homeRequestPending || stopRequested ||
      gripPhase != GripPhase::OFF) {
    Serial.println(F("[CAM CAL BLOCKED] Espera parada/desarmado y PWM OFF."));
    return;
  }
  cameraAlignment.beginCalibration();
  autoAuthorized = false;
  autoTargetActive = false;
  oledDirty = true;
  Serial.println(F("[CAM CAL] Busca AprilTags 0..3; se requieren 5 muestras estables por tag."));
}

void serviceCamera() {
  cameraAlignment.service();
  if (automaticOperation() && autoAuthorized &&
      cameraAlignment.lastPacketAgeMs() > Config::CAMERA_TIMEOUT_MS) {
    autoAuthorized = false;
    autoTargetActive = false;
    reopenMenuAfterStop = true;
    pauseSimulation(F("timeout de camara"));
    Serial.println(F("[AUTO SAFETY] Camara sin datos durante 1500 ms; desarmando."));
  }
  uint16_t command = 0u;
  if (cameraAlignment.consumeCommand(command)) {
    lastCameraCommand = command;
    lastCameraCommandMs = 0u;
    sendCameraCommand(command);
  }
  if ((cameraAlignment.state() == CameraAlignment::State::WAIT_TAG_MODE ||
       cameraAlignment.state() == CameraAlignment::State::WAIT_PIECE_MODE) &&
      millis() - lastCameraCommandMs >= Config::CAMERA_COMMAND_RETRY_MS &&
      lastCameraCommand != 0xFFFFu) {
    sendCameraCommand(lastCameraCommand);
  }
  // Observations are consumed only by the complete pickup coordinator, with
  // their encoder-associated timestamp. Camera reception never rearms DELTA.

  const uint32_t now = millis();
  if (automaticOperation() &&
      now - lastCameraDiagnosticMs >= Config::CAMERA_DIAGNOSTIC_PERIOD_MS) {
    lastCameraDiagnosticMs = now;
    if (cameraAlignment.state() == CameraAlignment::State::COLLECTING_TAGS ||
        cameraAlignment.state() == CameraAlignment::State::WAIT_TAG_MODE ||
        cameraAlignment.state() == CameraAlignment::State::SOLVING) {
      Serial.print(F("[CAM PROGRESS] state="));
      Serial.print(cameraAlignment.stateName());
      Serial.print(F(" tags="));
      for (uint8_t tag = 0u; tag < 4u; ++tag) {
        if (tag != 0u) Serial.print(',');
        Serial.print(tag); Serial.print(':');
        Serial.print(cameraAlignment.calibrationSamples(tag));
        Serial.print(F("/5"));
      }
      Serial.print(F(" packets=")); Serial.print(cameraPacketsValid);
      Serial.print(F(" raw_count=")); Serial.print(cameraAlignment.lastRawTagCount());
      Serial.print(F(" raw_id/tag="));
      Serial.print(cameraAlignment.lastRawLearnedId()); Serial.print('/');
      Serial.print(cameraAlignment.lastRawTagContent());
      Serial.print(F(" raw_xy=")); Serial.print(cameraAlignment.lastRawTagX());
      Serial.print(','); Serial.print(cameraAlignment.lastRawTagY());
      Serial.print(F(" frames/objects/bad_content="));
      Serial.print(cameraAlignment.tagFramesSeen()); Serial.print('/');
      Serial.print(cameraAlignment.tagObjectsSeen()); Serial.print('/');
      Serial.print(cameraAlignment.invalidTagContents());
      Serial.print(F(" age_ms="));
      const uint32_t age = cameraAlignment.lastPacketAgeMs();
      if (age == 0xFFFFFFFFu) Serial.println(F("N/A"));
      else Serial.println(age);
    } else if (cameraAlignment.state() == CameraAlignment::State::READY) {
      Serial.print(F("[CAM VIEW] state="));
      Serial.print(cameraAlignment.observationName());
      Serial.print(F(" detected/eligible="));
      Serial.print(cameraAlignment.detectedObjects()); Serial.print('/');
      Serial.print(cameraAlignment.eligibleObjects());
      Serial.print(F(" id=")); Serial.print(cameraAlignment.observedPieceId());
      Serial.print(F(" piece="));
      Serial.print(cameraPieceClassName(cameraAlignment.observedPieceId()));
      Serial.print(F(" samples="));
      Serial.print(cameraAlignment.observationSamples()); Serial.print(F("/3"));
      Serial.print(F(" robot_y=")); Serial.print(cameraAlignment.observedRobotY(), 2);
      Serial.print(F(" empty=")); Serial.print(cameraAlignment.emptyFrames());
      Serial.print(F(" latched=")); Serial.print(cameraAlignment.pieceLatched() ? 1 : 0);
      Serial.print(F(" pending=")); Serial.print(cameraAlignment.targetPending() ? 1 : 0);
      Serial.print(F(" authorized=")); Serial.println(autoAuthorized ? 1 : 0);
    } else {
      Serial.print(F("[CAM PROGRESS] state="));
      Serial.print(cameraAlignment.stateName());
      Serial.print(F(" fault="));
      Serial.println(cameraAlignment.fault());
    }
  }
}

void pauseSimulation(const __FlashStringHelper *reason) {
  nucleo.cancelPoint();
  autoLocalPhase=LocalPointPhase::IDLE;autoLocalToken=0u;autoDirectMove=false;
  // Freeze STOP context before abort clears the state. Never infer BOX15
  // merely from proximity or a previous checkpoint at a different target.
  if (!stopRequested) {
    pendingAutoStopFlags = 0u;
    if (operationMode == OperationMode::AUTO_PICKUP && autoRcActive &&
        (activeStreamOption & DELTA_STREAM_OPTION_AUTOMATIC)) {
      pendingAutoStopFlags = pendingSetpoint ? DELTA_AUTO_STOP_POLICY :
        AutoStopPolicy::requestFlags(autoPickup.state(),referencePose.v[1],referencePose.v[2]);
    }
  }
  // Cancel the future PTP destination before STOP. remoteRaw still describes
  // only the last short segment, never the distant IK endpoint.
  autoJointPtp.cancel();
  autoPtpToken = 0u;
  autoPtpSegmentPending = false;
  stopGripper();
  if (operationMode == OperationMode::AUTO_PICKUP || autoPickup.active() ||
      autoRcActive || autoRcPending || autoCapabilityPending) {
    autoAuthorized = false;
    if (autoPickup.active()) autoPickup.abort();
    autoRcActive = false;
    autoPiecePending = false;
    autoEncoderAssociated = false;
    autoPointHold = false;
    autoPointSettling = false;
    autoCapabilityPending = false;
    autoCapabilityPassed = false;
    // Keep an in-flight autoRcPending marker until its ACK is absorbed by the
    // AUTO completion branch. Never misinterpret a late AUTO ACK as MANUAL JOG.
    cameraAlignment.clearPendingTargetForArm();
  }
  if (runEnabled || nucleo.streamActive() || nucleo.armed()) {
    Serial.print(F("[PAUSE] "));
    Serial.println(reason);
  }
  runEnabled = false;
  stopRequested = nucleo.streamActive() || nucleo.armed() ||
                  nucleo.recoveryActive() ||
                  startPhase != StartPhase::IDLE || homeRequestPending ||
                  stopRequested;
  startPhase = StartPhase::IDLE;
  automaticBandMove = false;
  autoTargetActive = false;
  resetAxisHysteresis();
  commandedVelocity = Vec3{{0.0f, 0.0f, 0.0f}};
  oledDirty = true;
}

void revokePhysicalPermit(const __FlashStringHelper *reason) {
  pauseSimulation(reason);
}

void openMenu(bool afterStop = false) {
  stopGripper();
  showValueScreen = false;
  menuOpen = true;
  selectedMenuItem = MenuItem::HOME;
  confirmation = Confirmation::NONE;
  confirmationDeadlineMs = 0u;
  if (runEnabled || nucleo.streamActive() || nucleo.armed() ||
      startPhase != StartPhase::IDLE || nucleo.recoveryActive()) {
    reopenMenuAfterStop = true;
    pauseSimulation(afterStop ? F("STOP verificado") : F("OPTIONS abrio menu"));
  }
  printMenu();
}

bool requestStreamBegin() {
  if (gripPhase != GripPhase::OFF) return false;
  if (!simulationValid) {
    Serial.println(F("[STREAM BLOCKED] Modelo local invalido."));
    return false;
  } else if (!gamepadFresh || !axesCenteredLongEnough()) {
    Serial.println(F("[STREAM BLOCKED] Gamepad FRESH y centrado 500 ms requerido."));
    return false;
  } else if (!nucleo.ready() || nucleo.waitingResponse()) {
    Serial.println(F("[STREAM BLOCKED] Enlace Nucleo no disponible."));
    return false;
  } else if (nucleo.dryRun() || nucleo.remoteFault()) {
    Serial.println(F("[STREAM BLOCKED] Requiere 10C fisica, sin FAULT."));
    return false;
  } else if (!nucleo.homeValid() || !nucleo.armed()) {
    Serial.println(F("[STREAM BLOCKED] Ejecuta HOME y ARMAR E INICIAR desde el menu."));
    return false;
  } else {
    const uint16_t requestedOption = streamOption();
    if (!nucleo.sendStreamBegin(requestedOption)) return false;
    activeStreamOption = requestedOption;
    pendingAutoStopFlags = 0u;
    stopRequested = false;
    startPhase = StartPhase::WAIT_STREAM_ACK;
    Serial.print(F("[STREAM REQUEST] BEGIN "));
    Serial.print(modeName());
    Serial.print(F(" ")); Serial.print(rangeName());
    Serial.println(F(" enviado; espera ACK."));
    return true;
  }
}

void activateMenuItem() {
  if (!menuOpen) return;
  if (gripPhase != GripPhase::OFF) return;
  if (stopRequested || streamEndVerificationPending || homeRequestPending ||
      startPhase != StartPhase::IDLE) {
    Serial.println(F("[MENU WAIT] Espera cierre/verificacion de la orden actual; CIRCULO cancela HOME."));
    return;
  }
  const uint32_t now = millis();
  if (selectedMenuItem == MenuItem::FAULT_RECOVER) {
    if (!gamepadFresh || now - lastGamepadMs > 150u || !axesCenteredLongEnough() ||
        !centerCalibrated || centerCalibrationActive || gamepad.dpad || (gamepad.buttons & 0xF0u) ||
        !nucleo.ready() || nucleo.waitingResponse() || nucleo.recoveryActive() ||
        !nucleo.remoteFault() || nucleo.dryRun() || nucleo.armed() || nucleo.busy() ||
        nucleo.streamActive() || runEnabled || autoAuthorized || autoPickup.active()) {
      confirmation = Confirmation::NONE;
      Serial.println(F("[FAULT RECOVERY BLOCKED] FAULT de posicion, parado/desarmado, mando centrado y enlaces frescos requeridos."));
      return;
    }
    if (confirmation != Confirmation::FAULT_RECOVER || now > confirmationDeadlineMs) {
      confirmation = Confirmation::FAULT_RECOVER;
      confirmationDeadlineMs = now + Config::CONFIRM_TIMEOUT_MS;
      Serial.println(F("[FAULT RECOVERY CONFIRM] Revisa apoyo/holguras/cables. Segundo X en 5 s SOLO lee HTS; NO mueve ni rearma."));
      oledDirty = true;
      return;
    }
    confirmation = Confirmation::NONE;
    if (nucleo.sendFaultRecovery()) {
      simulationValid = false;
      autoAuthorized = autoTargetActive = false;
      pendingSetpoint = false;
      Serial.println(F("[FAULT RECOVERY WAIT] Lectura nueva de los 3 HTS, PWM OFF; espera resultado."));
    }
    return;
  }
  if (selectedMenuItem == MenuItem::GRIP_Z ||
      selectedMenuItem == MenuItem::GRIP_X ||
      selectedMenuItem == MenuItem::GRIP_PUSH ||
      selectedMenuItem == MenuItem::GRIP_MANUAL) {
    if (operationMode != OperationMode::MANUAL || !gamepadFresh ||
        !axesCenteredLongEnough() || !nucleo.ready() ||
        nucleo.waitingResponse() || nucleo.armed() || nucleo.busy() ||
        nucleo.streamActive() || nucleo.remoteFault() || nucleo.dryRun() ||
        gripReferenceUncertain || gripStateQueryNeeded || runEnabled || (gamepad.buttons & 0xF0u) ||
        gamepad.dpad) {
      Serial.println(F("[GRIP BLOCKED] MANUAL, DELTA parado, enlaces OK y controles libres. Si referencia incierta: menu12 RECUPERAR PALA, con V_GRIP apagado y revision fisica."));
      return;
    }
    if (confirmation != Confirmation::GRIP || now > confirmationDeadlineMs) {
      confirmation = Confirmation::GRIP;
      confirmationDeadlineMs = now + Config::CONFIRM_TIMEOUT_MS;
      Serial.println(F("[CONFIRM] PALA: segundo X en 5 s activa PWM del canal seleccionado. Corte V_GRIP accesible."));
      oledDirty = true;
      return;
    }
    confirmation = Confirmation::NONE;
    gripRuntime = false;
    gripGroupMode = selectedMenuItem == MenuItem::GRIP_MANUAL;
    if(gripGroupMode) {
      for(unsigned i=0;i<3;++i)
        gripTargetUs[i]=gripPendingGroupUs[i]=gripUs[i];
      gripChannel=1u;
      if(nucleo.sendGripperGroup(DELTA_MSG_GRIP_GROUP_BEGIN,gripUs)) {
        gripPhase=GripPhase::BEGIN_PENDING;gripEndSent=false;menuOpen=false;
        autoAuthorized=false;autoTargetActive=false;cameraAlignment.turnOff();
        showValueScreen=false;oledDirty=true;printGripper();
      } else gripGroupMode=false;
      return;
    }
    gripChannel = static_cast<uint8_t>(selectedMenuItem) -
                  static_cast<uint8_t>(MenuItem::GRIP_Z) + 1u;
    gripPendingUs = gripUs[gripChannel - 1u];
    if (nucleo.sendGripper(DELTA_MSG_GRIP_BEGIN, gripChannel, gripPendingUs)) {
      gripPhase = GripPhase::BEGIN_PENDING;
      gripEndSent = false;
      menuOpen = false;
      autoAuthorized = false;
      autoTargetActive = false;
      cameraAlignment.turnOff();
      oledDirty = true;
      printGripper();
    }
    return;
  }
  if (selectedMenuItem == MenuItem::GRIP_RECOVER) {
    if (operationMode != OperationMode::MANUAL || !gamepadFresh ||
        now - lastGamepadMs > 150u || centerCalibrationActive || !centerCalibrated ||
        !axesCenteredLongEnough() || (gamepad.buttons & 0xF0u) || gamepad.dpad ||
        !nucleo.ready() || nucleo.busy() || nucleo.remoteFault() || nucleo.dryRun() ||
        nucleo.armed() || nucleo.streamActive() || runEnabled) {
      confirmation = Confirmation::NONE;
      Serial.println(F("[PALA RECOVERY BLOCKED] MANUAL, parado/desarmado/sin FAULT y controles libres. No borra FAULT ni sustituye HOME."));
      return;
    }
    if (nucleo.waitingResponse()) {
      Serial.println(F("[PALA RECOVERY WAIT] Espera ACK y vuelve a pulsar X."));
      return;
    }
    if (!gripStateStopped()) {
      confirmation = Confirmation::NONE;
      gripStateQueryNeeded = true;
      Serial.println(F("[PALA RECOVERY WAIT] Falta lectura nueva con PWM OFF; requiere Nucleo10C v2.5."));
      return;
    }
    const uint16_t *remoteUs = nucleo.gripperStateUs();
    if (confirmation != Confirmation::GRIP_RECOVER || now > confirmationDeadlineMs ||
        gripRecoveryPreviewGeneration != nucleo.sessionGeneration() ||
        memcmp(gripRecoveryPreviewUs, remoteUs, sizeof(gripRecoveryPreviewUs)) != 0) {
      memcpy(gripRecoveryPreviewUs, remoteUs, sizeof(gripRecoveryPreviewUs));
      gripRecoveryPreviewGeneration = nucleo.sessionGeneration();
      confirmation = Confirmation::GRIP_RECOVER;
      confirmationDeadlineMs = now + Config::CONFIRM_TIMEOUT_MS;
      Serial.print(F("[PALA RECOVERY CONFIRM] Nucleo us_Z_X_PALO="));
      for (uint8_t i=0u;i<3u;++i) { if(i) Serial.print(','); Serial.print(remoteUs[i]); }
      Serial.println(F(". V_GRIP APAGADO, pala sostenida y referencia fisica comprobada: segundo X en5s acepta SOLO consignas. No mide ni centra; siguiente PWM puede mover si la referencia no coincide."));
      oledDirty = true;
      return;
    }

    for (uint8_t i=0u;i<3u;++i)
      gripUs[i] = gripTargetUs[i] = gripPendingGroupUs[i] = remoteUs[i];
    gripPendingUs = gripUs[gripChannel - 1u];
    gripReferenceUncertain = false;
    gripStateQueryNeeded = false;
    confirmation = Confirmation::NONE;
    Serial.println(F("[PALA RECOVERY OK] Consignas sincronizadas; PWM OFF, DELTA desarmado. HOME y ARM se confirman aparte; no encender V_GRIP sin referencia fisica segura."));
    oledDirty = true;
    printPositionMark();
    return;
  }
  if (selectedMenuItem == MenuItem::HOME) {
    if (confirmation != Confirmation::HOME || now > confirmationDeadlineMs) {
      confirmation = Confirmation::HOME;
      confirmationDeadlineMs = now + Config::CONFIRM_TIMEOUT_MS;
      Serial.println(F("[CONFIRM] Pulsa X otra vez en 5 s para ejecutar HOME."));
      return;
    }
    confirmation = Confirmation::NONE;
    if (!nucleo.ready() || nucleo.waitingResponse() || nucleo.dryRun() ||
        nucleo.remoteFault() || nucleo.busy() || nucleo.streamActive()) {
      Serial.println(F("[HOME BLOCKED] Requiere Nucleo fisica READY, libre y sin FAULT."));
    } else if (nucleo.sendHome()) {
      simulationValid = false;
      homeRequestPending = true;
      homeRequestAcknowledged = false;
      streamEndVerificationPending = false;
      Serial.println(F("[HOME REQUEST] Enviado; espera VERIFY PASS y home=1."));
    }
    return;
  }

  if (selectedMenuItem == MenuItem::BAND_SAFE) {
    if (confirmation != Confirmation::BAND_SAFE ||
        now > confirmationDeadlineMs) {
      confirmation = Confirmation::BAND_SAFE;
      confirmationDeadlineMs = now + Config::CONFIRM_TIMEOUT_MS;
      Serial.println(F("[CONFIRM] Pulsa X otra vez para ir a Z=-367.0 (1.5 mm bajo contacto calibrado)."));
      return;
    }
    confirmation = Confirmation::NONE;
    if (!gamepadFresh || !centerCalibrated || centerCalibrationActive ||
               !axesCenteredLongEnough()) {
      Serial.println(F("[BAND BLOCKED] Calibra y centra el mando antes del descenso."));
    } else if (!simulationValid || !nucleo.ready() || nucleo.waitingResponse() ||
               nucleo.dryRun() || nucleo.remoteFault() || !nucleo.homeValid() ||
               nucleo.busy() || nucleo.armed() || nucleo.streamActive() ||
               magnitude(subtract(referencePose,
                 Vec3{{Config::HOME_X_MM, Config::HOME_Y_MM,
                       Config::HOME_Z_MM}})) > 1.0f) {
      Serial.println(F("[BAND BLOCKED] Requiere HOME real, libre, desarmado y modelo en HOME."));
    } else if (nucleo.sendArm()) {
      automaticBandMove = true;
      startPhase = StartPhase::WAIT_ARM_ACK;
      Serial.println(F("[BAND REQUEST] ARM enviado; descenso automatico limitado a 30 mm/s."));
    }
    return;
  }

  if (selectedMenuItem == MenuItem::OPERATION) {
    confirmation = Confirmation::NONE;
    if (operationMode == OperationMode::MANUAL && !autoHomePositionVerified()) {
      showAutoHomeRequired();
      return;
    }
    operationMode = operationMode == OperationMode::MANUAL
      ? OperationMode::AUTO_PICKUP : OperationMode::MANUAL;
    autoAuthorized = false;
    autoTargetActive = false;
    if (automaticOperation()) {
      /* La homografia cubre la banda completa. AUTO no debe heredar R1 y
         rechazar una coordenada valida de camara; HOME cabe holgadamente en
         R3. Al volver a MANUAL se conserva R3 para permitir el mismo alcance. */
      startCameraCalibration();
      Serial.println(F("[OPERATION] AUTOMATICO: recogida completa de una pieza; configuracion fisica y 10C v2.6 requeridas. Permanece desarmado."));
    } else {
      cameraAlignment.turnOff();
      Serial.println(F("[OPERATION] MANUAL seleccionado."));
    }
    printMenu();
  } else if (selectedMenuItem == MenuItem::SPEED) {
    confirmation = Confirmation::NONE;
    speedProfile = static_cast<SpeedProfile>(
      (static_cast<uint8_t>(speedProfile) + 1u) %
      static_cast<uint8_t>(SpeedProfile::COUNT));
    Serial.print(F("[SPEED] ")); Serial.println(speedName());
    printMenu();
  } else if (selectedMenuItem == MenuItem::CAMERA) {
    if (confirmation != Confirmation::CAMERA || now > confirmationDeadlineMs) {
      confirmation = Confirmation::CAMERA;
      confirmationDeadlineMs = now + Config::CONFIRM_TIMEOUT_MS;
      Serial.println(F("[CONFIRM] Pulsa X otra vez para recalibrar AprilTags 0..3."));
      return;
    }
    confirmation = Confirmation::NONE;
    if (runEnabled || nucleo.armed() || nucleo.streamActive() || nucleo.busy()) {
      Serial.println(F("[CAM BLOCKED] Calibracion requiere robot detenido y desarmado."));
    } else if (!autoHomePositionVerified()) {
      showAutoHomeRequired();
    } else {
      if (operationMode == OperationMode::MANUAL) operationMode = OperationMode::AUTO_PICKUP;
      startCameraCalibration();
      printMenu();
    }
  } else if (selectedMenuItem == MenuItem::ARM_START) {
    if (confirmation != Confirmation::ARM_START ||
        now > confirmationDeadlineMs) {
      confirmation = Confirmation::ARM_START;
      confirmationDeadlineMs = now + Config::CONFIRM_TIMEOUT_MS;
      if (operationMode == OperationMode::MANUAL)
        Serial.println(F("[CONFIRM] ARM preparara PALA a1730/1390/1360us y luego DELTA+RC. Pala despejada/corte V_GRIP accesible; primer PWM puede mover desde pose desconocida. Centra y libera controles; segundo X en5s."));
      else
        Serial.println(F("[CONFIRM] AUTOMATICO: RC+DELTA, una pieza y retorno; bandeja despejada/corte V_GRIP accesible. Centra/libera controles; segundo X en5s."));
      return;
    }
    confirmation = Confirmation::NONE;
    if (!gamepadFresh || centerCalibrationActive || !centerCalibrated ||
        !axesCenteredLongEnough()) {
      Serial.println(F("[ARM BLOCKED] Gamepad calibrado y centrado 500 ms requerido."));
      printCenterDiagnostic(F("[CENTER DIAG]"));
      if (gamepadFresh && !centerCalibrationActive && !centerCalibrated) {
        startCenterCalibration();
      }
    } else if (automaticOperation() &&
               !cameraAlignment.ready()) {
      Serial.println(F("[ARM BLOCKED] AUTO requiere calibracion 0..3 y modelo 128 READY."));
    } else if (automaticOperation() &&
               !autoHomePositionVerified()) {
      showAutoHomeRequired();
    } else if (operationMode == OperationMode::AUTO_PICKUP &&
               (!AutoPickup::Sequence::configValid(AutoPickupSetup::configuration()) ||
                GRIP_AUTO_READY != 1u)) {
      Serial.println(F("[AUTO PICKUP BLOCKED] Configuracion invalida o ensayo deshabilitado sin confirmaciones. Revisar auto_pickup_config.h; no activa PWM ni ARM."));
    } else if (operationMode == OperationMode::AUTO_PICKUP &&
               ((gamepad.buttons & 0xF0u) || gamepad.dpad || now - lastGamepadMs > 150u)) {
      Serial.println(F("[ARM BLOCKED] AUTO RC requiere controles libres y mando fresco."));
    } else if (operationMode == OperationMode::MANUAL &&
               ((gamepad.buttons & 0xF0u) || gamepad.dpad ||
                now - lastGamepadMs > 150u)) {
      Serial.println(F("[ARM BLOCKED] PALA: libera gatillos/cruceta y requiere mando fresco. ARM sincroniza/prepara sin menu12."));
    } else if (!nucleo.ready()) {
      Serial.println(F("[ARM BLOCKED] Enlace Nucleo OFF; revisar UART y READY."));
    } else if (nucleo.remoteFault() || nucleo.dryRun()) {
      Serial.println(F("[ARM BLOCKED] Nucleo FAULT o DRY_RUN; no se puede armar."));
    } else if (nucleo.busy()) {
      Serial.println(F("[ARM BLOCKED] Nucleo ocupada: espera HOME/VERIFY PASS."));
    } else if (!simulationValid || !nucleo.homeValid()) {
      Serial.println(F("[ARM BLOCKED] HOME no verificado o cancelado. Ejecuta HOME y espera VERIFY PASS; RAW410 solo no basta."));
    } else if (nucleo.armed() || nucleo.streamActive()) {
      Serial.println(F("[ARM BLOCKED] Primero detener/desarmar la sesion actual."));
    } else if (nucleo.waitingResponse()) {
      armAfterAckPending = true;
      armAfterAckRequestedMs = millis();
      Serial.println(F("[ARM WAIT] Doble X confirmada; esperando ACK UART (max350ms), sin movimiento."));
    } else if (operationMode == OperationMode::MANUAL) {
      // This second X authorizes synchronization AND intentional RC positioning.
      // Always request a NEW snapshot, never trust a cached pre-reset reference.
      if (nucleo.sendGripperStateRequest()) {
        gripArmStartedMs = now;
        gripArmGeneration = nucleo.sessionGeneration();
        gripArmAtTargetMs = 0u;
        gripArmStateReceived = false;
        startPhase = StartPhase::WAIT_PALA_REF;
        showValueScreen = false;
        autoAuthorized = false;
        autoTargetActive = false;
        Serial.println(F("[PALA ARM PREP] Lectura interna y posicion1730/1390/1360us; DELTA sigue desarmado. OPTIONS/CIRC cancela."));
        oledDirty = true;
      }
    } else if (operationMode == OperationMode::AUTO_PICKUP) {
      if (nucleo.sendGripperStateRequest()) {
        if (autoPickup.active()) autoPickup.abort();
        autoStartGeneration = nucleo.sessionGeneration();
        autoStartMs = now;
        autoSnapshotReceived = false;
        autoRcPending = autoRcBeginPending = autoRcActive = false;
        autoCapabilityPending = false;
        autoCapabilityPassed = false;
        autoActionToken = autoCheckpointToken = 0u;
        autoPointHold = autoPiecePending = autoEncoderAssociated = false;
        autoPointSettling = false;
        cameraAlignment.clearPendingTargetForArm();
        startPhase = StartPhase::WAIT_AUTO_REF;
        Serial.println(F("[AUTO PICKUP REF] Sincronizando consignas RC sin PWM; ARM solo tras validar referencia."));
      }
    }
  } else if (selectedMenuItem == MenuItem::STATUS) {
    confirmation = Confirmation::NONE;
    showValueScreen = true;
    oledDirty = true;
    if (!nucleo.waitingResponse() && !nucleo.busy()) nucleo.sendStatusRequest();
    printStatus();
    printPositionMark();
  }
}

void serviceMenuArm() {
  if (!armAfterAckPending) return;
  const uint32_t now = millis();
  if (!menuOpen || selectedMenuItem != MenuItem::ARM_START || !gamepadFresh ||
      now - lastGamepadMs > 150u || now - armAfterAckRequestedMs > 350u ||
      stopRequested || streamEndVerificationPending || homeRequestPending ||
      startPhase != StartPhase::IDLE || !nucleo.ready() || nucleo.remoteFault()) {
    armAfterAckPending = false;
    Serial.println(F("[ARM CANCELLED] Espera ACK cancelada; permanece desarmado. Repite doble X con enlaces estables."));
    return;
  }
  if (nucleo.waitingResponse()) return;
  armAfterAckPending = false;
  // Continue only the explicitly confirmed request; recheck every ARM guard.
  confirmation = Confirmation::ARM_START;
  confirmationDeadlineMs = now + 1u;
  activateMenuItem();
}

void abortManualPalaPreparation(const __FlashStringHelper *reason) {
  gripReferenceUncertain = true;
  gripStateQueryNeeded = true;
  gripArmStateReceived = false;
  menuOpen = true;
  reopenMenuAfterStop = false;
  Serial.print(F("[PALA ARM CANCELLED] ")); Serial.println(reason);
  pauseSimulation(reason); // cancels phase; removes PWM, no later auto-retry
}

void serviceManualPalaPreparation() {
  if (!preparingManualPala()) return;
  const uint32_t now = millis();
  if (operationMode != OperationMode::MANUAL || !gamepadFresh ||
      now - lastGamepadMs > 150u || !centerCalibrated ||
      centerCalibrationActive || !axesCentered() ||
      (gamepad.buttons & 0xF0u) || gamepad.dpad ||
      !menuOpen || selectedMenuItem != MenuItem::ARM_START ||
      !nucleo.ready() || nucleo.sessionGeneration() != gripArmGeneration ||
      nucleo.remoteFault() || nucleo.dryRun() || nucleo.busy() ||
      nucleo.armed() || nucleo.streamActive() || !nucleo.homeValid() ||
      !simulationValid || runEnabled || stopRequested ||
      homeRequestPending || streamEndVerificationPending ||
      now - gripArmStartedMs > GRIP_ARM_TIMEOUT_MS) {
    abortManualPalaPreparation(F("Permiso/controles/enlace/tiempo invalido; repetir doble X."));
    return;
  }
  if (nucleo.waitingResponse()) return;
  if (startPhase == StartPhase::WAIT_PALA_REF) {
    if (!gripArmStateReceived || !gripStateStopped()) {
      abortManualPalaPreparation(F("Sin snapshot RC nuevo/PWM OFF; requiere10C v2.5."));
      return;
    }
    const uint16_t *remoteUs = nucleo.gripperStateUs();
    for (uint8_t i=0u;i<3u;++i)
      gripUs[i] = gripPendingGroupUs[i] = remoteUs[i];
    gripTargetUs[0] = GRIP_Z_PICKUP_US;
    gripTargetUs[1] = GRIP_X_LEVEL_US;
    gripTargetUs[2] = GRIP_PUSH_WAIT_US;
    for (uint8_t i=0u;i<3u;++i) {
      if (gripTargetUs[i] < GRIP_MIN_US[i] || gripTargetUs[i] > GRIP_MAX_US[i]) {
        abortManualPalaPreparation(F("Pose RC fuera de rango."));
        return;
      }
    }
    gripReferenceUncertain = false; // explicit double-X ARM permit, not a passive read
    gripStateQueryNeeded = false;
    gripGroupMode = true; gripRuntime = false; gripChannel = 1u;
    gripEndSent = false;
    if (!nucleo.sendGripperGroup(DELTA_MSG_GRIP_GROUP_BEGIN, gripUs, false)) {
      abortManualPalaPreparation(F("No se pudo habilitar preparacion RC."));
      return;
    }
    gripPhase = GripPhase::BEGIN_PENDING;
    startPhase = StartPhase::WAIT_PALA_POSE;
    Serial.println(F("[PALA ARM SYNC] BEGIN usa consignas Nucleo; preparacion gradual sin ARM DELTA."));
    oledDirty = true;
  } else if (startPhase == StartPhase::WAIT_PALA_POSE) {
    if (gripPhase != GripPhase::ACTIVE) {
      abortManualPalaPreparation(F("PWM preparacion no activo."));
      return;
    }
    if (memcmp(gripUs, gripTargetUs, sizeof(gripUs)) == 0) {
      if (gripArmAtTargetMs == 0u) gripArmAtTargetMs = now;
      if (now - gripArmAtTargetMs >= GRIP_ARM_SETTLE_MS) {
        Serial.println(F("[PALA ARM POSE] ACK1730/1390/1360us +300ms; no feedback fisico. Cerrando permiso aislado."));
        startPhase = StartPhase::WAIT_PALA_END;
        stopGripper();
      }
    }
  } else if (startPhase == StartPhase::WAIT_PALA_ARM) {
    if (gripPhase != GripPhase::OFF || gripReferenceUncertain ||
        memcmp(gripUs, gripTargetUs, sizeof(gripUs)) != 0) {
      abortManualPalaPreparation(F("Cierre/referencia RC no confirmado."));
      return;
    }
    if (!nucleo.sendArm()) {
      abortManualPalaPreparation(F("ARM no enviado."));
      return;
    }
    startPhase = StartPhase::WAIT_ARM_ACK;
    Serial.println(F("[ARM REQUEST] Pala preparada; ARM DELTA, luego STREAM y PALA READY."));
    oledDirty = true;
  }
}

bool resetSimulation() {
  referencePose = Vec3{{Config::HOME_X_MM, Config::HOME_Y_MM, Config::HOME_Z_MM}};
  resetAxisHysteresis();
  commandedVelocity = Vec3{{0.0f, 0.0f, 0.0f}};
  runEnabled = false;
  simulationValid = deltaInverse(referencePose, simulatedThetaRad);
  Vec3 checkPose{};
  simulationValid = simulationValid && deltaForward(simulatedThetaRad, checkPose) &&
                    magnitude(subtract(referencePose, checkPose)) < 0.10f &&
                    thetaRawSafe(simulatedThetaRad);
  if (simulationValid) {
    Mat3 initialJacobian{};
    Mat3 initialInverse{};
    float initialDeterminant = 0.0f;
    simulationValid = numericalJacobian(simulatedThetaRad, initialJacobian) &&
                      invert(initialJacobian, initialInverse, initialDeterminant);
    if (simulationValid) {
      lastDeterminant = initialDeterminant;
      lastConditionF = frobeniusNorm(initialJacobian) * frobeniusNorm(initialInverse);
      lastErrorMm = 0.0f;
      lastInverseResidualMm = 0.0f;
      lastTransposeResidualMm = 0.0f;
      lastDlsResidualMm = 0.0f;
      lastExactResidualMm = 0.0f;
    }
  }
  if (simulationValid) {
    Serial.print(F("[RESET OK] HOME simulado pose="));
    Serial.print(checkPose.v[0], 3); Serial.print(',');
    Serial.print(checkPose.v[1], 3); Serial.print(',');
    Serial.print(checkPose.v[2], 3); Serial.print(F(" raw="));
    Serial.print(thetaToRaw(simulatedThetaRad.v[0], 0u)); Serial.print(',');
    Serial.print(thetaToRaw(simulatedThetaRad.v[1], 1u)); Serial.print(',');
    Serial.println(thetaToRaw(simulatedThetaRad.v[2], 2u));
  } else {
    Serial.println(F("[RESET ERROR] FK/IK de HOME no son coherentes."));
  }
  return simulationValid;
}

void markGamepadLost(const __FlashStringHelper *reason) {
  /* Los heartbeats seguros del ESP siguen llegando con CONNECTED=0.  La
     pérdida debe procesarse una sola vez: repetir turnOff() en cada paquete
     volvía a armar CAMERA_OFF y saturaba el enlace con cuatro órdenes/s. */
  const bool wasAvailable = gamepadFresh || previousGamepadConnected;
  if (!wasAvailable) return;
  Serial.print(F("[GAMEPAD LOST] "));
  Serial.println(reason);
  ++gamepadLossEvents;
  armAfterAckPending = false;
  gamepadFresh = false;
  previousGamepadConnected = false;
  resetCenterCalibration();
  previousButtons = 0u;
  previousMisc = 0u;
  previousDpad = 0u;
  menuOpen = true; // Visible diagnostics only: loss still revokes all motion permits.
  reopenMenuAfterStop = false;
  operationMode = OperationMode::MANUAL;
  speedProfile = SpeedProfile::PRECISION;
  autoAuthorized = false;
  autoTargetActive = false;
  cameraAlignment.turnOff();
  revokePhysicalPermit(F("mando no disponible"));
}

void handlePalaManualButtons(uint16_t pressedButtons, uint8_t pressedDpad) {
  (void)pressedDpad; // Held cruceta/gatillos are sampled in serviceGripper().
  if (pressedButtons & (Config::BUTTON_A | 0x04u)) {
    printGripper();
    if (gripRuntime) printPositionMark();
  }
  if (pressedButtons & 0x08u) {
    showValueScreen = !showValueScreen;
    printPositionMark(); // Values only, never a pickup/deposit pose in MANUAL.
  }
  oledDirty = true;
}

void handleGamepadEdges() {
  const uint16_t pressedButtons = gamepad.buttons & ~previousButtons;
  const uint16_t pressedMisc = gamepad.misc & ~previousMisc;
  const uint8_t pressedDpad = gamepad.dpad & ~previousDpad;
  previousButtons = gamepad.buttons;
  previousMisc = gamepad.misc;
  previousDpad = gamepad.dpad;

  // Navigation or cancellation revokes an ARM explicitly queued behind an ACK.
  if ((pressedDpad & (Config::DPAD_UP | Config::DPAD_DOWN)) ||
      (pressedMisc & Config::MISC_START) || (pressedButtons & Config::BUTTON_B))
    armAfterAckPending = false;

  if (pressedMisc & Config::MISC_START) showValueScreen = false;
  if (pressedButtons & Config::BUTTON_B) showValueScreen = false;
  if ((pressedButtons & 0x0008u) &&
      !gripGroupMode &&
      !(pressedMisc & Config::MISC_START) && !(pressedButtons & Config::BUTTON_B)) {
    showValueScreen = !showValueScreen;
    oledDirty = true;
    if (!runEnabled && !nucleo.busy() && !nucleo.waitingResponse())
      nucleo.sendStatusRequest();
    printPositionMark();
    return;
  }

  if (gripPhase != GripPhase::OFF && !gripRuntime && !menuOpen) {
    if (pressedMisc & Config::MISC_START)
      openMenu(); // Do not wait for GRIP_END ACK to expose the menu.
    else if (pressedButtons & Config::BUTTON_B)
      stopGripper();
    else if (gripGroupMode && gripPhase == GripPhase::ACTIVE) {
      handlePalaManualButtons(pressedButtons, pressedDpad);
    } else if (pressedButtons & Config::BUTTON_A)
      printGripper();
    return; // Sticks and all DELTA/menu commands ignored during calibration.
  }

  if (pressedMisc & Config::MISC_START) {
    autoAuthorized = false;
    autoTargetActive = false;
    if (runEnabled || nucleo.streamActive() || nucleo.armed() ||
        startPhase != StartPhase::IDLE) {
      openMenu(); // Commands remain blocked until the physical stop/VERIFY.
    } else {
      openMenu();
    }
    return;
  }
  if (pressedButtons & Config::BUTTON_B) {
    confirmation = Confirmation::NONE;
    menuOpen = false;
    reopenMenuAfterStop = false;
    autoAuthorized = false;
    autoTargetActive = false;
    pauseSimulation(F("CIRCULO: cancelar y desarmar"));
    return;
  }
  if (gripRuntime && gripPhase == GripPhase::ACTIVE &&
      operationMode == OperationMode::MANUAL) {
    handlePalaManualButtons(pressedButtons, pressedDpad);
    return; // Joysticks remain active in serviceController, not in this edge handler.
  }
  if (operationMode == OperationMode::MANUAL && !menuOpen &&
      ((pressedButtons & 0xF0u) || (pressedDpad & 0x0Cu)))
    Serial.println(F("[PALA BLOCKED] No esta habilitada: ARMAR E INICIAR (doble X) o menu11 PALA MANUAL. Revisar GRIP ACK y V_GRIP."));
  if (showValueScreen) return; // Hidden menu must not accept HOME/ARM/navigation.
  if (menuOpen) {
    if (pressedDpad & Config::DPAD_UP) {
      uint8_t item = static_cast<uint8_t>(selectedMenuItem);
      item = item == 0u ? static_cast<uint8_t>(MenuItem::COUNT) - 1u
                        : item - 1u;
      selectedMenuItem = static_cast<MenuItem>(item);
      confirmation = Confirmation::NONE;
      printMenu();
    } else if (pressedDpad & Config::DPAD_DOWN) {
      const uint8_t item = (static_cast<uint8_t>(selectedMenuItem) + 1u) %
                           static_cast<uint8_t>(MenuItem::COUNT);
      selectedMenuItem = static_cast<MenuItem>(item);
      confirmation = Confirmation::NONE;
      printMenu();
    }
    if (pressedButtons & Config::BUTTON_A) activateMenuItem();
  }
}

void acceptGamepadPacket(const GamepadPacket &packet) {
  const bool newSession = !gamepadKnown || packet.session != activeGamepadSession;
  if (newSession) {
    armAfterAckPending = false;
    activeGamepadSession = packet.session;
    lastGamepadSequence = packet.sequence;
    previousButtons = 0u;
    previousMisc = 0u;
    previousDpad = 0u;
    resetCenterCalibration();
    operationMode = OperationMode::MANUAL;
    speedProfile = SpeedProfile::PRECISION;
    autoAuthorized = false;
    autoTargetActive = false;
    cameraAlignment.turnOff();
    menuOpen = false;
    reopenMenuAfterStop = false;
    revokePhysicalPermit(F("sesion nueva de gamepad"));
  } else if (!sequenceIsNewer(packet.sequence, lastGamepadSequence)) {
    ++udpSequenceErrors;
    return;
  } else {
    lastGamepadSequence = packet.sequence;
  }

  if (!newSession && gamepadKnown) {
    const uint32_t gap = millis() - lastGamepadMs;
    if (gap > gamepadGapMaxMs) gamepadGapMaxMs = gap;
  }
  gamepad = packet;
  gamepadKnown = true;
  lastGamepadMs = millis();
  ++udpValid;
  const bool connected = (packet.flags & GAMEPAD_FLAG_CONNECTED) != 0u;
  if (!connected) {
    markGamepadLost(F("mando reportado OFF"));
    return;
  }
  if (!previousGamepadConnected) {
    Serial.println(F("[GAMEPAD] Conectado; queda en PAUSE por seguridad."));
    resetCenterCalibration();
    revokePhysicalPermit(F("conexion/reconexion del mando"));
    startCenterCalibration();
  }
  previousGamepadConnected = true;
  gamepadFresh = true;
  sampleCenterCalibration();
  if (axesCentered()) {
    if (centeredSinceMs == 0u) centeredSinceMs = millis();
  } else {
    centeredSinceMs = 0u;
  }
  handleGamepadEdges();
}

void serviceUdp() {
  if (!apReady) return;
  int packetSize = 0;
  while ((packetSize = udp.parsePacket()) > 0) {
    if (packetSize == static_cast<int>(sizeof(GamepadPacket))) {
      GamepadPacket packet{};
      const int read = udp.read(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      if (read != static_cast<int>(sizeof(packet))) {
        ++udpLengthErrors;
      } else if (!gamepadPacketValid(&packet)) {
        ++udpCrcErrors;
      } else {
        espAddress = udp.remoteIP();
        acceptGamepadPacket(packet);
      }
    } else if (packetSize == static_cast<int>(sizeof(CameraPacket))) {
      CameraPacket packet{};
      const int read = udp.read(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      if (read != static_cast<int>(sizeof(packet))) {
        ++cameraLengthErrors;
      } else if (!cameraPacketValid(&packet)) {
        ++cameraCrcErrors;
      } else {
        espAddress = udp.remoteIP();
        // CAM y GAMEPAD comparten sessionId de la ESP. Un datagrama de un boot
        // anterior no puede confirmar OFF ni reiniciar la calibracion actual.
        if (gamepadKnown && packet.session != activeGamepadSession) {
          ++cameraSessionChanges;
          continue;
        }
        if (lastCameraSession != 0u && lastCameraSession != packet.session) {
          ++cameraSessionChanges;
          const bool physicalActivity = runEnabled || nucleo.armed() ||
              nucleo.streamActive() || startPhase != StartPhase::IDLE ||
              homeRequestPending || stopRequested || gripPhase != GripPhase::OFF;
          autoAuthorized = false;
          autoTargetActive = false;
          armAfterAckPending = false;
          confirmation = Confirmation::NONE;
          if (physicalActivity)
            revokePhysicalPermit(F("sesion de camara cambio"));
          if (physicalActivity ||
              (!cameraAlignment.awaitingOffAck() &&
               cameraAlignment.state() != CameraAlignment::State::OFF))
            cameraAlignment.invalidateRemoteSession();
          // Solo el OFF ACK de una recalibracion EXPLICITA permite rebind.
          // No reiniciar aqui beginCalibration(): renovaria plazo y reabriria.
          Serial.println(F("[CAM SAFETY] Sesion ESP/camara cambio; AUTO desautorizado."));
        }
        lastCameraSession = packet.session;
        ++cameraPacketsValid;
        const CameraAlignment::State previousCameraState = cameraAlignment.state();
        uint8_t previousTagCount[4];
        for (uint8_t id = 0u; id < 4u; ++id)
          previousTagCount[id] = cameraAlignment.calibrationSamples(id);
        cameraAlignment.onPacket(packet);
        for (uint8_t id = 0u; id < 4u; ++id) {
          const uint8_t count = cameraAlignment.calibrationSamples(id);
          if (count != previousTagCount[id]) {
            Serial.print(F("[CAM TAG] id=")); Serial.print(id);
            Serial.print(F(" samples=")); Serial.print(count); Serial.println(F("/5"));
          }
        }
        if (cameraAlignment.state() != previousCameraState) {
          Serial.print(F("[CAM STATE] "));
          Serial.println(cameraAlignment.stateName());
          if (cameraAlignment.ready()) {
            Serial.print(F("[CAM CAL PASS] reprojection_mm="));
            Serial.println(cameraAlignment.reprojectionMm(), 3);
          } else if (cameraAlignment.state() == CameraAlignment::State::FAULT) {
            Serial.print(F("[CAM FAULT] ")); Serial.println(cameraAlignment.fault());
          }
        }
        oledDirty = true;
      }
    } else {
      uint8_t discard[64];
      while (udp.available() > 0) udp.read(discard, sizeof(discard));
      ++udpLengthErrors;
    }
  }
  const uint32_t gamepadAgeMs = millis() - lastGamepadMs;
  const bool cameraSwitchGrace =
      automaticOperation() &&
      !runEnabled && !nucleo.armed() && !nucleo.streamActive() &&
      startPhase == StartPhase::IDLE &&
      (lastCameraCommand == CAMERA_COMMAND_TAG_MODE ||
       lastCameraCommand == CAMERA_COMMAND_PIECE_MODE) &&
      millis() - lastCameraCommandMs <=
          Config::CAMERA_SWITCH_GAMEPAD_TIMEOUT_MS;
  const uint32_t effectiveGamepadTimeoutMs = cameraSwitchGrace
      ? Config::CAMERA_SWITCH_GAMEPAD_TIMEOUT_MS
      : Config::GAMEPAD_TIMEOUT_MS;
  if (cameraSwitchGrace && gamepadFresh &&
      gamepadAgeMs > Config::GAMEPAD_TIMEOUT_MS) {
    if (!cameraSwitchGraceReported) {
      cameraSwitchGraceReported = true;
      Serial.print(F("[GAMEPAD GRACE] HUSKYLENS cambiando modelo; age_ms="));
      Serial.println(gamepadAgeMs);
    }
  } else if (gamepadAgeMs <= Config::GAMEPAD_TIMEOUT_MS) {
    cameraSwitchGraceReported = false;
  }
  if (gamepadFresh && gamepadAgeMs > effectiveGamepadTimeoutMs) {
    markGamepadLost(cameraSwitchGrace
        ? F("timeout durante cambio de modelo de camara")
        : F("timeout 600 ms"));
  }
}

void serviceAp() {
  if (apReady) return;
  const uint32_t now = millis();
  if (lastApAttemptMs != 0u && now - lastApAttemptMs < Config::AP_RETRY_MS) return;
  lastApAttemptMs = now;
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("[AP ERROR] Modulo Wi-Fi no disponible."));
    return;
  }
  WiFi.config(IPAddress(10, 0, 0, 1));
  const int status = WiFi.beginAP(Config::AP_SSID, Config::AP_PASSWORD);
  if (status != WL_AP_LISTENING && status != WL_AP_CONNECTED) {
    Serial.print(F("[AP ERROR] WiFi.beginAP retorno="));
    Serial.println(status);
    return;
  }
  if (!udp.begin(Config::UDP_PORT)) {
    Serial.println(F("[UDP ERROR] No se pudo abrir puerto 1234."));
    return;
  }
  apReady = true;
  Serial.print(F("[AP READY] SSID=DELTA_PORTENTA IP="));
  Serial.print(WiFi.localIP());
  Serial.println(F(" UDP=1234"));
}

bool workspacePointValid(const Vec3 &pose) {
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (!isfinite(pose.v[i])) return false;
  }
  const float epsilon = Config::WORKSPACE_POSITION_EPSILON_MM;
  if (pose.v[0] < Config::X_MIN_MM - epsilon ||
      pose.v[0] > Config::X_MAX_MM + epsilon ||
      pose.v[1] < Config::Y_MIN_MM - epsilon ||
      pose.v[1] > Config::Y_MAX_MM + epsilon ||
      pose.v[2] < Config::Z_MIN_MM - epsilon ||
      pose.v[2] > Config::Z_MAX_MM + epsilon) return false;
  Vec3 theta{};
  Vec3 check{};
  if (!deltaInverse(pose, theta) || !thetaRawSafeForPose(theta, pose) ||
      !deltaForward(theta, check) ||
      magnitude(subtract(pose, check)) > Config::MAX_NUMERIC_RESIDUAL_MM) {
    return false;
  }
  Mat3 jacobian{};
  Mat3 inverseJacobian{};
  float ignoredDeterminant = 0.0f;
  if (!numericalJacobian(theta, jacobian) ||
      !invert(jacobian, inverseJacobian, ignoredDeterminant)) return false;
  const float condition = frobeniusNorm(jacobian) *
                          frobeniusNorm(inverseJacobian);
  return isfinite(condition) && condition <= Config::MAX_CONDITION_F;
}

Vec3 fitWorkspaceReference(const Vec3 &requested) {
  Vec3 bounded = requested;
  bounded.v[0] = constrain(bounded.v[0], Config::X_MIN_MM, Config::X_MAX_MM);
  bounded.v[1] = constrain(bounded.v[1], Config::Y_MIN_MM, Config::Y_MAX_MM);
  bounded.v[2] = constrain(bounded.v[2], Config::Z_MIN_MM, Config::Z_MAX_MM);
  const bool boxLimited = bounded.v[0] != requested.v[0] ||
                          bounded.v[1] != requested.v[1] ||
                          bounded.v[2] != requested.v[2];
  if (workspacePointValid(bounded) && !boxLimited) return bounded;
  if (workspacePointValid(bounded) && boxLimited) {
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
      if (bounded.v[axis] != requested.v[axis]) commandedVelocity.v[axis] = 0.0f;
    }
    ++workspaceLimits;
    const uint32_t now = millis();
    if (now - lastWorkspaceLimitPrintMs >= Config::WORKSPACE_PRINT_PERIOD_MS) {
      lastWorkspaceLimitPrintMs = now;
      Serial.println(F("[WORKSPACE LIMIT] Limite cartesiano Z/XYZ alcanzado; velocidad saturada."));
    }
    return bounded;
  }

  /* Si el vector completo sale del workspace, probar combinaciones de sus
     componentes. Asi un intento de seguir saliendo en Y no cancela un X/Z
     tangencial que si es seguro. Se elige la combinacion valida que conserva
     la mayor parte cuadratica del desplazamiento solicitado. */
  const Vec3 delta = subtract(bounded, referencePose);
  Vec3 componentBest = referencePose;
  float componentBestScore = 0.0f;
  uint8_t componentBestMask = 0u;
  for (uint8_t mask = 1u; mask < 7u; ++mask) {
    Vec3 candidate = referencePose;
    float score = 0.0f;
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
      if ((mask & (uint8_t)(1u << axis)) != 0u) {
        candidate.v[axis] += delta.v[axis];
        score += delta.v[axis] * delta.v[axis];
      }
    }
    if (score > componentBestScore && workspacePointValid(candidate)) {
      componentBest = candidate;
      componentBestScore = score;
      componentBestMask = mask;
    }
  }
  if (componentBestMask != 0u) {
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
      if ((componentBestMask & (uint8_t)(1u << axis)) == 0u) {
        commandedVelocity.v[axis] = 0.0f;
      }
    }
    ++workspaceLimits;
    /* RETREAT_FOR_Z y LOWER_FOR_XY describen la maniobra de alto nivel. No
       sobrescribirlos cada ciclo con el detalle interno de proyeccion. */
    if (workspaceAssistMode == WorkspaceAssistMode::NONE) {
      setWorkspaceAssistMode(WorkspaceAssistMode::COMPONENT_PROJECTED);
    }
    return componentBest;
  }

  Vec3 best = referencePose;
  float low = 0.0f;
  float high = 1.0f;
  for (uint8_t iteration = 0u;
       iteration < Config::WORKSPACE_BISECTION_STEPS; ++iteration) {
    const float fraction = 0.5f * (low + high);
    const Vec3 candidate = add(referencePose, scale(delta, fraction));
    if (workspacePointValid(candidate)) {
      best = candidate;
      low = fraction;
    } else {
      high = fraction;
    }
  }
  if (low > 0.001f) {
    commandedVelocity = scale(commandedVelocity, low);
  } else {
    commandedVelocity = Vec3{{0.0f, 0.0f, 0.0f}};
  }
  ++workspaceLimits;
  const uint32_t now = millis();
  if (now - lastWorkspaceLimitPrintMs >= Config::WORKSPACE_PRINT_PERIOD_MS) {
    lastWorkspaceLimitPrintMs = now;
    Serial.print(F("[WORKSPACE LIMIT] range=")); Serial.print(rangeName());
    Serial.print(F(" accepted="));
    Serial.print(best.v[0], 1); Serial.print(',');
    Serial.print(best.v[1], 1); Serial.print(',');
    Serial.println(best.v[2], 1);
  }
  return best;
}

Vec3 assistManualWorkspaceVelocity(const Vec3 &requestedVelocity,
                                   float speedLimit, float dt) {
  if (operationMode != OperationMode::MANUAL ||
      manualMode != ManualMode::XYZ_MANUAL) {
    setWorkspaceAssistMode(WorkspaceAssistMode::NONE);
    return requestedVelocity;
  }

  const float lateralDemand = sqrtf(
    requestedVelocity.v[0] * requestedVelocity.v[0] +
    requestedVelocity.v[1] * requestedVelocity.v[1]);
  const float epsilon = Config::WORKSPACE_POSITION_EPSILON_MM;
  const Vec3 directPoint = add(referencePose,
                               scale(requestedVelocity, dt));
  if (workspacePointValid(directPoint)) {
    setWorkspaceAssistMode(WorkspaceAssistMode::NONE);
    return requestedVelocity;
  }

  // No bajar por estar alto: solo asistir si el paso lateral REAL fue rechazado.
  if (lateralDemand > 0.10f &&
      referencePose.v[2] > Config::HIGH_XY_ASSIST_Z_MM + epsilon) {
    const float assistLimit = fminf(speedLimit, Config::WORKSPACE_ASSIST_MAX_SPEED_MM_S);
    for (uint8_t step=1u; step<=Config::WORKSPACE_ASSIST_SEARCH_STEPS; ++step) {
      Vec3 candidateVelocity = requestedVelocity;
      candidateVelocity.v[2] = fminf(candidateVelocity.v[2],
        -assistLimit * static_cast<float>(step) / Config::WORKSPACE_ASSIST_SEARCH_STEPS);
      candidateVelocity = limitMagnitude(candidateVelocity, speedLimit);
      if (workspacePointValid(add(referencePose, scale(candidateVelocity, dt)))) {
        setWorkspaceAssistMode(WorkspaceAssistMode::LOWER_FOR_XY);
        return candidateVelocity;
      }
    }
  }

  if (requestedVelocity.v[2] < -0.10f) {
    const float radius = sqrtf(referencePose.v[0] * referencePose.v[0] +
                               referencePose.v[1] * referencePose.v[1]);
    if (radius > 0.50f) {
      const float inwardX = -referencePose.v[0] / radius;
      const float inwardY = -referencePose.v[1] / radius;
      const float assistLimit = fminf(speedLimit,
        Config::WORKSPACE_ASSIST_MAX_SPEED_MM_S);
      for (uint8_t step = 1u;
           step <= Config::WORKSPACE_ASSIST_SEARCH_STEPS; ++step) {
        const float inwardSpeed = assistLimit *
          static_cast<float>(step) /
          static_cast<float>(Config::WORKSPACE_ASSIST_SEARCH_STEPS);
        Vec3 candidateVelocity = requestedVelocity;
        candidateVelocity.v[0] += inwardX * inwardSpeed;
        candidateVelocity.v[1] += inwardY * inwardSpeed;
        candidateVelocity = limitMagnitude(candidateVelocity, speedLimit);
        const Vec3 candidatePoint = add(referencePose,
                                        scale(candidateVelocity, dt));
        if (workspacePointValid(candidatePoint)) {
          setWorkspaceAssistMode(WorkspaceAssistMode::RETREAT_FOR_Z);
          return candidateVelocity;
        }
      }

      Vec3 retreatVelocity{{inwardX * assistLimit,
                            inwardY * assistLimit, 0.0f}};
      if (workspacePointValid(add(referencePose,
                                  scale(retreatVelocity, dt)))) {
        setWorkspaceAssistMode(WorkspaceAssistMode::RETREAT_FOR_Z);
        return retreatVelocity;
      }
    }
  }

  setWorkspaceAssistMode(WorkspaceAssistMode::NONE);
  return requestedVelocity;
}

void controllerSafetyStop(const __FlashStringHelper *reason) {
  ++safetyStops;
  Serial.print(F("[JACOBIAN STOP] "));
  Serial.println(reason);
  reopenMenuAfterStop = true;
  pauseSimulation(F("proteccion matematica"));
}

void holdAtWorkspaceLimit() {
  commandedVelocity = Vec3{{0.0f, 0.0f, 0.0f}};
  ++workspaceLimits;
  if (operationMode == OperationMode::AUTO_PICKUP) {
    reopenMenuAfterStop = true;
    pauseSimulation(F("AUTO recogida: trayectoria fuera de limites"));
    return;
  }
  if (automaticBandMove) {
    Serial.println(F("[BAND BLOCKED] Trayectoria limitada; se verifica la ultima posicion."));
    reopenMenuAfterStop = true;
    pauseSimulation(F("frontera en descenso automatico"));
    return;
  }
  const uint32_t now = millis();
  if (now - lastWorkspaceLimitPrintMs >= Config::WORKSPACE_PRINT_PERIOD_MS) {
    lastWorkspaceLimitPrintMs = now;
    Serial.println(F("[WORKSPACE LIMIT] Manteniendo ultimo destino; puedes regresar con el joystick."));
  }
  // STREAM necesita setpoints incluso detenido en una frontera.
  // Dejar de transmitir aqui disparaba el watchdog de 250 ms.
  if (!nucleo.sendStreamSetpoint(nucleo.remoteRaw(),
        Config::STREAM_MOVE_TIME_MS, activeStreamOption)) return;
  pendingThetaRad = simulatedThetaRad;
  pendingReferencePose = referencePose;
  pendingConditionF = lastConditionF;
  pendingDeterminant = lastDeterminant;
  pendingErrorMm = lastErrorMm;
  pendingInverseResidualMm = lastInverseResidualMm;
  pendingTransposeResidualMm = lastTransposeResidualMm;
  pendingDlsResidualMm = lastDlsResidualMm;
  pendingExactResidualMm = lastExactResidualMm;
  pendingSetpoint = true;
  ++streamCommandsSent;
}

void sendAutoPickupHold() {
  if (!nucleo.sendStreamSetpoint(nucleo.remoteRaw(), Config::STREAM_MOVE_TIME_MS,
                                activeStreamOption)) return;
  autoPtpSegmentPending = false;
  pendingThetaRad = simulatedThetaRad;
  pendingReferencePose = referencePose;
  pendingConditionF = lastConditionF;
  pendingDeterminant = lastDeterminant;
  pendingErrorMm = lastErrorMm;
  pendingInverseResidualMm = lastInverseResidualMm;
  pendingTransposeResidualMm = lastTransposeResidualMm;
  pendingDlsResidualMm = lastDlsResidualMm;
  pendingExactResidualMm = lastExactResidualMm;
  pendingSetpoint = true;
  ++streamCommandsSent;
}

bool autoPtpWorkspacePoint(const float q[3], float xyz[3]) {
  const Vec3 theta{{q[0],q[1],q[2]}};
  Vec3 pose{};
  if (!deltaForward(theta,pose) || !thetaRawSafeForPose(theta,pose)) return false;
  // Same Cartesian box/Jacobian guards, evaluated from this joint sample.
  // Do not solve another IK or substitute a Cartesian chord for its FK arc.
  const float epsilon=Config::WORKSPACE_POSITION_EPSILON_MM;
  if (!isfinite(pose.v[0]) || !isfinite(pose.v[1]) || !isfinite(pose.v[2]) ||
      pose.v[0]<Config::X_MIN_MM-epsilon || pose.v[0]>Config::X_MAX_MM+epsilon ||
      pose.v[1]<Config::Y_MIN_MM-epsilon || pose.v[1]>Config::Y_MAX_MM+epsilon ||
      pose.v[2]<Config::Z_MIN_MM-epsilon || pose.v[2]>Config::Z_MAX_MM+epsilon) return false;
  Mat3 jacobian{},inverseJacobian{};float determinant=NAN;
  if (!numericalJacobian(theta,jacobian) || !invert(jacobian,inverseJacobian,determinant)) return false;
  const float condition=frobeniusNorm(jacobian)*frobeniusNorm(inverseJacobian);
  if (!isfinite(condition) || condition>Config::MAX_CONDITION_F) return false;
  for (unsigned i=0u;i<3u;++i) xyz[i]=pose.v[i];
  return true; // numerical envelope only; no physical collision certificate
}

bool autoPtpCurvePoint(const float q[3],float xyz[3]) {
  if (!autoPtpWorkspacePoint(q,xyz)) return false;
  return autoJointPtp.combined() ? autoJointPtp.pointSafe(xyz)
    : xyz[2]+0.001f >= AutoPickupHeight::targetZ(xyz[1])+
                         AutoPickupSetup::APPROACH_CLEARANCE_MM;
}

void autoRawToTheta(const uint16_t raw[3],float q[3]) {
  for (uint8_t i=0u;i<3u;++i)
    q[i]=(Config::THETA_HOME_DEG[i]+((float)raw[i]-Config::SERVO_HOME_RAW[i])/
      (Config::SERVO_SIGN[i]*Config::SERVO_UNITS_PER_DEG))*DEG_TO_RAD;
}

bool autoLocalCurvePoint(const float q[3],float xyz[3]) {
  return autoPtpWorkspacePoint(q,xyz) && xyz[2]>=
    AutoPickupHeight::targetZ(xyz[1])+AutoPickupSetup::APPROACH_CLEARANCE_MM;
}

bool prepareAutoLocalPoint(const uint16_t start[3],const Vec3 &goal,
                            DeltaPointProfile &profile,uint32_t begun) {
  Vec3 q{};uint16_t raw[3];
  if (!deltaInverse(goal,q)) return false;
  for (uint8_t i=0u;i<3u;++i) raw[i]=(uint16_t)thetaToRaw(q.v[i],i);
  const auto evaluate=[begun](const float theta[3],float xyz[3]) {
    return micros()-begun<=20000u && autoLocalCurvePoint(theta,xyz);
  };
  const bool valid=AutoLocalPoint::prepare(start,raw,profile,
    Config::MAX_JOINT_STEP_RAD*1000.0f/Config::CONTROL_PERIOD_MS,
    AutoSpeedProfile::cruiseLimit(autoPickup.state(),AutoPickupSetup::speedProfileConfig()),
    AutoPickupSetup::FINAL_APPROACH_SPEED_MM_S,AutoPickupSetup::FINAL_APPROACH_DISTANCE_MM,
    Config::ACCEL_MM_S2,Config::DECEL_MM_S2,autoRawToTheta,evaluate);
  return valid && micros()-begun<=20000u;
}

bool prepareAutoDirectMove(const uint16_t start[3],const Vec3 &goal,
                           AutoDirectEndpoint::Profile &profile,uint32_t begun,
                           AutoPickup::State state,
                           uint16_t samples=AutoDirectEndpoint::SAMPLES) {
  Vec3 q{};uint16_t raw[3];float startTheta[3],startXyz[3];
  if (!deltaInverse(goal,q)) return false;
  for (uint8_t i=0u;i<3u;++i) raw[i]=(uint16_t)thetaToRaw(q.v[i],i);
  autoRawToTheta(start,startTheta);
  if (!autoPtpWorkspacePoint(startTheta,startXyz)) return false;
  const Vec3 startPose{{startXyz[0],startXyz[1],startXyz[2]}};
  const bool vertical=AutoPickupSetup::verticalPositionStage(state);
  if (vertical && hypotf(goal.v[0]-startPose.v[0],goal.v[1]-startPose.v[1])>
                  AutoPickupSetup::verticalOriginCapMm(state)) return false;
  const float rising=goal.v[2]>=startPose.v[2] ? 1.0f : -1.0f;
  const float zLow=fminf(startPose.v[2],goal.v[2])-0.75f;
  const float zHigh=fmaxf(startPose.v[2],goal.v[2])+0.75f;
  const auto evaluate=[begun](const float theta[3],float xyz[3]) {
    return micros()-begun<=20000u && autoPtpWorkspacePoint(theta,xyz);
  };
  const auto validate=[vertical,rising,zLow,zHigh,startPose,goal](
      const float xyz[3],const float previous[3],uint16_t sample) {
    if (vertical) {
      // Measure bow away from the bounded measured-origin -> nominal-goal
      // segment, not the permitted initial physical checkpoint error itself.
      const float sx=startPose.v[0],sy=startPose.v[1];
      const float dx=goal.v[0]-sx,dy=goal.v[1]-sy,length2=dx*dx+dy*dy;
      float t=length2>1e-6f ? ((xyz[0]-sx)*dx+(xyz[1]-sy)*dy)/length2 : 0.0f;
      t=fminf(1.0f,fmaxf(0.0f,t));
      const float drift=hypotf(xyz[0]-(sx+t*dx),xyz[1]-(sy+t*dy));
      if (!isfinite(drift) || drift>Config::DIRECT_LIFT_MAX_XY_DRIFT_MM ||
          xyz[2]<zLow || xyz[2]>zHigh) return false;
      return !sample || rising*(xyz[2]-previous[2])>=-0.75f;
    }
    return xyz[2]+0.001f+AutoPickupSetup::POSITION_CLEARANCE_QUANTIZATION_MM>=
      AutoPickupHeight::targetZ(xyz[1])+AutoPickupSetup::APPROACH_CLEARANCE_MM;
  };
  if (!AutoDirectEndpoint::prepare(start,raw,profile,
      AutoPickupSetup::positionCruiseMmS(state),autoRawToTheta,evaluate,validate,samples))
    return false;
  // Preserve the validated curve and both speed ceilings: only LENGTHEN a short move.
  if (AutoPickupSetup::DIRECT_PROFILE250_ENABLED &&
      profile.durationMs<AutoPickupSetup::positionDurationFloor(state))
    profile.durationMs=AutoPickupSetup::positionDurationFloor(state);
  return micros()-begun<=20000u;
}

bool prepareAutoEndpointRetry(const uint16_t measured[3],AutoDirectEndpoint::Profile &profile,
                              uint32_t begun) {
  uint16_t effective[3];
  if (!deltaEndpointRetryPlanTolerance(measured,autoEndpointNominal,effective,
      DELTA_ENDPOINT_RETRY_MAX_ERROR_RAW,AutoPickupSetup::endpointRecoveryTolerance(autoPickup.state()))) return false;
  const auto evaluate=[begun](const float q[3],float xyz[3]) {
    return micros()-begun<=20000u && autoLocalCurvePoint(q,xyz);
  };
  const auto validate=[](const float[3],const float[3],uint16_t) {return true;};
  // Validate the actual single/multiple-axis recovery curve, not the old route
  // and not a fictitious origin. Axes within15 hold their measured positions.
  if (!AutoDirectEndpoint::prepare(measured,effective,profile,
      AutoPickupSetup::positionCruiseMmS(autoPickup.state()),autoRawToTheta,evaluate,validate))
    return false;
  if (profile.durationMs<autoEndpointDurationFloor) profile.durationMs=autoEndpointDurationFloor;
  if(autoEndpointStepPending){
    const uint16_t count=deltaEndpointStepCount(measured,effective);
    uint16_t raw[3];float q[3],xyz[3];
    for(uint16_t i=0u;i<=count;++i){
      deltaEndpointStepTarget(measured,effective,i,count,raw);
      autoRawToTheta(raw,q);
      if(micros()-begun>20000u || !autoLocalCurvePoint(q,xyz))return false;
    }
    const uint16_t budget=deltaEndpointStepBudget(measured,effective);
    if(!budget)return false;
    if(profile.durationMs<budget)profile.durationMs=budget;
  }
  return micros()-begun<=20000u;
}

// Returns true when it owns this controller tick; false only during the
// precisely Cartesian vertical escape (a joint-space arc need not be vertical).
bool serviceAutoLocalPoint() {
  if (autoLocalToken!=autoPickup.action().token) {
    controllerSafetyStop(F("AUTO punto token invalido"));return true;
  }
  if (autoDirectMove) {
    if (autoPickupHealthFailure(millis(),false,true)!=nullptr || !runEnabled || stopRequested) {
      controllerSafetyStop(F("AUTO DIRECT permisos antes de verificar"));return true;
    }
    if (autoLocalPhase==LocalPointPhase::RUNNING && nucleo.directRetryReady()) {
      autoLocalPhase=LocalPointPhase::RETRY_READY;
      autoDirectRcRenewalKnown=false;
      Serial.println(F("[AUTO DIRECT RETRY] parada confirmada; validando FK/ruta desde pose real, intento 1."));
    }
    if (autoLocalPhase==LocalPointPhase::PRECHECK) {
      const uint16_t originPolicy=AutoPickupSetup::precheckFlags(autoPickup.state());
      if (nucleo.sendStreamCheckpoint(activeStreamOption | originPolicy)) {
        autoLocalPhase=LocalPointPhase::WAIT_PRECHECK;
        autoDirectRcRenewalKnown=false;
        Serial.print(F("[AUTO DIRECT PRECHECK] state="));
        Serial.print(AutoPickup::Sequence::stateName(autoPickup.state()));
        Serial.print(F(" token="));Serial.println(autoLocalToken);
      }
      return true;
    }
    if (autoLocalPhase==LocalPointPhase::WAIT_PRECHECK) {
      DeltaResultCode result;uint16_t measured[3];
      if (nucleo.consumeCheckpoint(result,measured)) {
        const bool accepted = result == ((autoEndpointRetryPending || AutoPickupSetup::precheckFlags(autoPickup.state()) == DELTA_CHECKPOINT_OBSERVE)
          ? DELTA_RESULT_OBSERVED : DELTA_RESULT_OK);
        if (!accepted || !nucleo.directOriginKnown()) {
          controllerSafetyStop(F("AUTO DIRECT referencia fisica no valida"));return true;
        }
        autoLocalPhase=LocalPointPhase::READY;
        if(autoEndpointRetryPending){
          autoEndpointOriginRefreshed=true;
          bool inside=true;
          for(uint8_t i=0u;i<3u;++i)
            if(abs((int)measured[i]-(int)autoEndpointNominal[i])>
                AutoPickupSetup::endpointRecoveryTolerance(autoPickup.state()))inside=false;
          if(inside){
            autoEndpointRetryPending=false;autoDirectMove=false;autoPointHold=false;
            autoLocalPhase=LocalPointPhase::FINISHED;
            if(AutoPickupSetup::centerRecovery(autoPickup.state())){
              autoCenterPrecisionPending=true;autoPointHold=true;autoCheckpointToken=0u;
              Serial.println(F("[AUTO CENTER RECOVERED] referencia fresca dentro12; CP5 fisico pendiente."));
              return true;
            }
            if(!autoPickup.markCartesianVerified(autoLocalToken,millis()))
              controllerSafetyStop(F("AUTO recovery fresh checkpoint token invalido"));
            return true; // physical arrival during the wait; no redundant motion
          }
        }
        Serial.print(result == DELTA_RESULT_OBSERVED ? F("[AUTO DIRECT ORIGIN OBSERVED] measured=") :
          F("[AUTO DIRECT ORIGIN PASS] measured="));
        for (uint8_t i=0u;i<3u;++i) {if(i)Serial.print(',');Serial.print(measured[i]);}
        Serial.println(F("; consigna nominal conservada"));
      } else if (!nucleo.waitingResponse()) {
        sendAutoPickupHold(); // identical nominal renewal; never measured RAW
      }
      return true;
    }
  }
  if (autoLocalPhase==LocalPointPhase::ESCAPE) {
    if (!autoPointHold) return false;
    autoPointHold=false;autoTargetPose=autoLocalGoal;
    autoLocalPhase=LocalPointPhase::READY;
  }
  if (autoLocalPhase==LocalPointPhase::READY || autoLocalPhase==LocalPointPhase::RETRY_READY) {
    const bool retry=autoLocalPhase==LocalPointPhase::RETRY_READY;
    if(autoEndpointRetryPending && millis()-autoEndpointRetryQueuedMs<DELTA_ENDPOINT_RETRY_WAIT_MS){
      sendAutoPickupHold();return true; // no delay(): leases/STOP/camera keep being serviced
    }
    if(autoEndpointRetryPending && !autoEndpointOriginRefreshed){
      if(nucleo.sendStreamCheckpoint(activeStreamOption|DELTA_CHECKPOINT_OBSERVE,false,false,false,
          AutoPickupSetup::centerRecovery(autoPickup.state()))){
        autoLocalPhase=LocalPointPhase::WAIT_PRECHECK;
        autoDirectRcRenewalKnown=false;
      }
      return true; // wait100 then re-read; never reuse a snapshot made stale by the wait
    }
    if (!autoDirectMove && (nucleo.remoteFlags() & DELTA_FLAG_FINE_REQUIRED)) {
      return false; // existing fine DLS steps finish the compensated bridge first
    }
    if (autoDirectMove) {
      if (!(retry ? nucleo.directRetryReady() : nucleo.directOriginKnown())) {
        controllerSafetyStop(F("AUTO DIRECT referencia cambio tras PASS"));return true;
      }
      // PREP/READ can block: require an acknowledged identical RC renewal.
      // Its age starts at TX, conservatively before the receiver refreshes PWM.
      if (!autoDirectRcRenewalKnown ||
          millis()-autoDirectRcRenewalMs>DIRECT_RC_PREPARE_MAX_AGE_MS) {
        for (uint8_t i=0u;i<3u;++i) autoRcPendingUs[i]=gripUs[i];
        const uint32_t renewalTxMs=millis();
        if (nucleo.sendGripperGroup(DELTA_MSG_GRIP_GROUP_SET,autoRcPendingUs,true,true)) {
          autoRcPending=true;autoRcBeginPending=false;autoRcPendingToken=autoLocalToken;
          autoDirectRcRenewalMs=renewalTxMs;autoDirectRcRenewalKnown=true;
        }
        return true;
      }
      if (autoRcPending || nucleo.waitingResponse()) return true;
    }
    DeltaPointProfile profile{};
    AutoDirectEndpoint::Profile direct{};
    const uint32_t begun=micros();
    const bool endpointRetry=autoDirectMove && autoEndpointRetryPending;
    const bool planned=endpointRetry ? prepareAutoEndpointRetry(nucleo.directOriginRaw(),direct,begun) : autoDirectMove
      ? prepareAutoDirectMove(retry ? nucleo.directRetryOriginRaw() : nucleo.directOriginRaw(),autoLocalGoal,direct,begun,
          autoPickup.state())
      : prepareAutoLocalPoint(nucleo.remoteRaw(),autoLocalGoal,profile,begun);
    if (retry && direct.durationMs<nucleo.directRetryDurationFloor())
      direct.durationMs=nucleo.directRetryDurationFloor(); // no catch-up acceleration
    const uint16_t duration=autoDirectMove ? direct.durationMs : profile.durationMs;
    if (!planned || millis()-autoLocalStartedMs+duration+2000u+
          AutoPickupSetup::cartesianSettleMs(autoPickup.state())+
          DELTA_AUTO_CHECKPOINT_TIMEOUT_MS>AutoPickupSetup::CARTESIAN_STAGE_TIMEOUT_MS) {
      controllerSafetyStop(F("AUTO movimiento FK/RAW/Jacobiano/despeje/presupuesto"));return true;
    }
    // Preflight consumed bounded CPU time. Recheck live permissions BEFORE TX.
    if (autoPickupHealthFailure(millis(),false,true)!=nullptr || !runEnabled || stopRequested) {
      controllerSafetyStop(F("AUTO punto permisos vencidos en preflight"));return true;
    }
    if (autoDirectMove && millis()-autoDirectRcRenewalMs>DIRECT_RC_PREPARE_MAX_AGE_MS)
      return true; // renew next tick instead of extending the receiver PWM lease
    const bool sent=autoDirectMove
      ? (endpointRetry ? nucleo.sendEndpointRetry(autoEndpointNominal,direct.durationMs,activeStreamOption,autoEndpointStepPending) :
        retry ? nucleo.sendDirectRetry(direct.goal,direct.durationMs,activeStreamOption) :
          nucleo.sendValidatedDirect(direct.goal,direct.durationMs,activeStreamOption,
            AutoPickupSetup::postPickupDelayedProbe(autoPickup.state()),
            AutoPickupSetup::finalReadTransit(autoPickup.state())))
      : nucleo.sendPoint(profile.goal,profile.durationMs,activeStreamOption);
    if (!sent) {
      controllerSafetyStop(F("AUTO movimiento envio rechazado"));return true;
    }
    if (autoDirectMove) {
      if (endpointRetry) {
        ++autoEndpointRetryAttempts;autoEndpointRetryPending=false;
        Serial.print(F("[AUTO ENDPOINT RETRY SENT] mode="));Serial.print(autoEndpointStepPending?F("STEPS4/100ms"):F("DIRECT"));
        Serial.print(F(" attempt="));Serial.println(autoEndpointRetryAttempts);
      } else if (!retry) autoEndpointDurationFloor=direct.durationMs;
    }
    autoLocalPhase=LocalPointPhase::RUNNING;
    Serial.print(autoDirectMove ? F("[AUTO DIRECT MOVE] state=") : F("[AUTO LOCAL POINT] state="));
    Serial.print(AutoPickup::Sequence::stateName(autoPickup.state()));
    Serial.print(F(" duration_ms="));Serial.print(duration);
    if (autoDirectMove) {
      Serial.print(F(" tx_count=1; endpoint validado, raw_s_max=120 techo_mm_s="));
      Serial.print(AutoPickupSetup::positionCruiseMmS(autoPickup.state()),0);
      if (AutoPickupSetup::DIRECT_PROFILE250_ENABLED) {
        Serial.print(F("; C2.50 TX5/REC5; REC350 + STEPS4/100; centro CP5; duracion_min_ms="));
        Serial.print(AutoPickupSetup::positionDurationFloor(autoPickup.state()));
        Serial.print(F(" post_pickup_read_window_ms=300"));
      }
      else if (AutoPickupSetup::postPickupDelayedProbe(autoPickup.state()))
        Serial.print(F("; C2.32 copias iniciales=2; primera sonda diferida 200ms (tramo corto conserva 100ms)"));
      Serial.println(F("; sin reenvio en ruta; PASS fisico pendiente."));
    }
    else
      Serial.println(F("; UN destino B->C, trayectoria local cancelable; PASS fisico pendiente."));
  }
  return true;
}

bool autoCombinedCurvePoint(const float xyz[3],float q[3],const AutoJointPtp::Plan& plan) {
  if (!plan.pointSafe(xyz)) return false;
  const Vec3 reference{{xyz[0],xyz[1],xyz[2]}};
  Vec3 theta{};
  if (!deltaInverse(reference,theta)) return false;
  float actual[3];
  if (!autoPtpWorkspacePoint(theta.v,actual) || !plan.pointSafe(actual)) return false;
  const Vec3 actualPose{{actual[0],actual[1],actual[2]}};
  if (magnitude(subtract(reference,actualPose))>Config::MAX_NUMERIC_RESIDUAL_MM) return false;
  for (unsigned a=0u;a<3u;++a)q[a]=theta.v[a];
  return true;
}

bool beginAutoJointPtp() {
  // One loaded action from CARRY to final box when combined is enabled.
  // Only the short initial exit is vertical; Z continues while XY blends in.
  // No intermediate LIFT PASS or box-height adjustment is inserted.
  Vec3 goal{};
  if (!AutoJointPtp::eligible(autoPickup.state())) return false;
  const uint32_t begun=micros();
  bool planned=false;
  if (AutoPickupSetup::COMBINED_LOADED_TRANSFER_ENABLED) {
    const float clearanceZ=AutoPickupHeight::clearanceZBetween(referencePose.v[1],
      autoTargetPose.v[1],AutoPickupSetup::APPROACH_CLEARANCE_MM);
    const auto evaluateXYZ=[begun](const float p[3],float q[3]) {
      return micros()-begun<=20000u && autoCombinedCurvePoint(p,q,autoJointPtp);
    };
    planned=autoJointPtp.beginCombined(referencePose.v,autoTargetPose.v,clearanceZ,
      Config::MAX_JOINT_STEP_RAD*1000.0f/Config::CONTROL_PERIOD_MS,
      AutoPickupSetup::LOADED_TRANSFER_SPEED_MM_S,AutoPickupSetup::FINAL_APPROACH_SPEED_MM_S,
      AutoPickupSetup::FINAL_APPROACH_DISTANCE_MM,Config::ACCEL_MM_S2,Config::DECEL_MM_S2,evaluateXYZ);
    // Reserve the existing physical verification and settle within the stage
    // budget. Delayed ACKs still time out safely; never extend its deadline.
    if (autoJointPtp.durationMs()+AutoPickupSetup::LOADED_CARTESIAN_SETTLE_MS+
        DELTA_AUTO_CHECKPOINT_TIMEOUT_MS+Config::CONTROL_PERIOD_MS >
        AutoPickupSetup::CARTESIAN_STAGE_TIMEOUT_MS) planned=false;
  } else if (deltaInverse(autoTargetPose,goal)) {
    const auto evaluate=[begun](const float q[3],float p[3]) {
      return micros()-begun<=20000u && autoPtpCurvePoint(q,p);
    };
    planned=autoJointPtp.begin(simulatedThetaRad.v,goal.v,
        Config::MAX_JOINT_STEP_RAD*1000.0f/Config::CONTROL_PERIOD_MS,
        AutoPickupSetup::LOADED_TRANSFER_SPEED_MM_S,
        AutoPickupSetup::FINAL_APPROACH_SPEED_MM_S,
        AutoPickupSetup::FINAL_APPROACH_DISTANCE_MM,
        Config::ACCEL_MM_S2,Config::DECEL_MM_S2,evaluate);
  }
  if (!planned || micros()-begun>20000u) {
    // Include the cost of the LAST FK/Jacobian sample, too. The per-sample
    // guard alone only measures the time immediately before that evaluation.
    autoJointPtp.cancel();
    if (micros()-begun>20000u)
      Serial.println(F("[AUTO PTP REJECT] PREFLIGHT_BUDGET 20000 us; no movimiento TRANSFER."));
    else Serial.println(F("[AUTO PTP REJECT] FK/RAW/Jacobiano/despeje/perfil; no movimiento TRANSFER."));
    return false;
  }
  autoPtpToken=autoPickup.action().token;
  autoPtpSegmentPending=false;
  Serial.print(autoJointPtp.combined()
    ? F("[AUTO XYZ] salida corta + XYZ continuo a caja final; duration_ms=")
    : F("[AUTO PTP] TRANSFER IK destino una vez; quintica articular duration_ms="));
  Serial.print(autoJointPtp.durationMs());
  Serial.print(F(" preflight_us="));Serial.print(micros()-begun);
  Serial.println(F("; STREAM corto/ACK + PASS15 final; sin checkpoint intermedio ni MOVE largo."));
  return true;
}

void serviceAutoJointPtp(bool coarseTransport,float dt,float jointStepLimit) {
  if (!autoJointPtp.active() || autoPtpToken!=autoPickup.action().token) {
    controllerSafetyStop(F("AUTO PTP plan cancelado/invalido"));return;
  }
  if (autoJointPtp.complete()) {
    autoPointHold=true;
    commandedVelocity=Vec3{{0.0f,0.0f,0.0f}};
    sendAutoPickupHold();return; // final ACK then existing settle/checkpoint15
  }
  AutoJointPtp::Sample candidate{};
  const uint32_t interval=coarseTransport ? AutoTransport::MODEL_PERIOD_MS : Config::CONTROL_PERIOD_MS;
  if (!autoJointPtp.next(interval,candidate)) {
    controllerSafetyStop(F("AUTO PTP intervalo invalido"));return;
  }
  if (candidate.cartesian && !autoCombinedCurvePoint(candidate.xyz,candidate.theta,autoJointPtp)) {
    controllerSafetyStop(F("AUTO XYZ IK/FK/despeje invalido"));return;
  }
  float xyz[3];
  if (!autoPtpCurvePoint(candidate.theta,xyz)) {
    controllerSafetyStop(F("AUTO PTP FK/RAW/Jacobiano/despeje invalido"));return;
  }
  const Vec3 theta{{candidate.theta[0],candidate.theta[1],candidate.theta[2]}};
  const Vec3 pose{{xyz[0],xyz[1],xyz[2]}};
  const Vec3 velocity=scale(subtract(pose,referencePose),1.0f/dt);
  const float distance=magnitude(subtract(autoTargetPose,pose));
  const float previousDistance=magnitude(subtract(autoTargetPose,referencePose));
  const float speedLimit=fminf(distance,previousDistance)<=AutoPickupSetup::FINAL_APPROACH_DISTANCE_MM
    ? AutoPickupSetup::FINAL_APPROACH_SPEED_MM_S : AutoPickupSetup::LOADED_TRANSFER_SPEED_MM_S;
  const bool braking=magnitude(velocity)<magnitude(commandedVelocity);
  const float accelerationLimit=braking ? Config::DECEL_MM_S2 : Config::ACCEL_MM_S2;
  if (magnitude(velocity)>speedLimit+0.001f ||
      magnitude(subtract(velocity,commandedVelocity))>accelerationLimit*dt+0.001f) {
    controllerSafetyStop(F("AUTO PTP velocidad/aceleracion excedida"));return;
  }
  uint16_t raw[3];
  for (unsigned i=0u;i<3u;++i) {
    raw[i]=static_cast<uint16_t>(thetaToRaw(theta.v[i],i));
    if (fabsf(theta.v[i]-simulatedThetaRad.v[i])>jointStepLimit+0.000001f ||
        abs(static_cast<int>(raw[i])-static_cast<int>(nucleo.remoteRaw()[i]))>
          (coarseTransport ? AutoTransport::MAX_STEP_RAW : Config::STREAM_RAW_MAX_STEP)) {
      controllerSafetyStop(F("AUTO PTP paso articular/RAW excedido"));return;
    }
  }
  Mat3 jacobian{},inverseJacobian{};float determinant=NAN;
  if (!numericalJacobian(theta,jacobian) || !invert(jacobian,inverseJacobian,determinant)) {
    controllerSafetyStop(F("AUTO PTP Jacobiano singular"));return;
  }
  const float condition=frobeniusNorm(jacobian)*frobeniusNorm(inverseJacobian);
  if (!isfinite(condition) || condition>Config::MAX_CONDITION_F) {
    controllerSafetyStop(F("AUTO PTP condicion fuera de limite"));return;
  }
  if (!nucleo.sendStreamSetpoint(raw,
        coarseTransport ? AutoTransport::MOVE_TIME_MS : Config::STREAM_MOVE_TIME_MS,
        activeStreamOption | (coarseTransport ? DELTA_STREAM_SETPOINT_COARSE : 0u))) return;
  pendingThetaRad=theta;pendingReferencePose=pose;
  pendingConditionF=condition;pendingDeterminant=determinant;
  pendingErrorMm=0.0f; // reference IS the FK of this synchronized joint sample
  pendingInverseResidualMm=pendingTransposeResidualMm=pendingDlsResidualMm=pendingExactResidualMm=0.0f;
  commandedVelocity=velocity;
  autoPtpPendingElapsedMs=candidate.elapsedMs;
  autoPtpSegmentPending=true;
  pendingSetpoint=true;++streamCommandsSent;
  // Model phase is committed only by this segment's ACK. Late ACKs never
  // catch up elapsed wall time or inject the final endpoint into STOP.
}

bool autoPickupPathValid(const CameraPieceTarget &observation) {
  if (!std::isfinite(observation.robotY) || observation.robotY < -295.0f ||
      observation.robotY > 280.0f) return false;
  CameraPieceTarget piece = observation;
  piece.robotY = AutoPickupHeight::clampPickupY(observation.robotY);
  unsigned box = AutoBoxCalibration::COUNT;
  for (unsigned i=0u;i<AutoBoxCalibration::COUNT;++i)
    if (AutoBoxCalibration::PIECE_ID[i] == piece.id) box = i;
  if (box == AutoBoxCalibration::COUNT) return false;
  const float high = AutoPickupSetup::TRANSIT_Z_MM;
  const float wait = AutoPickupSetup::WAIT_Z_MM;
  const float x = AutoPickupSetup::PICKUP_X_MM;
  const float bx = AutoBoxCalibration::TARGET_MM[box][0];
  const float by = AutoBoxCalibration::TARGET_MM[box][1];
  const float bz = AutoBoxCalibration::TARGET_MM[box][2];
  const float pickupZ = AutoPickupHeight::targetZ(piece.robotY);
  const float approachZ = AutoPickupSetup::pickupApproachZ(piece.robotY);
  if (!std::isfinite(pickupZ) || pickupZ >= high ||
      !std::isfinite(approachZ) || approachZ <= pickupZ ||
      (AutoPickupSetup::COMBINED_APPROACH_ENABLED && approachZ >= wait)) return false;
  const Vec3 returnHigh = AutoPickupSetup::RETURN_XY_AT_DROP_Z_ENABLED
    ? Vec3{{0.0f,0.0f,bz}} : Vec3{{bx,by,high}};
  const Vec3 legacyRoute[] = {
    referencePose, {{x,piece.robotY,approachZ}}, {{x,piece.robotY,pickupZ}},
    {{x,piece.robotY,high}}, {{bx,by,high}}, {{bx,by,bz}},
    returnHigh, {{0.0f,0.0f,wait}},
    {{0.0f,0.0f,AutoPickupSetup::CONTINUOUS_ENABLED ? wait : Config::HOME_Z_MM}}
  };
  const Vec3 combinedRoute[] = {
    referencePose, {{x,piece.robotY,approachZ}}, {{x,piece.robotY,pickupZ}},
    {{bx,by,bz}}, returnHigh, {{0.0f,0.0f,wait}},
    {{0.0f,0.0f,AutoPickupSetup::CONTINUOUS_ENABLED ? wait : Config::HOME_Z_MM}}
  };
  const Vec3 rightRoute[] = {
    referencePose, {{x,piece.robotY,approachZ}}, {{x,piece.robotY,pickupZ}},
    {{x,piece.robotY,high}}, {{bx,by,bz}}, returnHigh,
    {{0.0f,0.0f,wait}},
    {{0.0f,0.0f,AutoPickupSetup::CONTINUOUS_ENABLED ? wait : Config::HOME_Z_MM}}
  };
  const Vec3 directRoute[] = {
    referencePose, {{x,piece.robotY,approachZ}}, {{x,piece.robotY,pickupZ}},
    {{x,piece.robotY,bz}}, {{bx,by,bz}},
    returnHigh,
    {{0.0f,0.0f,wait}},
    {{0.0f,0.0f,AutoPickupSetup::CONTINUOUS_ENABLED ? wait : Config::HOME_Z_MM}}
  };
  const bool combined=AutoPickupSetup::COMBINED_LOADED_TRANSFER_ENABLED;
  const bool twoDirect=combined && AutoPickupSetup::TWO_DIRECT_LOADED_MOVES_ENABLED;
  const AutoPickup::State directStages[] = {
    AutoPickup::State::IDLE,AutoPickup::State::ALIGN_ABOVE,
    AutoPickup::State::LOWER_PICKUP,AutoPickup::State::LIFT,
    AutoPickup::State::TRANSFER,
    AutoPickupSetup::RETURN_XY_AT_DROP_Z_ENABLED ? AutoPickup::State::RETURN_XY : AutoPickup::State::LIFT_RETURN,
    AutoPickupSetup::RETURN_XY_AT_DROP_Z_ENABLED ? AutoPickup::State::RETURN_HOME : AutoPickup::State::RETURN_XY,
    AutoPickup::State::RETURN_HOME
  };
  static_assert(sizeof(directStages)/sizeof(directStages[0]) ==
                sizeof(directRoute)/sizeof(directRoute[0]),"one stage per direct segment");
  const bool rightFirst=combined && AutoPickupSetup::RIGHT_LIFT_FIRST_ENABLED &&
    piece.robotY>=AutoPickupSetup::RIGHT_LIFT_THRESHOLD_Y_MM;
  /* TWO_DIRECT is a post-pickup policy for every accepted piece. Its first
     endpoint is vertical over pickup at the selected box Z; its second has
     the same Z and the selected box X/Y. Legacy right-first keeps TRANSIT_Z. */
  const bool liftThenBox=twoDirect || rightFirst;
  const Vec3* route=twoDirect ? directRoute :
    (rightFirst ? rightRoute : (combined ? combinedRoute : legacyRoute));
  const unsigned routeCount=twoDirect ? sizeof(directRoute)/sizeof(directRoute[0])
    : rightFirst ? sizeof(rightRoute)/sizeof(rightRoute[0])
    : combined ? sizeof(combinedRoute)/sizeof(combinedRoute[0])
               : sizeof(legacyRoute)/sizeof(legacyRoute[0]);
  const unsigned loadedSegment=liftThenBox ? 4u : 3u;
  const unsigned returnSegment=liftThenBox ? 6u : (combined ? 5u : 7u);
  const uint32_t begun = micros();
  for (unsigned i=1u;i<routeCount;++i) {
    // Continuous operation ends at WAIT, so the last route entry repeats it.
    // Validate the stationary pose once rather than budgeting hundreds of
    // identical FK/Jacobian evaluations for a non-existent movement.
    if (magnitude(subtract(route[i],route[i-1u]))<0.001f) {
      if (micros()-begun>20000u || !workspacePointValid(route[i]) ||
          micros()-begun>20000u) return false;
      continue;
    }
    if (twoDirect && AutoPickupSetup::positionMove(directStages[i])) {
      Vec3 q{};uint16_t raw[3];AutoDirectEndpoint::Profile direct{};
      if (micros()-begun>20000u || !deltaInverse(route[i-1u],q)) return false;
      for (uint8_t a=0u;a<3u;++a) raw[a]=(uint16_t)thetaToRaw(q.v[a],a);
      if (!prepareAutoDirectMove(raw,route[i],direct,begun,directStages[i],
                                AutoPickupSetup::ROUTE_PREFLIGHT_SAMPLES) ||
          direct.durationMs+2000u+AutoPickupSetup::cartesianSettleMs(directStages[i])+
          DELTA_AUTO_CHECKPOINT_TIMEOUT_MS>AutoPickupSetup::CARTESIAN_STAGE_TIMEOUT_MS)
        return false;
      continue;
    }
    if (AutoPickupSetup::LOCAL_POINT_TRANSIT_ENABLED && combined && i>=loadedSegment) {
      Vec3 start=route[i-1u];
      if (i==loadedSegment) {
        const float exitZ=fmaxf(start.v[2],AutoPickupHeight::clearanceZBetween(start.v[1],
          route[i].v[1],AutoPickupSetup::APPROACH_CLEARANCE_MM)+3.0f);
        const unsigned liftSteps=(unsigned)ceilf((exitZ-start.v[2])/5.0f)+1u;
        for (unsigned j=0u;j<=liftSteps;++j) {
          Vec3 p=start;p.v[2]+=(exitZ-start.v[2])*(float)j/(float)liftSteps;
          if (micros()-begun>20000u || !workspacePointValid(p)) return false;
        }
        start.v[2]=exitZ;
      }
      Vec3 q{};uint16_t raw[3];DeltaPointProfile profile{};
      if (!deltaInverse(start,q)) return false;
      for (uint8_t a=0u;a<3u;++a) raw[a]=(uint16_t)thetaToRaw(q.v[a],a);
      if (!prepareAutoLocalPoint(raw,route[i],profile,begun) ||
          profile.durationMs+2000u+AutoPickupSetup::LOADED_CARTESIAN_SETTLE_MS+
          DELTA_AUTO_CHECKPOINT_TIMEOUT_MS>AutoPickupSetup::CARTESIAN_STAGE_TIMEOUT_MS) return false;
      continue;
    }
    if (combined && i==loadedSegment) {
      AutoJointPtp::Plan check;
      const float clearance=AutoPickupHeight::clearanceZBetween(piece.robotY,by,
        AutoPickupSetup::APPROACH_CLEARANCE_MM);
      const auto evaluate=[begun,&check](const float p[3],float q[3]) {
        return micros()-begun<=20000u && autoCombinedCurvePoint(p,q,check);
      };
      if (!check.beginCombined(route[i-1u].v,route[i].v,clearance,
          Config::MAX_JOINT_STEP_RAD*1000.0f/Config::CONTROL_PERIOD_MS,
          AutoPickupSetup::LOADED_TRANSFER_SPEED_MM_S,AutoPickupSetup::FINAL_APPROACH_SPEED_MM_S,
          AutoPickupSetup::FINAL_APPROACH_DISTANCE_MM,Config::ACCEL_MM_S2,Config::DECEL_MM_S2,evaluate) ||
          check.durationMs()+AutoPickupSetup::LOADED_CARTESIAN_SETTLE_MS+
            DELTA_AUTO_CHECKPOINT_TIMEOUT_MS+Config::CONTROL_PERIOD_MS >
            AutoPickupSetup::CARTESIAN_STAGE_TIMEOUT_MS || micros()-begun>20000u) return false;
      continue;
    }
    const Vec3 delta = subtract(route[i],route[i-1u]);
    const unsigned steps = static_cast<unsigned>(ceilf(magnitude(delta)/10.0f)) + 1u;
    if (steps > 64u) return false;
    for (unsigned j=0u;j<=steps;++j) {
      const Vec3 sample = add(route[i-1u],scale(delta,static_cast<float>(j)/steps));
      if (micros()-begun > 20000u || !workspacePointValid(sample)) return false;
      // Model envelope on the combined empty approach and closed empty return.
      // The final vertical pickup leg intentionally crosses this clearance.
      // 0.001 mm covers float interpolation rounding, not a physical tolerance.
      if (((i == 1u && AutoPickupSetup::COMBINED_APPROACH_ENABLED) || i == returnSegment) &&
          sample.v[2] + 0.001f < AutoPickupHeight::targetZ(sample.v[1]) +
                                AutoPickupSetup::APPROACH_CLEARANCE_MM) return false;
    }
  }
  return micros()-begun<=20000u; // numeric path only, NOT a collision certificate
}

// Return the first live-health failure, or nullptr when every required permit
// is still valid. The nullptr contract is also used by the bounded point
// preflight immediately before transmitting POINT_START.
const __FlashStringHelper *autoPickupHealthFailure(uint32_t now, bool preparing,
                                                 bool requireEncoder) {
  if (!gamepadFresh) return F("GAMEPAD_OFF_OR_LOST");
  if (now-lastGamepadMs > (preparing ? 150u : AutoPickupSetup::GAMEPAD_RUNTIME_MAX_AGE_MS)) return F("GAMEPAD_STALE");
  if (preparing) {
    if (!axesCentered()) return F("AXES_NOT_CENTERED");
    if (gamepad.dpad || (gamepad.buttons & 0xF0u)) return F("MANUAL_INPUT");
    if (stopRequested) return F("STOP_REQUESTED");
    if (now-autoStartMs > GRIP_ARM_TIMEOUT_MS) return F("PREPARATION_TIMEOUT");
  }
  if (!nucleo.ready()) return F("LINK_NOT_READY");
  if (nucleo.sessionGeneration()!=autoStartGeneration) return F("LINK_SESSION_CHANGED");
  if (nucleo.remoteFault()) return F("NUCLEO_FAULT");
  if (nucleo.dryRun()) return F("DRY_RUN");
  if (preparing) {
    if (!autoHomePositionVerified()) return F("HOME_NOT_VERIFIED");
  } else {
    if (!nucleo.armed()) return F("NOT_ARMED");
    if (!nucleo.streamActive()) return F("STREAM_INACTIVE");
  }
  if (!cameraAlignment.ready()) return F("CAMERA_NOT_READY");
  if (!preparing && cameraAlignment.lastPacketAgeMs()>Config::CAMERA_TIMEOUT_MS)
    return F("CAMERA_STALE");
  if (requireEncoder) {
    if (!nucleo.encoderKnown()) return F("ENCODER_UNKNOWN");
    if (nucleo.encoderIntervalMs()>200u) return F("ENCODER_INTERVAL");
    if (now-nucleo.encoderReceivedMs()>200u) return F("ENCODER_STALE");
    if (!nucleo.encoderFastFresh()) return F("ENCODER_NOT_FRESH");
  }
  return nullptr;
}

void pauseAutoPickupHealthFailure(const __FlashStringHelper *context,
                                 const __FlashStringHelper *pauseReason,
                                 uint32_t now, bool preparing, bool requireEncoder) {
  // Capture the failing state before pause clears authorization and aborts.
  const __FlashStringHelper *failure = autoPickupHealthFailure(now,preparing,requireEncoder);
  // The outer guard can observe an additional condition or a state can recover
  // between its decision and this diagnostic snapshot. Never print a null
  // pointer; retain the generic explanation only for that diagnostic race.
  if (failure==nullptr) failure=F("HEALTH_STATE_CHANGED");
  const AutoPickup::State stoppedState = autoPickup.state();
  const uint32_t gamepadAge = now-lastGamepadMs;
  const bool encoderKnown = nucleo.encoderKnown();
  const uint32_t encoderAge = nucleo.encoderAgeMs();
  const uint16_t encoderInterval = nucleo.encoderIntervalMs();
  const uint16_t encoderSequence = nucleo.encoderSequence();
  const uint32_t cameraAge = cameraAlignment.lastPacketAgeMs();
  const bool linkReady = nucleo.ready();
  const uint8_t linkFlags = nucleo.remoteFlags();
  const bool checkpointActive = nucleo.checkpointActive();
  const uint16_t checkpointSequence = nucleo.checkpointSequence();
  reopenMenuAfterStop = true;
  pauseSimulation(pauseReason); // STOP/PWM shutdown takes priority over extra logging.
  Serial.print(F("[AUTO HEALTH] context=")); Serial.print(context);
  Serial.print(F(" reason=")); Serial.print(failure);
  Serial.print(F(" state=")); Serial.print(AutoPickup::Sequence::stateName(stoppedState));
  Serial.print(F(" gp_age_ms=")); Serial.print(gamepadAge);
  Serial.print(F(" gp_session=0x")); Serial.print(activeGamepadSession,HEX);
  Serial.print(F(" gp_seq=")); Serial.print(lastGamepadSequence);
  Serial.print(F(" gp_rx_gap_max_ms=")); Serial.print(gamepadGapMaxMs);
  Serial.print(F(" cam_age_ms=")); Serial.print(cameraAge);
  Serial.print(F(" encoder_known=")); Serial.print(encoderKnown);
  Serial.print(F(" encoder_age_ms="));
  if (encoderKnown) Serial.print(encoderAge);
  else Serial.print(F("SIN_DATO"));
  Serial.print(F(" encoder_dt_ms=")); Serial.print(encoderInterval);
  Serial.print(F(" encoder_seq=")); Serial.print(encoderSequence);
  Serial.print(F(" ready=")); Serial.print(linkReady);
  Serial.print(F(" flags=0x")); Serial.print(linkFlags,HEX);
  Serial.print(F(" checkpoint=")); Serial.print(checkpointActive);
  Serial.print(F(" checkpoint_seq=")); Serial.println(checkpointSequence);
}

const __FlashStringHelper *autoPickupStoppedResultMessage() {
  if (autoPickup.completedCycles()!=0u && autoPickup.piece().id==0u)
    return F("[AUTO PICKUP RESULT] ESPERA_FINALIZADA; ciclos previos completos, parada VERIFY PASS, PWM OFF. Nueva sesion requiere HOME/doble X.");
  if (autoPickup.state()==AutoPickup::State::COMPLETE)
    return F("[AUTO PICKUP RESULT] CICLO_COMPLETO; parada VERIFY PASS, PWM OFF. Comprobar pieza/recorrido antes de nueva doble X.");
  return F("[AUTO PICKUP RESULT] CICLO_INCOMPLETO; parada VERIFY PASS, PWM OFF. No confirma recogida; nueva doble X requerida.");
}

void serviceAutoPickup() {
  if (operationMode != OperationMode::AUTO_PICKUP) return;
  uint32_t now = millis();
  // A valid datagram may have arrived since loop() drained UDP. Check once at
  // the boundary, using the normal CRC/session/sequence/button handlers. This
  // is NOT a grace interval: absent/invalid frames stop at 150ms preparation /
  // GAMEPAD_RUNTIME_MAX_AGE_MS runtime; explicit OFF still stops immediately.
  if (gamepadFresh && now-lastGamepadMs >
      (startPhase == StartPhase::WAIT_AUTO_REF ? 150u : AutoPickupSetup::GAMEPAD_RUNTIME_MAX_AGE_MS)) {
    serviceUdp();
    now = millis();
    // Processing UDP can itself handle OPTIONS, OFF or a new ESP session.
    if (operationMode != OperationMode::AUTO_PICKUP || stopRequested) return;
  }
  if (startPhase == StartPhase::WAIT_AUTO_REF) {
    if (!gamepadFresh || now-lastGamepadMs > 150u || !axesCentered() ||
        gamepad.dpad || (gamepad.buttons & 0xF0u) || stopRequested ||
        now-autoStartMs > GRIP_ARM_TIMEOUT_MS || !nucleo.ready() ||
        nucleo.sessionGeneration()!=autoStartGeneration || nucleo.remoteFault() ||
        nucleo.dryRun() || !autoHomePositionVerified() || !cameraAlignment.ready()) {
      pauseAutoPickupHealthFailure(F("PREPARACION"),F("AUTO preparacion interrumpida"),now,true,false);
      return;
    }
    if (nucleo.waitingResponse() || !autoSnapshotReceived) return;
    if (!gripStateStopped()) {
      reopenMenuAfterStop = true;
      pauseSimulation(F("AUTO referencia RC no valida"));
      return;
    }
    const uint16_t *rc = nucleo.gripperStateUs();
    for (unsigned i=0u;i<3u;++i) gripUs[i]=autoRcPendingUs[i]=rc[i];
    if (nucleo.sendArm()) {
      autoAuthorized = true;
      cameraAlignment.clearPendingTargetForArm();
      startPhase = StartPhase::WAIT_ARM_ACK;
      Serial.println(F("[AUTO PICKUP AUTH] Continuo hasta menu/STOP; preparacion RC y espera nueva observacion tras armado."));
    }
    return;
  }
  if (!autoAuthorized || !runEnabled || stopRequested) return;
  if (autoCapabilityPending) {
    // New command support is proved while holding HOME and PWM is still OFF.
    // An old 10C therefore cannot start automatic RC positioning first.
    DeltaResultCode result;
    uint16_t measured[3];
    if (!nucleo.ready() || nucleo.sessionGeneration()!=autoStartGeneration ||
        nucleo.remoteFault() || nucleo.dryRun() || !nucleo.armed() ||
        !nucleo.streamActive() || !gamepadFresh ||
        now-lastGamepadMs>AutoPickupSetup::GAMEPAD_RUNTIME_MAX_AGE_MS ||
        !cameraAlignment.ready() ||
        cameraAlignment.lastPacketAgeMs()>Config::CAMERA_TIMEOUT_MS) {
      pauseAutoPickupHealthFailure(F("CAPACIDAD"),F("AUTO prueba de capacidad interrumpida"),now,false,false);
      return;
    }
    if (nucleo.consumeCheckpoint(result,measured)) {
      if (result!=DELTA_RESULT_OK) {
        reopenMenuAfterStop=true;
        pauseSimulation(F("AUTO requiere STEPS5/DIRECT300/TX5/REC5/REC350/CP5 antes de PWM; usar 10C v2.50"));
        return;
      }
      autoCapabilityPassed=true;
    }
    // A renewal ACK can still be in flight when the physical PASS arrives.
    // Defer the BEGIN until that same UART command slot is free; do not treat
    // the deferred PASS as a new verification or lose it on consumption.
    if (autoCapabilityPassed && !nucleo.waitingResponse()) {
      autoCapabilityPending=false;
      autoCapabilityPassed=false;
      if (!nucleo.sendGripperGroup(DELTA_MSG_GRIP_GROUP_BEGIN,autoRcPendingUs,true,true)) {
        reopenMenuAfterStop=true;
        pauseSimulation(F("AUTO inicio RC rechazado tras checkpoint"));
        return;
      }
      autoRcPending=true; autoRcBeginPending=true; autoRcPendingToken=0u;
    }
    return;
  }
  if (!autoRcActive) return;
  if (!nucleo.encoderFastFresh() && now-autoRuntimeStartMs<=200u) return;
  if (!gamepadFresh || now-lastGamepadMs > AutoPickupSetup::GAMEPAD_RUNTIME_MAX_AGE_MS || !nucleo.ready() ||
      nucleo.sessionGeneration()!=autoStartGeneration || nucleo.remoteFault() ||
      nucleo.dryRun() || !nucleo.armed() || !nucleo.streamActive() ||
      !cameraAlignment.ready() || cameraAlignment.lastPacketAgeMs()>Config::CAMERA_TIMEOUT_MS ||
      !nucleo.encoderFastFresh()) {
    pauseAutoPickupHealthFailure(F("RUNTIME"),F("AUTO requiere mando/camara/encoder/enlace frescos"),now,false,true);
    return;
  }
  const float advance = autoEncoderAssociated ? AutoPickupSetup::advanceMm(
    autoEncoderAtFrame,nucleo.encoderPosition()) : 0.0f;
  const uint32_t arrivalWaitMs = autoPickup.state()==AutoPickup::State::WAIT_ARRIVAL
    ? autoPickup.stateAgeMs(now) : 0u;
  // Diagnostic only: a threshold reached while still lowering must NOT bypass
  // pickup PASS. It explains whether lateness is geometric or arm readiness.
  const AutoGateEncoder::Target& gateTarget = autoPickup.gateTarget();
  if (!autoGateThresholdSeen && autoEncoderAssociated && nucleo.encoderFastFresh() &&
      gateTarget.reached(nucleo.encoderPosition()) &&
      (autoPickup.state()==AutoPickup::State::ALIGN_ABOVE ||
       autoPickup.state()==AutoPickup::State::LOWER_PICKUP ||
       autoPickup.state()==AutoPickup::State::WAIT_ARRIVAL)) {
    autoGateThresholdSeen = true;
    autoGateThresholdSeenMs = now;
    Serial.print(F("[AUTO GATE COUNT REACHED] state="));
    Serial.print(AutoPickup::Sequence::stateName(autoPickup.state()));
    Serial.print(F(" encoder=")); Serial.print(nucleo.encoderPosition());
    Serial.print(F(" target_bits=")); Serial.println(gateTarget.targetCountBits);
  }
  autoPickup.tickEncoder(now,nucleo.encoderFastFresh(),nucleo.encoderPosition());
  DeltaResultCode checkpointResult;
  uint16_t measured[3];
  if (!(autoDirectMove && autoLocalPhase==LocalPointPhase::WAIT_PRECHECK) &&
      nucleo.consumeCheckpoint(checkpointResult,measured)) {
    const bool pickupVerified = autoPickup.state()==AutoPickup::State::LOWER_PICKUP;
    const bool centerVerified = AutoPickupSetup::centerRecovery(autoPickup.state());
    if (AutoPickupSetup::ENDPOINT_RETRY_ENABLED && checkpointResult==DELTA_RESULT_OBSERVED &&
        autoCheckpointToken!=0u && autoCheckpointToken==autoPickup.action().token &&
        autoPickup.action().kind==AutoPickup::Kind::CARTESIAN) {
      bool missed=false,tooFar=false;
      for (uint8_t i=0u;i<3u;++i) {
        autoEndpointNominal[i]=nucleo.remoteRaw()[i];
        const int error=abs((int)measured[i]-(int)autoEndpointNominal[i]);
        missed|=error>AutoPickupSetup::endpointRecoveryTolerance(autoPickup.state());
        tooFar|=error>DELTA_ENDPOINT_RETRY_MAX_ERROR_RAW;
      }
      if (missed) {
        Serial.print(F("[AUTO ENDPOINT MISS] measured="));
        for(uint8_t i=0u;i<3u;++i){if(i)Serial.print(',');Serial.print(measured[i]);}
        Serial.print(F(" attempt="));Serial.println(autoEndpointRetryAttempts);
        if (tooFar || autoEndpointRetryAttempts>=DELTA_ENDPOINT_RETRY_MAX_ATTEMPTS) {
          pauseSimulation(F("AUTO endpoint fuera de recuperacion350 o dos reintentos agotados; no continuar"));return;
        }
        autoCheckpointToken=0u;autoPointHold=autoPointSettling=false;
        autoEndpointRetryPending=true;autoDirectMove=true;
        autoEndpointStepPending=autoEndpointRetryAttempts>=1u;
        autoEndpointOriginRefreshed=false;
        autoEndpointRetryQueuedMs=now;
        Serial.print(F("[AUTO ENDPOINT RECOVERY] wait_ms=100 next="));
        Serial.println(autoEndpointStepPending?F("STEPS4/100; fresh CP before retry"):F("DIRECT; fresh CP before retry"));
        autoDirectRcRenewalKnown=false;autoLocalPhase=LocalPointPhase::READY;
        return; // same action/token/destination. No dumping or next target yet.
      }
    }
    if(checkpointResult==DELTA_RESULT_OBSERVED && AutoPickupSetup::centerRecovery(autoPickup.state()) &&
        !autoCenterPrecisionPending && autoCheckpointToken!=0u &&
        autoCheckpointToken==autoPickup.action().token){
      autoCenterPrecisionPending=true;autoCheckpointToken=0u;autoPointHold=true;
      autoDirectMove=false;autoLocalPhase=LocalPointPhase::FINISHED;autoPointSettling=false;
      Serial.println(F("[AUTO CENTER RECOVERED] dentro12; no avanzar sin CP5 fisico."));
      return;
    }
    if (!AutoPickupSetup::checkpointResultAccepted(autoPickup.state(),checkpointResult) || autoCheckpointToken==0u ||
        autoCheckpointToken!=autoPickup.action().token ||
        !autoPickup.markCartesianVerified(autoCheckpointToken,now)) {
      reopenMenuAfterStop = true;
      pauseSimulation(F("AUTO checkpoint invalido/fallido"));
      return;
    }
    if (checkpointResult == DELTA_RESULT_OBSERVED) {
      Serial.print(F("[AUTO POSITION OBSERVED] state="));
      Serial.print(AutoPickup::Sequence::stateName(autoPickup.state()));
      Serial.print(F(" measured="));
      for (uint8_t i=0u;i<3u;++i) {if(i)Serial.print(',');Serial.print(measured[i]);}
      Serial.println(F("; llegada dentro15; siguiente ruta usa feedback real."));
    }
    if (pickupVerified) {
      Serial.print(F("[AUTO PICKUP VERIFIED] tolerance_raw=5 measured="));
      for (uint8_t i=0u;i<3u;++i) {
        if (i) Serial.print(',');
        Serial.print(measured[i]);
      }
      Serial.println(F("; feedback PASS antes de esperar llegada/cerrar."));
    }
    if(centerVerified)
      Serial.println(F("[AUTO CENTER VERIFIED] CP5 fisico PASS; retorno puede continuar."));
    autoCheckpointToken = 0u;
    autoPointHold = false;
    autoPointSettling = false;
  }
  const AutoPickup::Action &action = autoPickup.action();
  if (action.kind==AutoPickup::Kind::FAULT) {
    Serial.print(F("[AUTO PICKUP FAULT] reason="));
    Serial.println(static_cast<unsigned>(autoPickup.fault()));
    reopenMenuAfterStop = true;
    pauseSimulation(F("AUTO secuencia abortada"));
    return;
  }
  if (action.token!=autoActionToken) {
    autoEndpointRetryPending=autoEndpointStepPending=false;autoEndpointRetryAttempts=0u;autoEndpointDurationFloor=0u;
    autoCenterPrecisionPending=false;
    autoLocalPhase=LocalPointPhase::IDLE;autoLocalToken=0u;autoDirectMove=false;
    autoDirectRcRenewalKnown=false;
    autoJointPtp.cancel();
    autoPtpToken=0u;
    autoPtpSegmentPending=false;
    autoActionToken = action.token;
    autoTargetActive = action.kind==AutoPickup::Kind::CARTESIAN;
    autoPointHold = false;
    autoPointSettling = false;
    commandedVelocity = Vec3{{0.0f,0.0f,0.0f}};
    Serial.print(F("[AUTO PICKUP STATE] "));
    Serial.println(AutoPickup::Sequence::stateName(autoPickup.state()));
    if (autoPickup.state()==AutoPickup::State::CLOSE_GATE) {
      autoGateTriggerMs = now;
      calibrationGateEvent('T', now);
      autoGateFirstAckLogged = autoGateFinalAckLogged = false;
      Serial.print(F("[AUTO CLOSE TRIGGER] distance_mm="));
      Serial.print(autoPickup.piece().distanceToArrivalMm,1);
      Serial.print(F(" advance_mm=")); Serial.print(advance,1);
      Serial.print(F(" remaining_mm=")); Serial.print(autoPickup.piece().distanceToArrivalMm-advance,1);
      Serial.print(F(" close_lead_mm=")); Serial.print(autoPickup.closeLeadMm(),2);
      Serial.print(F(" arrival_window_mm=")); Serial.print(AutoPickupSetup::ARRIVAL_TOLERANCE_MM,1);
      Serial.print(F(" wait_ms=")); Serial.print(arrivalWaitMs);
      Serial.print(F(" count_due_age_ms="));
      Serial.println(autoGateThresholdSeen ? now-autoGateThresholdSeenMs : 0u);
      Serial.print(F("[AUTO GATE RAMP] period_ms="));
      Serial.print(AutoPickupSetup::rcRampPeriodMs(autoPickup.state()));
      Serial.print(F(" step_us=")); Serial.println(AutoPickupSetup::RC_RAMP_STEP_US);
    }
    if (autoPickup.state()==AutoPickup::State::WAIT_PIECE) {
      autoPiecePending = false;
      if (autoPickup.completedCycles() == 0u) cameraAlignment.clearPendingTargetForArm();
      else cameraAlignment.clearPendingTargetForNextCycle();
      Serial.println(F("[AUTO PICKUP READY] Presenta UNA pieza nueva; no recoge una observacion anterior."));
    }
    if (action.kind==AutoPickup::Kind::CARTESIAN) {
      autoTargetPose = Vec3{{action.cartesian.x,action.cartesian.y,action.cartesian.z}};
      if (action.concurrentRc)
        Serial.println(F("[AUTO PICKUP OVERLAP] X1270->1510 junto con ALIGN_ABOVE; checkpoint y bajada esperan consigna RC ACK + settle. Sin feedback RC fisico."));
      if (AutoPickupSetup::positionMove(autoPickup.state()))
        Serial.println(F("[AUTO ROUTE] POR POSICION: endpoint validado por etapa y checkpoint final."));
      else if (autoPickup.state()==AutoPickup::State::ALIGN_ABOVE ||
               autoPickup.state()==AutoPickup::State::LOWER_PICKUP) {
        Serial.print(F("[AUTO ROUTE] FINO DLS/STREAM: max_step_raw="));
        Serial.print(Config::STREAM_RAW_MAX_STEP);
        Serial.print(F(" period_ms="));Serial.print(Config::CONTROL_PERIOD_MS);
        Serial.println(F("; avance tras ACK y checkpoint fisico final CP5."));
      }
      Serial.print(F("[AUTO SPEED PROFILE] state="));
      Serial.print(AutoPickup::Sequence::stateName(autoPickup.state()));
      Serial.print(F(" profile=")); Serial.print(AutoSpeedProfile::name(autoPickup.state()));
      Serial.print(F(" cruise_mm_s=")); Serial.print(AutoSpeedProfile::cruiseLimit(
        autoPickup.state(),AutoPickupSetup::speedProfileConfig()),1);
      Serial.print(F(" final_mm_s=")); Serial.print(AutoPickupSetup::FINAL_APPROACH_SPEED_MM_S,1);
      Serial.print(F(" final_zone_mm=")); Serial.println(AutoPickupSetup::FINAL_APPROACH_DISTANCE_MM,1);
      if (!workspacePointValid(autoTargetPose)) {
        reopenMenuAfterStop = true;
        pauseSimulation(F("AUTO destino fuera de IK/FK/RAW/Jacobiano"));
        return;
      }
      if (AutoPickupSetup::positionMove(autoPickup.state())) {
        autoLocalGoal=autoTargetPose;autoLocalToken=action.token;autoLocalStartedMs=now;
        autoDirectMove=true;autoLocalPhase=LocalPointPhase::PRECHECK;
      } else if (AutoPickupSetup::LOCAL_POINT_TRANSIT_ENABLED &&
                 AutoLocalPoint::eligible(autoPickup.state())) {
        autoLocalGoal=autoTargetPose;autoLocalToken=action.token;autoLocalStartedMs=now;
        autoDirectMove=false;autoLocalPhase=LocalPointPhase::READY;
        const float exitZ=AutoPickupHeight::clearanceZBetween(referencePose.v[1],
          autoTargetPose.v[1],AutoPickupSetup::APPROACH_CLEARANCE_MM)+3.0f;
        if (autoPickup.state()==AutoPickup::State::TRANSFER && referencePose.v[2]<exitZ) {
          autoLocalPhase=LocalPointPhase::ESCAPE;
          autoTargetPose=Vec3{{referencePose.v[0],referencePose.v[1],exitZ}};
          if (!workspacePointValid(autoTargetPose)) {
            pauseSimulation(F("AUTO salida vertical invalida"));return;
          }
        }
      } else if (AutoJointPtp::eligible(autoPickup.state()) && !beginAutoJointPtp()) {
        reopenMenuAfterStop=true;
        pauseSimulation(F("AUTO PTP preflight articular rechazado antes de TRANSFER"));
        return;
      }
    }
    oledDirty = true;
  }
  if (action.kind==AutoPickup::Kind::COMPLETE) {
    if (autoPickup.continueAfterComplete(now)) {
      autoPiecePending = false;
      autoEncoderAssociated = false;
      autoPieceId = 0u;
      autoTargetActive = false;
      autoPointHold = false;
      autoPointSettling = false;
      Serial.print(F("[AUTO PICKUP COMPLETE] cycles="));
      Serial.print(autoPickup.completedCycles());
      Serial.println(F("; retorno verificado a espera AUTO. Sigue armado; esperando pieza NUEVA sin otro ARM."));
      return; // next scheduler turn observes the new WAIT_PIECE token
    }
    autoAuthorized = false;
    autoRcActive = false;
    runEnabled = false;
    stopRequested = true;
    reopenMenuAfterStop = true;
    Serial.println(F("[AUTO PICKUP COMPLETE] Retorno verificado; cerrando STREAM/PWM. Nueva pieza requiere doble X, nunca rearme automatico."));
    return;
  }
  if (autoPickup.state()==AutoPickup::State::WAIT_PIECE) {
    if (!autoPiecePending) autoPiecePending = cameraAlignment.consumePieceTarget(autoPendingPiece);
    if (autoPiecePending) {
      // Wait for an encoder sample AFTER the camera observation so history can
      // bracket it. Never replace its detection count with the current count.
      if (now-autoPendingPiece.observedMs>AutoPickup::Sequence::CAMERA_MAX_AGE_MS) {
        autoPiecePending = false;
      } else if (static_cast<int32_t>(nucleo.encoderReceivedMs()-autoPendingPiece.observedMs)>=0) {
        int32_t count;
        if (!nucleo.encoderPositionAt(autoPendingPiece.observedMs,count) ||
            !autoPickupPathValid(autoPendingPiece)) {
          reopenMenuAfterStop = true;
          pauseSimulation(F("AUTO observacion sin encoder asociado/ruta numerica valida"));
          return;
        }
        AutoPickup::Piece piece;
        piece.id=autoPendingPiece.id;
        piece.robotYMm=AutoPickupHeight::clampPickupY(autoPendingPiece.robotY);
        if (piece.robotYMm != autoPendingPiece.robotY) {
          Serial.print(F("[AUTO PICKUP LATERAL LIMIT] requested_y="));Serial.print(autoPendingPiece.robotY,2);
          Serial.print(F(" target_y="));Serial.print(piece.robotYMm,2);
          Serial.println(F("; limite local de recogida, X robot=0; no se persigue fuera de banda."));
        }
        const float acceptBeltMmS = nucleo.encoderFresh() ? nucleo.beltSpeedMmS() : NAN;
        const float gateOffsetMm = AutoPickupSetup::gateCloseOffsetMm(piece.id,acceptBeltMmS);
        piece.distanceToArrivalMm=AutoPickupSetup::distanceToArrival(
          autoPendingPiece.worldLongitudinalMm,piece.id,acceptBeltMmS);
        piece.observedAtMs=autoPendingPiece.observedMs;
        piece.encoderAtFrame=count;
        const AutoGateEncoder::LeadSelection gateLead = AutoGateEncoder::selectLead(
          AutoPickupSetup::SUPERVISED_TRIAL_ENABLED &&
            AutoPickupSetup::GATE_EXPERIMENTAL_LEAD_ENABLED,piece.id,piece.robotYMm,
          acceptBeltMmS,AutoPickupSetup::CLOSE_LEAD_MM,AutoPickupSetup::ARRIVAL_TOLERANCE_MM);
        piece.closeLeadMm=static_cast<float>(gateLead.closeLeadMm);
        autoGateThresholdSeen=false;
        autoGateThresholdSeenMs=0u;
        autoEncoderAtFrame=count; autoEncoderAssociated=true;
        autoPiecePending=false;
        if (!autoPickup.acceptPiece(piece,now,AutoPickupSetup::advanceMm(count,nucleo.encoderPosition()))) {
          reopenMenuAfterStop=true;
          pauseSimulation(F("AUTO pieza tardia/invalida"));
        } else {
          ++calibrationUsbTrial;
          if (calibrationUsbTrial == 0u) ++calibrationUsbTrial;
          if (calibrationUsb.active(now)) {
            char line[CalibrationUsb::PACKET_BYTES];
            calibrationUsb.push(line, CalibrationUsb::piece(line, calibrationEpoch(), now,
              calibrationUsbTrial, count, piece.id, piece.distanceToArrivalMm, piece.robotYMm));
          }
          autoPieceId = piece.id;
          Serial.print(F("[AUTO PICKUP PIECE] id=")); Serial.print(piece.id);
          Serial.print(F(" Y=")); Serial.print(piece.robotYMm,2);
          Serial.print(F(" pickup_z_mm=")); Serial.print(autoPickup.pickupZMm(),2);
          Serial.print(F(" approach_z_mm=")); Serial.print(autoPickup.approachZMm(),2);
          Serial.print(F(" lateral_cal=")); Serial.print(AutoPickupHeight::calibrated(piece.robotYMm) ? 1 : 0);
          Serial.print(F(" distance_mm=")); Serial.print(piece.distanceToArrivalMm,2);
          Serial.print(F(" encoder_at_frame=")); Serial.println(count);
          Serial.print(F("[AUTO GATE ARRIVAL] longitudinal_mm="));
          Serial.print(autoPendingPiece.worldLongitudinalMm,2);
          Serial.print(F(" gate_line_mm=")); Serial.print(AutoPickupSetup::GATE_LINE_FROM_IMAGE_CENTRE_MM,1);
          Serial.print(F(" geometry_mm="));
          Serial.print(AutoPickupSetup::distanceToArrival(autoPendingPiece.worldLongitudinalMm),2);
          Serial.print(F(" offset_mm=")); Serial.print(gateOffsetMm,2);
          Serial.print(F(" (all=")); Serial.print(AutoPickupSetup::GATE_CLOSE_OFFSET_MM,1);
          Serial.print(F(" class=")); Serial.print(piece.id==1u ? AutoPickupSetup::GATE_CLOSE_OFFSET_PIECE_C_MM
            : piece.id==2u ? AutoPickupSetup::GATE_CLOSE_OFFSET_SMALL_MM : 0.0f,1);
          Serial.print(F(" ms=")); Serial.print(AutoPickupSetup::GATE_CLOSE_OFFSET_MS,0);
          Serial.print(F(")")); Serial.print(F(" belt_mm_s=")); Serial.println(acceptBeltMmS,1);
          Serial.print(F("[AUTO GATE TARGET] counts_per_mm="));
          Serial.print(AutoPickupSetup::ENCODER_COUNTS_PER_MM,6);
          Serial.print(F(" advance_counts=")); Serial.print(autoPickup.gateTarget().requiredAdvanceCounts);
          Serial.print(F(" target_bits=")); Serial.print(autoPickup.gateTarget().targetCountBits);
          Serial.print(F(" lead_mm=")); Serial.print(autoPickup.closeLeadMm(),2);
          Serial.print(F(" source=")); Serial.println(gateLead.experimental
            ? F("EXPERIMENTAL_C_SESSION_20261015") : F("DEFAULT_TRIAL"));
        }
        return; // observe new token before any motion/RC change
      }
    }
  }
  if (nucleo.waitingResponse() || pendingSetpoint || autoRcPending) return;
  if (autoPointHold && action.kind==AutoPickup::Kind::CARTESIAN &&
      autoLocalPhase!=LocalPointPhase::ESCAPE &&
      (!action.concurrentRc || autoPickup.rcTargetSettled(now)) &&
      !nucleo.checkpointActive() && autoCheckpointToken==0u) {
    const uint32_t settleMs = AutoPickupSetup::cartesianSettleMs(autoPickup.state());
    // Start only after the final hold/setpoint ACK is absorbed (guards above).
    // Identical delta/RC lease renewals must not restart the same action's wait.
    if (settleMs>0u && !autoPointSettling) {
      autoPointSettling = true;
      autoPointSettleStartedMs = now;
      Serial.print(F("[AUTO CART SETTLE] state="));
      Serial.print(AutoPickup::Sequence::stateName(autoPickup.state()));
      Serial.print(F(" wait_ms=")); Serial.println(settleMs);
    }
    if (settleMs==0u || now-autoPointSettleStartedMs>=settleMs) {
      const bool centerObserve=AutoPickupSetup::centerRecovery(autoPickup.state()) && !autoCenterPrecisionPending;
      const uint16_t checkpointFlags = centerObserve ? DELTA_CHECKPOINT_OBSERVE :
        AutoPickupSetup::checkpointRequestFlags(autoPickup.state());
      if (nucleo.sendStreamCheckpoint(activeStreamOption | checkpointFlags,false,false,false,centerObserve,
          AutoPickupSetup::finePickupCorrection(autoPickup.state()))) {
        autoCheckpointToken=action.token;
        Serial.print(F("[AUTO CHECKPOINT] state="));
        Serial.print(AutoPickup::Sequence::stateName(autoPickup.state()));
        Serial.print(F(" tolerance_raw="));
        if(centerObserve)Serial.println(F("CENTER OBSERVE REC13..200 max2; despues CP5 obligatorio"));
        else if (checkpointFlags == DELTA_CHECKPOINT_OBSERVE) Serial.println(F("OBSERVE + reintento selectivo final16..200 max2; llegada15, lecturas validas obligatorias"));
        else Serial.println(checkpointFlags == DELTA_CHECKPOINT_AUTO15_REC30 ? DELTA_AUTO15_TOLERANCE_RAW :
          (checkpointFlags == DELTA_CHECKPOINT_BOX15
          ? DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW
          : (checkpointFlags ? DELTA_CHECKPOINT_AUTO9_TOLERANCE_RAW :
                               DELTA_CHECKPOINT_PRECISION_RAW)));
      }
      return;
    }
    // Non-blocking hold: continue RC renewals below and delta holds in controller.
    // No Cartesian transition or gate opening until the subsequent physical PASS.
  }
  const uint32_t rcPeriod = (action.kind==AutoPickup::Kind::RC || action.concurrentRc) &&
      !nucleo.checkpointActive() ? AutoPickupSetup::rcRampPeriodMs(autoPickup.state()) : GRIP_RUNTIME_RENEW_MS;
  if (now-autoRcLastAckMs<rcPeriod || now-lastControlMs>=180u) return;
  // A lease renewal while a checkpoint is reading must be an identical RC hold.
  for (unsigned i=0u;i<3u;++i) autoRcPendingUs[i]=gripUs[i];
  if ((action.kind==AutoPickup::Kind::RC || action.concurrentRc) && !nucleo.checkpointActive()) {
    const uint16_t target[3]={action.rc.z,action.rc.x,action.rc.palo};
    for (unsigned i=0u;i<3u;++i) autoRcPendingUs[i]=static_cast<uint16_t>(
      static_cast<int>(gripUs[i])+constrain(static_cast<int>(target[i])-gripUs[i],
        -static_cast<int>(AutoPickupSetup::rcRampStepUs(autoPickup.state(),(uint8_t)i)),
         static_cast<int>(AutoPickupSetup::rcRampStepUs(autoPickup.state(),(uint8_t)i))));
  }
  if (nucleo.sendGripperGroup(DELTA_MSG_GRIP_GROUP_SET,autoRcPendingUs,true,true)) {
    autoRcPending=true; autoRcBeginPending=false; autoRcPendingToken=action.token;
  }
}

void serviceController() {
  const uint32_t now = millis();
  const bool approach5=operationMode==OperationMode::AUTO_PICKUP &&
    AutoPickupSetup::fastApproach(autoPickup.state()) &&
    !(nucleo.remoteFlags() & DELTA_FLAG_FINE_REQUIRED) && !autoPointHold &&
    autoPickup.action().kind==AutoPickup::Kind::CARTESIAN &&
    autoActionToken==autoPickup.action().token;
  if (lastControlMs == 0u) {
    lastControlMs = now;
    return;
  }
  const bool coarseTransport = operationMode==OperationMode::AUTO_PICKUP &&
      !AutoPickupSetup::positionMove(autoPickup.state()) &&
      AutoTransport::coarse(autoPickup.state(),
        magnitude(subtract(autoTargetPose, referencePose)),
        !(nucleo.remoteFlags() & DELTA_FLAG_FINE_REQUIRED) &&
        (!autoJointPtp.combined() || referencePose.v[2]+0.001f >=
          AutoPickupHeight::targetZ(referencePose.v[1])+AutoPickupSetup::APPROACH_CLEARANCE_MM) &&
        !autoPointHold && autoPickup.action().kind==AutoPickup::Kind::CARTESIAN &&
        autoActionToken==autoPickup.action().token,
        hypotf(referencePose.v[0]-AutoPickupSetup::PICKUP_X_MM,
               referencePose.v[1]-autoPickup.piece().robotYMm));
  const uint32_t dispatchPeriod = coarseTransport ? AutoTransport::DISPATCH_PERIOD_MS
                                                  : Config::CONTROL_PERIOD_MS;
  // READY after the dedicated RC ACK is a command handoff, not a Cartesian
  // integration step. Use that fresh lease immediately instead of spending
  // another 50/110 ms waiting for the motion-model cadence.
  const bool directAckReady=operationMode==OperationMode::AUTO_PICKUP &&
      autoDirectMove && (autoLocalPhase==LocalPointPhase::READY ||
                        autoLocalPhase==LocalPointPhase::RETRY_READY) &&
      autoDirectRcRenewalKnown && !autoRcPending && !nucleo.waitingResponse();
  if (now - lastControlMs < dispatchPeriod && !directAckReady) return;
  const uint32_t autoRcPeriod = (autoPickup.action().kind==AutoPickup::Kind::RC ||
      autoPickup.action().concurrentRc) &&
      !nucleo.checkpointActive() ? AutoPickupSetup::rcRampPeriodMs(autoPickup.state()) : GRIP_RUNTIME_RENEW_MS;
  if (autoRcActive && operationMode==OperationMode::AUTO_PICKUP &&
      now-autoRcLastAckMs>=autoRcPeriod && now-lastControlMs<180u)
    return; // bounded RC lease slot, same scheduling rule as MANUAL
  if (gripRuntime && gripPhase == GripPhase::ACTIVE &&
      now - gripLastAckMs >= GRIP_RUNTIME_RENEW_MS &&
      now - lastControlMs < 180u)
    return; // One RC renewal slot; do not starve its independent 300 ms watchdog.
  if (!simulationValid || !runEnabled || !gamepadFresh ||
      pendingSetpoint || stopRequested || !nucleo.ready() || nucleo.dryRun() ||
      nucleo.remoteFault() || !nucleo.streamActive() ||
      nucleo.waitingResponse()) return;
  // Do not advance the model while the receiver's coarse ACK guard is closed.
  // This also covers the fine -> coarse transition after any accepted hold.
  if (coarseTransport && !nucleo.coarseDispatchReady()) return;
  if (approach5 && !nucleo.approachDispatchReady()) return;
  // Un ACK tardio reduce la frecuencia efectiva: nunca acumular tiempo
  // para recuperarlo con un salto mayor en la siguiente trama.
  const float dt = static_cast<float>(coarseTransport ? AutoTransport::MODEL_PERIOD_MS
                                                     : Config::CONTROL_PERIOD_MS) / 1000.0f;

  const float jointStepLimit = approach5
    ? AutoPickupSetup::APPROACH_JOINT_STEP_DEG * Config::DEG_TO_RAD_VALUE
    : Config::MAX_JOINT_STEP_RAD *
      (coarseTransport ? static_cast<float>(AutoTransport::MODEL_PERIOD_MS) /
                         Config::CONTROL_PERIOD_MS : 1.0f);
  lastControlMs = now;
  const bool boundedPointState=
      AutoPickupSetup::positionMove(autoPickup.state()) ||
      (AutoPickupSetup::LOCAL_POINT_TRANSIT_ENABLED &&
       AutoLocalPoint::eligible(autoPickup.state()));
  if (operationMode==OperationMode::AUTO_PICKUP && boundedPointState && !autoPointHold &&
      autoActionToken==autoPickup.action().token &&
      serviceAutoLocalPoint()) return;
  if (operationMode==OperationMode::AUTO_PICKUP && AutoPickupSetup::LOCAL_POINT_TRANSIT_ENABLED &&
      autoLocalPhase==LocalPointPhase::ESCAPE && autoPointHold) {
    serviceAutoLocalPoint();return;
  }
  if (operationMode==OperationMode::AUTO_PICKUP &&
      (autoPointHold || autoPickup.action().kind!=AutoPickup::Kind::CARTESIAN ||
       autoActionToken!=autoPickup.action().token)) {
    commandedVelocity=Vec3{{0.0f,0.0f,0.0f}};
    sendAutoPickupHold();
    return;
  }

  // Estado y error al comienzo del intervalo k. La referencia todavia no se
  // adelanta: asi la realimentacion y el Jacobiano pertenecen al mismo instante.
  if (operationMode==OperationMode::AUTO_PICKUP && AutoJointPtp::eligible(autoPickup.state()) &&
      !AutoPickupSetup::LOCAL_POINT_TRANSIT_ENABLED) {
    serviceAutoJointPtp(coarseTransport,dt,jointStepLimit);
    return;
  }
  Vec3 estimatedPose{};
  if (!deltaForward(simulatedThetaRad, estimatedPose)) {
    controllerSafetyStop(F("FK sin solucion"));
    return;
  }
  const Vec3 positionError = subtract(referencePose, estimatedPose);

  const float speedLimit = automaticOperation()
    ? AutoSpeedProfile::cruiseLimit(autoPickup.state(),AutoPickupSetup::speedProfileConfig())
    : automaticBandMove ? fminf(selectedSpeedMmS(), Config::NORMAL_SPEED_MM_S)
                        : selectedSpeedMmS();
  float xCommand = 0.0f;
  float yCommand = 0.0f;
  if (operationMode == OperationMode::MANUAL &&
      manualMode == ManualMode::XYZ_MANUAL) {
    xCommand = -normalizedAxis(gamepad.ly, gamepadNeutral[1],
                               axisDriveActive[0]);
    yCommand = normalizedAxis(gamepad.lx, gamepadNeutral[0],
                              axisDriveActive[1]);
  } else {
    axisDriveActive[0] = false;
    axisDriveActive[1] = false;
  }
  float zCommand = operationMode == OperationMode::MANUAL
    ? -normalizedAxis(gamepad.ry, gamepadNeutral[3], axisDriveActive[2])
    : 0.0f;
  Vec3 sampledManualCommand{{xCommand, yCommand, zCommand}};
  const bool manualStickControl = operationMode == OperationMode::MANUAL &&
                                  !automaticBandMove;
  if (manualStickControl && now - lastGamepadMs > 150u) {
    sampledManualCommand = Vec3{{0.0f, 0.0f, 0.0f}};
    resetAxisHysteresis(); // Old stick data cannot keep demanding motion.
  }
  if (manualStickControl) {
    const float alpha = dt / (Config::STICK_FILTER_TAU_S + dt);
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
      if (!axisDriveActive[axis] ||
          fabsf(sampledManualCommand.v[axis]) < 0.0001f) {
        /* Liberar el stick no deja una cola de movimiento: DECEL hace el
           frenado suave desde commandedVelocity. */
        filteredManualCommand.v[axis] = 0.0f;
        heldManualCommand.v[axis] = 0.0f;
      } else {
        heldManualCommand.v[axis] = ManualAntiJitter::stableAxis(
          sampledManualCommand.v[axis], heldManualCommand.v[axis],
          Config::STICK_COMMAND_HYSTERESIS);
        filteredManualCommand.v[axis] += alpha *
          (heldManualCommand.v[axis] - filteredManualCommand.v[axis]);
      }
    }
  } else {
    filteredManualCommand = Vec3{{0.0f, 0.0f, 0.0f}};
    heldManualCommand = Vec3{{0.0f, 0.0f, 0.0f}};
  }
  Vec3 targetVelocity = scale(filteredManualCommand, speedLimit);
  if (automaticOperation()) {
    axisDriveActive[0] = axisDriveActive[1] = axisDriveActive[2] = false;
    const Vec3 remaining = subtract(autoTargetPose, referencePose);
    const float distance = magnitude(remaining);
    if (distance <= Config::AUTO_TARGET_TOLERANCE_MM &&
        magnitude(commandedVelocity) < 1.0f) {
      autoPointHold=true;
      commandedVelocity=Vec3{{0.0f,0.0f,0.0f}};
      sendAutoPickupHold();
      return; // physical checkpoint, never STREAM_END while holding a piece
    }
    const float approachLimit = AutoSpeedProfile::maximumSpeed(
      speedLimit,distance,AutoPickupSetup::speedProfileConfig(),Config::DECEL_MM_S2,dt);
    const float approachSpeed = AutoCartesianArrival::nextSpeed(
      distance, magnitude(commandedVelocity), approachLimit,
      Config::ACCEL_MM_S2, Config::DECEL_MM_S2, dt,
      Config::AUTO_TARGET_TOLERANCE_MM);
    if (!isfinite(approachSpeed)) {
      controllerSafetyStop(F("AUTO perfil de llegada invalido"));
      return;
    }
    targetVelocity = scale(remaining, approachSpeed /
                           fmaxf(distance, 0.001f));
  } else if (automaticBandMove) {
    const Vec3 bandTarget{{Config::BAND_SAFE_X_MM, Config::BAND_SAFE_Y_MM,
                           Config::BAND_SAFE_Z_MM}};
    const Vec3 remaining = subtract(bandTarget, referencePose);
    const float distance = magnitude(remaining);
    if (distance <= Config::BAND_TARGET_TOLERANCE_MM &&
        magnitude(commandedVelocity) < 1.0f) {
      Serial.println(F("[BAND SAFE] Destino cartesiano alcanzado; cerrando STREAM para VERIFY."));
      reopenMenuAfterStop = true;
      pauseSimulation(F("plano seguro alcanzado"));
      return;
    }
    const float brakingSpeed = sqrtf(2.0f * Config::DECEL_MM_S2 * distance);
    targetVelocity = scale(remaining, fminf(speedLimit, brakingSpeed) /
                           fmaxf(distance, 0.001f));
  }
  targetVelocity = limitMagnitude(targetVelocity, speedLimit);
  targetVelocity = assistManualWorkspaceVelocity(targetVelocity, speedLimit, dt);
  if (automaticOperation()) {
    // AUTO already ramps against its discrete stopping envelope. A second
    // vector ramp would undo the endpoint cap and retain the old direction.
    commandedVelocity = targetVelocity;
  } else {
    const bool braking = magnitude(targetVelocity) < magnitude(commandedVelocity) ||
                         dot(targetVelocity, commandedVelocity) < 0.0f;
    const float rate = braking ? Config::DECEL_MM_S2 : Config::ACCEL_MM_S2;
    const Vec3 velocityDelta = limitMagnitude(
      subtract(targetVelocity, commandedVelocity), rate * dt);
    commandedVelocity = limitMagnitude(
      add(commandedVelocity, velocityDelta), speedLimit);
  }

  // Construir r[k+1] sin publicarla hasta que todos los calculos sean validos.
  // En un limite, la velocidad efectiva de referencia es cero en ese eje.
  const Vec3 requestedReference=add(referencePose,scale(commandedVelocity,dt));
  if (operationMode==OperationMode::AUTO_PICKUP && !workspacePointValid(requestedReference)) {
    controllerSafetyStop(F("AUTO no permite proyectar/desviar trayectoria"));
    return;
  }
  Vec3 nextReference = operationMode==OperationMode::AUTO_PICKUP
    ? requestedReference : fitWorkspaceReference(requestedReference);
  const Vec3 referenceVelocity = scale(
    subtract(nextReference, referencePose), 1.0f / dt);
  Vec3 correctiveVelocity = add(referenceVelocity,
    scale(positionError, Config::POSITION_KP_S));
  correctiveVelocity = limitMagnitude(correctiveVelocity, speedLimit);
  Vec3 cartesianStep = scale(correctiveVelocity, dt);

  Mat3 jacobian{};
  Mat3 inverseJacobian{};
  float candidateDeterminant = NAN;
  if (!numericalJacobian(simulatedThetaRad, jacobian) ||
      !invert(jacobian, inverseJacobian, candidateDeterminant)) {
    controllerSafetyStop(F("Jacobiano no invertible"));
    return;
  }
  const float candidateConditionF =
    frobeniusNorm(jacobian) * frobeniusNorm(inverseJacobian);
  if (!isfinite(candidateConditionF) ||
      candidateConditionF > Config::MAX_CONDITION_F) {
    controllerSafetyStop(F("condicion Jacobiana fuera de limite"));
    return;
  }

  // Escalar tambien la referencia cuando limita la velocidad articular.
  // Recortar solo theta dejaba una referencia inalcanzable que acumulaba
  // residuo y disparaba FAULT, especialmente a 50/60 mm/s.
  Vec3 unrestrictedStep{};
  if (!dlsStep(jacobian, cartesianStep, unrestrictedStep)) {
    controllerSafetyStop(F("DLS sin solucion"));
    return;
  }
  const float largestJointStep = fmaxf(fabsf(unrestrictedStep.v[0]),
    fmaxf(fabsf(unrestrictedStep.v[1]), fabsf(unrestrictedStep.v[2])));
  if (largestJointStep > jointStepLimit) {
    const float fraction = jointStepLimit / largestJointStep;
    cartesianStep = scale(cartesianStep, fraction);
    nextReference = add(estimatedPose, cartesianStep);
    commandedVelocity = scale(commandedVelocity, fraction);
    if (!workspacePointValid(nextReference)) {
      holdAtWorkspaceLimit();
      return;
    }
  }

  Vec3 inverseJointStep = matVec(inverseJacobian, cartesianStep);
  Vec3 transposeJointStep{};
  Vec3 dlsJointStep{};
  if (!transposeStep(jacobian, cartesianStep, transposeJointStep) ||
      !dlsStep(jacobian, cartesianStep, dlsJointStep)) {
    controllerSafetyStop(F("metodo transpuesta/DLS sin solucion"));
    return;
  }
  // El limite es por servo, no la norma del vector de los tres servos.
  for (uint8_t i = 0u; i < 3u; ++i) {
    if (fabsf(dlsJointStep.v[i]) > jointStepLimit + 0.000001f) {
      controllerSafetyStop(F("paso DLS articular fuera de limite"));
      return;
    }
  }

  const float candidateInverseResidual = predictedResidual(
    simulatedThetaRad, inverseJointStep, nextReference);
  const float candidateTransposeResidual = predictedResidual(
    simulatedThetaRad, transposeJointStep, nextReference);
  const float candidateDlsResidual = predictedResidual(
    simulatedThetaRad, dlsJointStep, nextReference);
  Vec3 exactTheta{};
  Vec3 exactPose{};
  if (!deltaInverse(nextReference, exactTheta) || !deltaForward(exactTheta, exactPose)) {
    controllerSafetyStop(F("IK exacta sin solucion en referencia"));
    return;
  }
  const float candidateExactResidual = magnitude(subtract(nextReference, exactPose));
  if (!isfinite(candidateInverseResidual) ||
      !isfinite(candidateTransposeResidual) ||
      !isfinite(candidateDlsResidual) || !isfinite(candidateExactResidual) ||
      candidateDlsResidual > Config::MAX_NUMERIC_RESIDUAL_MM ||
      candidateExactResidual > Config::MAX_NUMERIC_RESIDUAL_MM) {
    controllerSafetyStop(F("residuo numerico fuera de limite"));
    return;
  }

  const Vec3 nextTheta = add(simulatedThetaRad, dlsJointStep);
  Vec3 resultingPose{};
  if (!deltaForward(nextTheta, resultingPose)) {
    controllerSafetyStop(F("FK posterior sin solucion"));
    return;
  }
  if (!thetaRawSafeForPose(nextTheta, resultingPose)) {
    holdAtWorkspaceLimit();
    return;
  }
  if (!workspacePointValid(resultingPose)) {
    holdAtWorkspaceLimit();
    return;
  }
  const float candidateError = magnitude(subtract(nextReference, resultingPose));
  if (!isfinite(candidateError) ||
      candidateError > Config::MAX_NUMERIC_RESIDUAL_MM) {
    controllerSafetyStop(F("error FK/DLS fuera de limite"));
    return;
  }
  uint16_t raw[3] = {
    static_cast<uint16_t>(thetaToRaw(nextTheta.v[0], 0u)),
    static_cast<uint16_t>(thetaToRaw(nextTheta.v[1], 1u)),
    static_cast<uint16_t>(thetaToRaw(nextTheta.v[2], 2u))
  };
  const uint16_t *remote = nucleo.remoteRaw();
  const bool manualRawHysteresis = manualStickControl &&
                                  manualMode == ManualMode::XYZ_MANUAL;
  if (manualRawHysteresis) {
    for (uint8_t index = 0u; index < 3u; ++index) {
      const int candidateRaw = ManualAntiJitter::stableRaw(
        thetaToRawValue(nextTheta.v[index], index), remote[index],
        Config::STREAM_RAW_HYSTERESIS_RAW);
      if (candidateRaw < selectedRawMin() || candidateRaw > selectedRawMax()) {
        holdAtWorkspaceLimit();
        return;
      }
      raw[index] = static_cast<uint16_t>(candidateRaw);
    }
  }
  const int rawMinimum = min(raw[0], min(raw[1], raw[2]));
  const int rawMaximum = max(raw[0], max(raw[1], raw[2]));
  if (operationMode == OperationMode::MANUAL &&
      manualMode == ManualMode::Z_ONLY &&
      rawMaximum - rawMinimum > Config::STREAM_Z_SPREAD_MAX) {
    controllerSafetyStop(F("setpoint dejo de ser Z simetrico"));
    return;
  }
  for (uint8_t index = 0u; index < 3u; ++index) {
    if (abs(static_cast<int>(raw[index]) - static_cast<int>(remote[index])) >
        (approach5 ? DELTA_APPROACH_STEP_RAW :
         coarseTransport ? AutoTransport::MAX_STEP_RAW : Config::STREAM_RAW_MAX_STEP)) {
      controllerSafetyStop(F("salto STREAM supera limite del tramo"));
      return;
    }
  }
  if (!nucleo.sendStreamSetpoint(raw,
        coarseTransport ? AutoTransport::MOVE_TIME_MS : Config::STREAM_MOVE_TIME_MS,
        activeStreamOption | (coarseTransport ? DELTA_STREAM_SETPOINT_COARSE : 0u),approach5)) return;

  // Commit diferido: el modelo local solo avanza cuando llega ACK de 10C.
  pendingThetaRad = nextTheta;
  pendingReferencePose = nextReference;
  pendingConditionF = candidateConditionF;
  pendingDeterminant = candidateDeterminant;
  pendingErrorMm = candidateError;
  pendingInverseResidualMm = candidateInverseResidual;
  pendingTransposeResidualMm = candidateTransposeResidual;
  pendingDlsResidualMm = candidateDlsResidual;
  pendingExactResidualMm = candidateExactResidual;
  pendingSetpoint = true;
  ++streamCommandsSent;
}

void serviceNucleo() {
  nucleo.service();
  if (!runEnabled || stopRequested || !nucleo.ready() || nucleo.remoteFault()) {
    autoJointPtp.cancel();autoPtpToken=0u;autoPtpSegmentPending=false;
    nucleo.cancelPoint();autoLocalPhase=LocalPointPhase::IDLE;autoLocalToken=0u;autoDirectMove=false;
  }
  DeltaResultCode pointResult;
  if (nucleo.consumePoint(pointResult)) {
    if (pointResult!=DELTA_RESULT_OK || !runEnabled || stopRequested || !autoAuthorized ||
        operationMode!=OperationMode::AUTO_PICKUP || autoLocalPhase!=LocalPointPhase::RUNNING ||
        autoLocalToken!=autoPickup.action().token) {
      pauseSimulation(F("AUTO movimiento directo/local interrumpido/rechazado"));
    } else {
      Vec3 q{};float xyz[3];autoRawToTheta(nucleo.remoteRaw(),q.v);
      if (!(autoDirectMove ? autoPtpWorkspacePoint(q.v,xyz) : autoLocalCurvePoint(q.v,xyz)))
        pauseSimulation(F("AUTO punto final FK invalido"));
      else {
        simulatedThetaRad=q;referencePose=Vec3{{xyz[0],xyz[1],xyz[2]}};
        commandedVelocity=Vec3{};autoPointHold=true;autoPointSettling=false;
        autoLocalPhase=LocalPointPhase::FINISHED;lastControlMs=millis();
        Serial.println(autoDirectMove
          ? F("[AUTO DIRECT MOVE MODEL END] tiempo terminado; ahora lectura final real obligatoria, ACK no es PASS.")
          : F("[AUTO LOCAL POINT MODEL END] ahora se requiere checkpoint fisico; ACK no es PASS."));
        autoDirectMove=false;
      }
    }
  }

  // Session generation catches a reconnect even if READY changed twice in one
  // service() call. Never retain permission or assume RC references survived.
  if (gripSessionGeneration != nucleo.sessionGeneration()) {
    if (gripSessionGeneration != 0u) gripReferenceUncertain = true;
    gripSessionGeneration = nucleo.sessionGeneration();
    gripStateQueryNeeded = true;
    confirmation = Confirmation::NONE;
    armAfterAckPending = false;
  }

  if (previousNucleoReady && !nucleo.ready()) {
    autoRcActive = autoRcPending = autoRcBeginPending = autoCapabilityPending = autoCapabilityPassed = false;
    autoPiecePending = autoEncoderAssociated = autoPointHold = false;
    autoPointSettling = false;
    if (autoPickup.active()) autoPickup.abort();
    gripReferenceUncertain = true; // Also when PWM was already OFF.
    gripStateQueryNeeded = true;
    confirmation = Confirmation::NONE;
    armAfterAckPending = false;
    if (gripPhase != GripPhase::OFF) {
      gripEndSent = false;
      stopGripper();
    }
    homeRequestPending = false;
    homeRequestAcknowledged = false;
    automaticBandMove = false;
    autoAuthorized = false;
    autoTargetActive = false;
    streamEndVerificationPending = false;
    simulationValid = false;
    runEnabled = false;
    pendingSetpoint = false;
    stopRequested = false;
    startPhase = StartPhase::IDLE;
    commandedVelocity = Vec3{{0.0f, 0.0f, 0.0f}};
    Serial.println(F("[SAFETY] Sesion STREAM perdida; HOME local invalidado."));
  }
  if (!previousNucleoReady && nucleo.ready()) {
    Serial.println(nucleo.dryRun()
      ? F("[SAFETY] Firmware Nucleo DRY_RUN: no habra movimiento fisico.")
      : F("[NUCLEO] Firmware 10C fisico detectado; sistema permanece desarmado."));
  }
  if (nucleo.ready() && nucleo.remoteFault() && !nucleo.recoveryActive() &&
      (runEnabled || nucleo.armed() || startPhase != StartPhase::IDLE ||
       !menuOpen)) {
    autoRcActive = autoRcPending = autoRcBeginPending = autoCapabilityPending = autoCapabilityPassed = false;
    autoPiecePending = autoEncoderAssociated = autoPointHold = false;
    autoPointSettling = false;
    if (autoPickup.active()) autoPickup.abort();
    homeRequestPending = false;
    homeRequestAcknowledged = false;
    automaticBandMove = false;
    autoAuthorized = false;
    autoTargetActive = false;
    runEnabled = false;
    pendingSetpoint = false;
    stopRequested = false;
    startPhase = StartPhase::IDLE;
    commandedVelocity = Vec3{{0.0f, 0.0f, 0.0f}};
    menuOpen = true;
    Serial.println(F("[SAFETY] FAULT Nucleo; menu13 RECUPERAR FAULT para posicion estable. No reanuda AUTO."));
    printMenu();
  }
  previousNucleoReady = nucleo.ready();

  DeltaResultCode recoveryResult;
  if (nucleo.consumeFaultRecovery(recoveryResult)) {
    simulationValid = runEnabled = autoAuthorized = autoTargetActive = pendingSetpoint = false;
    menuOpen = true;
    confirmation = Confirmation::NONE;
    gripReferenceUncertain = gripStateQueryNeeded = true;
    Serial.println(recoveryResult == DELTA_RESULT_OK
      ? F("[FAULT RECOVERY OK] Sin reinicio; desarmado/PWM OFF. HOME y ARM se confirman aparte, tras revision fisica.")
      : F("[FAULT RECOVERY FAILED] No se autoriza movimiento; revisa HTS/alimentacion/lecturas. FAULT no recuperable requiere revision/reinicio."));
    oledDirty = true;
  }

  uint8_t completedType = 0u;
  DeltaResultCode result = DELTA_RESULT_OK;
  if (nucleo.consumeCompletion(completedType, result)) {
    if ((completedType==DELTA_MSG_GRIP_GROUP_BEGIN ||
         completedType==DELTA_MSG_GRIP_GROUP_SET) && autoRcPending) {
      const bool wasBegin = autoRcBeginPending;
      autoRcPending=autoRcBeginPending=false;
      if (!autoAuthorized || stopRequested || operationMode!=OperationMode::AUTO_PICKUP) {
        // The first stop cause is authoritative. Absorb a late completion,
        // without another pause, adopting its pose, or restarting the sequence.
        Serial.print(F("[AUTO RC COMPLETION IGNORED] cancelled result="));
        Serial.println(static_cast<unsigned>(result));
      } else if (result!=DELTA_RESULT_OK ||
          nucleo.sessionGeneration()!=autoStartGeneration) {
        reopenMenuAfterStop=true;
        pauseSimulation(F("AUTO consigna RC rechazada/cancelada"));
      } else {
        const uint16_t previousGateUs = gripUs[2];
        for (unsigned i=0u;i<3u;++i) gripUs[i]=autoRcPendingUs[i];
        autoRcLastAckMs=millis();
        if (wasBegin) {
          autoRcActive=true;
          autoRuntimeStartMs=millis();
          if (!autoPickup.begin(AutoPickupSetup::configuration(),millis())) {
            reopenMenuAfterStop=true;
            pauseSimulation(F("AUTO configuracion invalida"));
          }
        } else if ((autoPickup.action().kind==AutoPickup::Kind::RC ||
                    autoPickup.action().concurrentRc) &&
                   autoRcPendingToken==autoPickup.action().token) {
          const AutoPickup::RcPose &target=autoPickup.action().rc;
          if (autoPickup.state()==AutoPickup::State::CLOSE_GATE &&
              gripUs[2]!=previousGateUs && !autoGateFirstAckLogged) {
            autoGateFirstAckLogged = true;
            calibrationGateEvent('F', millis());
            Serial.print(F("[AUTO GATE FIRST COMMAND ACK] from_trigger_ms="));
            Serial.print(millis()-autoGateTriggerMs);
            Serial.print(F(" palo_us=")); Serial.println(gripUs[2]);
          }
          if (gripUs[0]==target.z && gripUs[1]==target.x && gripUs[2]==target.palo) {
            autoPickup.markRcTargetAck(autoRcPendingToken,millis());
            if (autoPickup.state()==AutoPickup::State::CLOSE_GATE && !autoGateFinalAckLogged) {
              autoGateFinalAckLogged = true;
              calibrationGateEvent('A', millis());
              Serial.print(F("[AUTO GATE TARGET COMMAND ACK] from_trigger_ms="));
              Serial.print(millis()-autoGateTriggerMs);
              Serial.println(F("; commands only, no physical gate feedback"));
            }
          }
        }
      }
      oledDirty=true;
    } else if (completedType==DELTA_MSG_STREAM_CHECKPOINT || completedType==DELTA_MSG_ENDPOINT_REC200_PROBE ||
               completedType==DELTA_MSG_ENDPOINT_STEP_PROBE || completedType==DELTA_MSG_CENTER_RECOVERY_PROBE ||
               completedType==DELTA_MSG_CENTER_OBSERVE || completedType==DELTA_MSG_PICKUP_FINE20_PROBE ||
               completedType==DELTA_MSG_PICKUP_FINE20_CHECKPOINT || completedType==DELTA_MSG_DIRECT_REPEAT5_PROBE ||
               completedType==DELTA_MSG_APPROACH_STEPS_PROBE) {
      if (result!=DELTA_RESULT_OK) {
        reopenMenuAfterStop=true;
        pauseSimulation(F("AUTO checkpoint no admitido"));
      }
    } else if (completedType == DELTA_MSG_GRIP_STATE_REQUEST) {
      autoSnapshotReceived=startPhase==StartPhase::WAIT_AUTO_REF &&
                           result==DELTA_RESULT_OK && gripStateStopped();
      gripArmStateReceived = startPhase == StartPhase::WAIT_PALA_REF &&
                             result == DELTA_RESULT_OK && gripStateStopped();
      if (result != DELTA_RESULT_OK || !gripStateStopped()) {
        gripReferenceUncertain = true;
        gripStateQueryNeeded = true;
        Serial.println(F("[PALA REF UNAVAILABLE] Requiere 10C v2.5, estado nuevo y PWM OFF; no habilitar RC."));
      } else {
        gripStateQueryNeeded = false;
        const uint16_t *remoteUs = nucleo.gripperStateUs();
        const bool matches = memcmp(gripUs, remoteUs, sizeof(gripUs)) == 0;
        if (!matches) gripReferenceUncertain = true;
        // Passive reading never clears uncertainty. Menu12 adopts without PWM;
        // explicit ARM confirmation instead permits synchronization + RC motion.
        Serial.print(F("[PALA REF READ] Nucleo us_Z_X_PALO="));
        for (uint8_t i=0u;i<3u;++i) { if(i) Serial.print(','); Serial.print(remoteUs[i]); }
        Serial.println(gripReferenceUncertain
          ? F(" PWM OFF; ARM sincroniza/prepara pala; menu12 opcional para calibracion. Sin feedback RC.")
          : F(" PWM OFF; consignas coinciden. Sin feedback RC."));
      }
      oledDirty = true;
    } else if (completedType == DELTA_MSG_GRIP_BEGIN ||
        completedType == DELTA_MSG_GRIP_SET ||
        completedType == DELTA_MSG_GRIP_GROUP_BEGIN ||
        completedType == DELTA_MSG_GRIP_GROUP_SET ||
        completedType == DELTA_MSG_GRIP_END) {
      if (result != DELTA_RESULT_OK) {
        Serial.print(F("[GRIP REJECT] result=")); Serial.println((unsigned)result);
        gripReferenceUncertain = true;
        gripStateQueryNeeded = true;
        if (preparingManualPala())
          abortManualPalaPreparation(F("NACK de preparacion RC."));
        if (gripRuntime) {
          pauseSimulation(F("PALA rechazada durante MANUAL: parar DELTA"));
        }
        stopGripper();
        if (completedType == DELTA_MSG_GRIP_END) {
          gripPhase = GripPhase::OFF;
          gripGroupMode = false;
          gripRuntime = false;
          gripEndSent = false;
          gripReferenceUncertain = true;
          Serial.println(F("[GRIP BLOCKED] Sin confirmacion de cierre. Quitar V_GRIP; revisar version 10C. Watchdog no se renueva."));
          openMenu();
        }
      } else if (completedType == DELTA_MSG_GRIP_END) {
        const bool preparationEnded = startPhase == StartPhase::WAIT_PALA_END &&
                                      !stopRequested;
        gripPhase = GripPhase::OFF;
        gripGroupMode = false;
        gripRuntime = false;
        gripEndSent = false;
        gripStateQueryNeeded = !preparationEnded;
        Serial.println(F("[GRIP OFF] PWM retirado. No equivale a cortar V_GRIP ni confirma sujecion."));
        if (preparationEnded) {
          startPhase = StartPhase::WAIT_PALA_ARM; // recheck permits before ARM
          oledDirty = true;
        } else openMenu();
      } else {
        if(completedType==DELTA_MSG_GRIP_GROUP_SET ||
           completedType==DELTA_MSG_GRIP_GROUP_BEGIN) {
          for(unsigned i=0;i<3;++i) gripUs[i]=gripPendingGroupUs[i];
        } else gripUs[gripChannel - 1u] = gripPendingUs;
        gripLastAckMs = millis();
        if ((completedType == DELTA_MSG_GRIP_BEGIN ||
             completedType == DELTA_MSG_GRIP_GROUP_BEGIN) &&
            gripPhase == GripPhase::BEGIN_PENDING)
          gripPhase = GripPhase::ACTIVE;
        if (completedType == DELTA_MSG_GRIP_GROUP_BEGIN && gripRuntime &&
            gripPhase == GripPhase::ACTIVE && !stopRequested) {
          runEnabled = true;
          lastControlMs = millis();
          Serial.println(F("[PALA READY] JOG: sostener cruz izq/der Z, L1/R1 X, L2/R2 palo. Soltar detiene avance. X/CUAD valores, TRI pantalla."));
        }
      }
      oledDirty = true;
    } else if (completedType == DELTA_MSG_STREAM_SETPOINT || completedType==DELTA_MSG_APPROACH_SETPOINT) {
      if (result == DELTA_RESULT_OK && pendingSetpoint) {
        if (autoPtpSegmentPending && runEnabled && !stopRequested && autoAuthorized &&
            operationMode==OperationMode::AUTO_PICKUP &&
            autoPtpToken==autoPickup.action().token &&
            AutoJointPtp::eligible(autoPickup.state())) {
          if (!autoJointPtp.commit(autoPtpPendingElapsedMs)) {
            controllerSafetyStop(F("AUTO PTP ACK fuera de plan"));
          }
        }
        autoPtpSegmentPending=false;
        // AUTO cadence and RC lease arbitration start at ACK receipt, not at
        // pre-TX calculation time. Fixed integration dt/step limits remain.
        if (operationMode == OperationMode::AUTO_PICKUP) lastControlMs = millis();
        simulatedThetaRad = pendingThetaRad;
        referencePose = pendingReferencePose;
        lastConditionF = pendingConditionF;
        lastDeterminant = pendingDeterminant;
        lastErrorMm = pendingErrorMm;
        lastInverseResidualMm = pendingInverseResidualMm;
        lastTransposeResidualMm = pendingTransposeResidualMm;
        lastDlsResidualMm = pendingDlsResidualMm;
        lastExactResidualMm = pendingExactResidualMm;
        ++controllerCycles;
      } else {
        ++streamCommandsRejected;
        reopenMenuAfterStop = true;
        pauseSimulation(F("NACK de setpoint STREAM"));
      }
      pendingSetpoint = false;
    } else if (completedType == DELTA_MSG_HOME) {
      homeRequestAcknowledged = result == DELTA_RESULT_OK;
      if (!homeRequestAcknowledged) {
        homeRequestPending = false;
        Serial.println(F("[HOME REJECTED] Modelo permanece invalido."));
      }
    } else if (completedType == DELTA_MSG_ARM) {
      if (result == DELTA_RESULT_OK &&
          startPhase == StartPhase::WAIT_ARM_ACK && !stopRequested) {
        Serial.println(F("[ARM ACK] Iniciando STREAM automaticamente."));
        if (!requestStreamBegin()) {
          startPhase = StartPhase::IDLE;
          stopRequested = true;
          Serial.println(F("[STREAM CHAIN ABORTED] ARM sera cancelado con DISARM."));
        }
      } else {
        startPhase = StartPhase::IDLE;
        if (result != DELTA_RESULT_OK) {
          Serial.print(F("[ARM REJECTED] result="));
          Serial.println(static_cast<unsigned>(result));
        } else {
          stopRequested = true;
          Serial.println(F("[ARM CANCELLED] Se desarmara sin iniciar STREAM."));
        }
      }
    } else if (completedType == DELTA_MSG_STREAM_BEGIN) {
      if (result == DELTA_RESULT_OK && nucleo.streamActive() &&
          startPhase == StartPhase::WAIT_STREAM_ACK && !stopRequested) {
        streamEndVerificationPending = false;
        const bool startPalaRuntime = operationMode == OperationMode::MANUAL &&
                                      !automaticBandMove;
        runEnabled = !startPalaRuntime;
        if (operationMode==OperationMode::AUTO_PICKUP) {
          autoCapabilityPassed=false;
          autoCapabilityPending=nucleo.sendStreamCheckpoint(activeStreamOption | DELTA_AUTO_STOP_POLICY |
            (AutoPickupSetup::ENDPOINT_RETRY_ENABLED ? DELTA_ENDPOINT_RETRY_CAPABILITY :
             AutoPickupSetup::TRANSIT_FINAL_READ_ENABLED ? DELTA_DIRECT_FINAL_READ_CAPABILITY :
              DELTA_CHECKPOINT_OBSERVE) |
            (AutoPickupSetup::LOCAL_POINT_TRANSIT_ENABLED ? DELTA_LOCAL_POINT_CAPABILITY : 0u) |
            (AutoPickupSetup::POSITION_MOVES_ALL_STAGES || AutoPickupSetup::TWO_DIRECT_LOADED_MOVES_ENABLED
              ? (DELTA_DIRECT_ENDPOINT_CAPABILITY |
                 DELTA_DIRECT_TRANSACTION_CAPABILITY | DELTA_DIRECT_VALIDATED_PATH_CAPABILITY |
                 (AutoPickupSetup::DIRECT_PROFILE250_ENABLED ||
                  AutoPickupSetup::POST_PICKUP_LIFT_DELAYED_PROBE_ENABLED ||
                  AutoPickupSetup::POST_PICKUP_TRANSFER_DELAYED_PROBE_ENABLED ||
                  AutoPickupSetup::POST_DEPOSIT_RETURN_DELAYED_PROBE_ENABLED
                   ? DELTA_DIRECT_LIFT_DELAY_CAPABILITY : 0u) |
                 (AutoPickupSetup::DIRECT_PROFILE250_ENABLED
                   ? DELTA_DIRECT_PROFILE250_CAPABILITY |
                     (AutoPickupSetup::DIRECT_COORDINATED_RETRY_ENABLED
                       ? DELTA_DIRECT_RETRY_CAPABILITY : 0u) : 0u)) : 0u),
            AutoPickupSetup::ENDPOINT_RETRY_ENABLED,AutoPickupSetup::ENDPOINT_RETRY_ENABLED,
            AutoPickupSetup::ENDPOINT_RETRY_ENABLED,false,true,true,
            AutoPickupSetup::PICKUP_APPROACH_STEP5_ENABLED);
          if (!autoCapabilityPending) {
            runEnabled=false; stopRequested=true;
            Serial.println(F("[AUTO CAPABILITY FAILED] PWM sigue OFF; parar STREAM."));
          } else Serial.println(F("[AUTO CAPABILITY] Verificando HOME con checkpoint; PWM sigue OFF."));
        } else if (startPalaRuntime) {
          gripRuntime = true; gripGroupMode = true; gripChannel = 1u;
          gripEndSent = false;
          for (uint8_t i=0;i<3u;++i)
            gripTargetUs[i] = gripPendingGroupUs[i] = gripUs[i];
          gripPhase = GripPhase::BEGIN_PENDING;
          if (!nucleo.sendGripperGroup(DELTA_MSG_GRIP_GROUP_BEGIN, gripUs, true)) {
            stopGripper(); stopRequested = true;
            Serial.println(F("[PALA INIT FAILED] Parar STREAM, no movimiento DELTA."));
          } else Serial.println(F("[PALA INIT] Esperando ACK de las 3 salidas; DELTA espera."));
        }
        menuOpen = false;
        reopenMenuAfterStop = false;
        startPhase = StartPhase::IDLE;
        if (gripPhase != GripPhase::STOPPING &&
            (operationMode!=OperationMode::AUTO_PICKUP || autoCapabilityPending))
          stopRequested = false;
        lastControlMs = millis();
        resetAxisHysteresis();
        Serial.print(F("[STREAM RUN] operation=")); Serial.print(operationName());
        Serial.print(F(" mode=")); Serial.print(modeName());
        Serial.print(F(" speed=")); Serial.print(speedName());
        Serial.print(F(" range=")); Serial.print(rangeName());
        Serial.println(F("; Jacobiano de velocidad y setpoints interpolados a 20 Hz."));
      } else {
        startPhase = StartPhase::IDLE;
        runEnabled = false;
        stopRequested = nucleo.streamActive() || nucleo.armed();
        Serial.print(F("[STREAM BEGIN CANCELLED] result="));
        Serial.println(static_cast<unsigned>(result));
      }
    } else if (completedType == DELTA_MSG_STREAM_END) {
      runEnabled = false;
      stopRequested = false;
      pendingSetpoint = false;
      commandedVelocity = Vec3{{0.0f, 0.0f, 0.0f}};
      if (result == DELTA_RESULT_OK) {
        streamEndVerificationPending = true;
      Serial.println(F("[STREAM STOP ACK] Destino congelado; 10C espera, mide y puede corregir hasta 4 rondas."));
      } else {
        streamEndVerificationPending = false;
        Serial.println(F("[STREAM STOP ERROR] STREAM_END rechazado."));
        if (reopenMenuAfterStop) openMenu(true);
      }
    } else if (completedType == DELTA_MSG_DISARM) {
      if (homeRequestPending) simulationValid = false;
      homeRequestPending = false;
      homeRequestAcknowledged = false;
      runEnabled = false;
      stopRequested = false;
      pendingSetpoint = false;
      streamEndVerificationPending = false;
      commandedVelocity = Vec3{{0.0f, 0.0f, 0.0f}};
      Serial.println(F("[STREAM STOPPED] DISARM confirmado; destino no verificado."));
      if (reopenMenuAfterStop) {
        reopenMenuAfterStop = false;
        openMenu(true);
      }
    } else if (result != DELTA_RESULT_OK) {
      Serial.print(F("[COMMAND REJECTED] type="));
      Serial.print(completedType);
      Serial.print(F(" result="));
      Serial.println(static_cast<unsigned>(result));
    }
  }

  if (homeRequestPending && homeRequestAcknowledged && !stopRequested &&
      nucleo.ready() && nucleo.homeValid() && !nucleo.busy() &&
      !nucleo.remoteFault()) {
    homeRequestPending = false;
    homeRequestAcknowledged = false;
    resetSimulation();
    Serial.print(F("[HOME SYNC] Feedback valido; modelo reiniciado a 0,0,"));
    Serial.println(Config::HOME_Z_MM, 1);
  }

  if (streamEndVerificationPending && nucleo.ready()) {
    if (nucleo.remoteFault()) {
      streamEndVerificationPending = false;
      autoAuthorized = false;
      Serial.println(F("[STREAM RESULT] VERIFY FAULT; reinicia Nucleo antes de continuar."));
      if (reopenMenuAfterStop) {
        reopenMenuAfterStop = false;
        openMenu(true);
      }
    } else if (!nucleo.busy()) {
      streamEndVerificationPending = false;
      Serial.println(operationMode==OperationMode::AUTO_PICKUP
        ? autoPickupStoppedResultMessage()
        : F("[STREAM RESULT] VERIFY PASS; pose conservada. Puedes ARMAR E INICIAR otra vez sin HOME."));
      if (reopenMenuAfterStop) {
        reopenMenuAfterStop = false;
        openMenu(true);
      }
    }
  }

  if (stopRequested && nucleo.ready() && !nucleo.waitingResponse()) {
    if (nucleo.streamActive()) {
      if (nucleo.sendStreamEnd(activeStreamOption | pendingAutoStopFlags))
        Serial.println(F("[STREAM STOP] STREAM_END enviado."));
    } else if (nucleo.armed() || homeRequestPending || nucleo.recoveryActive()) {
      if (nucleo.sendDisarm()) Serial.println(F("[STREAM STOP] DISARM enviado."));
    } else {
      stopRequested = false;
    }
  }
}

void printTelemetry() {
  Vec3 estimatedPose{};
  if (!simulationValid || !deltaForward(simulatedThetaRad, estimatedPose)) return;
  Serial.print(F("[JAC] run=")); Serial.print(runEnabled ? 1 : 0);
  Serial.print(F(" ref="));
  Serial.print(referencePose.v[0], 2); Serial.print(',');
  Serial.print(referencePose.v[1], 2); Serial.print(',');
  Serial.print(referencePose.v[2], 2);
  Serial.print(F(" est="));
  Serial.print(estimatedPose.v[0], 2); Serial.print(',');
  Serial.print(estimatedPose.v[1], 2); Serial.print(',');
  Serial.print(estimatedPose.v[2], 2);
  Serial.print(F(" err=")); Serial.print(lastErrorMm, 3);
  Serial.print(F(" vel=")); Serial.print(magnitude(commandedVelocity), 2);
  Serial.print(F(" condF=")); Serial.print(lastConditionF, 2);
  Serial.print(F(" residual inv/T/DLS/exact="));
  Serial.print(lastInverseResidualMm, 3); Serial.print('/');
  Serial.print(lastTransposeResidualMm, 3); Serial.print('/');
  Serial.print(lastDlsResidualMm, 3); Serial.print('/');
  Serial.print(lastExactResidualMm, 3);
  Serial.print(F(" raw="));
  Serial.print(thetaToRaw(simulatedThetaRad.v[0], 0u)); Serial.print(',');
  Serial.print(thetaToRaw(simulatedThetaRad.v[1], 1u)); Serial.print(',');
  Serial.println(thetaToRaw(simulatedThetaRad.v[2], 2u));
}

void printPositionMark() {
  const uint16_t *command = nucleo.remoteRaw();
  const uint16_t *measured = nucleo.measuredRawSnapshot();
  Serial.print(F("[POS MARK] xyz_ref_mm="));
  for (unsigned i=0; i<3; ++i) {
    if (i) Serial.print(',');
    Serial.print(referencePose.v[i], 2);
  }
  Serial.print(F(" RAW_destino="));
  for (unsigned i=0; i<3; ++i) {
    if (i) Serial.print(',');
    Serial.print(command[i]);
  }
  Serial.print(F(" RAW_ultima_verificacion="));
  if (nucleo.feedbackSnapshotKnown()) {
    for (unsigned i=0; i<3; ++i) {
      if (i) Serial.print(',');
      Serial.print(measured[i]);
    }
    Serial.print(F(" lectura_age_ms=")); Serial.print(nucleo.feedbackAgeMs());
  } else Serial.print(F("SIN_DATO_VALIDO"));
  Serial.print(F(" RC_us_ACK_Z_X_PALO="));
  for (unsigned i=0; i<3; ++i) {
    if (i) Serial.print(',');
    Serial.print(gripUs[i]);
  }
  Serial.print(F(" rc_referencia="));
  Serial.print(gripReferenceUncertain ? F("INCIERTA") :
               gripStateQueryNeeded ? F("LECTURA_PENDIENTE") : F("CONSIGNA"));
  Serial.print(F(" banda_mm_s="));
  if (nucleo.encoderFresh()) Serial.print(nucleo.beltSpeedMmS(), 1);
  else Serial.print(F("SIN_DATO"));
  Serial.println(F(" RC sin feedback; XYZ es referencia calculada."));
}

void printStatus() {
  Serial.print(F("[TIMING] loop_max_us=")); Serial.print(loopMaxUs);
  Serial.print(F(" gp_gap_max_ms=")); Serial.print(gamepadGapMaxMs);
  Serial.print(F(" gp_last_rx_seq=")); Serial.print(lastGamepadSequence);
  Serial.print(F(" gp_loss_events=")); Serial.println(gamepadLossEvents);
  Serial.print(F("[STATUS] stage=MANUAL_AUTO_CAM_OLED ap="));
  Serial.print(apReady ? F("ON") : F("OFF"));
  Serial.print(F(" gp="));
  Serial.print(gamepadFresh ? F("FRESH") :
    (gamepad.flags & GAMEPAD_FLAG_CONNECTED) ? F("STALE") : F("OFF"));
  Serial.print(F(" age_ms="));
  if (gamepadKnown) Serial.print(millis() - lastGamepadMs); else Serial.print(F("N/A"));
  Serial.print(F(" axes="));
  Serial.print(gamepad.lx); Serial.print(','); Serial.print(gamepad.ly); Serial.print(',');
  Serial.print(gamepad.rx); Serial.print(','); Serial.print(gamepad.ry);
  Serial.print(F(" neutral="));
  Serial.print(gamepadNeutral[0]); Serial.print(','); Serial.print(gamepadNeutral[1]);
  Serial.print(','); Serial.print(gamepadNeutral[2]); Serial.print(',');
  Serial.print(gamepadNeutral[3]);
  Serial.print(F(" center=")); Serial.print(axesCentered() ? 1 : 0);
  Serial.print(F(" cal="));
  if (centerCalibrationActive) Serial.print(F("ACTIVE"));
  else Serial.print(centerCalibrated ? F("OK") : F("DEFAULT"));
  Serial.print(F(" menu=")); Serial.print(menuOpen ? 1 : 0);
  Serial.print(F(" operation=")); Serial.print(operationName());
  Serial.print(F(" mode=")); Serial.print(modeName());
  Serial.print(F(" speed=")); Serial.print(speedName());
  Serial.print(F(" range=")); Serial.print(rangeName());
  Serial.print(F(" raw_guard=")); Serial.print(selectedRawMin());
  Serial.print(F("..")); Serial.print(selectedRawMax());
  Serial.print(F(" verify_end="));
  Serial.print(streamEndVerificationPending ? 1 : 0);
  Serial.print(F(" run=")); Serial.print(runEnabled);
  Serial.print(F(" auto_band=")); Serial.print(automaticBandMove ? 1 : 0);
  Serial.print(F(" auto_auth/target/id=")); Serial.print(autoAuthorized ? 1 : 0);
  Serial.print('/'); Serial.print(autoTargetActive ? 1 : 0);
  Serial.print('/'); Serial.print(autoPieceId);
  Serial.print(F(" camera=")); Serial.print(cameraAlignment.stateName());
  Serial.print(F(" cam_view/id/samples/y="));
  Serial.print(cameraAlignment.observationName()); Serial.print('/');
  Serial.print(cameraAlignment.observedPieceId()); Serial.print('/');
  Serial.print(cameraAlignment.observationSamples()); Serial.print('/');
  Serial.print(cameraAlignment.observedRobotY(), 2);
  Serial.print(F(" cam_pending="));
  Serial.print(cameraAlignment.targetPending() ? 1 : 0);
  Serial.print(F(" tags="));
  Serial.print(cameraAlignment.calibrationSamples(0u)); Serial.print(',');
  Serial.print(cameraAlignment.calibrationSamples(1u)); Serial.print(',');
  Serial.print(cameraAlignment.calibrationSamples(2u)); Serial.print(',');
  Serial.print(cameraAlignment.calibrationSamples(3u));
  Serial.print(F(" reproj_mm=")); Serial.print(cameraAlignment.reprojectionMm(), 3);
  Serial.print(F(" cam_fault=")); Serial.print(cameraAlignment.fault());
  Serial.print(F(" cam_age="));
  const uint32_t cameraAge = cameraAlignment.lastPacketAgeMs();
  if (cameraAge == 0xFFFFFFFFu) Serial.print(F("N/A")); else Serial.print(cameraAge);
  Serial.print(F(" cam_pkt/len/crc/session=")); Serial.print(cameraPacketsValid);
  Serial.print('/'); Serial.print(cameraLengthErrors);
  Serial.print('/'); Serial.print(cameraCrcErrors);
  Serial.print('/'); Serial.print(cameraSessionChanges);
  Serial.print(F(" oled=")); Serial.print(oledReady ? F("OK") : F("WARN"));
  Serial.print(F(" simulation=")); Serial.print(simulationValid ? F("VALID") : F("INVALID"));
  Serial.print(F(" pose="));
  Serial.print(referencePose.v[0], 2); Serial.print(',');
  Serial.print(referencePose.v[1], 2); Serial.print(',');
  Serial.print(referencePose.v[2], 2);
  Serial.print(F(" feedback="));
  if (nucleo.remoteFault()) Serial.print(F("FAULT"));
  else if (nucleo.busy()) Serial.print(F("VERIFYING"));
  else if (nucleo.homeValid()) Serial.print(F("REFERENCE_VALID"));
  else Serial.print(F("REFERENCE_REQUIRED"));
  const uint16_t *remoteRaw = nucleo.remoteRaw();
  Serial.print(F(" raw=")); Serial.print(remoteRaw[0]); Serial.print(',');
  Serial.print(remoteRaw[1]); Serial.print(','); Serial.print(remoteRaw[2]);
  Serial.print(F(" udp valid/len/crc/seq="));
  Serial.print(udpValid); Serial.print('/'); Serial.print(udpLengthErrors); Serial.print('/');
  Serial.print(udpCrcErrors); Serial.print('/'); Serial.print(udpSequenceErrors);
  Serial.print(F(" cycles/stops=")); Serial.print(controllerCycles); Serial.print('/');
  Serial.print(safetyStops);
  Serial.print(F(" workspace_limits=")); Serial.print(workspaceLimits);
  Serial.print(F(" sent/rejected=")); Serial.print(streamCommandsSent);
  Serial.print('/'); Serial.println(streamCommandsRejected);
  nucleo.printStatus();
  Serial.print(F("[BANDA STATUS] pos_x4=")); Serial.print(nucleo.encoderPosition());
  Serial.print(F(" velocidad_mm_s="));
  if (nucleo.encoderFresh()) Serial.println(nucleo.beltSpeedMmS(), 1);
  else Serial.println(F("SIN_DATO"));
}

uint32_t calibrationEpoch() {
  return cameraCommandSession ^ (nucleo.sessionGeneration() * 0x9E3779B9u);
}
void calibrationGateEvent(char event, uint32_t now) {
  if (!calibrationUsb.active(now)) return;
  char line[CalibrationUsb::PACKET_BYTES];
  calibrationUsb.push(line, CalibrationUsb::gate(line, calibrationEpoch(), now,
                                                calibrationUsbTrial, event));
}
void serviceCalibrationUsb() {
  const uint32_t now = millis();
  if (!calibrationUsb.active(now) || !_SerialUSB.connected()) return;
  if (now - calibrationUsbLastMs >= CalibrationUsb::PERIOD_MS) {
    calibrationUsbLastMs = now;
    uint8_t flags = static_cast<uint8_t>(autoPickup.state()) & 0x1fu;
    if (nucleo.encoderFastFresh()) flags |= 0x20u;
    if (autoAuthorized && runEnabled && operationMode == OperationMode::AUTO_PICKUP &&
        autoPickup.active()) flags |= 0x40u;
    if (nucleo.remoteFault() || stopRequested) flags |= 0x80u;
    char line[CalibrationUsb::PACKET_BYTES];
    calibrationUsb.push(line, CalibrationUsb::sample(line, calibrationEpoch(), now,
      nucleo.encoderReceivedMs(), nucleo.encoderPosition(), calibrationUsbTrial,
      flags, gripUs[2]), true);
  }
  const CalibrationUsb::Packet* packet = calibrationUsb.front();
  if (packet) {
    uint32_t actual = 0u;
    _SerialUSB.send_nb(const_cast<uint8_t*>(packet->data), packet->length, &actual);
    calibrationUsb.sent(actual); // never wait for USB to become available
  }
}

void printHelp() {
  Serial.println(F("Control normal: OPTIONS abre menu; D-pad navega; X elige; CIRCULO cancela/desarma."));
  Serial.println(F("Durante STREAM: LX/LY controlan Y/X; RY controla Z; RX sin uso."));
  Serial.println(F("XYZ + R3 EXPERIMENTAL=215..690 RAW fijos; guardias IK/FK/Jacobiano siguen."));
  Serial.println(F("R3: Z=-367..-180, X=+-270, Y=-295..280; caja NO alcanzable completa: IK/RAW/condF limitan cada paso."));
  Serial.println(F("MANUAL: sostener L2/R2 palo, L1/R1 X, cruz izq/der Z; soltar detiene. X/CUAD valores, TRI pantalla."));
  Serial.println(F("La calibracion individual y PALA MANUAL del menu siguen disponibles con DELTA detenido."));
  Serial.println(F("Garra contacto RAW~495/Z~-365.5; PLANO BANDA usa Z=-367.0 (RAW~497)."));
  Serial.println(F("AUTO: tags 0..3, solo pieza_C/ pieza_chica por nombre, todo el cuadro, 3 muestras."));
  Serial.println(F("OPERACION: MANUAL <-> AUTOMATICO. 10C v2.50 compatible + 10B v3.73; mando AUTO max500ms; X RC simultaneo en aproximacion STEPS5; bajada FINA4 REC20 CP5; POST DIRECT300; STOP retiene."));
  Serial.print(F("[AUTO GAMEPAD] max_age_ms="));Serial.print(AutoPickupSetup::GAMEPAD_RUNTIME_MAX_AGE_MS);
  Serial.println(F("; preparacion150ms, OFF/STOP inmediato; manual sin cambios."));
  Serial.print(F("[AUTO ROUTE CONFIG] right_lift_y_mm="));Serial.print(AutoPickupSetup::RIGHT_LIFT_THRESHOLD_Y_MM,3);
  Serial.println(F(" lift_z=box_z; LIFT vertical + TRANSFER XY directos; aproximacion STEPS5 encima + bajada FINA4."));
  Serial.print(F("[AUTO APPROACH] direct="));Serial.print(AutoPickupSetup::PICKUP_APPROACH_DIRECT_ENABLED);
  Serial.println(F("; step5=1 max_raw=5 period_ms=50 move_ms=100; bajada conserva4."));
  Serial.print(F("[AUTO LOWER] direct="));Serial.print(AutoPickupSetup::LOWER_PICKUP_DIRECT_ENABLED);
  Serial.print(F(" fine_max_step_raw="));Serial.print(Config::STREAM_RAW_MAX_STEP);
  Serial.print(F(" fine_period_ms="));Serial.print(Config::CONTROL_PERIOD_MS);
  Serial.println(F("; CP5 con afinado; derecha Z=-313.59 y RC X=1510."));
  Serial.println(F("AUTO: primero ensayo en vacio/banda lenta; parametros en auto_pickup_config.h. HOME, mando, camara y checkpoints obligatorios."));
  Serial.println(F("Comandos USB opcionales de diagnostico:"));
  Serial.println(F("  MENU | CALIBRATE CENTER | STOP"));
  Serial.println(F("  REMOTE_STATUS   solicita estado a la Nucleo"));
  Serial.println(F("  STATUS          muestra ambos enlaces y contadores"));
  Serial.println(F("  HELP"));
  Serial.println(F("  CALTRAIN ON | CALTRAIN OFF  registro pasivo USB; ON caduca en 15s sin renovar"));
  Serial.println(F("El monitor USB puede permanecer cerrado; no forma parte del control."));
}

void processUsbLine(char *line) {
  for (char *cursor = line; *cursor != '\0'; ++cursor) {
    *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
  }
  if (strcmp(line, "CALTRAIN ON") == 0) {
    calibrationUsb.enable(millis()); // logging permission only, NEVER ARM/motion
  } else if (strcmp(line, "CALTRAIN OFF") == 0) {
    calibrationUsb.disable();
  } else if (strcmp(line, "MENU") == 0) {
    openMenu();
  } else if (strcmp(line, "CALIBRATE CENTER") == 0) {
    if (!gamepadFresh || runEnabled || nucleo.armed() ||
        nucleo.streamActive() || nucleo.busy()) {
      Serial.println(F("[CENTER CAL BLOCKED] Requiere gamepad FRESH y robot detenido/desarmado."));
    } else {
      startCenterCalibration();
    }
  } else if (strcmp(line, "STOP") == 0) {
    reopenMenuAfterStop = false;
    pauseSimulation(F("USB diagnostico"));
  } else if (strcmp(line, "REMOTE_STATUS") == 0) {
    if (!nucleo.sendStatusRequest())
      Serial.println(F("[STATUS BLOCKED] Enlace ocupado o no disponible."));
  } else if (strcmp(line, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(line, "HELP") == 0) {
    printHelp();
  } else if (line[0] != '\0') {
    Serial.println(F("[USB ERROR] Comando desconocido; envia HELP."));
  }
}

void serviceUsb() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\r' || character == '\n') {
      if (usbLineLength > 0u) {
        usbLine[usbLineLength] = '\0';
        processUsbLine(usbLine);
        usbLineLength = 0u;
      }
    } else if (usbLineLength + 1u < sizeof(usbLine)) {
      usbLine[usbLineLength++] = character;
    } else {
      usbLineLength = 0u;
      Serial.println(F("[USB ERROR] Linea demasiado larga."));
    }
  }
}

void setup() {
  Serial.begin(Config::USB_BAUD);
  delay(1200u);
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("PORTENTA 10B v3.73 - MANDO AUTO500 + X RC SIMULTANEO + APROXIMACION STEPS5 + BAJADA FINA + POST DIRECT300 + UART CLASICA TX5 REC350x5"));
  Serial.println(F("AUTO: recogida/centro CP5; traslados/cajas15 con reintento selectivo final16..350 max2; STOP y paradas criticas activas"));
  Serial.println(F("Arranque: XYZ + R3 EXPERIMENTAL + PRECISION10 + DESARMADO"));
  Serial.println(F("HOME Z=-287; R3 techo experimental Z=-180. Ampliacion NO validada fisicamente."));
  Serial.println(F("Serial1 115200 protocolo v3 hacia Nucleo 10C"));
  Serial.println(F("================================================"));
  resetSimulation();
  if (!initializeCameraCommandSession()) {
    Serial.println(F("[CAM CMD FAULT] Sin sesion propia; comandos inhibidos."));
  }
  Serial.print(F("[AUTO TRIAL CONFIG] enabled=")); Serial.print(AutoPickupSetup::SUPERVISED_TRIAL_ENABLED ? 1 : 0);
  Serial.print(F(" continuous=")); Serial.print(AutoPickupSetup::CONTINUOUS_ENABLED ? 1 : 0);
  Serial.print(F(" pickup_z_mm=")); Serial.print(GRIP_PICKUP_Z_MM,2);
  Serial.print(F(" lateral_right_y_z=")); Serial.print(AutoPickupHeight::RIGHT_Y_MM,2);
  Serial.print(','); Serial.print(AutoPickupHeight::RIGHT_Z_MM,2);
  Serial.print(F(" lateral_left_y_z=")); Serial.print(AutoPickupHeight::LEFT_Y_MM,2);
  Serial.print(','); Serial.print(AutoPickupHeight::LEFT_Z_MM,2);
  Serial.print(F(" lateral_center_y_z=")); Serial.print(AutoPickupHeight::CENTER_Y_MM,2);
  Serial.print(','); Serial.print(AutoPickupHeight::CENTER_Z_MM,2);
  Serial.print(F(" belt_center_y_z=")); Serial.print(AutoPickupHeight::BELT_CENTER_Y_MM,2);
  Serial.print(','); Serial.print(AutoPickupHeight::BELT_CENTER_Z_MM,2);
  Serial.print(F(" transit_z_mm=")); Serial.print(AutoPickupSetup::TRANSIT_Z_MM,2);
  Serial.print(F(" wait_z_mm=")); Serial.print(AutoPickupSetup::WAIT_Z_MM,2);
  Serial.print(F(" approach_clearance_mm=")); Serial.print(AutoPickupSetup::APPROACH_CLEARANCE_MM,2);
  Serial.print(F(" pickup_x_us=")); Serial.print(GRIP_X_PICKUP_US);
  Serial.print(F(" dump_x_us=")); Serial.print(AutoBoxCalibration::DUMP_X_US);
  Serial.print(F(" rc_step_us=")); Serial.print(AutoPickupSetup::RC_RAMP_STEP_US);
  Serial.print(F(" rc_period_ms=")); Serial.print(AutoPickupSetup::RC_RAMP_PERIOD_MS);
  Serial.print(F(" settle_ms=")); Serial.print(AutoPickupSetup::RC_SETTLE_MS);
  Serial.print(F(" dump_ms=")); Serial.print(AutoPickupSetup::DUMP_SETTLE_MS);
  Serial.print(F(" loaded_settle_ms=")); Serial.print(AutoPickupSetup::LOADED_CARTESIAN_SETTLE_MS);
  Serial.print(F(" arrival_mm=")); Serial.print(AutoPickupSetup::ARRIVAL_TOLERANCE_MM, 1);
  Serial.print(F(" close_lead_mm=")); Serial.println(AutoPickupSetup::CLOSE_LEAD_MM, 1);
  Serial.print(F("[AUTO GATE CONFIG] gate_line_mm=")); Serial.print(AutoPickupSetup::GATE_LINE_FROM_IMAGE_CENTRE_MM,1);
  Serial.print(F(" offset_mm=")); Serial.print(AutoPickupSetup::GATE_CLOSE_OFFSET_MM,1);
  Serial.print(F(" offset_ms=")); Serial.print(AutoPickupSetup::GATE_CLOSE_OFFSET_MS,0);
  Serial.print(F(" pieza_C_mm=")); Serial.print(AutoPickupSetup::GATE_CLOSE_OFFSET_PIECE_C_MM,1);
  Serial.print(F(" pieza_chica_mm=")); Serial.print(AutoPickupSetup::GATE_CLOSE_OFFSET_SMALL_MM,1);
  Serial.print(F(" experimental_lead=")); Serial.print(AutoPickupSetup::GATE_EXPERIMENTAL_LEAD_ENABLED ? 1 : 0);
  Serial.println(F(" sign=+tarde/-temprano"));
  Serial.print(F("[AUTO SPEED CONFIG] approach_mm_s=")); Serial.print(AutoPickupSetup::APPROACH_SPEED_MM_S,1);
  Serial.print(F(" loaded_mm_s=")); Serial.print(AutoPickupSetup::LOADED_TRANSFER_SPEED_MM_S,1);
  Serial.print(F(" empty_mm_s=")); Serial.print(AutoPickupSetup::EMPTY_RETURN_SPEED_MM_S,1);
  Serial.print(F(" final_mm_s=")); Serial.print(AutoPickupSetup::FINAL_APPROACH_SPEED_MM_S,1);
  Serial.print(F(" final_zone_mm=")); Serial.println(AutoPickupSetup::FINAL_APPROACH_DISTANCE_MM,1);
  for (unsigned i=0u;i<AutoBoxCalibration::COUNT;++i) {
    Serial.print(F("[AUTO BOX CONFIG] piece_id=")); Serial.print(AutoBoxCalibration::PIECE_ID[i]);
    Serial.print(F(" xyz_mm=")); Serial.print(AutoBoxCalibration::TARGET_MM[i][0],2);
    Serial.print(','); Serial.print(AutoBoxCalibration::TARGET_MM[i][1],2);
    Serial.print(','); Serial.println(AutoBoxCalibration::TARGET_MM[i][2],2);
  }
  printHelp();
  nucleo.begin(Config::NUCLEO_BAUD);
  oledReady = oled.begin(Wire);
  if (oledReady) {
    Serial.print(F("[OLED READY] SH1106 128x64 address=0x"));
    Serial.println(oled.address(), HEX);
  } else {
    Serial.println(F("[OLED WARN] No detectada en 0x3C/0x3D; control continua."));
  }
  oledDirty = true;
  serviceAp();
}

void loop() {
  const uint32_t loopStartedUs = micros();
  serviceNucleo();
  serviceAp();
  serviceUdp();
  serviceCamera();
  serviceUsb();
  serviceNucleo();
  serviceMenuArm();
  serviceManualPalaPreparation();
  serviceGripReference();
  serviceAutoPickup(); // sequencer/checkpoint/RC leases; never blocks on a wait
  serviceController(); // Hard 50 ms DELTA deadline before optional RC updates.
  serviceGripper();
  serviceBeltDisplay();
  serviceOled();
  serviceCalibrationUsb(); // optional diagnostics AFTER all control/lease services

  const uint32_t now = millis();
  if (runEnabled && now - lastTelemetryMs >= Config::TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    printTelemetry();
  }
  /* [JAC] ya mantiene pose/velocidad/RAW durante RUN. El STATUS extenso se
     difiere hasta detenerse para que una rafaga USB no altere la cadencia. */
  if (!runEnabled && now - lastStatusMs >= Config::STATUS_PERIOD_MS) {
    lastStatusMs = now;
    printStatus();
  }
  const uint32_t loopElapsedUs = micros() - loopStartedUs;
  if (loopElapsedUs > loopMaxUs) loopMaxUs = loopElapsedUs;
}
