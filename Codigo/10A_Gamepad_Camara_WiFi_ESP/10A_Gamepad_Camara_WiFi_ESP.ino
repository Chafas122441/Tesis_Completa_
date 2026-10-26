/**
 * @file 10A_Gamepad_Camara_WiFi_ESP.ino
 * @brief Adquisicion del mando Bluetooth y de HUSKYLENS 2 en el ESP32.
 * Envia datagramas UDP con sesion y secuencia hacia la Portenta. La camara se
 * atiende en una tarea separada. UART2 usa RX GPIO16 y TX GPIO17 a 115200.
 * Las confirmaciones de enlace no sustituyen una lectura visual completa.
 */
#include <Arduino.h>
#include <Bluepad32.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <errno.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "DFRobot_HuskylensV2.h"
#include "camera_packet.h"
#include "camera_piece_classes.h"
#include "camera_mode_uart.h"
#include "gamepad_packet.h"

/* DualShock 4 -> Bluepad32 -> UDP -> Portenta
   HUSKYLENS 2 -> UART2 GPIO16/17 -> UDP binario -> Portenta. */

namespace Config {
constexpr uint32_t USB_BAUD = 115200u;
constexpr uint32_t HUSKY_BAUD = 115200u;
constexpr int8_t HUSKY_RX_PIN = 16;
constexpr int8_t HUSKY_TX_PIN = 17;
constexpr char WIFI_SSID[] = "DELTA_PORTENTA";
constexpr char WIFI_PASSWORD[] = "PortentaDelta2026";
constexpr uint16_t PORTENTA_UDP_PORT = 1234u;
constexpr uint16_t ESP_UDP_PORT = 1235u;
const IPAddress PORTENTA_IP(10, 0, 0, 1);
constexpr uint32_t SEND_MIN_PERIOD_MS = 20u;
constexpr uint32_t BUTTON_SEND_MIN_PERIOD_MS = 10u;
constexpr uint32_t GAMEPAD_SEND_RETRY_MS = 25u;
constexpr uint32_t GAMEPAD_EDGE_MAX_AGE_MS = 150u;
constexpr uint32_t UDP_BIND_RETRY_MS = 25u;
constexpr uint32_t UDP_SEND_FAILURES_BEFORE_REBIND = 3u;
/* Redundancia de ESTADO actual a50Hz, incluso con sticks quietos. No es la
   frecuencia de movimiento del DELTA (20Hz); no reproducir tramas antiguas.
   Mayor numero de oportunidades dentro del mismo corte AUTO de200ms. */
constexpr uint32_t GAMEPAD_HEARTBEAT_MS = 20u;
constexpr uint32_t WIFI_RETRY_PERIOD_MS = 10000u;
constexpr uint32_t WIFI_LATENCY_RETRY_MS = 1000u;
constexpr uint32_t CAMERA_CONNECT_RETRY_MS = 1000u;
/* Una sola orden de modelo, hasta 15 s para su ACK y luego 8 s de carga.
   El worker cede CPU; SETTLING no consulta UART ni bloquea el mando. */
constexpr uint32_t CAMERA_HANDSHAKE_TIMEOUT_MS = 5000u;
constexpr uint32_t CAMERA_MODE_ACK_TIMEOUT_MS = 15000u;
constexpr uint32_t CAMERA_SETTLE_MS = 8000u;
// Tras cargar, exigir una lectura UART completa; no anunciar READY por reloj.
constexpr uint32_t CAMERA_FIRST_FRAME_TIMEOUT_MS = 15000u;
constexpr uint32_t CAMERA_FRAME_UART_TIMEOUT_MS = 1500u;
constexpr uint32_t CAMERA_READ_PERIOD_MS = 100u;
constexpr uint32_t CAMERA_STATUS_PERIOD_MS = 1000u;
constexpr uint32_t CAMERA_RAW_DIAGNOSTIC_MS = 1000u;
constexpr uint32_t STATUS_PERIOD_MS = 2000u;
constexpr uint8_t CAMERA_CONNECT_ATTEMPTS = 1u;
constexpr uint8_t CAMERA_READ_FAILURES = 3u;
constexpr uint8_t CAMERA_MAX_RESULTS = 10u;
constexpr uint16_t CAMERA_WORKER_STACK_SIZE = 6144u;
constexpr int16_t AXIS_CHANGE_THRESHOLD = 8;
constexpr int32_t TRIGGER_PRESS = 192;
constexpr int32_t TRIGGER_RELEASE = 96;
constexpr uint16_t SONY_VENDOR_ID = 0x054Cu;
constexpr uint16_t DS4_V1_PRODUCT_ID = 0x05C4u;
constexpr uint16_t DS4_V2_PRODUCT_ID = 0x09CCu;
constexpr uint32_t GAMEPAD_FIRST_REPORT_TIMEOUT_MS = 5000u;
constexpr size_t USB_LINE_CAPACITY = 40u;
constexpr eAlgorithm_t TAG_ALGORITHM = ALGORITHM_TAG_RECOGNITION;
constexpr eAlgorithm_t PIECE_ALGORITHM = static_cast<eAlgorithm_t>(128);
}

enum class CameraState : uint8_t { OFF = 0u, CONNECTING, SETTLING, READY, FAULT };
enum class CameraWorkerOperation : uint8_t { SWITCH_MODE = 0u, READ_FRAME };

struct CameraObservation {
  int16_t rawId;
  int16_t learnedId;
  int16_t tagContent;
  int16_t xCenter;
  int16_t yCenter;
  int16_t width;
  int16_t height;
  char name[24];
  char content[24];
};

struct CameraWorkerRequest {
  CameraWorkerOperation operation;
  uint32_t generation;
  int16_t algorithm;
  uint8_t forceReconnect;
};

struct CameraWorkerEvent {
  CameraWorkerOperation operation;
  uint32_t generation;
  int16_t algorithm;
  uint16_t durationMs;
  uint8_t success;
  uint8_t linkEstablished;
  uint8_t failurePhase; // 1=KNOCK, 2=SET_ALGORITHM, 0=ninguna.
  uint8_t uartCode;
  uint16_t phaseDurationMs;
  uint8_t hasReturnValue;
  int8_t returnValue;
  uint8_t replyAlgorithm;
  uint8_t replyLength;
  uint16_t ignoredFrames;
  uint16_t badChecksums;
  uint8_t returnTextLength;
  uint8_t returnText[245];
  uint8_t validatedFrame;
  uint8_t frameAlgorithm;
  uint8_t count;
  CameraObservation observations[Config::CAMERA_MAX_RESULTS];
};

ControllerPtr primaryController = nullptr;
ControllerPtr pendingController = nullptr;
uint32_t pendingControllerStartedMs = 0u;
ControllerPtr rejectedControllers[BP32_MAX_CONTROLLERS]{};
portMUX_TYPE controllerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool closePairingRequested = false;
bool pairingOpen = true;
bool triggerL2Active = false;
bool triggerR2Active = false;
uint32_t acceptedControllerCount = 0u;
uint32_t rejectedControllerCount = 0u;
uint32_t controllerDisconnectCount = 0u;
uint32_t controllerCandidateCount = 0u;
uint32_t controllerFirstReportTimeoutCount = 0u;
const char *lastControllerRejectReason = "NONE";
HardwareSerial huskySerial(2);
HuskylensV2 huskylens;
WiFiUDP udp;
bool dualShock4Connected = false;
bool udpBound = false;
uint32_t sessionId = 0u;
uint32_t nextGamepadSequence = 1u;
uint32_t nextCameraSequence = 1u;
uint32_t cameraFrameSequence = 1u;
uint32_t commandSession = 0u;
uint32_t lastCommandSequence = 0u;
GamepadPacket lastSentPacket{};
bool haveLastPacket = false;
uint32_t sentGamepadPackets = 0u;
uint32_t sentCameraPackets = 0u;
uint32_t failedPackets = 0u;
uint32_t badCameraCommands = 0u;
uint32_t lastGamepadSendMs = 0u;
uint32_t lastGamepadAttemptMs = 0u;
bool lastGamepadAttemptFailed = false;
uint32_t gamepadSendFailures = 0u;
uint32_t consecutiveGamepadFailures = 0u;
uint32_t gamepadSendGapMaxMs = 0u;
uint32_t gamepadSendGapLastMs = 0u;
uint32_t lastGamepadFailedSequence = 0u;
uint32_t lastUdpBindAttemptMs = 0u;
uint32_t udpRecoveries = 0u;
uint32_t udpBeginErrors = 0u;
uint32_t udpWriteEndErrors = 0u;
uint32_t udpSendFailureStreak = 0u;
uint8_t udpLastFailurePhase = 0u;  // 0=NONE, 1=BEGIN, 2=WRITE, 3=END.
int udpLastFailureErrno = 0;
uint32_t udpLastFailureMs = 0u;
uint32_t udpLastFailureDurationUs = 0u;
uint32_t udpSendDurationMaxUs = 0u;
uint32_t pendingEdgesSinceMs = 0u;
uint16_t observedButtons = 0u;
uint16_t observedMisc = 0u;
uint8_t observedDpad = 0u;
uint32_t espLoopMaxUs = 0u;
uint32_t lastWifiAttemptMs = 0u;
bool wifiLatencyVerified = false;
bool wifiLatencyAttempted = false;
uint32_t wifiLatencyLastAttemptMs = 0u;
int32_t wifiSleepMode = -1;
int32_t wifiLatencyError = 0;
uint32_t lastStatusMs = 0u;
uint32_t lastCameraStatusMs = 0u;
char usbLine[Config::USB_LINE_CAPACITY];
size_t usbLineLength = 0u;

CameraState cameraState = CameraState::OFF;
int16_t requestedAlgorithm = -1;
int16_t activeAlgorithm = -1;
uint16_t pendingCommandValue = CAMERA_COMMAND_OFF;
uint8_t cameraConnectAttempts = 0u;
uint8_t consecutiveReadFailures = 0u;
uint16_t cameraLastFault = CAMERA_FAULT_NONE;
uint32_t lastCameraConnectMs = 0u;
uint32_t cameraStateStartedMs = 0u;
uint32_t lastCameraReadMs = 0u;
uint16_t lastCameraReadDurationMs = 0u;
bool cameraLinkEstablished = false;
bool cameraModeProvisional = false;
uint32_t lastCameraProbeDiagnosticMs = 0u;
QueueHandle_t cameraRequestQueue = nullptr;
QueueHandle_t cameraEventQueue = nullptr;
TaskHandle_t cameraWorkerHandle = nullptr;
uint32_t cameraGeneration = 1u;
bool cameraWorkerBusy = false;
uint32_t lastCameraRawDiagnosticMs = 0u;
uint16_t pendingButtonEdges = 0u;
uint16_t pendingMiscEdges = 0u;
uint8_t pendingDpadEdges = 0u;

bool sequenceIsNewer(uint32_t value, uint32_t previous) {
  return static_cast<int32_t>(value - previous) > 0;
}

const char *cameraStateName() {
  switch (cameraState) {
    case CameraState::OFF: return "OFF";
    case CameraState::CONNECTING: return "CONNECTING";
    case CameraState::SETTLING: return "SETTLING";
    case CameraState::READY: return "READY";
    case CameraState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXTERNAL";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "OTHER_WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    default: return "OTHER";
  }
}

void printControllerProperties(ControllerPtr controller) {
  const ControllerProperties properties = controller->getProperties();
  Serial.print(F("[GAMEPAD] model="));
  Serial.print(controller->getModelName().c_str());
  Serial.print(F(" VID=0x")); Serial.print(properties.vendor_id, HEX);
  Serial.print(F(" PID=0x")); Serial.print(properties.product_id, HEX);
  Serial.print(F(" mac="));
  for (uint8_t index = 0u; index < 6u; ++index) {
    if (index != 0u) Serial.print(':');
    if (properties.btaddr[index] < 0x10u) Serial.print('0');
    Serial.print(properties.btaddr[index], HEX);
  }
  Serial.print(F(" class=")); Serial.print(static_cast<int>(controller->getClass()));
  Serial.print(F(" has_data=")); Serial.println(controller->hasData() ? 1 : 0);
}

ControllerPtr controllerSnapshot() {
  portENTER_CRITICAL(&controllerMux);
  ControllerPtr controller = primaryController;
  portEXIT_CRITICAL(&controllerMux);
  return controller;
}

ControllerPtr pendingControllerSnapshot() {
  portENTER_CRITICAL(&controllerMux);
  ControllerPtr controller = pendingController;
  portEXIT_CRITICAL(&controllerMux);
  return controller;
}

bool controllerSlotReserved() {
  portENTER_CRITICAL(&controllerMux);
  const bool reserved = primaryController != nullptr || pendingController != nullptr;
  portEXIT_CRITICAL(&controllerMux);
  return reserved;
}

void queueRejectedController(ControllerPtr controller) {
  if (controller == nullptr) return;
  const int8_t index = controller->index();
  if (index < 0 || index >= BP32_MAX_CONTROLLERS) return;
  portENTER_CRITICAL(&controllerMux);
  rejectedControllers[index] = controller;
  portEXIT_CRITICAL(&controllerMux);
}

void rejectController(ControllerPtr controller, const char *reason) {
  ++rejectedControllerCount;
  lastControllerRejectReason = reason;
  if (controller != nullptr) printControllerProperties(controller);
  Serial.print(F("[GAMEPAD REJECT] reason=")); Serial.println(reason);
  queueRejectedController(controller);
}

void onConnectedController(ControllerPtr controller) {
  if (controller == nullptr) {
    rejectController(controller, "NULL_CONTROLLER");
    return;
  }

  const ControllerProperties properties = controller->getProperties();
  const int model = controller->getModel();
  const bool genuineDs4 = properties.vendor_id == Config::SONY_VENDOR_ID &&
    (properties.product_id == Config::DS4_V1_PRODUCT_ID ||
     properties.product_id == Config::DS4_V2_PRODUCT_ID);
  const bool parsedAsDs4 = model == CONTROLLER_TYPE_PS4Controller ||
                           model == CONTROLLER_TYPE_XInputPS4Controller;
  if (!genuineDs4 && !parsedAsDs4) {
    rejectController(controller, "NOT_DS4");
    return;
  }
  // Bluepad32 4.1 can call onConnected before the FIRST input report.
  // Without fresh data the class may also belong to an earlier slot occupant.
  if (controller->hasData() && controller->getClass() != UNI_CONTROLLER_CLASS_NONE &&
      !controller->isGamepad()) {
    rejectController(controller, "NOT_GAMEPAD_CLASS");
    return;
  }

  bool reserved = false;
  const uint32_t now = millis();
  portENTER_CRITICAL(&controllerMux);
  if (primaryController == nullptr && pendingController == nullptr) {
    pendingController = controller;
    pendingControllerStartedMs = now;
    reserved = true;
  }
  portEXIT_CRITICAL(&controllerMux);

  if (!reserved) {
    rejectController(controller, "SLOT_RESERVED");
    return;
  }

  ++controllerCandidateCount;
  printControllerProperties(controller);
  Serial.print(F("[GAMEPAD WAIT] DS4 identificado; esperando primer reporte, timeout_ms="));
  Serial.println(Config::GAMEPAD_FIRST_REPORT_TIMEOUT_MS);
}

void onDisconnectedController(ControllerPtr controller) {
  bool wasPrimary = false;
  bool wasPending = false;
  portENTER_CRITICAL(&controllerMux);
  if (controller != nullptr && controller == primaryController) {
    primaryController = nullptr;
    wasPrimary = true;
  }
  if (controller != nullptr && controller == pendingController) {
    pendingController = nullptr;
    pendingControllerStartedMs = 0u;
    wasPending = true;
  }
  const int8_t index = controller == nullptr ? -1 : controller->index();
  if (index >= 0 && index < BP32_MAX_CONTROLLERS &&
      rejectedControllers[index] == controller) {
    rejectedControllers[index] = nullptr;
  }
  portEXIT_CRITICAL(&controllerMux);
  if (wasPending) Serial.println(F("[GAMEPAD WAIT] desconectado antes del primer reporte."));
  if (!wasPrimary) return;
  ++controllerDisconnectCount;
  dualShock4Connected = false;
  haveLastPacket = false;
  pendingButtonEdges = 0u;
  pendingMiscEdges = 0u;
  pendingDpadEdges = 0u;
  lastGamepadSendMs = 0u;
  Serial.println(F("[GAMEPAD] desconectado; se enviara estado seguro."));
}

void servicePendingController() {
  ControllerPtr controller = pendingControllerSnapshot();
  if (controller == nullptr) return;
  const uint32_t waitedMs = millis() - pendingControllerStartedMs;
  const char *reason = nullptr;
  if (!controller->isConnected()) reason = "DISCONNECTED_BEFORE_REPORT";
  else if (controller->hasData() && controller->getClass() != UNI_CONTROLLER_CLASS_NONE &&
           !controller->isGamepad())
    reason = "NOT_GAMEPAD_CLASS";
  else if (waitedMs >= Config::GAMEPAD_FIRST_REPORT_TIMEOUT_MS) {
    reason = "FIRST_REPORT_TIMEOUT";
    ++controllerFirstReportTimeoutCount;
  }
  if (reason != nullptr) {
    portENTER_CRITICAL(&controllerMux);
    pendingController = nullptr;
    pendingControllerStartedMs = 0u;
    portEXIT_CRITICAL(&controllerMux);
    rejectController(controller, reason);
    if (!controller->hasData())
      Serial.println(F("[GAMEPAD] Sin reporte fresco; apaga el mando antes de reintentar."));
    return;
  }
  // hasData is refreshed by BP32.update(). Do not trust a stale class from a
  // previous connection that reused this slot. No connected UDP flag yet.
  if (!controller->isGamepad() || !controller->hasData()) return;
  portENTER_CRITICAL(&controllerMux);
  pendingController = nullptr;
  pendingControllerStartedMs = 0u;
  primaryController = controller;
  closePairingRequested = true;
  portEXIT_CRITICAL(&controllerMux);
  ++acceptedControllerCount;
  dualShock4Connected = true;
  haveLastPacket = false;
  pendingButtonEdges = 0u;
  pendingMiscEdges = 0u;
  pendingDpadEdges = 0u;
  printControllerProperties(controller);
  Serial.print(F("[GAMEPAD READY] primer reporte valido; wait_ms=")); Serial.println(waitedMs);
  Serial.println(F("[GAMEPAD] conectado; profile=DUALSHOCK4 X=confirmar CIRCULO=cancelar OPTIONS=menu."));
}

void serviceControllerLifecycle() {
  servicePendingController();
  bool closePairing = false;
  portENTER_CRITICAL(&controllerMux);
  if (closePairingRequested) {
    closePairingRequested = false;
    closePairing = true;
  }
  portEXIT_CRITICAL(&controllerMux);

  if (closePairing) {
    BP32.enableNewBluetoothConnections(false);
    pairingOpen = false;
    Serial.println(F("[BLUETOOTH] busqueda cerrada; segundo mando se rechaza (sin filtro MAC)."));
  }

  for (int8_t index = 0; index < BP32_MAX_CONTROLLERS; ++index) {
    ControllerPtr rejected = nullptr;
    portENTER_CRITICAL(&controllerMux);
    rejected = rejectedControllers[index];
    // Bluepad32 4.1 processes disconnect requests on controller reports.
    // Do not enqueue a request for a silent device: its index could be reused
    // before that request runs. Keep it ignored until data or disconnection.
    if (rejected != nullptr && rejected->isConnected() && !rejected->hasData())
      rejected = nullptr;
    else rejectedControllers[index] = nullptr;
    portEXIT_CRITICAL(&controllerMux);
    if (rejected != nullptr && rejected->isConnected()) rejected->disconnect();
  }
}

GamepadPacket currentGamepadPacket() {
  GamepadPacket packet{};
  packet.session = sessionId;
  packet.sequence = nextGamepadSequence;
  ControllerPtr controller = controllerSnapshot();
  if (controller != nullptr && controller->isConnected() &&
      controller->isGamepad()) {
    packet.flags = GAMEPAD_FLAG_CONNECTED;
    packet.lx = static_cast<int16_t>(controller->axisX());
    packet.ly = static_cast<int16_t>(controller->axisY());
    packet.rx = static_cast<int16_t>(controller->axisRX());
    packet.ry = static_cast<int16_t>(controller->axisRY());
    const uint16_t rawButtons = controller->buttons();
    // Some DS4 clones expose L2/R2 as analog brake/throttle only.
    triggerL2Active = (rawButtons & 0x40u) ||
      controller->brake() > (triggerL2Active ? Config::TRIGGER_RELEASE : Config::TRIGGER_PRESS);
    triggerR2Active = (rawButtons & 0x80u) ||
      controller->throttle() > (triggerR2Active ? Config::TRIGGER_RELEASE : Config::TRIGGER_PRESS);
    packet.buttons = rawButtons |
      (triggerL2Active ? 0x40u : 0u) | (triggerR2Active ? 0x80u : 0u);
    packet.misc = controller->miscButtons();
    packet.dpad = static_cast<uint8_t>(controller->dpad());
  } else {
    triggerL2Active = triggerR2Active = false;
  }
  gamepadFinalize(&packet);
  return packet;
}

bool axisChanged(int16_t current, int16_t previous) {
  return abs(static_cast<int32_t>(current) - previous) >=
         Config::AXIS_CHANGE_THRESHOLD;
}

bool gamepadPacketChanged(const GamepadPacket &current,
                          const GamepadPacket &previous) {
  return current.flags != previous.flags ||
         current.buttons != previous.buttons ||
         current.misc != previous.misc || current.dpad != previous.dpad ||
         axisChanged(current.lx, previous.lx) ||
         axisChanged(current.ly, previous.ly) ||
         axisChanged(current.rx, previous.rx) ||
         axisChanged(current.ry, previous.ry);
}

void recoverUdpSocket() {
  // Recover only UDP; never reboot Wi-Fi/Bluetooth or reopen pairing.
  udp.stop();
  udpBound = false;
  // Rebind on the next loop, not after a compulsory 250ms blackout.
  // Unsigned subtraction also works across millis() rollover.
  lastUdpBindAttemptMs = millis() - Config::UDP_BIND_RETRY_MS;
  // serviceWifi may restore the socket before serviceGamepadPackets runs.
  // Drop old menu/ARM edges here, not only in its unbound branch.
  pendingButtonEdges = pendingMiscEdges = 0u;
  pendingDpadEdges = 0u;
  pendingEdgesSinceMs = 0u;
  ++udpRecoveries;
}

void recordUdpSendFailure(uint8_t phase, int error, uint32_t startedUs,
                          bool rebuildNow) {
  ++failedPackets;
  if (phase == 1u) ++udpBeginErrors;
  else ++udpWriteEndErrors;
  ++udpSendFailureStreak;
  udpLastFailurePhase = phase;
  udpLastFailureErrno = error;
  udpLastFailureMs = millis();
  udpLastFailureDurationUs = micros() - startedUs;
  if (udpLastFailureDurationUs > udpSendDurationMaxUs)
    udpSendDurationMaxUs = udpLastFailureDurationUs;
  // A nonblocking sendto error does not by itself invalidate the socket.
  // Keep it for isolated errors; rebuild only if broken or failures persist.
  if (rebuildNow ||
      udpSendFailureStreak >= Config::UDP_SEND_FAILURES_BEFORE_REBIND)
    recoverUdpSocket();
  // No Serial here: diagnostics are emitted by the existing 2s STATUS.
}

bool sendBytes(const uint8_t *bytes, size_t length) {
  if (WiFi.status() != WL_CONNECTED || !udpBound) {
    ++failedPackets;
    return false;
  }
  const uint32_t startedUs = micros();
  errno = 0;
  const bool began = udp.beginPacket(Config::PORTENTA_IP, Config::PORTENTA_UDP_PORT) == 1;
  const int beginError = began ? 0 : errno;
  if (!began) {
    recordUdpSendFailure(1u, beginError, startedUs, true);
    return false;
  }
  errno = 0;
  const size_t written = udp.write(bytes, length);
  const int writeError = written == length ? 0 : errno;
  if (written != length) {
    // Never put a partial gamepad/camera datagram on the wire.
    recordUdpSendFailure(2u, writeError, startedUs, true);
    return false;
  }
  errno = 0;
  const bool ended = udp.endPacket() == 1;
  const int endError = ended ? 0 : errno;
  if (!ended) {
    const bool brokenSocket = endError == EBADF || endError == ENOTSOCK;
    recordUdpSendFailure(3u, endError, startedUs, brokenSocket);
    return false;
  }
  udpSendFailureStreak = 0u;
  const uint32_t durationUs = micros() - startedUs;
  if (durationUs > udpSendDurationMaxUs) udpSendDurationMaxUs = durationUs;
  return true;
}

bool sendGamepadPacket(GamepadPacket packet) {
  packet.sequence = nextGamepadSequence++;
  if (nextGamepadSequence == 0u) nextGamepadSequence = 1u;
  gamepadFinalize(&packet);
  if (!sendBytes(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet))) {
    lastGamepadFailedSequence = packet.sequence;
    ++gamepadSendFailures;
    ++consecutiveGamepadFailures;
    lastGamepadAttemptFailed = true;
    return false;
  }
  const uint32_t now = millis();
  if (haveLastPacket) {
    const uint32_t gap = now - lastGamepadSendMs;
    gamepadSendGapLastMs = gap;
    if (gap > gamepadSendGapMaxMs) gamepadSendGapMaxMs = gap;
  }
  consecutiveGamepadFailures = 0u;
  lastGamepadAttemptFailed = false;
  ++sentGamepadPackets;
  lastSentPacket = packet;
  haveLastPacket = true;
  lastGamepadSendMs = now;
  return true;
}

bool sendCameraPacket(CameraPacket packet) {
  packet.session = sessionId;
  packet.sequence = nextCameraSequence++;
  if (nextCameraSequence == 0u) nextCameraSequence = 1u;
  cameraFinalize(&packet);
  if (!sendBytes(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet))) {
    return false;
  }
  ++sentCameraPackets;
  return true;
}

void sendCameraStatus(uint8_t type = CAMERA_TYPE_STATUS,
                      uint16_t value = CAMERA_FAULT_NONE) {
  CameraPacket packet{};
  packet.type = type;
  packet.algorithm = activeAlgorithm;
  packet.readMs = lastCameraReadDurationMs;
  packet.value = value;
  if (cameraState == CameraState::READY ||
      cameraState == CameraState::SETTLING) packet.flags |= CAMERA_FLAG_UART_READY;
  if (cameraState == CameraState::READY) packet.flags |= CAMERA_FLAG_MODE_READY;
  if (cameraState == CameraState::SETTLING) packet.flags |= CAMERA_FLAG_SETTLING;
  sendCameraPacket(packet);
  lastCameraStatusMs = millis();
}

void clearHuskyInput() {
  while (huskySerial.available() > 0) huskySerial.read();
}

void cameraOff() {
  ++cameraGeneration;
  if (cameraGeneration == 0u) cameraGeneration = 1u;
  requestedAlgorithm = -1;
  activeAlgorithm = -1;
  pendingCommandValue = CAMERA_COMMAND_OFF;
  cameraState = CameraState::OFF;
  cameraModeProvisional = false;
  cameraConnectAttempts = 0u;
  consecutiveReadFailures = 0u;
  cameraLastFault = CAMERA_FAULT_NONE;
  sendCameraStatus(CAMERA_TYPE_ACK, CAMERA_COMMAND_OFF);
  Serial.println(F("[CAMERA] OFF solicitado; no se consultara HUSKYLENS."));
}

void cameraFault(uint16_t code, const __FlashStringHelper *reason) {
  cameraState = CameraState::FAULT;
  cameraModeProvisional = false;
  cameraLastFault = code;
  activeAlgorithm = -1;
  if (code == CAMERA_FAULT_UART || code == CAMERA_FAULT_READ ||
      code == CAMERA_FAULT_MODE)
    cameraLinkEstablished = false;
  Serial.print(F("[CAMERA FAULT] ")); Serial.println(reason);
  sendCameraStatus(CAMERA_TYPE_FAULT, code);
}

void requestCameraAlgorithm(uint16_t command) {
  if (command != CAMERA_COMMAND_TAG_MODE &&
      command != CAMERA_COMMAND_PIECE_MODE) {
    cameraFault(CAMERA_FAULT_BAD_COMMAND, F("Comando de algoritmo invalido."));
    return;
  }
  ++cameraGeneration;
  if (cameraGeneration == 0u) cameraGeneration = 1u;
  requestedAlgorithm = static_cast<int16_t>(command);
  pendingCommandValue = command;
  activeAlgorithm = -1;
  cameraState = CameraState::CONNECTING;
  cameraModeProvisional = false;
  cameraConnectAttempts = 0u;
  consecutiveReadFailures = 0u;
  lastCameraConnectMs = 0u;
  cameraLastFault = CAMERA_FAULT_NONE;
  Serial.print(F("[CAMERA] solicitando algoritmo "));
  Serial.println(requestedAlgorithm);
}

void serviceCameraConnection() {
  if (cameraState != CameraState::CONNECTING) return;
  if (cameraWorkerBusy) return;
  const uint32_t now = millis();
  if (lastCameraConnectMs != 0u &&
      now - lastCameraConnectMs < Config::CAMERA_CONNECT_RETRY_MS) return;
  if (cameraConnectAttempts >= Config::CAMERA_CONNECT_ATTEMPTS) {
    cameraFault(CAMERA_FAULT_UART, F("HUSKYLENS no respondio por UART."));
    return;
  }
  lastCameraConnectMs = now;
  ++cameraConnectAttempts;
  if (!cameraLinkEstablished) {
    Serial.print(F("[CAMERA] handshake UART "));
    Serial.print(cameraConnectAttempts); Serial.print('/');
    Serial.println(Config::CAMERA_CONNECT_ATTEMPTS);
  } else {
    Serial.print(F("[CAMERA] enlace UART activo; cambiando a algoritmo "));
    Serial.println(requestedAlgorithm);
  }
  CameraWorkerRequest request{};
  request.operation = CameraWorkerOperation::SWITCH_MODE;
  request.generation = cameraGeneration;
  request.algorithm = requestedAlgorithm;
  request.forceReconnect = cameraLinkEstablished ? 0u : 1u;
  if (cameraRequestQueue == nullptr ||
      xQueueSend(cameraRequestQueue, &request, 0u) != pdTRUE) {
    cameraFault(CAMERA_FAULT_UART, F("No se pudo iniciar tarea HUSKYLENS."));
    return;
  }
  cameraWorkerBusy = true;
}

int16_t parseTagContent(const Result &result) {
  const char *text = result.content.c_str();
  if (text == nullptr) return -1;
  while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') ++text;
  if (*text < '0' || *text > '9') return -1;
  int16_t value = 0;
  while (*text >= '0' && *text <= '9') {
    value = static_cast<int16_t>(value * 10 + (*text - '0'));
    if (value > 3) return -1; // Evitar overflow con contenido decimal largo.
    ++text;
  }
  while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') ++text;
  return *text == '\0' && value >= 0 && value <= 3 ? value : -1;
}

int16_t parsePieceLearnedId(const Result &result) {
  // La trama conserva el campo learnedId, pero en PIECE_MODE lleva clase
  // interna 1/2/0. rawId se conserva exclusivamente para diagnostico local.
  return cameraPieceClassFromName(result.name.c_str());
}

void copyCameraText(char *destination, size_t capacity, const String &source) {
  if (destination == nullptr || capacity == 0u) return;
  const char *text = source.c_str();
  if (text == nullptr) text = "";
  size_t index = 0u;
  while (index + 1u < capacity && text[index] != '\0') {
    destination[index] = text[index];
    ++index;
  }
  destination[index] = '\0';
}

void cameraWorkerTask(void *) {
  bool linkEstablished = false;
  CameraModeUart::EmptyFramePolicy emptyPolicy{};
  CameraWorkerRequest request{};
  for (;;) {
    if (xQueueReceive(cameraRequestQueue, &request, portMAX_DELAY) != pdTRUE) continue;
    CameraWorkerEvent event{};
    event.operation = request.operation;
    event.generation = request.generation;
    event.algorithm = request.algorithm;
    const uint32_t startedMs = millis();
    const eAlgorithm_t algorithm = request.algorithm == CAMERA_COMMAND_TAG_MODE
      ? Config::TAG_ALGORITHM : Config::PIECE_ALGORITHM;

    if (request.operation == CameraWorkerOperation::SWITCH_MODE) {
      emptyPolicy.select(request.generation, request.algorithm, false);
      if (request.forceReconnect != 0u) linkEstablished = false;
      if (!linkEstablished) {
        clearHuskyInput();
        huskylens.stream = &huskySerial;
        huskylens.wire = nullptr;
        const CameraModeUart::Reply reply = CameraModeUart::transact(
          huskySerial, static_cast<uint8_t>(COMMAND_KNOCK),
          static_cast<uint8_t>(BOARD_LARGE_RAM), Config::CAMERA_HANDSHAKE_TIMEOUT_MS,
          []() { return millis(); }, []() { vTaskDelay(1u); });
        linkEstablished = reply.code == CameraModeUart::Code::OK;
        event.failurePhase = linkEstablished ? 0u : 1u;
        event.uartCode = static_cast<uint8_t>(reply.code);
        event.phaseDurationMs = static_cast<uint16_t>(reply.elapsedMs);
        event.hasReturnValue = reply.hasReturnValue ? 1u : 0u;
        event.returnValue = reply.returnValue;
        event.replyAlgorithm = reply.replyAlgorithm;
        event.replyLength = reply.replyLength;
        event.ignoredFrames = reply.ignoredFrames;
        event.badChecksums = reply.badChecksums;
        event.returnTextLength = reply.returnTextLength;
        memcpy(event.returnText, reply.returnText, reply.returnTextLength);
      }
      bool switched = false;
      if (linkEstablished) {
        clearHuskyInput();
        const CameraModeUart::Reply reply = CameraModeUart::transact(
          huskySerial, static_cast<uint8_t>(COMMAND_SET_ALGORITHM),
          static_cast<uint8_t>(algorithm), Config::CAMERA_MODE_ACK_TIMEOUT_MS,
          []() { return millis(); }, []() { vTaskDelay(1u); });
        switched = reply.code == CameraModeUart::Code::OK;
        event.failurePhase = switched ? 0u : 2u;
        event.uartCode = static_cast<uint8_t>(reply.code);
        event.phaseDurationMs = static_cast<uint16_t>(reply.elapsedMs);
        event.hasReturnValue = reply.hasReturnValue ? 1u : 0u;
        event.returnValue = reply.returnValue;
        event.replyAlgorithm = reply.replyAlgorithm;
        event.replyLength = reply.replyLength;
        event.ignoredFrames += reply.ignoredFrames;
        event.badChecksums += reply.badChecksums;
        event.returnTextLength = reply.returnTextLength;
        memcpy(event.returnText, reply.returnText, reply.returnTextLength);
#ifdef LARGE_MEMORY
        // Misma inicializacion de cache/mapeo que HuskylensV2::switchAlgorithm.
        huskylens.customAlgoNum = 0u;
        memset(huskylens.customId, 0, sizeof(huskylens.customId));
        if (switched && algorithm >= ALGORITHM_CUSTOM_BEGIN)
          huskylens.customId[0] = static_cast<uint8_t>(algorithm);
#endif
      }
      // ret=1 es fallo de la orden, NO exito. Si KNOCK fue valido, conservar
      // UART exclusivamente para comprobar el modelo durante SETTLING finito.
      const bool provisional = !switched && linkEstablished &&
        event.failurePhase == 2u && event.hasReturnValue != 0u &&
        event.returnValue == 1 &&
        event.uartCode == static_cast<uint8_t>(CameraModeUart::Code::NACK);
      if (!switched && !provisional) linkEstablished = false;
      emptyPolicy.select(request.generation, request.algorithm, switched);
      event.success = switched ? 1u : 0u;
    } else if (linkEstablished) {
      const CameraModeUart::FrameReply reply = CameraModeUart::readFrame(
        huskySerial, static_cast<uint8_t>(algorithm),
        Config::CAMERA_FRAME_UART_TIMEOUT_MS,
        []() { return millis(); }, []() { vTaskDelay(1u); },
        [&](const uint8_t *packet) {
          // El constructor SDK espera dos strings aun sin nombres. Normalizar
          // el caso payload10 ya validado, sin leer checksum como longitud.
          uint8_t noStrings[18]{};
          if (packet[4] == 10u) {
            memcpy(noStrings, packet, 15u);
            noStrings[4] = 12u;
            packet = noStrings;
          }
          Result result(packet);
          CameraObservation &observation = event.observations[event.count++];
          observation.rawId = static_cast<int16_t>(result.ID);
          observation.learnedId = request.algorithm == CAMERA_COMMAND_PIECE_MODE
            ? parsePieceLearnedId(result) : static_cast<int16_t>(result.ID);
          observation.tagContent = request.algorithm == CAMERA_COMMAND_TAG_MODE
            ? parseTagContent(result) : -1;
          observation.xCenter = static_cast<int16_t>(result.xCenter);
          observation.yCenter = static_cast<int16_t>(result.yCenter);
          observation.width = static_cast<int16_t>(result.width);
          observation.height = static_cast<int16_t>(result.height);
          copyCameraText(observation.name, sizeof(observation.name), result.name);
          copyCameraText(observation.content, sizeof(observation.content), result.content);
        }, emptyPolicy.allows(request.generation, request.algorithm));
      event.uartCode = static_cast<uint8_t>(reply.code);
      event.frameAlgorithm = reply.actualAlgorithm;
      event.ignoredFrames = reply.ignoredFrames;
      event.badChecksums = reply.badChecksums;
      event.validatedFrame = reply.code == CameraModeUart::Code::OK &&
        reply.actualAlgorithm == static_cast<uint8_t>(algorithm) &&
        event.count == reply.count ? 1u : 0u;
      event.success = event.validatedFrame;
      emptyPolicy.observe(request.generation, request.algorithm, event.success != 0u);
      if (!event.success) event.count = 0u; // Nunca emitir un cuadro parcial.
    }
    event.linkEstablished = linkEstablished ? 1u : 0u;
    const uint32_t durationMs = millis() - startedMs;
    event.durationMs = static_cast<uint16_t>(durationMs > 65535u ? 65535u : durationMs);
    xQueueSend(cameraEventQueue, &event, portMAX_DELAY);
  }
}

void sendCameraFrame(uint32_t frame, uint8_t index, uint8_t count,
                     const CameraObservation *result, uint16_t readMs) {
  CameraPacket packet{};
  packet.type = CAMERA_TYPE_FRAME;
  packet.frame = frame;
  packet.algorithm = activeAlgorithm;
  packet.index = index;
  packet.count = count;
  packet.readMs = readMs;
  if (result == nullptr) {
    packet.flags = CAMERA_FLAG_UART_READY | CAMERA_FLAG_MODE_READY |
                   CAMERA_FLAG_EMPTY_FRAME;
    packet.learnedId = -1;
    packet.tagContent = -1;
  } else {
    packet.flags = CAMERA_FLAG_UART_READY | CAMERA_FLAG_MODE_READY;
    packet.learnedId = result->learnedId;
    packet.tagContent = result->tagContent;
    packet.xCenter = result->xCenter;
    packet.yCenter = result->yCenter;
    packet.width = result->width;
    packet.height = result->height;
    if (activeAlgorithm == CAMERA_COMMAND_TAG_MODE &&
        millis() - lastCameraRawDiagnosticMs >= Config::CAMERA_RAW_DIAGNOSTIC_MS) {
      lastCameraRawDiagnosticMs = millis();
      Serial.print(F("[CAMERA TAG RAW] frame=")); Serial.print(frame);
      Serial.print(F(" objects=")); Serial.print(count);
      Serial.print(F(" learned=")); Serial.print(result->rawId);
      Serial.print(F(" content=\"")); Serial.print(result->content);
      Serial.print(F("\" parsed=")); Serial.print(packet.tagContent);
      Serial.print(F(" pixel=")); Serial.print(result->xCenter);
      Serial.print(','); Serial.println(result->yCenter);
    }
    if (activeAlgorithm == CAMERA_COMMAND_PIECE_MODE &&
        millis() - lastCameraRawDiagnosticMs >= Config::CAMERA_RAW_DIAGNOSTIC_MS) {
      lastCameraRawDiagnosticMs = millis();
      Serial.print(F("[CAMERA PIECE RAW] frame=")); Serial.print(frame);
      Serial.print(F(" objects=")); Serial.print(count);
      Serial.print(F(" id=")); Serial.print(result->rawId);
      Serial.print(F(" name=\"")); Serial.print(result->name);
      Serial.print(F("\" content=\"")); Serial.print(result->content);
      Serial.print(F("\" mapped=")); Serial.print(packet.learnedId);
      Serial.print(F(" class="));
      Serial.print(cameraPieceClassName(static_cast<uint8_t>(packet.learnedId)));
      Serial.print(F(" eligible=")); Serial.print(packet.learnedId != 0 ? 1 : 0);
      Serial.print(F(" pixel=")); Serial.print(result->xCenter);
      Serial.print(','); Serial.println(result->yCenter);
    }
  }
  sendCameraPacket(packet);
}

void serviceCameraWorkerEvents() {
  if (cameraEventQueue == nullptr) return;
  CameraWorkerEvent event{};
  while (xQueueReceive(cameraEventQueue, &event, 0u) == pdTRUE) {
    cameraWorkerBusy = false;
    if (event.generation != cameraGeneration) continue;
    if (event.operation == CameraWorkerOperation::SWITCH_MODE &&
        (cameraState != CameraState::CONNECTING ||
         event.algorithm != requestedAlgorithm)) continue;
    cameraLinkEstablished = event.linkEstablished != 0u;
    lastCameraReadDurationMs = event.durationMs;

    if (event.operation == CameraWorkerOperation::SWITCH_MODE) {
      Serial.print(F("[CAMERA UART] phase="));
      Serial.print(event.failurePhase == 1u ? F("KNOCK") :
                   (event.failurePhase == 2u ? F("SET_ALGORITHM") : F("CONFIRMED")));
      Serial.print(F(" code=")); Serial.print(event.uartCode);
      Serial.print(F(" elapsed_ms=")); Serial.print(event.durationMs);
      Serial.print(F(" phase_ms=")); Serial.print(event.phaseDurationMs);
      Serial.print(F(" algorithm=")); Serial.print(event.algorithm);
      Serial.print(F(" ret="));
      if (event.hasReturnValue) Serial.print(static_cast<int>(event.returnValue));
      else Serial.print(F("NONE"));
      Serial.print(F(" reply_algo/len=")); Serial.print(event.replyAlgorithm);
      Serial.print('/'); Serial.print(event.replyLength);
      Serial.print(F(" ignored=")); Serial.print(event.ignoredFrames);
      Serial.print(F(" checksum_bad=")); Serial.println(event.badChecksums);
      if (event.returnTextLength != 0u) {
        // Preservar los bytes del error, incluso si el formato de strings
        // no coincide. Texto legible acotado, hex completo para diagnostico.
        const char hexDigits[] = "0123456789ABCDEF";
        Serial.print(F("[CAMERA RETURN] payload_hex="));
        for (uint16_t i = 0u; i < event.returnTextLength; ++i) {
          Serial.print(hexDigits[event.returnText[i] >> 4u]);
          Serial.print(hexDigits[event.returnText[i] & 15u]);
        }
        Serial.print(F(" text="));
        uint16_t cursor = 0u;
        for (uint8_t field = 0u; field < 2u && cursor < event.returnTextLength; ++field) {
          const uint8_t length = event.returnText[cursor++];
          if (cursor + length > event.returnTextLength) break;
          Serial.print('"');
          for (uint16_t i = 0u; i < length; ++i) {
            const uint8_t c = event.returnText[cursor++];
            Serial.print(static_cast<char>(c >= 32u && c <= 126u ? c : '.'));
          }
          Serial.print(F("\" "));
        }
        Serial.println();
      }
      const bool provisional = event.failurePhase == 2u &&
        event.uartCode == static_cast<uint8_t>(CameraModeUart::Code::NACK) &&
        event.hasReturnValue != 0u && event.returnValue == 1 &&
        event.linkEstablished != 0u;
      if (event.success != 0u || provisional) {
        activeAlgorithm = event.algorithm;
        cameraModeProvisional = provisional;
        cameraState = CameraState::SETTLING;
        cameraStateStartedMs = millis();
        Serial.print(provisional
          ? F("[CAMERA PROVISIONAL] SET rechazado; comprobar modelo real sin reabrir; algoritmo=")
          : F("[CAMERA] cambio confirmado sin bloquear gamepad; algoritmo="));
        Serial.print(activeAlgorithm); Serial.print(F(" elapsed_ms="));
        Serial.print(event.durationMs); Serial.print(F(" settle_ms="));
        Serial.println(Config::CAMERA_SETTLE_MS);
        sendCameraStatus();
      } else if (cameraConnectAttempts >= Config::CAMERA_CONNECT_ATTEMPTS) {
        // Fallos de enlace o retornos distintos de1 no admiten reconciliacion.
        cameraFault(event.failurePhase == 1u ? CAMERA_FAULT_UART : CAMERA_FAULT_MODE,
          event.uartCode == static_cast<uint8_t>(CameraModeUart::Code::NACK)
            ? F("HUSKYLENS rechazo la orden (NACK); ver ret/fase. OFF y recalibrar.")
            : (event.uartCode == static_cast<uint8_t>(CameraModeUart::Code::TX_SHORT)
              ? F("Escritura UART incompleta; revisar enlace. OFF y recalibrar.")
              : (event.failurePhase == 1u
                ? F("Timeout real KNOCK 5 s; usar OFF y recalibrar.")
                : F("Timeout real de modelo 15 s; usar OFF y recalibrar."))));
      } else {
        lastCameraConnectMs = millis();
        Serial.print(F("[CAMERA RETRY] cambio no confirmado; reconectando, attempt="));
        Serial.print(cameraConnectAttempts + 1u); Serial.print('/');
        Serial.println(Config::CAMERA_CONNECT_ATTEMPTS);
      }
      continue;
    }

    if ((cameraState != CameraState::SETTLING && cameraState != CameraState::READY) ||
        event.algorithm != activeAlgorithm) continue;
    const bool validFrame = event.success != 0u && event.validatedFrame != 0u &&
      event.frameAlgorithm == static_cast<uint8_t>(activeAlgorithm);
    if (cameraState == CameraState::SETTLING) {
      if (millis() - cameraStateStartedMs >= Config::CAMERA_SETTLE_MS +
          Config::CAMERA_FIRST_FRAME_TIMEOUT_MS) {
        cameraFault(CAMERA_FAULT_READ, F("Sin primer cuadro UART valido tras carga; OFF y recalibrar."));
        continue;
      }
      // El modelo puede tardar mas que su ACK. Leer de nuevo, NO reabrirlo.
      if (!validFrame) {
        if (millis() - lastCameraProbeDiagnosticMs >= 1000u) {
          lastCameraProbeDiagnosticMs = millis();
          Serial.print(F("[CAMERA PROBE] code=")); Serial.print(event.uartCode);
          Serial.print(F(" requested/actual=")); Serial.print(activeAlgorithm);
          Serial.print('/'); Serial.print(event.frameAlgorithm);
          Serial.print(F(" checksum_bad=")); Serial.println(event.badChecksums);
        }
        continue;
      }
      cameraState = CameraState::READY;
      if (cameraModeProvisional)
        Serial.println(F("[CAMERA RECOVERED] modelo real y cuadro completo confirmados tras NACK."));
      cameraModeProvisional = false;
      consecutiveReadFailures = 0u;
      sendCameraStatus(CAMERA_TYPE_ACK, pendingCommandValue);
      Serial.print(F("[CAMERA] algoritmo listo con lectura UART="));
      Serial.println(activeAlgorithm);
    }
    if (!validFrame) {
      ++consecutiveReadFailures;
      if (consecutiveReadFailures >= Config::CAMERA_READ_FAILURES) {
        cameraFault(CAMERA_FAULT_READ, F("Tres lecturas UART consecutivas fallaron."));
      }
      continue;
    }

    consecutiveReadFailures = 0u;
    const uint32_t frame = cameraFrameSequence++;
    if (cameraFrameSequence == 0u) cameraFrameSequence = 1u;
    if (event.count == 0u) {
      sendCameraFrame(frame, 0u, 0u, nullptr, event.durationMs);
    } else {
      for (uint8_t index = 0u; index < event.count; ++index) {
        sendCameraFrame(frame, index, event.count,
                        &event.observations[index], event.durationMs);
      }
    }
  }
}

void serviceCameraRead() {
  const uint32_t now = millis();
  if (cameraState == CameraState::SETTLING) {
    if (now - cameraStateStartedMs < Config::CAMERA_SETTLE_MS) return;
    if (now - cameraStateStartedMs >= Config::CAMERA_SETTLE_MS +
        Config::CAMERA_FIRST_FRAME_TIMEOUT_MS) {
      cameraFault(CAMERA_FAULT_READ, F("Sin primer cuadro UART valido tras carga; OFF y recalibrar."));
      return;
    }
  }
  if ((cameraState != CameraState::READY && cameraState != CameraState::SETTLING) ||
      cameraWorkerBusy ||
      now - lastCameraReadMs < Config::CAMERA_READ_PERIOD_MS) return;
  lastCameraReadMs = now;
  CameraWorkerRequest request{};
  request.operation = CameraWorkerOperation::READ_FRAME;
  request.generation = cameraGeneration;
  request.algorithm = activeAlgorithm;
  if (cameraRequestQueue != nullptr &&
      xQueueSend(cameraRequestQueue, &request, 0u) == pdTRUE) {
    cameraWorkerBusy = true;
  }
}

void handleCameraCommand(const CameraPacket &packet) {
  if (packet.type != CAMERA_TYPE_COMMAND) return;
  if (commandSession != packet.session) {
    commandSession = packet.session;
    lastCommandSequence = packet.sequence;
  } else if (!sequenceIsNewer(packet.sequence, lastCommandSequence)) {
    ++badCameraCommands;
    return;
  } else {
    lastCommandSequence = packet.sequence;
  }
  switch (packet.value) {
    case CAMERA_COMMAND_OFF:
      /* Un reintento de la misma orden OFF no debe reinicializar el estado ni
         llenar el monitor; siempre se confirma para conservar idempotencia. */
      if (cameraState == CameraState::OFF && requestedAlgorithm < 0) {
        sendCameraStatus(CAMERA_TYPE_ACK, CAMERA_COMMAND_OFF);
      } else {
        cameraOff();
      }
      break;
    case CAMERA_COMMAND_TAG_MODE:
    case CAMERA_COMMAND_PIECE_MODE:
      /* FAULT queda enclavado para la misma orden: UDP repetido no reabre.
         La recalibracion explicita en Portenta envia OFF antes del nuevo TAG. */
      if (requestedAlgorithm == static_cast<int16_t>(packet.value) &&
          cameraState == CameraState::FAULT) {
        sendCameraStatus(CAMERA_TYPE_FAULT, cameraLastFault);
        break;
      }
      if (requestedAlgorithm == static_cast<int16_t>(packet.value) &&
          (cameraState == CameraState::CONNECTING ||
           cameraState == CameraState::SETTLING ||
           cameraState == CameraState::READY)) {
        sendCameraStatus(cameraState == CameraState::READY
          ? CAMERA_TYPE_ACK : CAMERA_TYPE_STATUS, packet.value);
      } else {
        requestCameraAlgorithm(packet.value);
      }
      break;
    case CAMERA_COMMAND_STATUS: sendCameraStatus(); break;
    default:
      ++badCameraCommands;
      sendCameraStatus(CAMERA_TYPE_FAULT, CAMERA_FAULT_BAD_COMMAND);
      break;
  }
}

void serviceInboundPackets() {
  int packetSize = udp.parsePacket();
  while (packetSize > 0) {
    if (packetSize == static_cast<int>(sizeof(CameraPacket))) {
      CameraPacket packet{};
      const int read = udp.read(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      if (read == static_cast<int>(sizeof(packet)) && cameraPacketValid(&packet)) {
        handleCameraCommand(packet);
      } else ++badCameraCommands;
    } else {
      while (udp.available() > 0) udp.read();
      ++badCameraCommands;
    }
    packetSize = udp.parsePacket();
  }
}

void serviceWifiLatency() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiLatencyVerified = wifiLatencyAttempted = false;
    wifiSleepMode = -1;
    return;
  }
  const uint32_t now = millis();
  if (wifiLatencyVerified || (wifiLatencyAttempted &&
      now-wifiLatencyLastAttemptMs < Config::WIFI_LATENCY_RETRY_MS)) return;
  wifiLatencyAttempted = true;
  wifiLatencyLastAttemptMs = now;
  wifi_ps_type_t mode = WIFI_PS_MIN_MODEM;
  esp_err_t error = esp_wifi_get_ps(&mode);
  // This Bluepad32 SDK aborts if modem sleep is disabled with Bluetooth on.
  // Keep the compatible minimum-modem policy, including after reconnection.
  if (error == ESP_OK && mode != WIFI_PS_MIN_MODEM) {
    error = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    if (error == ESP_OK) error = esp_wifi_get_ps(&mode);
  }
  wifiSleepMode = error == ESP_OK ? static_cast<int32_t>(mode) : -1;
  wifiLatencyError = static_cast<int32_t>(error);
  wifiLatencyVerified = error == ESP_OK && mode == WIFI_PS_MIN_MODEM;
  Serial.print(F("[WIFI COEX] verified=")); Serial.print(wifiLatencyVerified);
  Serial.print(F(" ps=")); Serial.print(wifiSleepMode);
  Serial.print(F(" error=")); Serial.println(wifiLatencyError);
}

void serviceWifi() {
  serviceWifiLatency();
  const wl_status_t status = WiFi.status();
  const uint32_t now = millis();
  if (status == WL_CONNECTED) {
    if (!udpBound && now - lastUdpBindAttemptMs >= Config::UDP_BIND_RETRY_MS) {
      lastUdpBindAttemptMs = now;
      udp.stop();
      udpBound = udp.begin(Config::ESP_UDP_PORT) == 1;
      if (udpBound) udpSendFailureStreak = 0u;
      // Failed binds retry at25ms without filling Serial or blocking the loop.
    }
    return;
  }
  if (udpBound) udp.stop();
  udpBound = false;
  pendingButtonEdges = 0u;
  pendingMiscEdges = 0u;
  pendingDpadEdges = 0u;
  if (now - lastWifiAttemptMs < Config::WIFI_RETRY_PERIOD_MS) return;
  lastWifiAttemptMs = now;
  /* No abortar una asociación en curso con disconnect()+begin(). Bluepad32 y
     Wi-Fi comparten el radio; reducir churn evita castigar la pila BTstack. */
  if (status == WL_IDLE_STATUS) return;
  Serial.println(F("[WIFI] reintentando DELTA_PORTENTA sin reiniciar la interfaz..."));
  WiFi.reconnect();
}

void serviceGamepadPackets() {
  GamepadPacket packet = currentGamepadPacket();
  const uint16_t newButtons = packet.buttons & ~observedButtons;
  const uint16_t newMisc = packet.misc & ~observedMisc;
  const uint8_t newDpad = packet.dpad & ~observedDpad;
  observedButtons = packet.buttons;
  observedMisc = packet.misc;
  observedDpad = packet.dpad;
  if (WiFi.status() != WL_CONNECTED || !udpBound) {
    // Never replay menu/ARM presses that occurred while transport was down.
    pendingButtonEdges = pendingMiscEdges = 0u;
    pendingDpadEdges = 0u;
    return;
  }
  const uint32_t now = millis();
  if (now - pendingEdgesSinceMs > Config::GAMEPAD_EDGE_MAX_AGE_MS) {
    pendingButtonEdges = pendingMiscEdges = 0u;
    pendingDpadEdges = 0u;
  }
  if ((packet.flags & GAMEPAD_FLAG_CONNECTED) == 0u) {
    pendingButtonEdges = 0u;
    pendingMiscEdges = 0u;
    pendingDpadEdges = 0u;
  } else {
    /* Retain only NEW rising edges until a local send succeeds. Retaining a
       held level each loop delayed releases and could merge two X presses.
       UDP local success is not confirmation of reception by Portenta. */
    if (!(pendingButtonEdges || pendingMiscEdges || pendingDpadEdges) &&
        (newButtons || newMisc || newDpad)) pendingEdgesSinceMs = now;
    pendingButtonEdges |= newButtons;
    pendingMiscEdges |= newMisc;
    pendingDpadEdges |= newDpad;
    packet.buttons |= pendingButtonEdges;
    packet.misc |= pendingMiscEdges;
    packet.dpad |= pendingDpadEdges;
  }
  const bool changed = !haveLastPacket ||
                       gamepadPacketChanged(packet, lastSentPacket);
  const bool buttonChanged = !haveLastPacket ||
      packet.flags != lastSentPacket.flags || packet.buttons != lastSentPacket.buttons ||
      packet.misc != lastSentPacket.misc || packet.dpad != lastSentPacket.dpad;
  // Preserve25ms failure backoff, not a retry on every1ms loop or catch-up burst.
  if (lastGamepadAttemptFailed &&
      now - lastGamepadAttemptMs < Config::GAMEPAD_SEND_RETRY_MS) return;
  if ((buttonChanged && now - lastGamepadSendMs >= Config::BUTTON_SEND_MIN_PERIOD_MS) ||
      (changed && now - lastGamepadSendMs >= Config::SEND_MIN_PERIOD_MS) ||
      now - lastGamepadSendMs >= Config::GAMEPAD_HEARTBEAT_MS) {
    lastGamepadAttemptMs = now;
    if (sendGamepadPacket(packet)) {
      pendingButtonEdges = 0u;
      pendingMiscEdges = 0u;
      pendingDpadEdges = 0u;
    }
  }
}

void printStatus() {
  const GamepadPacket packet = currentGamepadPacket();
  ControllerPtr controller = controllerSnapshot();
  const uint16_t rawButtons = controller != nullptr && controller->isConnected()
    ? controller->buttons() : 0u;
  const uint16_t rawMisc = controller != nullptr && controller->isConnected()
    ? controller->miscButtons() : 0u;
  const uint8_t rawDpad = controller != nullptr && controller->isConnected()
    ? static_cast<uint8_t>(controller->dpad()) : 0u;
  Serial.print(F("[STATUS] stage=10A uptime_ms=")); Serial.print(millis());
  Serial.print(F(" wifi=")); Serial.print(WiFi.status() == WL_CONNECTED ? F("ON") : F("OFF"));
  Serial.print(F(" ip=")); Serial.print(WiFi.localIP());
  Serial.print(F(" gamepad="));
  Serial.print((packet.flags & GAMEPAD_FLAG_CONNECTED) != 0u ? F("ON") : F("OFF"));
  Serial.print(F(" profile="));
  Serial.print(dualShock4Connected ? F("DUALSHOCK4") : F("NONE"));
  Serial.print(F(" pairing=")); Serial.print(pairingOpen ? F("OPEN") : F("CLOSED"));
  Serial.print(F(" bt acc/rej/disc=")); Serial.print(acceptedControllerCount);
  Serial.print('/'); Serial.print(rejectedControllerCount);
  Serial.print('/'); Serial.print(controllerDisconnectCount);
  ControllerPtr pending = pendingControllerSnapshot();
  Serial.print(F(" bt_pending=")); Serial.print(pending != nullptr ? 1 : 0);
  Serial.print(F(" bt_wait_ms="));
  Serial.print(pending != nullptr ? millis() - pendingControllerStartedMs : 0u);
  Serial.print(F(" bt_candidates/timeouts=")); Serial.print(controllerCandidateCount);
  Serial.print('/'); Serial.print(controllerFirstReportTimeoutCount);
  Serial.print(F(" bt_last_reject=")); Serial.print(lastControllerRejectReason);
  Serial.print(F(" camera=")); Serial.print(cameraStateName());
  Serial.print(F(" alg=")); Serial.print(activeAlgorithm);
  Serial.print(F(" read_ms=")); Serial.print(lastCameraReadDurationMs);
  Serial.print(F(" gp/cam/fail=")); Serial.print(sentGamepadPackets);
  Serial.print('/'); Serial.print(sentCameraPackets); Serial.print('/'); Serial.print(failedPackets);
  Serial.print(F(" gp_fail/streak/gap_max_ms=")); Serial.print(gamepadSendFailures);
  Serial.print('/'); Serial.print(consecutiveGamepadFailures); Serial.print('/'); Serial.print(gamepadSendGapMaxMs);
  Serial.print(F(" udp_bound/recover/begin_err/end_err=")); Serial.print(udpBound);
  Serial.print('/'); Serial.print(udpRecoveries); Serial.print('/'); Serial.print(udpBeginErrors);
  Serial.print('/'); Serial.print(udpWriteEndErrors);
  Serial.print(F(" udp_last_phase/errno/at_ms/dur_us=")); Serial.print(udpLastFailurePhase);
  Serial.print('/'); Serial.print(udpLastFailureErrno);
  Serial.print('/'); Serial.print(udpLastFailureMs);
  Serial.print('/'); Serial.print(udpLastFailureDurationUs);
  Serial.print(F(" udp_streak/tx_max_us=")); Serial.print(udpSendFailureStreak);
  Serial.print('/'); Serial.print(udpSendDurationMaxUs);
  Serial.print(F(" gp_last_fail_seq/gap_ms=")); Serial.print(lastGamepadFailedSequence);
  Serial.print('/'); Serial.print(gamepadSendGapLastMs);
  Serial.print(F(" gp_last_tx_seq=")); Serial.print(haveLastPacket ? lastSentPacket.sequence : 0u);
  Serial.print(F(" rssi_dbm=")); Serial.print(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  Serial.print(F(" wifi_ps/verified/error=")); Serial.print(wifiSleepMode);
  Serial.print('/'); Serial.print(wifiLatencyVerified);
  Serial.print('/'); Serial.print(wifiLatencyError);
  Serial.print(F(" loop_max_us=")); Serial.print(espLoopMaxUs);
  Serial.print(F(" bad_cmd=")); Serial.print(badCameraCommands);
  Serial.print(F(" axes=")); Serial.print(packet.lx); Serial.print(',');
  Serial.print(packet.ly); Serial.print(','); Serial.print(packet.rx); Serial.print(','); Serial.print(packet.ry);
  Serial.print(F(" raw_btn/misc/dpad=0x")); Serial.print(rawButtons, HEX);
  Serial.print(F("/0x")); Serial.print(rawMisc, HEX);
  Serial.print(F("/0x")); Serial.print(rawDpad, HEX);
  Serial.print(F(" out_btn/misc/dpad=0x")); Serial.print(packet.buttons, HEX);
  Serial.print(F("/0x")); Serial.print(packet.misc, HEX);
  Serial.print(F("/0x")); Serial.print(packet.dpad, HEX);
  Serial.print(F(" trigger_L2_R2="));
  Serial.print(controller != nullptr && controller->isConnected() ? controller->brake() : 0);
  Serial.print('/');
  Serial.print(controller != nullptr && controller->isConnected() ? controller->throttle() : 0);
  Serial.print(F(" heap_free=")); Serial.println(ESP.getFreeHeap());
}

void printHelp() {
  Serial.println(F("Comandos USB: STATUS | OPEN_PAIRING | CLEAR_KEYS | CAMERA_OFF | TAG | PIECE | HELP"));
  Serial.println(F("Solo DualShock 4: OPTIONS=menu/STOP, D-pad=navegar, X=confirmar, CIRCULO=cancelar."));
  Serial.println(F("DS4 espera primer reporte valido; luego cierra busqueda y rechaza segundos mandos. Sin filtro MAC."));
  Serial.println(F("TAG/PIECE son diagnosticos; el uso normal lo ordena la Portenta."));
}

void processUsbLine(char *line) {
  for (char *cursor = line; *cursor != '\0'; ++cursor) {
    *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
  }
  if (strcmp(line, "STATUS") == 0) printStatus();
  else if (strcmp(line, "OPEN_PAIRING") == 0) {
    if (controllerSlotReserved()) {
      Serial.println(F("[USB BLOCKED] Ya existe un mando activo o esperando primer reporte."));
    } else {
      BP32.enableNewBluetoothConnections(true);
      pairingOpen = true;
      Serial.println(F("[USB] Pairing abierto hasta conectar el primer mando."));
    }
  }
  else if (strcmp(line, "CLEAR_KEYS") == 0) {
    if (controllerSlotReserved()) {
      Serial.println(F("[USB BLOCKED] Apaga/desconecta el mando primero."));
    } else {
      BP32.forgetBluetoothKeys();
      BP32.enableNewBluetoothConnections(true);
      pairingOpen = true;
      Serial.println(F("[USB] Claves eliminadas; pairing abierto para un solo mando."));
    }
  } else if (strcmp(line, "CAMERA_OFF") == 0) cameraOff();
  else if (strcmp(line, "TAG") == 0) requestCameraAlgorithm(CAMERA_COMMAND_TAG_MODE);
  else if (strcmp(line, "PIECE") == 0) requestCameraAlgorithm(CAMERA_COMMAND_PIECE_MODE);
  else if (strcmp(line, "HELP") == 0) printHelp();
  else Serial.println(F("[USB ERROR] Comando desconocido."));
}

void readUsb() {
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\r' || value == '\n') {
      if (usbLineLength > 0u) {
        usbLine[usbLineLength] = '\0';
        processUsbLine(usbLine);
        usbLineLength = 0u;
      }
    } else if (value >= 32 && value <= 126 &&
               usbLineLength + 1u < Config::USB_LINE_CAPACITY) {
      usbLine[usbLineLength++] = value;
    }
  }
}

void setup() {
  Serial.begin(Config::USB_BAUD);
  delay(1000u);
  Serial.println(F("ESP32 10A v2.15 - GAMEPAD UDP50HZ + WIFI/BT COEX"));
  sessionId = esp_random();
  if (sessionId == 0u) sessionId = 1u;
  Serial.print(F("[BOOT] esp_session=0x")); Serial.println(sessionId, HEX);
  huskySerial.begin(Config::HUSKY_BAUD, SERIAL_8N1,
                    Config::HUSKY_RX_PIN, Config::HUSKY_TX_PIN);
  /* La biblioteca espera hasta 5 s por intento y por defecto repite cinco
     veces. Una sola tentativa por trabajo limita la latencia; los reintentos
     y la reconexion se administran explicitamente sin detener Bluepad32. */
  huskylens.retry = 1;
  cameraRequestQueue = xQueueCreate(2u, sizeof(CameraWorkerRequest));
  cameraEventQueue = xQueueCreate(2u, sizeof(CameraWorkerEvent));
  if (cameraRequestQueue != nullptr && cameraEventQueue != nullptr) {
    const BaseType_t taskCreated = xTaskCreatePinnedToCore(
      cameraWorkerTask, "husky_uart", Config::CAMERA_WORKER_STACK_SIZE,
      nullptr, tskIDLE_PRIORITY, &cameraWorkerHandle, 1u);
    if (taskCreated != pdPASS) {
      vQueueDelete(cameraRequestQueue);
      vQueueDelete(cameraEventQueue);
      cameraRequestQueue = nullptr;
      cameraEventQueue = nullptr;
      cameraWorkerHandle = nullptr;
    }
  }
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("ESP32 10A - DUALSHOCK 4 + HUSKYLENS UART/UDP"));
  Serial.println(F("GAMEPAD ESTADO UDP50 Hz; DELTA20 Hz; CAMARA10 Hz SOLO BAJO ORDEN"));
  Serial.println(F("================================================"));
  Serial.print(F("[BOOT] reset_reason=")); Serial.println(resetReasonName(esp_reset_reason()));
  printHelp();
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
  BP32.enableNewBluetoothConnections(true);
  pairingOpen = true;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  // Cache this profile before association; the core reapplies it on STA_START.
  // false would request WIFI_PS_NONE and PANIC with this Bluepad32 SDK.
  // The return value can also be false when already applied; verify on connect.
  WiFi.setSleep(true);
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  lastWifiAttemptMs = millis();
}

void loop() {
  const uint32_t loopStartedUs = micros();
  BP32.update();
  serviceControllerLifecycle();
  serviceWifi();
  if (udpBound) serviceInboundPackets();
  serviceGamepadPackets();
  serviceCameraWorkerEvents();
  serviceCameraConnection();
  serviceCameraRead();
  if (cameraState != CameraState::OFF && udpBound &&
      millis() - lastCameraStatusMs >= Config::CAMERA_STATUS_PERIOD_MS) {
    sendCameraStatus();
  }
  readUsb();
  if (millis() - lastStatusMs >= Config::STATUS_PERIOD_MS) {
    lastStatusMs = millis();
    printStatus();
  }
  const uint32_t loopElapsedUs = micros() - loopStartedUs;
  if (loopElapsedUs > espLoopMaxUs) espLoopMaxUs = loopElapsedUs;
  delay(1u);
}
