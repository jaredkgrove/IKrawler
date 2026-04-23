#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <Wire.h>

#include "DualBoardServo.h"
#include "Hexapod.h"
#include "web_ui.h"

// ── Configuration ──
const char *AP_SSID = "IKrawler";
const char *AP_PASS = "hexapod123";

// ── Components ──
DualBoardServo dualBoardServos;
Hexapod hexapod(&dualBoardServos);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

unsigned long lastUpdateMs = 0;

// ── Robot mode ──
// Tracks the last user-commanded high-level state. Used to gate the
// identify command: only safe in STAND because stand() gives known,
// neutral joint angles and leaves the gait idle.
enum RobotMode { MODE_NONE, MODE_STAND, MODE_WALK, MODE_REST, MODE_STOPPING };
RobotMode currentMode = MODE_NONE;

// ── Leg identification ──
int identifyLeg = -1;
unsigned long identifyStartMs = 0;
float identifyOriginalFemur = 0.0f;
float identifyOriginalCoxa = 0.0f;
float identifyTargetFemur = 0.0f;
float identifyTargetCoxa = 0.0f;

// Safe envelope (degrees) for the angles identify is allowed to drive the
// coxa/femur to. Picked to stay well clear of the servos' mechanical stops
// so a stalled servo can't sit drawing stall current through the PCA9685.
// From STAND (coxa=90, femur=75) the ±30° wiggle lands at 120/45, both
// comfortably inside.
constexpr float IDENTIFY_FEMUR_MIN = 30.0f;
constexpr float IDENTIFY_FEMUR_MAX = 150.0f;
constexpr float IDENTIFY_COXA_MIN = 45.0f;
constexpr float IDENTIFY_COXA_MAX = 135.0f;
constexpr unsigned long IDENTIFY_DURATION_MS = 500;

// ── Single-joint sweep test ──
// Drives one joint from its current angle to +amplitude, then -amplitude,
// then back. Used to verify per-joint inversion: the serial log prints the
// expected physical direction for each phase; the operator watches the leg
// and reports any joint that moved the wrong way.
int sweepLeg = -1;
int sweepJoint = 0;
unsigned long sweepStartMs = 0;
float sweepBaseAngle = 0.0f;
int sweepLastPhase = -1;

constexpr float SWEEP_AMPLITUDE_DEG = 15.0f;
// Hard envelope that both +/- targets are clamped into. Well inside 0/180
// so no inversion + base-angle combo can drive a servo into its stop.
constexpr float SWEEP_ANGLE_MIN = 15.0f;
constexpr float SWEEP_ANGLE_MAX = 175.0f;
constexpr unsigned long SWEEP_PHASE_MS = 1500;
constexpr unsigned long SWEEP_RETURN_MS = 500;
constexpr unsigned long SWEEP_TOTAL_MS = 2 * SWEEP_PHASE_MS + SWEEP_RETURN_MS;

const char *const SWEEP_LEG_NAMES[6] = {"FR", "MR", "RR", "RL", "ML", "FL"};
const char *const SWEEP_JOINT_NAMES[3] = {"COXA", "FEMUR", "TIBIA"};

// Send current calibration offsets to all connected clients.
void broadcastCalState() {
  JsonDocument doc;
  doc["type"] = "cal_state";
  JsonArray arr = doc["offsets"].to<JsonArray>();
  for (int i = 0; i < DualBoardServo::NUM_SERVOS; i++) {
    arr.add(dualBoardServos.getOffset(i));
  }
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// Restore any active identify leg to its original pose and clear the state.
// Called before state transitions so the per-tick re-assert can't fight a
// newly-commanded gait/rest pose.
void cancelIdentify() {
  if (identifyLeg != -1) {
    hexapod.setServo(identifyLeg, Hexapod::FEMUR, identifyOriginalFemur);
    hexapod.setServo(identifyLeg, Hexapod::COXA, identifyOriginalCoxa);
    identifyLeg = -1;
  }
}

// Restore the joint under test to its base angle and clear sweep state.
void cancelSweep() {
  if (sweepLeg != -1) {
    hexapod.setServo(sweepLeg, sweepJoint, sweepBaseAngle);
    sweepLeg = -1;
    sweepLastPhase = -1;
  }
}

float clampSweepAngle(float a) {
  if (a < SWEEP_ANGLE_MIN)
    return SWEEP_ANGLE_MIN;
  if (a > SWEEP_ANGLE_MAX)
    return SWEEP_ANGLE_MAX;
  return a;
}

// Print the expected physical direction for the current sweep phase so the
// operator can compare against what the leg actually does.
void logSweepPhase(int phase, float target) {
  const char *legName = SWEEP_LEG_NAMES[sweepLeg];
  const char *jointName = SWEEP_JOINT_NAMES[sweepJoint];
  bool isRightSide = (sweepLeg < 3);
  const char *hint = "";

  if (phase == 0) { // +amplitude
    switch (sweepJoint) {
    case Hexapod::COXA:
      hint = isRightSide ? "expect foot swings FORWARD"
                         : "expect foot swings REARWARD";
      break;
    case Hexapod::FEMUR:
      hint = "expect thigh rotates DOWN (knee drops)";
      break;
    case Hexapod::TIBIA:
      hint = "expect knee BENDS MORE (foot curls IN)";
      break;
    }
  } else if (phase == 1) { // -amplitude
    switch (sweepJoint) {
    case Hexapod::COXA:
      hint = isRightSide ? "expect foot swings REARWARD"
                         : "expect foot swings FORWARD";
      break;
    case Hexapod::FEMUR:
      hint = "expect thigh rotates UP (knee lifts)";
      break;
    case Hexapod::TIBIA:
      hint = "expect knee STRAIGHTENS (foot pushes OUT)";
      break;
    }
  } else {
    hint = "returning to neutral";
  }

  Serial.printf("[TEST] Leg %d (%s) %s -> %.1f deg (%s)\n", sweepLeg, legName,
                jointName, target, hint);
}

// Re-issue the last commanded angle for a single servo so the new offset
// takes effect immediately (no need to wait for the next gait tick).
void refreshServo(int servoId) {
  if (servoId < 0 || servoId >= DualBoardServo::NUM_SERVOS)
    return;
  int leg = servoId / 3;
  int joint = servoId % 3;
  float current = hexapod.getServo(leg, joint);
  hexapod.setServo(leg, joint, current);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(),
                  client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, data, len);
      if (err) {
        Serial.printf("JSON parse failed: %s\n", err.c_str());
        return;
      }

      const char *msgType = doc["type"];
      if (!msgType)
        return;

      if (strcmp(msgType, "action") == 0) {
        const char *cmd = doc["cmd"];
        if (!cmd)
          return;

        Serial.printf("Action: %s\n", cmd);
        if (strcmp(cmd, "stand") == 0) {
          cancelIdentify();
          cancelSweep();
          hexapod.stand();
          currentMode = MODE_STAND;
        } else if (strcmp(cmd, "walk") == 0) {
          cancelIdentify();
          cancelSweep();
          hexapod.walk();
          currentMode = MODE_WALK;
        } else if (strcmp(cmd, "rest") == 0) {
          cancelIdentify();
          cancelSweep();
          hexapod.rest();
          currentMode = MODE_REST;
        } else if (strcmp(cmd, "stop") == 0) {
          cancelIdentify();
          cancelSweep();
          hexapod.stop();
          // stop() transitions through STOPPING and eventually calls stand();
          // hold the gate closed against identify until the user explicitly
          // presses STAND again.
          currentMode = MODE_STOPPING;
        } else if (strncmp(cmd, "identify_", 9) == 0) {
          // Refuse if identify is already in progress. Otherwise a second
          // press would cache the already-displaced angles as "original"
          // and each subsequent press would drift the leg further until a
          // servo stalled against its mechanical stop — a known burnout
          // path for the PCA9685.
          if (identifyLeg != -1) {
            Serial.println("Identify ignored: already active");
            return;
          }
          if (sweepLeg != -1) {
            Serial.println("Identify ignored: joint test active");
            return;
          }
          // Only safe in STAND. STAND gives known, neutral joint angles
          // (coxa=90, femur=75) so ±30° stays well inside the safe
          // envelope, and gait updates are idle so the pose isn't fought.
          if (currentMode != MODE_STAND) {
            Serial.println("Identify ignored: robot not in STAND");
            return;
          }
          int leg = atoi(cmd + 9);
          if (leg >= 0 && leg < 6) {
            identifyOriginalFemur = hexapod.getServo(leg, Hexapod::FEMUR);
            identifyOriginalCoxa = hexapod.getServo(leg, Hexapod::COXA);

            // Clamp target angles to the safe envelope before writing,
            // so even an unexpected starting pose can't push a servo
            // against its stop.
            float femurTarget = identifyOriginalFemur - 30.0f;
            float coxaTarget = identifyOriginalCoxa + 30.0f;
            if (femurTarget < IDENTIFY_FEMUR_MIN)
              femurTarget = IDENTIFY_FEMUR_MIN;
            if (femurTarget > IDENTIFY_FEMUR_MAX)
              femurTarget = IDENTIFY_FEMUR_MAX;
            if (coxaTarget < IDENTIFY_COXA_MIN)
              coxaTarget = IDENTIFY_COXA_MIN;
            if (coxaTarget > IDENTIFY_COXA_MAX)
              coxaTarget = IDENTIFY_COXA_MAX;

            identifyTargetFemur = femurTarget;
            identifyTargetCoxa = coxaTarget;
            identifyLeg = leg;
            identifyStartMs = millis();

            hexapod.setServo(leg, Hexapod::FEMUR, femurTarget);
            hexapod.setServo(leg, Hexapod::COXA, coxaTarget);
          }
        } else if (strncmp(cmd, "test_", 5) == 0) {
          // Format: test_<leg>_<joint>, e.g. test_0_1 = FR femur.
          // Sweeps a single joint +amp / -amp / back so the operator can
          // watch a known joint move and catch inverted directions.
          if (sweepLeg != -1) {
            Serial.println("Test ignored: already active");
            return;
          }
          if (identifyLeg != -1) {
            Serial.println("Test ignored: identify active");
            return;
          }
          if (currentMode != MODE_STAND) {
            Serial.println("Test ignored: robot not in STAND");
            return;
          }
          const char *p = cmd + 5;
          int leg = atoi(p);
          const char *us = strchr(p, '_');
          if (!us) {
            Serial.println("Test ignored: malformed cmd");
            return;
          }
          int joint = atoi(us + 1);
          if (leg < 0 || leg >= 6 || joint < 0 || joint >= 3) {
            Serial.println("Test ignored: bad leg/joint");
            return;
          }
          sweepLeg = leg;
          sweepJoint = joint;
          sweepBaseAngle = hexapod.getServo(leg, joint);
          sweepStartMs = millis();
          sweepLastPhase = -1;
          Serial.printf("[TEST] Starting Leg %d (%s) %s, base=%.1f\n", leg,
                        SWEEP_LEG_NAMES[leg], SWEEP_JOINT_NAMES[joint],
                        sweepBaseAngle);
        }
      } else if (strcmp(msgType, "move") == 0) {
        float heading = doc["heading"] | 0.0f;
        float turn = doc["turn"] | 0.0f;
        float stride = doc["stride"] | 0.0f;

        hexapod.setHeading(heading);
        hexapod.setTurnRate(turn);
        hexapod.setStrideLength(stride);
      } else if (strcmp(msgType, "cal_get") == 0) {
        broadcastCalState();
      } else if (strcmp(msgType, "cal_set") == 0) {
        int servo = doc["servo"] | -1;
        float offset = doc["offset"] | 0.0f;
        if (servo >= 0 && servo < DualBoardServo::NUM_SERVOS) {
          dualBoardServos.setOffset(servo, offset);
          refreshServo(servo);
          broadcastCalState();
        }
      } else if (strcmp(msgType, "cal_save") == 0) {
        bool ok = dualBoardServos.saveOffsets();
        JsonDocument reply;
        reply["type"] = "cal_saved";
        reply["ok"] = ok;
        String out;
        serializeJson(reply, out);
        ws.textAll(out);
      } else if (strcmp(msgType, "cal_reset") == 0) {
        cancelIdentify();
        cancelSweep();
        dualBoardServos.resetOffsets();
        hexapod.stand(); // re-apply neutral pose with zeroed offsets
        currentMode = MODE_STAND;
        broadcastCalState();
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- IKrawler Web Controller ---");

  // 1. Initialize servos and hexapod
  Serial.println("Initializing servos...");
  if (!dualBoardServos.begin()) {
    // Probe each address directly so the operator knows which board is dead
    // before any servo is driven blind.
    Wire.beginTransmission(0x40);
    bool board1Ok = (Wire.endTransmission() == 0);
    Wire.beginTransmission(0x41);
    bool board2Ok = (Wire.endTransmission() == 0);
    Serial.printf("ERROR: PCA9685 init failed (0x40=%s, 0x41=%s). "
                  "Halting before driving servos.\n",
                  board1Ok ? "ok" : "no ACK", board2Ok ? "ok" : "no ACK");
    while (1)
      delay(10);
  }

  // Not used directly by our DualBoardServo adapter (it hardcodes the
  // channels), but Hexapod.begin() requires it and sets servos to neutral.
  int dummyPins[18] = {0};
  hexapod.begin(dummyPins);
  hexapod.stand();
  currentMode = MODE_STAND;
  Serial.println("Hexapod standing by.");

  // 2. Setup WiFi AP
  Serial.printf("Starting WiFi AP: %s\n", AP_SSID);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false); // prevent scanning/sleep latency
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(IP);

  // 3. Setup Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", web_ui_html_start);
  });

  // 4. Setup WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.begin();
  Serial.println("HTTP & WebSocket server started.");

  lastUpdateMs = millis();
}

void loop() {
  // Clean up disconnected WebSocket clients
  ws.cleanupClients();

  // Update Hexapod kinematics
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  lastUpdateMs = now;

  hexapod.update(dt);

  // Drive the identify leg: re-assert the target pose each tick so nothing
  // else can clobber it, and restore the original angles when the window
  // expires. (With the STAND guard the gait is IDLE and won't fight us,
  // but this keeps the leg pinned regardless.)
  if (identifyLeg != -1) {
    if (millis() - identifyStartMs > IDENTIFY_DURATION_MS) {
      hexapod.setServo(identifyLeg, Hexapod::FEMUR, identifyOriginalFemur);
      hexapod.setServo(identifyLeg, Hexapod::COXA, identifyOriginalCoxa);
      identifyLeg = -1;
    } else {
      hexapod.setServo(identifyLeg, Hexapod::FEMUR, identifyTargetFemur);
      hexapod.setServo(identifyLeg, Hexapod::COXA, identifyTargetCoxa);
    }
  }

  // Drive the single-joint sweep: step through +amp / -amp / return phases
  // and re-assert the target each tick. Logs on phase transitions so the
  // operator sees exactly what direction is being tested.
  if (sweepLeg != -1) {
    unsigned long elapsed = millis() - sweepStartMs;
    int phase;
    float target;
    if (elapsed < SWEEP_PHASE_MS) {
      phase = 0;
      target = clampSweepAngle(sweepBaseAngle + SWEEP_AMPLITUDE_DEG);
    } else if (elapsed < 2 * SWEEP_PHASE_MS) {
      phase = 1;
      target = clampSweepAngle(sweepBaseAngle - SWEEP_AMPLITUDE_DEG);
    } else if (elapsed < SWEEP_TOTAL_MS) {
      phase = 2;
      target = sweepBaseAngle;
    } else {
      hexapod.setServo(sweepLeg, sweepJoint, sweepBaseAngle);
      Serial.printf("[TEST] Leg %d %s complete\n", sweepLeg,
                    SWEEP_JOINT_NAMES[sweepJoint]);
      sweepLeg = -1;
      sweepLastPhase = -1;
      phase = -1;
      target = sweepBaseAngle;
    }
    if (sweepLeg != -1) {
      if (phase != sweepLastPhase) {
        logSweepPhase(phase, target);
        sweepLastPhase = phase;
      }
      hexapod.setServo(sweepLeg, sweepJoint, target);
    }
  }

  // Minimal delay to prevent busy-looping and let WiFi tasks run
  delay(5);
}
