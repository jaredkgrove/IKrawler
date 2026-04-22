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

// Leg Identification State
int identifyLeg = -1;
unsigned long identifyStartMs = 0;
float identifyOriginalFemur = 0.0f;
float identifyOriginalCoxa = 0.0f;

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
        if (strcmp(cmd, "stand") == 0)
          hexapod.stand();
        else if (strcmp(cmd, "walk") == 0)
          hexapod.walk();
        else if (strcmp(cmd, "rest") == 0)
          hexapod.rest();
        else if (strcmp(cmd, "stop") == 0)
          hexapod.stop();
        else if (strncmp(cmd, "identify_", 9) == 0) {
          int leg = atoi(cmd + 9);
          if (leg >= 0 && leg < 6) {
            identifyLeg = leg;
            identifyOriginalFemur = hexapod.getServo(leg, Hexapod::FEMUR);
            identifyOriginalCoxa = hexapod.getServo(leg, Hexapod::COXA);

            // Lift leg (femur -30) and swing coxa +30
            hexapod.setServo(leg, Hexapod::FEMUR,
                             identifyOriginalFemur - 30.0f);
            hexapod.setServo(leg, Hexapod::COXA, identifyOriginalCoxa + 30.0f);

            identifyStartMs = millis();
          }
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
        dualBoardServos.resetOffsets();
        hexapod.stand(); // re-apply neutral pose with zeroed offsets
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

  // Safely restore identification leg
  if (identifyLeg != -1) {
    if (millis() - identifyStartMs > 500) {
      hexapod.setServo(identifyLeg, Hexapod::FEMUR, identifyOriginalFemur);
      hexapod.setServo(identifyLeg, Hexapod::COXA, identifyOriginalCoxa);
      identifyLeg = -1;
    }
  }

  // Minimal delay to prevent busy-looping and let WiFi tasks run
  delay(5);
}
