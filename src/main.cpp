#include "ESP32ServoImpl.h"
#include "Hexapod.h"
#include <Arduino.h>


// Servo pin assignments (18 servos total)
// Front-Right leg (0): pins 13, 12, 14
// Middle-Right leg (1): pins 27, 26, 25
// Rear-Right leg (2): pins 33, 32, 35
// Rear-Left leg (3): pins 34, 39, 36
// Middle-Left leg (4): pins 4, 16, 17
// Front-Left leg (5): pins 5, 18, 19
const int SERVO_PINS[18] = {
    // Leg 0: Front-Right
    13, 12, 14,
    // Leg 1: Middle-Right
    27, 26, 25,
    // Leg 2: Rear-Right
    33, 32, 35,
    // Leg 3: Rear-Left
    34, 39, 36,
    // Leg 4: Middle-Left
    4, 16, 17,
    // Leg 5: Front-Left
    5, 18, 19};

// Create servo implementation
ESP32ServoImpl servoImpl;

// Create hexapod instance
Hexapod hexapod(&servoImpl);

unsigned long lastUpdateTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Hexapod Starting...");

  // Initialize hexapod with servo pins
  if (!hexapod.begin(SERVO_PINS)) {
    Serial.println("ERROR: Failed to initialize servos!");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Hexapod initialized successfully!");

  // Move to standing position
  delay(1000);
  Serial.println("Moving to standing position...");
  hexapod.stand();

  lastUpdateTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdateTime) / 1000.0f;
  lastUpdateTime = currentTime;

  // Update hexapod logic
  hexapod.update(deltaTime);

  // Small delay to prevent overwhelming the servos
  delay(20);
}
