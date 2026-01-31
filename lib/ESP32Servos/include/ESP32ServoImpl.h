#pragma once

#include "ServoInterface.h"
#include <ESP32Servo.h>

/**
 * @brief ESP32 implementation of ServoInterface
 *
 * Uses the ESP32Servo library to control physical servos
 */
class ESP32ServoImpl : public ServoInterface {
public:
  ESP32ServoImpl();
  ~ESP32ServoImpl() override = default;

  bool attach(int servoId, int pin) override;
  void write(int servoId, float angle) override;
  float read(int servoId) override;
  void detach(int servoId) override;

private:
  static constexpr int HEXAPOD_MAX_SERVOS = 18;
  Servo servos_[HEXAPOD_MAX_SERVOS];
};
