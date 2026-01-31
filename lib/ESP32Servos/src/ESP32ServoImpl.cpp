#include "ESP32ServoImpl.h"

ESP32ServoImpl::ESP32ServoImpl() {
  // Constructor
}

bool ESP32ServoImpl::attach(int servoId, int pin) {
  if (servoId < 0 || servoId >= HEXAPOD_MAX_SERVOS) {
    return false;
  }

  // Attach servo with typical PWM range (500-2500 microseconds)
  servos_[servoId].attach(pin, 500, 2500);
  return true;
}

void ESP32ServoImpl::write(int servoId, float angle) {
  if (servoId >= 0 && servoId < HEXAPOD_MAX_SERVOS) {
    servos_[servoId].write(static_cast<int>(angle));
  }
}

float ESP32ServoImpl::read(int servoId) {
  if (servoId >= 0 && servoId < HEXAPOD_MAX_SERVOS) {
    return static_cast<float>(servos_[servoId].read());
  }
  return 0.0f;
}

void ESP32ServoImpl::detach(int servoId) {
  if (servoId >= 0 && servoId < HEXAPOD_MAX_SERVOS) {
    servos_[servoId].detach();
  }
}
