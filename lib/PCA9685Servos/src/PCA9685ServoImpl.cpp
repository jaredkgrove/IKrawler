#include "PCA9685ServoImpl.h"
#include <Wire.h>

PCA9685ServoImpl::PCA9685ServoImpl(uint8_t address, int sdaPin, int sclPin)
    : pca_(address), address_(address), sdaPin_(sdaPin), sclPin_(sclPin),
      initialized_(false) {
  // Initialize current angles tracking array
  for (int i = 0; i < PCA9685_MAX_CHANNELS; ++i) {
    currentAngles_[i] = 0.0f;
  }
}

bool PCA9685ServoImpl::begin() {
  if (initialized_)
    return true;

  // Ideally Wire.begin() should be called globally in main.cpp, but if
  // multiple libraries use it, providing pins here is safe.
  Wire.begin(sdaPin_, sclPin_);

  // Probe the I2C bus before issuing any commands. If the chip doesn't ACK,
  // refuse to initialize so the caller can halt before driving servos blind.
  Wire.beginTransmission(address_);
  if (Wire.endTransmission() != 0)
    return false;

  pca_.begin();
  // Set PWM frequency to 50Hz for standard servos (20ms period)
  pca_.setPWMFreq(50);

  // Small delay to allow oscillator to stabilize
  delay(10);

  initialized_ = true;
  return true;
}

bool PCA9685ServoImpl::attach(int servoId, int pin) {
  // In PCA9685 context, "pin" usually corresponds to the 0-15 channel on the
  // board. We'll enforce this mapping.
  if (pin < 0 || pin >= PCA9685_MAX_CHANNELS) {
    return false;
  }

  // We don't have a strict "attach" logic on PCA9685, since the channel is
  // always active once written to. We'll simply acknowledge the attachment.
  // The user should call begin() before doing any write operations.
  return true;
}

void PCA9685ServoImpl::write(int servoId, float angle) {
  // If we assume servoId == pin channel for simplicity, then:
  if (servoId >= 0 && servoId < PCA9685_MAX_CHANNELS) {
    // Constrain angle between 0 and 180
    if (angle < 0.0f)
      angle = 0.0f;
    if (angle > 180.0f)
      angle = 180.0f;

    // Store it for read()
    currentAngles_[servoId] = angle;

    uint16_t ticks = angleToPulseTicks(angle);

    // PCA9685 sets high output at tick 0, and low output at `ticks`
    pca_.setPWM(servoId, 0, ticks);
  }
}

float PCA9685ServoImpl::read(int servoId) {
  if (servoId >= 0 && servoId < PCA9685_MAX_CHANNELS) {
    return currentAngles_[servoId];
  }
  return 0.0f;
}

void PCA9685ServoImpl::detach(int servoId) {
  if (servoId >= 0 && servoId < PCA9685_MAX_CHANNELS) {
    // Sending a 0 to the ON tick parameter with 4096 to the OFF tick turns it
    // completely off
    pca_.setPWM(servoId, 0, 4096);
  }
}

uint16_t PCA9685ServoImpl::angleToPulseTicks(float angle) {
  // Map angle (0-180) to pulse width in microseconds (e.g. 500-2500)
  float pulseUs = minPulseUs_ + (angle / 180.0f) * (maxPulseUs_ - minPulseUs_);

  // Convert microseconds to PCA9685 ticks.
  // 50Hz = 20ms period = 20000us.
  // PCA9685 has 4096 ticks per period.
  // ticks = (pulseUs / 20000.0) * 4096.0
  return static_cast<uint16_t>((pulseUs * 4096.0f) / 20000.0f);
}
