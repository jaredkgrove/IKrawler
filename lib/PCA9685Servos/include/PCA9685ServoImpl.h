#pragma once

#include "ServoInterface.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

/**
 * @brief PCA9685 implementation of ServoInterface
 *
 * Uses the Adafruit_PWMServoDriver library to control servos via I2C.
 * This implementation supports 16 channels per board, mapping them directly
 * to servoIds or utilizing multiple boards depending on the address
 * configuration.
 */
class PCA9685ServoImpl : public ServoInterface {
public:
  /**
   * @brief Construct a new PCA9685 Servo Impl object
   *
   * @param address I2C address of the PCA9685 board (default 0x40)
   * @param sdaPin SDA pin for I2C (default 21 for ESP32)
   * @param sclPin SCL pin for I2C (default 22 for ESP32)
   */
  PCA9685ServoImpl(uint8_t address = 0x40, int sdaPin = 21, int sclPin = 22);
  ~PCA9685ServoImpl() override = default;

  /**
   * @brief Initialize the PCA9685 board and set default PWM frequency.
   * Call this in setup() after Wire.begin().
   *
   * @return true if initialization is successful
   */
  bool begin();

  // ServoInterface implementations
  bool attach(int servoId, int pin) override;
  void write(int servoId, float angle) override;
  float read(int servoId) override;
  void detach(int servoId) override;

private:
  Adafruit_PWMServoDriver pca_;
  uint8_t address_;
  int sdaPin_;
  int sclPin_;
  bool initialized_;

  // Track last written angles for the `read` method
  static constexpr int PCA9685_MAX_CHANNELS = 16;
  float currentAngles_[PCA9685_MAX_CHANNELS];

  // Configurable servo limits (microseconds)
  // MG996R typical: ~500us to 2500us
  uint16_t minPulseUs_ = 500;
  uint16_t maxPulseUs_ = 2500;

  // Helper method to convert angle to PCA9685 ticks (0-4095 range)
  uint16_t angleToPulseTicks(float angle);
};
