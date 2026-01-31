#pragma once

#include <ServoInterface.h>
#include <string>
#include <vector>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>


using namespace webots;

/**
 * @brief Webots implementation of ServoInterface
 *
 * Uses Webots Motor and PositionSensor devices
 */
class WebotsServoImpl : public ServoInterface {
public:
  WebotsServoImpl(Robot *robot);
  ~WebotsServoImpl() override = default;

  bool attach(int servoId, int pin) override;
  void write(int servoId, float angle) override;
  float read(int servoId) override;
  void detach(int servoId) override;

private:
  static constexpr int MAX_SERVOS = 18;
  Robot *robot_;
  Motor *motors_[MAX_SERVOS];
  PositionSensor *sensors_[MAX_SERVOS];

  // Convert degrees to radians
  double degToRad(float degrees) const {
    return degrees * 3.14159265359 / 180.0;
  }

  // Convert radians to degrees
  float radToDeg(double radians) const {
    return static_cast<float>(radians * 180.0 / 3.14159265359);
  }
};
