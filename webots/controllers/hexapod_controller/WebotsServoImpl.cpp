#include "WebotsServoImpl.h"
#include <iostream>

WebotsServoImpl::WebotsServoImpl(Robot *robot) : robot_(robot) {
  for (int i = 0; i < MAX_SERVOS; i++) {
    motors_[i] = nullptr;
    sensors_[i] = nullptr;
  }
}

bool WebotsServoImpl::attach(int servoId, int pin) {
  if (servoId < 0 || servoId >= MAX_SERVOS) {
    return false;
  }

  // In Webots, pin number is ignored - we use device names
  // Device names follow the pattern: servo_0, servo_1, etc.
  std::string motorName = "servo_" + std::to_string(servoId);
  std::string sensorName = "pos_" + std::to_string(servoId);

  // Get motor device
  motors_[servoId] = robot_->getMotor(motorName);
  if (motors_[servoId] == nullptr) {
    std::cerr << "Warning: Motor " << motorName << " not found!" << std::endl;
    return false;
  }

  // Get position sensor
  sensors_[servoId] = robot_->getPositionSensor(sensorName);
  if (sensors_[servoId] == nullptr) {
    std::cerr << "Warning: Position sensor " << sensorName << " not found!"
              << std::endl;
    return false;
  }

  // Enable the position sensor
  int timeStep = static_cast<int>(robot_->getBasicTimeStep());
  sensors_[servoId]->enable(timeStep);

  // Set motor to position control mode
  motors_[servoId]->setPosition(0.0);

  return true;
}

void WebotsServoImpl::write(int servoId, float angle) {
  if (servoId >= 0 && servoId < MAX_SERVOS && motors_[servoId] != nullptr) {
    // Convert angle from degrees to radians and set position
    // Note: Webots uses radians, and we need to offset by 90 degrees (π/2
    // radians) because servos use 0-180 degrees, but Webots joints are centered
    // at 0
    double radians = degToRad(angle - 90.0f);
    motors_[servoId]->setPosition(radians);
  }
}

float WebotsServoImpl::read(int servoId) {
  if (servoId >= 0 && servoId < MAX_SERVOS && sensors_[servoId] != nullptr) {
    // Read position in radians and convert to degrees
    double radians = sensors_[servoId]->getValue();
    return radToDeg(radians) + 90.0f;
  }
  return 0.0f;
}

void WebotsServoImpl::detach(int servoId) {
  if (servoId >= 0 && servoId < MAX_SERVOS) {
    // In Webots, we don't explicitly detach devices
    // Just disable the sensor
    if (sensors_[servoId] != nullptr) {
      sensors_[servoId]->disable();
    }
    motors_[servoId] = nullptr;
    sensors_[servoId] = nullptr;
  }
}
