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
    double radians;
    // Joint mapping: Coxa=0, Femur=1, Tibia=2
    int joint = servoId % 3;
    if (joint == 2) {
      // Tibia: IK outputs 0°=extended, 90°=right angle — matches Webots directly
      radians = degToRad(angle);
    } else if (joint == 1) {
      // Femur: IK increases downward to match physical servos, but Webots
      // Y-rotation positive = downward too... except the Webots model
      // has the femur hinge oriented so positive rotation lifts.
      // Invert: motor = degToRad(90 - angle)
      radians = degToRad(90.0f - angle);
    } else {
      // Coxa: 90° IK = 0 rad Webots center
      radians = degToRad(angle - 90.0f);
    }
    motors_[servoId]->setPosition(radians);
  }
}

float WebotsServoImpl::read(int servoId) {
  if (servoId >= 0 && servoId < MAX_SERVOS && sensors_[servoId] != nullptr) {
    double radians = sensors_[servoId]->getValue();
    int joint = servoId % 3;
    if (joint == 2) {
      return radToDeg(radians);
    } else if (joint == 1) {
      return 90.0f - radToDeg(radians);
    } else {
      return radToDeg(radians) + 90.0f;
    }
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
