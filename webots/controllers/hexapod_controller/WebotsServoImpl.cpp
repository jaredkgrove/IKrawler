#include "WebotsServoImpl.h"
#include <iostream>

// Hexapod's leg enum is clockwise from front-right: FR, MR, RR, RL, ML, FL.
// The .wbt file was authored against the prior enum (FR, FL, MR, ML, RR, RL),
// so servo_{N} names are still wired to that older leg order. Map each
// logical leg index to the slot it occupies in the .wbt file.
static constexpr int WEBOTS_LEG_SLOT[6] = {
    0, // FR → wbt slot 0
    2, // MR → wbt slot 2
    4, // RR → wbt slot 4
    5, // RL → wbt slot 5
    3, // ML → wbt slot 3
    1, // FL → wbt slot 1
};

static int toWebotsServoId(int servoId) {
  return WEBOTS_LEG_SLOT[servoId / 3] * 3 + (servoId % 3);
}

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

  int webotsId = toWebotsServoId(servoId);
  std::string motorName = "servo_" + std::to_string(webotsId);
  std::string sensorName = "pos_" + std::to_string(webotsId);

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
      // Tibia: IK outputs tibiaRelativeRad in degrees; Webots knee motor
      // is oriented so positive rotation = more extended, so motor position
      // equals tibiaRelativeRad directly.
      radians = degToRad(angle);
    } else if (joint == 1) {
      // Femur: IK outputs 90 + femurRad_deg, where femurRad > 0 means
      // femur above horizontal (knee up). Webots femur hinge has positive
      // rotation pointing the tip downward, so we negate.
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
