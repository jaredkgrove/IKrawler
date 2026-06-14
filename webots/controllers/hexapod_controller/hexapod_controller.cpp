#include "WebotsServoImpl.h"
#include <Hexapod.h>
#include <iostream>
#include <webots/Robot.hpp>

using namespace webots;

// Dummy pin array (not used in Webots, but required by Hexapod::begin())
const int SERVO_PINS[18] = {0, 1,  2,  3,  4,  5,  6,  7,  8,
                            9, 10, 11, 12, 13, 14, 15, 16, 17};

int main(int argc, char **argv) {
  // Create robot instance
  Robot *robot = new Robot();

  // Get simulation time step
  int timeStep = static_cast<int>(robot->getBasicTimeStep());

  std::cout << "Hexapod Controller Starting..." << std::endl;
  std::cout << "Time step: " << timeStep << " ms" << std::endl;

  // Create Webots servo implementation
  WebotsServoImpl servoImpl(robot);

  // Create hexapod instance
  Hexapod hexapod(&servoImpl);

  // Initialize hexapod with servo pins (pins not used in Webots)
  if (!hexapod.begin(SERVO_PINS)) {
    std::cerr << "ERROR: Failed to initialize hexapod!" << std::endl;
    return 1;
  }

  std::cout << "Hexapod initialized successfully!" << std::endl;

  // Wait a moment before posing
  for (int i = 0; i < 50; i++) {
    robot->step(timeStep);
  }

  // Drive every servo to the same mechanical baseline the firmware's
  // "calibrate" mode uses (see hexapodNeutralPose() in src/main.cpp), so the
  // simulation's resting pose matches what the real robot is calibrated to.
  // Coxa and femur sit at a true 90°; the tibia is commanded 15° past 90° to
  // account for the physical link bend. stand() runs first to force the gait
  // state to IDLE so update() won't clobber these direct servo writes.
  constexpr float NEUTRAL_TIBIA_BEND_DEG = 15.0f;
  std::cout << "Moving to calibration neutral pose..." << std::endl;
  hexapod.stand();
  for (int leg = 0; leg < Hexapod::NUM_LEGS; leg++) {
    for (int joint = 0; joint < Hexapod::JOINTS_PER_LEG; joint++) {
      float angle = (joint == Hexapod::TIBIA)
                        ? 90.0f + NEUTRAL_TIBIA_BEND_DEG
                        : 90.0f;
      hexapod.setServo(leg, joint, angle);
    }
  }
  hexapod.setStrideLength(0.06f);
  hexapod.setGaitSpeed(0.5f);
  hexapod.setTurnRate(0.0f);
  hexapod.walk();
  double lastTime = robot->getTime();
  while (robot->step(timeStep) != -1) {
    double currentTime = robot->getTime();
    float deltaTime = static_cast<float>(currentTime - lastTime);
    lastTime = currentTime;

    hexapod.update(deltaTime);
  }

  delete robot;
  return 0;
}
