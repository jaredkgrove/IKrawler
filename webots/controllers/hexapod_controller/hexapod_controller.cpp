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

  // Wait a moment, then move to standing position
  for (int i = 0; i < 50; i++) {
    robot->step(timeStep);
  }

  std::cout << "Moving to standing position..." << std::endl;
  hexapod.stand();

  // Wait for robot to stabilize in standing position
  for (int i = 0; i < 100; i++) {
    robot->step(timeStep);
  }

  // hexapod.setBodyPose(0, 0, 0.02, 0, 0, 0); // Raise body 2cm
  // hexapod.setBodyPose(0.02, 0, 0, 0, 0, 0); // Shift forward 2cm
  // hexapod.setBodyPose(0, 0, 0, 0, 0.2, 0); // Pitch nose down ~11°
  hexapod.setBodyPose(0, 0, 0, 0.2, 0, 0); // Roll right side down
  // ~11°

  // ===== INVERSE KINEMATICS TEST =====
  // std::cout << "Testing Inverse Kinematics..." << std::endl;

  // Test 1: Set all legs to a standing foot position using IK
  // Expected: x=0.14 (coxa + femur reach), y=0, z=-0.06 (below hip)
  // std::cout << "Test 1: All legs to standing position via IK" << std::endl;
  // for (int leg = 0; leg < 6; leg++) {
  //   bool success = hexapod.setFootPosition(leg, 0.12f, 0.0f, -0.06f);
  //   std::cout << "  Leg " << leg << ": " << (success ? "OK" : "FAILED")
  //             << std::endl;
  // }

  // for (int i = 0; i < 100; i++) {
  //   robot->step(timeStep);
  // }

  // Test 2: Lift front-right leg (leg 0)
  // std::cout << "Test 2: Lifting front-right leg" << std::endl;
  // hexapod.setFootPosition(0, 0.12f, 0.0f, -0.02f); // Raise foot

  // for (int i = 0; i < 100; i++) {
  //   robot->step(timeStep);
  // }

  // // Test 3: Extend front-right leg forward
  // std::cout << "Test 3: Extend front-right leg forward" << std::endl;
  // hexapod.setFootPosition(0, 0.10f, 0.04f, -0.02f); // Forward and up

  // for (int i = 0; i < 100; i++) {
  //   robot->step(timeStep);
  // }

  // // Test 4: Return to standing
  // std::cout << "Test 4: Return all legs to standing" << std::endl;
  // for (int leg = 0; leg < 6; leg++) {
  //   hexapod.setFootPosition(leg, 0.12f, 0.0f, -0.06f);
  // }

  // for (int i = 0; i < 100; i++) {
  //   robot->step(timeStep);
  // }

  // std::cout << "IK Test complete!" << std::endl;

  // // Keep running (no walking for now - just IK test)

  // // Main control loop
  // double lastTime = robot->getTime();
  // while (robot->step(timeStep) != -1) {
  //   // Calculate delta time
  //   double currentTime = robot->getTime();
  //   float deltaTime = static_cast<float>(currentTime - lastTime);
  //   lastTime = currentTime;

  //   // Update hexapod logic
  //   hexapod.update(deltaTime);
  // }

  delete robot;
  return 0;
}
