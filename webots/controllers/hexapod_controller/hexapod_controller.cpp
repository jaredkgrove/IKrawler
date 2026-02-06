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

  // hexapod.rest();

  // Set up for differential steering demo
  hexapod.setStrideLength(0.06f); // Stride length
  hexapod.setGaitSpeed(0.5f);     // Gait speed
  hexapod.setTurnRate(0.0f);      // Start going straight
  hexapod.walk();                 // Start walking (heading/turnRate set above)

  double lastTime = robot->getTime();
  // double loopStartTime = lastTime;
  float turnRate = 0.0f;
  hexapod.setTurnRate(turnRate);
  while (robot->step(timeStep) != -1) {
    double currentTime = robot->getTime();
    // Calculate delta time
    float deltaTime = static_cast<float>(currentTime - lastTime);
    lastTime = currentTime;

    // Update hexapod logic
    hexapod.update(deltaTime);

    // if (currentTime - loopStartTime > 2.0) {
    //   // Gradually increase turn rate to demonstrate differential steering
    //   turnRate += 0.05f;
    //   if (turnRate > 0.4f)
    //     turnRate = 0.4f; // Wrap around
    //   hexapod.setTurnRate(turnRate);
    //   loopStartTime = robot->getTime();
    // }
  }

  delete robot;
  return 0;
}
