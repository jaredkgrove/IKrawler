#include "../lib/HexapodCore/include/Hexapod.h"
#include "TestFramework.h"
#include <cstring>

// Define globals required by TestFramework
int tests_run = 0;
int tests_failed = 0;

// ==========================================
// MOCK SERVO
// ==========================================

class MockServo : public ServoInterface {
public:
  float angles[32]; // Store more than 18 just in case
  bool attached[32];

  MockServo() {
    for (int i = 0; i < 32; i++) {
      angles[i] = 0.0f;
      attached[i] = false;
    }
  }

  bool attach(int servoId, int pin) override {
    (void)pin; // Unused
    if (servoId >= 0 && servoId < 32) {
      attached[servoId] = true;
      return true;
    }
    return false;
  }

  void detach(int servoId) override {
    if (servoId >= 0 && servoId < 32) {
      attached[servoId] = false;
    }
  }

  void write(int servoId, float angle) override {
    if (servoId >= 0 && servoId < 32) {
      angles[servoId] = angle;
    }
  }

  float read(int servoId) override {
    if (servoId >= 0 && servoId < 32) {
      return angles[servoId];
    }
    return 0.0f;
  }
};

// ==========================================
// TESTS
// ==========================================

TEST(TestInitialization) {
  MockServo mock;
  Hexapod hexapod(&mock);

  // Dummy pins
  int pins[18] = {0};

  bool success = hexapod.begin(pins);
  EXPECT_TRUE(success);

  // Check if servos are attached
  EXPECT_TRUE(mock.attached[0]);
  EXPECT_TRUE(mock.attached[17]);
}

TEST(TestWalkCycle) {
  MockServo mock;
  Hexapod hexapod(&mock);
  int pins[18] = {0};
  hexapod.begin(pins);

  // Start walking
  hexapod.walk();

  // Initially (time 0), we might be standing or starting first step
  hexapod.update(0.0f);

  // Leg 0 (Front Right) is in Group A. Leg 1 (Front Left) is in Group B.
  // In Tripod gait, A and B should be out of phase.
  // One should be lifting/moving, other on ground.

  // Step forward a significant amount of time
  // Gait speed defaults to something, let's say 1.0Hz?
  // Let's set a known speed
  hexapod.setGaitSpeed(1.0f); // 1 cycle per second

  // Update by 0.25s (quarter cycle)
  hexapod.update(0.25f);

  float leg0_femur = mock.read(Hexapod::FRONT_RIGHT * 3 + Hexapod::FEMUR);
  float leg1_femur = mock.read(Hexapod::FRONT_LEFT * 3 + Hexapod::FEMUR);

  // If phase 0 is start of swing for Group A, then at 0.25 (mid swing) A should
  // be lifted.

  // Just ensure they are NOT identical, implying different phases
  bool sameAngle = std::abs(leg0_femur - leg1_femur) < 0.1f;
  EXPECT_FALSE(sameAngle);
}

TEST(TestStopSequence) {
  MockServo mock;
  Hexapod hexapod(&mock);
  int pins[18] = {0};
  hexapod.begin(pins);
  hexapod.walk();

  // Run for a bit
  hexapod.update(0.5f);

  // Request stop
  hexapod.stop();

  // Run for 2 seconds (2 cycles) to ensure completion
  hexapod.update(1.0f);
  hexapod.update(1.0f);

  // Check if repeated updates cause no change (idling)
  float angleBefore = mock.read(0);
  hexapod.update(0.1f);
  float angleAfter = mock.read(0);

  EXPECT_NEAR(angleBefore, angleAfter, 0.001f);
}

int main() {
  PRINT_RESULT("HEXAPOD INTEGRATION");

  TestInitialization();
  TestWalkCycle();
  TestStopSequence();

  if (tests_failed == 0) {
    return 0;
  } else {
    return 1;
  }
}
