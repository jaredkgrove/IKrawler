#include "../lib/HexapodCore/include/GaitMath.h"
#include "../lib/HexapodCore/include/Hexapod.h"
#include "TestFramework.h"
#include <cmath>

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

TEST(TestStrideClamping) {
  // Test that setStrideLength clamps to maxStride_
  MockServo mock;
  Hexapod hexapod(&mock);
  int pins[18] = {0};
  hexapod.begin(pins);

  // Try to set an impossibly large stride
  hexapod.setStrideLength(1.0f); // 1 meter is way too far

  float actualStride = hexapod.getStrideLength();

  // Should be clamped to maxStride_ (which is geometry-derived)
  EXPECT_TRUE(actualStride <= hexapod.getMaxStride());
  EXPECT_TRUE(actualStride > 0.0f); // Should still be positive
}

TEST(TestHeightChangeReclampsStride) {
  // When height is reduced, maxStride_ decreases, and strideLength_ should
  // re-clamp
  MockServo mock;
  Hexapod hexapod(&mock);
  int pins[18] = {0};
  hexapod.begin(pins);
  hexapod.stand();

  // Set stride to current max
  hexapod.setStrideLength(hexapod.getMaxStride());
  float strideAtStanding = hexapod.getStrideLength();

  // Now lower the body (rest position has lower maxStride)
  hexapod.rest();
  float strideAfterRest = hexapod.getStrideLength();

  // Stride should have been automatically reduced
  EXPECT_TRUE(strideAfterRest <= hexapod.getMaxStride());
  EXPECT_TRUE(strideAfterRest < strideAtStanding);
}

TEST(TestSetVelocity) {
  // setVelocity(vx, vy) should set strideLength to |v| (clamped) and heading
  // to atan2(vy, vx). Zero vector should not snap heading to 0.
  MockServo mock;
  Hexapod hexapod(&mock);
  int pins[18] = {0};
  hexapod.begin(pins);
  hexapod.stand();

  // Vector forward-left at 45 deg, magnitude 0.04 m
  float mag = 0.04f;
  float vx = mag * std::cos(GaitMath::PI / 4.0f);
  float vy = mag * std::sin(GaitMath::PI / 4.0f);
  hexapod.setVelocity(vx, vy);

  EXPECT_NEAR(hexapod.getStrideLength(), mag, 0.001f);

  // Strafe pure-Y (heading = 90 deg) — strideLength should match magnitude.
  hexapod.setVelocity(0.0f, 0.05f);
  EXPECT_NEAR(hexapod.getStrideLength(), 0.05f, 0.001f);

  // Strafing in pure-Y direction at progress=+0.5 should move foot in +y only.
  hexapod.walk();
  hexapod.setGaitSpeed(1.0f);
  hexapod.setTurnRate(0.0f);
  // We can't directly inspect foot pos, but we can verify stride magnitude
  // got accepted (clamped or not).
  EXPECT_TRUE(hexapod.getStrideLength() > 0.0f);

  // Zero vector: stride should clamp to MIN_STRIDE_LENGTH (since 0 < min).
  hexapod.setVelocity(0.0f, 0.0f);
  EXPECT_TRUE(hexapod.getStrideLength() > 0.0f); // bounded by MIN_STRIDE_LENGTH
}

TEST(TestCombinedStrideClamp) {
  // When user pushes both joysticks to max, the combined per-leg displacement
  // must not exceed maxStride_. The Hexapod should scale the effective inputs
  // down at apply time, but leave the raw setpoints alone (so a UI showing
  // them does not snap back).
  MockServo mock;
  Hexapod hexapod(&mock);
  int pins[18] = {0};
  hexapod.begin(pins);
  hexapod.stand();

  // Push stride to max and turn rate to max simultaneously
  float maxStride = hexapod.getMaxStride();
  hexapod.setStrideLength(maxStride);
  hexapod.setTurnRate(1.0f);

  // Raw setpoints should remain at the user's request (not snapped down)
  EXPECT_NEAR(hexapod.getStrideLength(), maxStride, 0.001f);

  // Walking should not blow up servo angle bounds — this is the real-world
  // smoke test that the clamp is active in the gait pipeline.
  hexapod.walk();
  hexapod.setGaitSpeed(1.0f);
  for (int step = 0; step < 20; step++) {
    hexapod.update(0.05f);
    for (int i = 0; i < 18; i++) {
      float angle = mock.read(i);
      EXPECT_TRUE(angle >= 0.0f);
      EXPECT_TRUE(angle <= 180.0f);
    }
  }
}

TEST(TestServoAngleBounds) {
  // Verify that servo angles stay in valid range [0, 180]
  MockServo mock;
  Hexapod hexapod(&mock);
  int pins[18] = {0};
  hexapod.begin(pins);

  hexapod.stand();
  hexapod.update(0.0f);

  // Check all 18 servo angles in standing pose
  for (int i = 0; i < 18; i++) {
    float angle = mock.read(i);
    EXPECT_TRUE(angle >= 0.0f);
    EXPECT_TRUE(angle <= 180.0f);
  }

  // Also check during walking (this was previously failing before IK fix)
  hexapod.walk();
  hexapod.update(0.1f);
  hexapod.update(0.1f);
  hexapod.update(0.1f);

  for (int i = 0; i < 18; i++) {
    float angle = mock.read(i);
    EXPECT_TRUE(angle >= 0.0f);
    EXPECT_TRUE(angle <= 180.0f);
  }
}

int main() {
  PRINT_RESULT("HEXAPOD INTEGRATION");

  TestInitialization();
  TestWalkCycle();
  TestStopSequence();
  TestStrideClamping();
  TestHeightChangeReclampsStride();
  TestSetVelocity();
  TestCombinedStrideClamp();
  TestServoAngleBounds();

  if (tests_failed == 0) {
    return 0;
  } else {
    return 1;
  }
}
