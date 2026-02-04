#include "../lib/HexapodCore/include/GaitMath.h"
#include <cmath>
#include <iostream>

// Simple minimal test framework
int tests_run = 0;
int tests_failed = 0;

#define EXPECT_NEAR(val1, val2, abs_error)                                     \
  do {                                                                         \
    float diff = std::abs((val1) - (val2));                                    \
    if (diff > (abs_error)) {                                                  \
      std::cout << "[FAILED] " << #val1 << " (" << (val1) << ") != " << #val2  \
                << " (" << (val2) << ") within " << abs_error << "\n";         \
      std::cout << "         at " << __FILE__ << ":" << __LINE__ << "\n";      \
      tests_failed++;                                                          \
    }                                                                          \
  } while (0)

#define EXPECT_TRUE(condition)                                                 \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cout << "[FAILED] " << #condition << " is false\n";                 \
      std::cout << "         at " << __FILE__ << ":" << __LINE__ << "\n";      \
      tests_failed++;                                                          \
    }                                                                          \
  } while (0)

#define EXPECT_FALSE(condition) EXPECT_TRUE(!(condition))

#define TEST(name) void name()

// ==========================================
// TESTS
// ==========================================

TEST(TestTripodPhases) {
  std::cout << "Running TestTripodPhases...\n";
  // 50% swing, 50% stance

  // Phase 0.0 (start of swing)
  auto res0 = GaitMath::calculatePhaseProgress(0.0f, 0.05f, 0.5f);
  EXPECT_TRUE(res0.isSwing);
  EXPECT_NEAR(res0.strideProgress, -0.5f, 0.001f);
  EXPECT_NEAR(res0.liftHeight, 0.0f, 0.001f);

  // Phase 0.25 (mid swing)
  auto res1 = GaitMath::calculatePhaseProgress(0.25f, 0.05f, 0.5f);
  EXPECT_TRUE(res1.isSwing);
  EXPECT_NEAR(res1.strideProgress, 0.0f, 0.001f);
  EXPECT_NEAR(res1.liftHeight, 0.05f, 0.001f);

  // Phase 0.5 (start of stance)
  auto res2 = GaitMath::calculatePhaseProgress(0.5f, 0.05f, 0.5f);
  EXPECT_FALSE(res2.isSwing);
  EXPECT_NEAR(res2.strideProgress, 0.5f, 0.001f);
  EXPECT_NEAR(res2.liftHeight, 0.0f, 0.001f);

  // Phase 0.75 (mid stance)
  auto res3 = GaitMath::calculatePhaseProgress(0.75f, 0.05f, 0.5f);
  EXPECT_FALSE(res3.isSwing);
  EXPECT_NEAR(res3.strideProgress, 0.0f, 0.001f);
  EXPECT_NEAR(res3.liftHeight, 0.0f, 0.001f);
}

TEST(TestWavePhases) {
  std::cout << "Running TestWavePhases...\n";
  // Wave gait: ~16.6% swing (1/6), ~83.3% stance
  float duty = 0.166f;

  // Phase 0.083 (mid swing) -> 0.083 / 0.166 = 0.5 swing progress
  auto res = GaitMath::calculatePhaseProgress(0.083f, 0.05f, duty);
  EXPECT_TRUE(res.isSwing);
  EXPECT_NEAR(res.strideProgress, 0.0f, 0.01f);
  EXPECT_NEAR(res.liftHeight, 0.05f, 0.01f);

  // Phase 0.5 (mid stance)
  // stance phase is 0.166 to 1.0 (length 0.834)
  // 0.5 is roughly 40% into stance?
  auto res2 = GaitMath::calculatePhaseProgress(0.5f, 0.05f, duty);
  EXPECT_FALSE(res2.isSwing);
  EXPECT_NEAR(res2.liftHeight, 0.0f, 0.001f);
}

TEST(TestStraightWalk) {
  std::cout << "Running TestStraightWalk...\n";
  // Stride 10cm, Heading 0 (Forward)
  // Stride Progress 0.5 (Front of stride)
  auto delta = GaitMath::calculateFootDelta(0.5f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f);

  EXPECT_NEAR(delta.x, 0.05f, 0.001f); // 0.5 * 0.1 = 0.05
  EXPECT_NEAR(delta.y, 0.0f, 0.001f);

  // Stride Progress -0.5 (Back of stride)
  delta = GaitMath::calculateFootDelta(-0.5f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f);
  EXPECT_NEAR(delta.x, -0.05f, 0.001f);
}

TEST(TestSideWalk) {
  std::cout << "Running TestSideWalk...\n";
  // Stride 10cm, Heading 90 deg (Left)
  // Stride Progress 0.5
  float heading = GaitMath::PI / 2.0f;
  auto delta =
      GaitMath::calculateFootDelta(0.5f, 0.0f, heading, 0.1f, 0.0f, 0.0f);

  EXPECT_NEAR(delta.x, 0.0f, 0.001f);
  EXPECT_NEAR(delta.y, 0.05f, 0.001f);
}

// ==========================================
// MAIN REPL
// ==========================================

int main() {
  std::cout << "Running Hexapod Unit Tests...\n";
  std::cout << "--------------------------------\n";

  TestTripodPhases();
  TestWavePhases();
  TestStraightWalk();
  TestSideWalk();

  std::cout << "--------------------------------\n";
  if (tests_failed == 0) {
    std::cout << "ALL TESTS PASSED\n";
    return 0;
  } else {
    std::cout << tests_failed << " TESTS FAILED\n";
    return 1;
  }
}
