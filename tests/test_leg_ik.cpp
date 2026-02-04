#include "../lib/HexapodCore/include/LegIK.h"
#include "TestFramework.h"
#include <cmath>

int tests_run = 0;
int tests_failed = 0;

// ==========================================
// TESTS
// ==========================================

TEST(TestNeutralReach) {
  std::cout << "Running TestNeutralReach...\n";
  LegIK::LegConfig config;
  // Default config: coxa=0.03, femur=0.06, tibia=0.08
  // Total horizontal reach = 0.03 + 0.06 = 0.09? No, that's not quite right.
  // Standard rest pose often has femur horizontal, tibia vertical?

  LegIK ik(config);

  // Test a known reachable point directly sideways (y=0)
  // At y=0, coxa should be 90 degrees (neutral)
  LegIK::FootPosition target;
  target.x = 0.12f; // 12cm reach
  target.y = 0.0f;
  target.z = -0.05f;

  LegIK::JointAngles angles = ik.solve(target);

  EXPECT_TRUE(angles.valid);
  EXPECT_NEAR(angles.coxa, 90.0f, 0.01f);
}

TEST(TestUnreachable) {
  std::cout << "Running TestUnreachable...\n";
  LegIK ik;

  LegIK::FootPosition target;
  target.x = 0.5f; // 50cm reach (way too far)
  target.y = 0.0f;
  target.z = 0.0f;

  LegIK::JointAngles angles = ik.solve(target);
  EXPECT_FALSE(angles.valid);
}

TEST(TestSymmetry) {
  std::cout << "Running TestSymmetry...\n";
  LegIK ik;

  LegIK::FootPosition left;
  left.x = 0.1f;
  left.y = 0.05f; // Forward/Left
  left.z = -0.05f;

  LegIK::FootPosition right;
  right.x = 0.1f;
  right.y = -0.05f; // Backward/Right
  right.z = -0.05f;

  auto leftAngles = ik.solve(left);
  auto rightAngles = ik.solve(right);

  EXPECT_TRUE(leftAngles.valid);
  EXPECT_TRUE(rightAngles.valid);

  // Coxa should mirror around 90 degrees
  // e.g. if left is 105 (+15), right should be 75 (-15)
  float leftDelta = leftAngles.coxa - 90.0f;
  float rightDelta = rightAngles.coxa - 90.0f;

  EXPECT_NEAR(leftDelta, -rightDelta, 0.01f);

  // Femur and Tibia should be identical (height/reach distance are same)
  EXPECT_NEAR(leftAngles.femur, rightAngles.femur, 0.01f);
  EXPECT_NEAR(leftAngles.tibia, rightAngles.tibia, 0.01f);
}

TEST(TestVerticalReach) {
  std::cout << "Running TestVerticalReach...\n";
  LegIK ik;
  LegIK::FootPosition target;

  // Check constraints: Min reach = |0.06 - 0.08| = 0.02
  // Max reach = 0.06 + 0.08 = 0.14

  // Test Case 1: Valid Reach
  target.x = 0.03f; // Coxa length
  target.y = 0.0f;
  target.z = -0.04f; // distance 0.04 from femur joint (valid > 0.02)

  auto angles = ik.solve(target);
  EXPECT_TRUE(angles.valid);
  EXPECT_TRUE(std::isfinite(angles.femur));
  EXPECT_TRUE(std::isfinite(angles.tibia));

  // Test Case 2: Too Close (Inside min radius)
  target.z = -0.01f; // distance 0.01 < 0.02
  angles = ik.solve(target);
  EXPECT_FALSE(angles.valid);
}

int main() {
  std::cout << "Running LegIK Unit Tests...\n";
  std::cout << "--------------------------------\n";

  TestNeutralReach();
  TestUnreachable();
  TestSymmetry();
  TestVerticalReach();

  PRINT_RESULT("LEG IK");
  if (tests_failed == 0) {
    return 0;
  } else {
    return 1;
  }
}
