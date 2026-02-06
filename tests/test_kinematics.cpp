#include "../lib/HexapodCore/include/Hexapod.h"
#include "../lib/HexapodCore/include/LegIK.h"
#include "TestFramework.h"
#include <cmath>

// Define globals required by TestFramework
int tests_run = 0;
int tests_failed = 0;

// Helper: Get leg length from default config
float getLegLength() {
  LegIK ik;
  const LegIK::LegConfig &config = ik.getConfig();
  return config.femurLength + config.tibiaLength;
}

// ==========================================
// TEST CASES
// ==========================================

TEST(TestStandingPoseReachability) {
  // Verify standing pose (from Hexapod::stand()) is kinematically valid
  LegIK ik;
  float legLength = getLegLength();

  float standingHeight = -legLength * Hexapod::STANDING_HEIGHT_RATIO;
  float standingReach = legLength * Hexapod::STANDING_REACH_RATIO;

  LegIK::FootPosition target;
  target.x = standingReach;
  target.y = 0.0f;
  target.z = standingHeight;

  LegIK::JointAngles angles = ik.solve(target);

  EXPECT_TRUE(angles.valid);
  EXPECT_FALSE(std::isnan(angles.coxa));
  EXPECT_FALSE(std::isnan(angles.femur));
  EXPECT_FALSE(std::isnan(angles.tibia));
}

TEST(TestRestPoseReachability) {
  // Verify rest pose (from Hexapod::rest()) is kinematically valid
  LegIK ik;
  float legLength = getLegLength();

  float restHeight = -legLength * Hexapod::REST_HEIGHT_RATIO;
  float restReach = legLength * Hexapod::REST_REACH_RATIO;

  LegIK::FootPosition target;
  target.x = restReach;
  target.y = 0.0f;
  target.z = restHeight;

  LegIK::JointAngles angles = ik.solve(target);

  EXPECT_TRUE(angles.valid);
}

TEST(TestHeightRangeReachability) {
  // Verify min/max height bounds are achievable
  LegIK ik;
  float legLength = getLegLength();
  float reach = legLength * Hexapod::REST_REACH_RATIO;

  LegIK::FootPosition target;
  target.x = reach;
  target.y = 0.0f;

  // Min height (body at lowest position)
  target.z = -legLength * Hexapod::MIN_HEIGHT_RATIO;
  LegIK::JointAngles anglesLow = ik.solve(target);
  EXPECT_TRUE(anglesLow.valid);

  // Max height (body at highest position)
  target.z = -legLength * Hexapod::MAX_HEIGHT_RATIO;
  LegIK::JointAngles anglesHigh = ik.solve(target);
  EXPECT_TRUE(anglesHigh.valid);
}

int main() {
  PRINT_RESULT("KINEMATIC VALIDATION");

  TestStandingPoseReachability();
  TestRestPoseReachability();
  TestHeightRangeReachability();

  if (tests_failed == 0) {
    return 0;
  } else {
    return 1;
  }
}
