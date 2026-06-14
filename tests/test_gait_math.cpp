#include "../lib/HexapodCore/include/GaitMath.h"
#include "TestFramework.h"
#include <cmath>

// Define globals required by TestFramework
int tests_run = 0;
int tests_failed = 0;

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

  // Phase 0.25 (mid swing, swingProgress = 0.5)
  // Horizontal: the cycloid's sin(2*pi*s) term vanishes at s=0.5, so
  // strideProgress is exactly 0.0 (mid-stride) here. Vertical: mid-swing sits on
  // the flat top of the trapezoidal profile (peak lift) for any plateau fraction.
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

  // Phase 0.083 (mid swing) -> 0.083 / 0.166 = 0.5 swing progress.
  // Mid-swing sits on the flat top of the trapezoidal profile -> peak lift.
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

TEST(TestSwingTrajectoryShape) {
  std::cout << "Running TestSwingTrajectoryShape...\n";
  // Trapezoidal flat-top vertical arc + cycloidal horizontal. The horizontal
  // cycloid matches the stance sweep velocity at both swing endpoints, so there
  // is no velocity jump across the swing<->stance boundary for a slew-limited
  // servo to overshoot -> scuff-free liftoff and touchdown. The vertical
  // smoothstep ramps land with ~zero vertical velocity (soft touchdown, no
  // impact bounce) while holding full clearance across the middle of swing.
  const float H = 0.05f;
  const float duty = 0.5f;
  const float stanceDuty = 1.0f - duty;

  auto spAt = [&](float ph) {
    return GaitMath::calculatePhaseProgress(ph, H, duty).strideProgress;
  };
  auto liftAt = [&](float ph) {
    return GaitMath::calculatePhaseProgress(ph, H, duty).liftHeight;
  };

  // Endpoints are preserved so the swing curve meshes with the linear stance
  // ramp: swing starts at -0.5 on the ground, ends at +0.5 on the ground.
  auto start = GaitMath::calculatePhaseProgress(0.0f, H, duty);
  EXPECT_NEAR(start.strideProgress, -0.5f, 0.001f);
  EXPECT_NEAR(start.liftHeight, 0.0f, 0.001f);

  // Sample very close to the end of swing (swingProgress -> 1): lift is 0 there
  // for any plateau fraction < 1, and stride has returned to +0.5.
  auto end = GaitMath::calculatePhaseProgress(0.49999f, H, duty);
  EXPECT_TRUE(end.isSwing);
  EXPECT_NEAR(end.strideProgress, 0.5f, 0.005f);
  EXPECT_NEAR(end.liftHeight, 0.0f, 0.001f);

  // Flat top: mid-swing (swingProgress 0.5) is held at peak lift.
  EXPECT_NEAR(liftAt(0.25f), H, 0.001f);

  // Scuff-free horizontal: the cycloid's slope at liftoff equals the stance
  // sweep slope, so foot velocity is CONTINUOUS across the boundary (no jump for
  // a servo to overshoot through). Stance sweeps at d(strideProgress)/dphase =
  // -1/stanceDuty; the swing must start at the same velocity.
  const float dph = 0.0005f;
  float stanceVel = (spAt(1.0f - dph) - spAt(1.0f - 2.0f * dph)) / dph;
  float swingStartVel = (spAt(dph) - spAt(0.0f)) / dph;
  EXPECT_NEAR(stanceVel, -1.0f / stanceDuty, 0.02f);
  EXPECT_NEAR(swingStartVel, stanceVel, 0.05f);

  // Soft, symmetric vertical endpoints (smoothstep -> zero slope at the ground):
  // the descent speed just before touchdown is far smaller than the peak descent
  // speed mid-fall-ramp, so the foot is set down gently instead of slammed.
  // The fall ramp spans swingProgress [1-ramp, 1] = phase [0.425, 0.5] here.
  float touchdownVSpeed = std::fabs(liftAt(0.5f - dph) - liftAt(0.5f - 2.0f * dph));
  float midFallVSpeed = std::fabs(liftAt(0.4625f) - liftAt(0.4625f - dph));
  EXPECT_TRUE(midFallVSpeed > 0.0f);
  EXPECT_TRUE(touchdownVSpeed < 0.25f * midFallVSpeed);

  // Symmetric: liftoff descent/rise speed near the ground mirrors touchdown.
  float liftoffVSpeed = std::fabs(liftAt(dph) - liftAt(0.0f));
  EXPECT_NEAR(liftoffVSpeed, touchdownVSpeed, 0.0005f);
}

TEST(TestSubFiftyDutyContinuity) {
  std::cout << "Running TestSubFiftyDutyContinuity...\n";
  // Sub-50% swing duty (double-support tripod). The cycloid derives its
  // endpoint slope from stanceDuty, so foot velocity must stay continuous
  // across the swing<->stance boundary at ANY duty, not just 0.5 -- otherwise
  // the scuff-free property would silently break when overlap is dialed in.
  const float H = 0.05f;
  const float duty = 0.4f; // stance 0.6
  const float stanceDuty = 1.0f - duty;

  auto spAt = [&](float ph) {
    return GaitMath::calculatePhaseProgress(ph, H, duty).strideProgress;
  };

  // Swing still spans the full stride and meets the ground at both ends.
  EXPECT_NEAR(spAt(0.0f), -0.5f, 0.001f);
  EXPECT_NEAR(spAt(duty - 0.00001f), 0.5f, 0.005f);

  // Velocity at liftoff (start of swing) matches the stance sweep velocity.
  const float dph = 0.0005f;
  float stanceVel = (spAt(1.0f - dph) - spAt(1.0f - 2.0f * dph)) / dph;
  float swingStartVel = (spAt(dph) - spAt(0.0f)) / dph;
  EXPECT_NEAR(stanceVel, -1.0f / stanceDuty, 0.02f);
  EXPECT_NEAR(swingStartVel, stanceVel, 0.05f);
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

TEST(TestPureSpin) {
  std::cout << "Running TestPureSpin...\n";
  // turnRate = 1.0 (max CCW), strideLength = 0.
  // Foot at (+x, 0): tangential motion is purely +y at strideProgress=+0.5.
  // Expected: ω × r = (-ω·y, +ω·x) = (0, ω·x)
  // With strideProgress = 0.5, omega = MAX_ROTATION_PER_CYCLE.
  float footX = 0.18f;
  float footY = 0.0f;
  auto delta =
      GaitMath::calculateFootDelta(0.5f, 1.0f, 0.0f, 0.0f, footX, footY);

  EXPECT_NEAR(delta.x, 0.0f, 0.001f);
  EXPECT_NEAR(delta.y, 0.5f * GaitMath::MAX_ROTATION_PER_CYCLE * footX, 0.001f);

  // Foot at (0, +y): tangential motion is purely -x at strideProgress=+0.5.
  // ω × r = (-ω·y, +ω·x) = (-ω·y, 0)
  auto delta2 =
      GaitMath::calculateFootDelta(0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.18f);
  EXPECT_NEAR(delta2.x, -0.5f * GaitMath::MAX_ROTATION_PER_CYCLE * 0.18f,
              0.001f);
  EXPECT_NEAR(delta2.y, 0.0f, 0.001f);
}

TEST(TestRotationSign) {
  std::cout << "Running TestRotationSign...\n";
  // Positive turnRate must rotate CCW (left). Foot at (+x, 0) at the front of
  // its stride should swing toward +y (left side of body).
  auto deltaPos =
      GaitMath::calculateFootDelta(0.5f, 1.0f, 0.0f, 0.0f, 0.18f, 0.0f);
  EXPECT_TRUE(deltaPos.y > 0.0f);

  // Negative turnRate must rotate CW (right). Same foot should go to -y.
  auto deltaNeg =
      GaitMath::calculateFootDelta(0.5f, -1.0f, 0.0f, 0.0f, 0.18f, 0.0f);
  EXPECT_TRUE(deltaNeg.y < 0.0f);
}

TEST(TestStrafeSpinSuperposition) {
  std::cout << "Running TestStrafeSpinSuperposition...\n";
  // The combined motion must equal the sum of pure-translation and
  // pure-rotation deltas. This is the holonomic invariant: the two joysticks
  // do not couple.
  float strideProgress = 0.5f;
  float turnRate = 0.5f;
  float heading = GaitMath::PI / 4.0f; // 45 deg (forward-left strafe)
  float strideLength = 0.06f;
  float footX = 0.12f;
  float footY = -0.08f;

  auto deltaTranslateOnly = GaitMath::calculateFootDelta(
      strideProgress, 0.0f, heading, strideLength, footX, footY);
  auto deltaRotateOnly = GaitMath::calculateFootDelta(
      strideProgress, turnRate, 0.0f, 0.0f, footX, footY);
  auto deltaCombined = GaitMath::calculateFootDelta(
      strideProgress, turnRate, heading, strideLength, footX, footY);

  EXPECT_NEAR(deltaCombined.x, deltaTranslateOnly.x + deltaRotateOnly.x,
              0.0001f);
  EXPECT_NEAR(deltaCombined.y, deltaTranslateOnly.y + deltaRotateOnly.y,
              0.0001f);
}

// ==========================================
// MAIN REPL
// ==========================================

int main() {
  std::cout << "Running Hexapod Unit Tests...\n";
  std::cout << "--------------------------------\n";

  TestTripodPhases();
  TestWavePhases();
  TestSwingTrajectoryShape();
  TestSubFiftyDutyContinuity();
  TestStraightWalk();
  TestSideWalk();
  TestPureSpin();
  TestRotationSign();
  TestStrafeSpinSuperposition();

  PRINT_RESULT("GAIT MATH");
  if (tests_failed == 0) {
    return 0;
  } else {
    return 1;
  }
}
