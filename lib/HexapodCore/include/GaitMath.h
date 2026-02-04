#pragma once

/**
 * @brief Pure math functions for gait trajectory calculation
 *
 * These functions are stateless and can be unit tested without hardware.
 * They handle the geometric calculations for foot placement during walking.
 */
namespace GaitMath {

// Tolerance for floating-point comparisons
constexpr float EPSILON = 0.001f;

// Base stride length for pure rotation (when strideLength is 0)
constexpr float ROTATION_STRIDE = 0.04f;

// Turn rate to turn radius scale factor
// turnRate=1.0 with strideLength=0.06 gives ~12cm turn radius
constexpr float TURN_RATE_SCALE = 0.5f;

constexpr float PI = 3.14159265359f;

/**
 * @brief Result of phase progress calculation
 */
struct PhaseResult {
  float strideProgress; // -0.5 to +0.5, position along stride
  float liftHeight;     // Vertical lift (0 during stance, >0 during swing)
  bool isSwing;         // True if leg is in swing phase (lifted)

  PhaseResult() : strideProgress(0.0f), liftHeight(0.0f), isSwing(false) {}
  PhaseResult(float sp, float lh, bool sw)
      : strideProgress(sp), liftHeight(lh), isSwing(sw) {}
};

/**
 * @brief XY movement delta for foot position
 */
struct FootDelta {
  float x; // Forward/backward delta in body frame
  float y; // Left/right delta in body frame

  FootDelta() : x(0.0f), y(0.0f) {}
  FootDelta(float x_, float y_) : x(x_), y(y_) {}
};

/**
 * @brief Calculate phase progress and lift height
 *
 * @param phase Current gait phase (0.0 to 1.0)
 * @param maxLiftHeight Maximum lift height during swing
 * @param swingDuty Fraction of cycle spent in swing phase (0.0-1.0)
 *                  0.5 = tripod gait (50% swing, 50% stance)
 *                  0.17 = wave gait (~17% swing, ~83% stance)
 * @return PhaseResult with stride progress, lift, and phase type
 */
PhaseResult calculatePhaseProgress(float phase, float maxLiftHeight,
                                   float swingDuty);

/**
 * @brief Calculate XY foot movement for a given stride progress
 *
 * Handles three cases:
 * 1. Straight line motion (turnRate ≈ 0)
 * 2. Pure rotation in place (strideLength ≈ 0)
 * 3. Arc trajectory (combined translation + rotation)
 *
 * @param strideProgress Position along stride (-0.5 to +0.5)
 * @param turnRate Rotation rate (-1.0 to 1.0, positive = turn left)
 * @param heading Direction of travel in radians (0 = forward)
 * @param strideLength Linear stride length in meters
 * @param footX Default foot X position in body frame
 * @param footY Default foot Y position in body frame
 * @return FootDelta with X and Y movement from default position
 */
FootDelta calculateFootDelta(float strideProgress, float turnRate,
                             float heading, float strideLength, float footX,
                             float footY);

} // namespace GaitMath
