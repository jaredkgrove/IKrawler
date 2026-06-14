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

// Angular displacement per gait cycle at turnRate = 1.0 (radians).
// Picked to roughly match the previous pure-rotation feel at the default body
// radius (~0.04 m arc / ~0.18 m radius).
constexpr float MAX_ROTATION_PER_CYCLE = 0.23f;

constexpr float PI = 3.14159265359f;

// Fraction of the swing phase the foot is held at peak lift (the flat top of
// the trapezoidal swing arc). The remaining time splits evenly into a rise and
// a fall ramp, so the foot clears the ground steeply at BOTH liftoff and
// touchdown and stays high while it translates forward — this avoids scuffing
// carpet tops (and the resulting stance-tripod slip) without biasing one end of
// swing over the other. Must stay in [0, 1); 0 reduces to a symmetric sine arc.
constexpr float SWING_PLATEAU = 0.7f;

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
