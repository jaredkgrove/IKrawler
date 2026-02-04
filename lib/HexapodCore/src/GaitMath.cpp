#include "GaitMath.h"
#include <cmath>

namespace GaitMath {

PhaseResult calculatePhaseProgress(float phase, float maxLiftHeight,
                                   float swingDuty) {
  PhaseResult result;

  // Clamp swingDuty to valid range
  if (swingDuty < 0.01f)
    swingDuty = 0.01f;
  if (swingDuty > 0.99f)
    swingDuty = 0.99f;

  float stanceDuty = 1.0f - swingDuty;

  if (phase < swingDuty) {
    // Swing phase: leg lifted and moving forward
    float swingProgress = phase / swingDuty;       // 0 to 1 during swing
    result.strideProgress = -0.5f + swingProgress; // -0.5 to +0.5
    result.liftHeight = std::sin(swingProgress * PI) * maxLiftHeight;
    result.isSwing = true;
  } else {
    // Stance phase: leg on ground, pushing back
    float stanceProgress =
        (phase - swingDuty) / stanceDuty;          // 0 to 1 during stance
    result.strideProgress = 0.5f - stanceProgress; // +0.5 to -0.5
    result.liftHeight = 0.0f;
    result.isSwing = false;
  }

  return result;
}

FootDelta calculateFootDelta(float strideProgress, float turnRate,
                             float heading, float strideLength, float footX,
                             float footY) {
  FootDelta delta;

  if (std::abs(turnRate) < EPSILON) {
    // Case 1: No turning - straight line motion in heading direction
    float strideAmount = strideProgress * strideLength;
    delta.x = strideAmount * std::cos(heading);
    delta.y = strideAmount * std::sin(heading);
  } else if (strideLength < EPSILON) {
    // Case 2: Pure rotation - ICR at body center
    // Each foot moves tangent to its radius from body center
    float radius = std::sqrt(footX * footX + footY * footY);
    if (radius > EPSILON) {
      // Tangent direction (perpendicular to radial)
      // For positive turnRate (CCW), tangent is (-y, +x) normalized
      float tangentX = -footY / radius;
      float tangentY = footX / radius;

      // Arc length = angular_velocity * radius * stride_progress
      float arcLength = strideProgress * ROTATION_STRIDE * turnRate;

      delta.x = arcLength * tangentX;
      delta.y = arcLength * tangentY;
    }
  } else {
    // Case 3: Arc turn - ICR is offset from body center
    // turn_radius = linear_velocity / angular_velocity
    float turnRadius = strideLength / (std::abs(turnRate) * TURN_RATE_SCALE);

    // ICR position: perpendicular to heading direction
    // Positive turnRate = turning left = ICR on left side
    float icrX = 0.0f;
    float icrY = (turnRate > 0) ? turnRadius : -turnRadius;

    // Rotate ICR position by heading angle
    float cosH = std::cos(heading);
    float sinH = std::sin(heading);
    float icrXRotated = icrX * cosH - icrY * sinH;
    float icrYRotated = icrX * sinH + icrY * cosH;

    // Vector from ICR to foot
    float toFootX = footX - icrXRotated;
    float toFootY = footY - icrYRotated;
    float radiusToICR = std::sqrt(toFootX * toFootX + toFootY * toFootY);

    if (radiusToICR > EPSILON) {
      // Tangent direction (perpendicular to radius from ICR)
      float tangentX, tangentY;
      if (turnRate > 0) {
        // CCW rotation: tangent is (-y, +x) relative to ICR
        tangentX = -toFootY / radiusToICR;
        tangentY = toFootX / radiusToICR;
      } else {
        // CW rotation: tangent is (+y, -x) relative to ICR
        tangentX = toFootY / radiusToICR;
        tangentY = -toFootX / radiusToICR;
      }

      // Arc length = body angular velocity * radius_to_ICR * stride_progress
      float bodyAngularVelocity = strideLength / turnRadius;
      float arcLength = strideProgress * bodyAngularVelocity * radiusToICR;

      delta.x = arcLength * tangentX;
      delta.y = arcLength * tangentY;
    }
  }

  return delta;
}

} // namespace GaitMath
