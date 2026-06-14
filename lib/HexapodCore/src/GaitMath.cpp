#include "GaitMath.h"
#include <cmath>

namespace GaitMath {

namespace {
// Smoothstep on [0,1]: 3u^2 - 2u^3. Its first derivative is zero at BOTH u=0
// and u=1, so ramps built from it meet the ground (and the plateau) with zero
// vertical velocity. A soft touchdown means the landing tripod is not slammed
// into the ground every step -- that impact is the main source of the per-step
// body bounce, especially on the compliant physical robot.
inline float smoothstep(float u) { return u * u * (3.0f - 2.0f * u); }
} // namespace

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
    // Swing phase: leg lifted and moving forward.
    float swingProgress = phase / swingDuty; // 0 to 1 during swing

    // Horizontal: cycloid. A linear or cosine-eased ramp leaves a velocity JUMP
    // at the swing<->stance boundary -- stance sweeps the foot backward at a
    // constant rate (-1/stanceDuty per phase), but those curves arrive at the
    // boundary with a different (for the cosine, zero) slope. On the physical
    // robot a slew-limited servo cannot cancel its stance velocity instantly,
    // so it overshoots through that jump and scrubs the foot across the ground
    // right at liftoff -- worst on the middle legs, whose coxa carries the
    // entire fore/aft sweep and so has the most velocity to overshoot.
    //
    // The cycloid removes the jump: its slope at BOTH endpoints equals the
    // stance slope, so foot velocity is continuous across the boundary.
    // Equivalently the foot's velocity relative to the GROUND is ~0 at liftoff
    // and touchdown (it keeps pace with the ground as it leaves and lands) --
    // the textbook scuff-free property.
    //
    //   strideProgress(s) = -0.5 + s + a*sin(2*pi*s),  a = -1/(2*pi*stanceDuty)
    //
    // Endpoints are preserved (-0.5 at s=0, +0.5 at s=1) so it still meshes with
    // the stance ramp. The interior is non-monotonic (a small backward dip just
    // after liftoff, a small forward overshoot just before touchdown) -- this is
    // unavoidable when both endpoint velocities point backward, and harmless
    // since the foot is lifted clear of the ground there.
    float a = -1.0f / (2.0f * PI * stanceDuty);
    result.strideProgress =
        -0.5f + swingProgress + a * std::sin(2.0f * PI * swingProgress);

    // Vertical: trapezoidal "flat-top" arc. Smoothstep rise, a flat hold at peak
    // across the middle (SWING_PLATEAU of the swing), then a smoothstep fall.
    // Smoothstep has zero slope at both ends of each ramp, so the foot leaves
    // and -- crucially -- lands with zero vertical velocity (soft touchdown, no
    // impact bounce) while still reaching full clearance across the plateau.
    // Symmetric, so neither end of swing is favored. Continuous: each ramp
    // reaches maxLiftHeight at the plateau edge and 0 at the endpoint.
    float ramp = 0.5f * (1.0f - SWING_PLATEAU); // fraction for each ramp
    if (ramp < EPSILON) {
      // Degenerate (plateau ~ full swing): square pulse, peak across interior.
      result.liftHeight = (swingProgress <= 0.0f || swingProgress >= 1.0f)
                              ? 0.0f
                              : maxLiftHeight;
    } else if (swingProgress < ramp) {
      result.liftHeight = smoothstep(swingProgress / ramp) * maxLiftHeight;
    } else if (swingProgress > 1.0f - ramp) {
      result.liftHeight =
          (1.0f - smoothstep((swingProgress - (1.0f - ramp)) / ramp)) *
          maxLiftHeight;
    } else {
      result.liftHeight = maxLiftHeight;
    }
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
  // Holonomic superposition: foot moves through (T + omega x r) per cycle,
  // where T is the body translation vector and omega is the body angular
  // displacement per cycle. strideProgress samples that span across the gait.
  //
  // Translation contribution: vector along heading, magnitude = strideLength.
  float Tx = strideLength * std::cos(heading);
  float Ty = strideLength * std::sin(heading);

  // Rotation contribution: omega x r in 2D = (-omega * footY, +omega * footX).
  float omega = turnRate * MAX_ROTATION_PER_CYCLE;
  float Rx = -omega * footY;
  float Ry = omega * footX;

  return FootDelta(strideProgress * (Tx + Rx), strideProgress * (Ty + Ry));
}

} // namespace GaitMath
