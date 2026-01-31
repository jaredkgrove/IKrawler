#include "LegIK.h"
#include <cmath>

LegIK::LegIK(const LegConfig &config) : config_(config) {}

float LegIK::radToDeg(float rad) { return rad * 180.0f / 3.14159265f; }

float LegIK::degToRad(float deg) { return deg * 3.14159265f / 180.0f; }

LegIK::JointAngles LegIK::solve(const FootPosition &target) const {
  JointAngles result;
  result.valid = false;

  // Step 1: Calculate coxa angle (horizontal rotation to point toward target)
  // atan2(y, x) gives angle in XY plane
  float coxaRad = std::atan2(target.y, target.x);
  result.coxa = radToDeg(coxaRad) + 90.0f; // Offset by 90 for servo neutral

  // Step 2: Calculate the distance from coxa joint to foot in the horizontal
  // plane
  float horizontalDist = std::sqrt(target.x * target.x + target.y * target.y);

  // Distance from femur joint to foot (subtract coxa length)
  float femurToFootHoriz = horizontalDist - config_.coxaLength;

  // Step 3: Solve 2D IK in the vertical plane (femur-tibia)
  // We're solving for a 2-link arm reaching a point (femurToFootHoriz,
  // target.z)
  float dx = femurToFootHoriz;
  float dz = target.z; // Usually negative (foot below hip)

  // Distance from femur joint to foot
  float distToFoot = std::sqrt(dx * dx + dz * dz);

  // Check if target is reachable
  float maxReach = config_.femurLength + config_.tibiaLength;
  float minReach = std::fabs(config_.femurLength - config_.tibiaLength);

  if (distToFoot > maxReach || distToFoot < minReach) {
    // Target out of reach
    return result;
  }

  // Use law of cosines to find knee angle
  // c² = a² + b² - 2ab*cos(C)
  // where a = femur, b = tibia, c = distToFoot
  float cosKnee =
      (config_.femurLength * config_.femurLength +
       config_.tibiaLength * config_.tibiaLength - distToFoot * distToFoot) /
      (2.0f * config_.femurLength * config_.tibiaLength);

  // Clamp to valid range for acos
  if (cosKnee < -1.0f)
    cosKnee = -1.0f;
  if (cosKnee > 1.0f)
    cosKnee = 1.0f;

  float kneeAngleRad = std::acos(cosKnee); // Internal angle at knee

  // Step 4: Calculate femur angle
  // Angle from horizontal to the line from femur joint to foot
  // Positive dz = foot above hip, negative dz = foot below hip
  float angleToFoot = std::atan2(dz, dx);

  // Angle between femur and the line to foot (using law of sines)
  float sinAlpha = config_.tibiaLength * std::sin(kneeAngleRad) / distToFoot;
  if (sinAlpha < -1.0f)
    sinAlpha = -1.0f;
  if (sinAlpha > 1.0f)
    sinAlpha = 1.0f;
  float alpha = std::asin(sinAlpha);

  // Femur angle relative to horizontal
  float femurRad = angleToFoot + alpha;
  result.femur =
      90.0f - radToDeg(femurRad); // Convert to servo angle (90 = horizontal)

  // Step 5: Calculate tibia angle
  // Tibia angle relative to femur
  float tibiaRelativeRad = 3.14159265f - kneeAngleRad; // External angle
  result.tibia = 90.0f + radToDeg(tibiaRelativeRad);   // Convert to servo angle

  result.valid = true;
  return result;
}
