#include "LegIK.h"
#include "Utils.h"
#include <cmath>

LegIK::LegIK(const LegConfig &config) : config_(config) {}

float LegIK::radToDeg(float rad) { return rad * 180.0f / PI; }

float LegIK::degToRad(float deg) { return deg * PI / 180.0f; }

LegIK::JointAngles LegIK::solve(const FootPosition &target) const {
  JointAngles result;
  result.valid = false;

  // Step 1: Calculate coxa angle (horizontal rotation)
  float coxaRad = std::atan2(target.y, target.x);
  result.coxa = radToDeg(coxaRad) + 90.0f;

  // Step 2: Calculate 2D reach in vertical plane
  float horizontalDist = std::sqrt(target.x * target.x + target.y * target.y);
  float femurToFootHoriz = horizontalDist - config_.coxaLength;

  // Step 3: Solve 2D IK for Femur/Tibia
  if (solve2LinkIK(femurToFootHoriz, target.z, result.femur, result.tibia)) {
    result.valid = true;
  }

  return result;
}

bool LegIK::solve2LinkIK(float targetDist, float targetHeight,
                         float &femurAngle, float &tibiaAngle) const {
  // Distance from femur joint to foot
  float distToFoot =
      std::sqrt(targetDist * targetDist + targetHeight * targetHeight);

  // Check reachability
  float maxReach = config_.femurLength + config_.tibiaLength;
  float minReach = std::fabs(config_.femurLength - config_.tibiaLength);

  if (distToFoot > maxReach || distToFoot < minReach) {
    return false;
  }

  // Law of Cosines for Knee Angle
  // c² = a² + b² - 2ab*cos(C) -> cos(C) = (a² + b² - c²) / 2ab
  float numerator =
      (config_.femurLength * config_.femurLength +
       config_.tibiaLength * config_.tibiaLength - distToFoot * distToFoot);
  float denominator = (2.0f * config_.femurLength * config_.tibiaLength);

  float cosKnee = Utils::clamp(numerator / denominator, -1.0f, 1.0f);
  float kneeAngleRad = std::acos(cosKnee);

  // Law of Sines/Trig for Femur Angle
  float angleToFoot = std::atan2(targetHeight, targetDist);

  float sinAlpha = config_.tibiaLength * std::sin(kneeAngleRad) / distToFoot;
  sinAlpha = Utils::clamp(sinAlpha, -1.0f, 1.0f);
  float alpha = std::asin(sinAlpha);

  float femurRad = angleToFoot + alpha;
  float tibiaRelativeRad = PI - kneeAngleRad;

  // Convert to degrees and apply servo offsets (90 = horizontal/neutral)
  femurAngle = 90.0f - radToDeg(femurRad);
  tibiaAngle = 90.0f + radToDeg(tibiaRelativeRad);

  return true;
}
