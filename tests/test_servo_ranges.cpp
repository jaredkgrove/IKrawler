// Diagnostic test: Calculate and print actual servo angle ranges
// This helps understand what range each joint needs for normal operation

#include "../lib/HexapodCore/include/Hexapod.h"
#include "../lib/HexapodCore/include/LegIK.h"
#include <algorithm>
#include <iomanip>
#include <iostream>


// Get leg length from default config
float getLegLength() {
  LegIK ik;
  const LegIK::LegConfig &config = ik.getConfig();
  return config.femurLength + config.tibiaLength;
}

void printAngles(const char *label, const LegIK::JointAngles &angles) {
  std::cout << std::setw(20) << label << " | "
            << "Coxa: " << std::setw(7) << std::fixed << std::setprecision(1)
            << angles.coxa << "° | "
            << "Femur: " << std::setw(7) << angles.femur << "° | "
            << "Tibia: " << std::setw(7) << angles.tibia << "°";
  if (!angles.valid)
    std::cout << " [INVALID]";
  std::cout << "\n";
}

int main() {
  LegIK ik;
  float legLength = getLegLength();

  std::cout << "\n========================================\n";
  std::cout << "  SERVO ANGLE RANGE ANALYSIS\n";
  std::cout << "========================================\n\n";

  std::cout << "Leg geometry: femur=" << ik.getConfig().femurLength * 1000
            << "mm, "
            << "tibia=" << ik.getConfig().tibiaLength * 1000 << "mm, "
            << "total=" << legLength * 1000 << "mm\n\n";

  // Standing pose
  float standHeight = -legLength * Hexapod::STANDING_HEIGHT_RATIO;
  float standReach = legLength * Hexapod::STANDING_REACH_RATIO;

  // Rest pose
  float restHeight = -legLength * Hexapod::REST_HEIGHT_RATIO;
  float restReach = legLength * Hexapod::REST_REACH_RATIO;

  // Min/Max heights
  float minHeight = -legLength * Hexapod::MIN_HEIGHT_RATIO;
  float maxHeight = -legLength * Hexapod::MAX_HEIGHT_RATIO;

  std::cout << "--- STATIC POSES (foot straight out, y=0) ---\n";
  std::cout << std::string(65, '-') << "\n";

  LegIK::FootPosition target;
  target.y = 0.0f;

  // Standing
  target.x = standReach;
  target.z = standHeight;
  printAngles("Standing", ik.solve(target));

  // Rest
  target.x = restReach;
  target.z = restHeight;
  printAngles("Rest (crouched)", ik.solve(target));

  // Min height (body lowest)
  target.x = restReach;
  target.z = minHeight;
  printAngles("Min height", ik.solve(target));

  // Max height (body highest)
  target.x = restReach;
  target.z = maxHeight;
  printAngles("Max height", ik.solve(target));

  std::cout << "\n--- WALKING GAIT EXTREMES ---\n";
  std::cout << "(Standing pose with stride forward/back)\n";
  std::cout << std::string(65, '-') << "\n";

  // Estimate max stride for standing
  float strideLen = 0.04f; // 4cm stride (typical)

  // Foot at front of stride
  target.x = standReach + strideLen / 2;
  target.z = standHeight;
  printAngles("Stride front", ik.solve(target));

  // Foot at back of stride
  target.x = standReach - strideLen / 2;
  target.z = standHeight;
  printAngles("Stride back", ik.solve(target));

  // Foot lifted (mid-swing)
  target.x = standReach;
  target.z = standHeight + 0.03f; // 3cm lift
  printAngles("Foot lifted 3cm", ik.solve(target));

  std::cout << "\n--- COXA RANGE (lateral foot movement) ---\n";
  std::cout << std::string(65, '-') << "\n";

  // Foot rotated forward
  target.x = standReach * 0.866f; // 30 degrees forward
  target.y = standReach * 0.5f;
  target.z = standHeight;
  printAngles("30° forward", ik.solve(target));

  // Foot straight out
  target.x = standReach;
  target.y = 0.0f;
  target.z = standHeight;
  printAngles("Straight out", ik.solve(target));

  // Foot rotated backward
  target.x = standReach * 0.866f;
  target.y = -standReach * 0.5f;
  target.z = standHeight;
  printAngles("30° backward", ik.solve(target));

  std::cout << "\n--- SUMMARY ---\n";
  std::cout << "90° = servo neutral/center position\n";
  std::cout << "Ideal: All poses should produce angles in ~30°-150° range\n";
  std::cout << "       (leaving margin from 0° and 180° limits)\n\n";

  return 0;
}
