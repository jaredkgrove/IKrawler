#pragma once

/**
 * @brief Inverse kinematics solver for a 3-DOF hexapod leg
 *
 * Coordinate system (relative to leg's hip mount):
 * - X: outward from body (positive = away from body)
 * - Y: along body axis (positive = forward for right legs, backward for left)
 * - Z: vertical (positive = up)
 */
class LegIK {
public:
  // Leg segment lengths in meters
  struct LegConfig {
    float coxaLength;  // Hip segment
    float femurLength; // Thigh segment
    float tibiaLength; // Shin segment (to foot)
    float mountAngle;  // Hip mounting angle in radians (relative to body)

    LegConfig()
        : coxaLength(0.02f), femurLength(0.050f), tibiaLength(0.080f),
          mountAngle(0.0f) {}
  };

  // Target foot position relative to hip
  struct FootPosition {
    float x; // Outward from hip
    float y; // Forward/backward
    float z; // Up/down (negative = below hip)

    FootPosition() : x(0.0f), y(0.0f), z(0.0f) {}
  };

  // Resulting joint angles in degrees
  struct JointAngles {
    float coxa;  // Hip rotation
    float femur; // Thigh angle
    float tibia; // Shin angle
    bool valid;  // True if solution found

    JointAngles() : coxa(90.0f), femur(90.0f), tibia(90.0f), valid(false) {}
  };

  /**
   * @brief Construct IK solver with leg configuration
   * @param config Leg segment lengths and mounting
   */
  explicit LegIK(const LegConfig &config = LegConfig());

  /**
   * @brief Solve inverse kinematics for a target foot position
   * @param target Desired foot position relative to hip
   * @return Joint angles (check .valid for success)
   */
  JointAngles solve(const FootPosition &target) const;

  /**
   * @brief Get the leg configuration
   */
  static constexpr float PI = 3.14159265359f;
  const LegConfig &getConfig() const { return config_; }

private:
  LegConfig config_;

  // Convert radians to degrees
  static float radToDeg(float rad);

  // Convert degrees to radians
  static float degToRad(float deg);

  /**
   * @brief Solve 2-link planar IK for femur and tibia
   * @param targetDist Horizontal distance from femur joint to foot
   * @param targetHeight Vertical distance from femur joint to foot (z)
   * @param femurAngle Output femur angle in degrees
   * @param tibiaAngle Output tibia angle in degrees
   * @return true if reachable
   */
  bool solve2LinkIK(float targetDist, float targetHeight, float &femurAngle,
                    float &tibiaAngle) const;
};
