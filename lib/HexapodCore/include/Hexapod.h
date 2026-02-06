#pragma once

#include "LegIK.h"
#include "ServoInterface.h"

/**
 * @brief Main hexapod robot control class
 *
 * Manages 18 servos arranged as:
 * - 6 legs (numbered 0-5, clockwise from front-right)
 * - 3 joints per leg: coxa (hip), femur (thigh), tibia (shin)
 *
 * Coordinate system (body frame):
 * - X: forward (positive = front of robot)
 * - Y: left (positive = left side of robot)
 * - Z: up (positive = above body)
 */
class Hexapod {
public:
  // Leg indices (matches servo numbering in .wbt file)
  enum Leg {
    FRONT_RIGHT = 0,  // servo_0, servo_1, servo_2
    FRONT_LEFT = 1,   // servo_3, servo_4, servo_5
    MIDDLE_RIGHT = 2, // servo_6, servo_7, servo_8
    MIDDLE_LEFT = 3,  // servo_9, servo_10, servo_11
    REAR_RIGHT = 4,   // servo_12, servo_13, servo_14
    REAR_LEFT = 5     // servo_15, servo_16, servo_17
  };

  // Joint indices within each leg
  enum Joint {
    COXA = 0,  // Hip joint (rotation around vertical axis)
    FEMUR = 1, // Thigh joint (rotation around horizontal axis)
    TIBIA = 2  // Shin joint (rotation around horizontal axis)
  };

  // Gait states
  enum GaitState {
    IDLE,        // Not moving
    TRIPOD_WALK, // Tripod gait walking
    STOPPING     // Transitioning to stop
  };

  // Hip mount configuration
  struct HipMount {
    float x;     // X position from body center (meters)
    float y;     // Y position from body center (meters)
    float angle; // Mounting angle in radians (0 = pointing forward)

    HipMount() : x(0.0f), y(0.0f), angle(0.0f) {}
    HipMount(float x_, float y_, float angle_) : x(x_), y(y_), angle(angle_) {}
  };

  static constexpr int NUM_LEGS = 6;
  static constexpr int JOINTS_PER_LEG = 3;
  static constexpr int TOTAL_SERVOS = NUM_LEGS * JOINTS_PER_LEG;

  // Pose percentages (fraction of leg length)
  // These define standing/rest positions relative to geometry
  static constexpr float STANDING_HEIGHT_RATIO = 0.50f; // 50% of leg length
  static constexpr float STANDING_REACH_RATIO = 0.70f;  // 70% of leg length
  static constexpr float REST_HEIGHT_RATIO = 0.25f;     // 25% of leg length
  static constexpr float REST_REACH_RATIO = 0.50f;      // 50% of leg length
  static constexpr float MIN_HEIGHT_RATIO = 0.85f;      // Body lowest
  static constexpr float MAX_HEIGHT_RATIO = 0.25f; // Body highest (crouched)

  /**
   * @brief Constructor
   * @param servoInterface Platform-specific servo implementation
   */
  explicit Hexapod(ServoInterface *servoInterface);

  /**
   * @brief Initialize all servos
   * @param servoPins Array of pin numbers (18 elements)
   * @return true if successful
   */
  bool begin(const int *servoPins);

  /**
   * @brief Set a specific servo angle
   * @param leg Leg index (0-5)
   * @param joint Joint index (0-2)
   * @param angle Angle in degrees
   */
  void setServo(int leg, int joint, float angle);

  /**
   * @brief Get a specific servo angle
   * @param leg Leg index (0-5)
   * @param joint Joint index (0-2)
   * @return Current angle in degrees
   */
  float getServo(int leg, int joint);

  /**
   * @brief Set foot position in hip-local coordinates
   * @param leg Leg index (0-5)
   * @param x Distance outward from hip (meters)
   * @param y Distance forward/backward in leg's frame (meters)
   * @param z Distance up/down from hip (meters, negative = down)
   * @return true if position is reachable
   */
  bool setFootPosition(int leg, float x, float y, float z);

  /**
   * @brief Set foot position in body coordinates
   * @param leg Leg index (0-5)
   * @param x Distance forward from body center (meters)
   * @param y Distance left from body center (meters)
   * @param z Distance up from body center (meters, negative = down)
   * @return true if position is reachable
   */
  bool setFootPositionBody(int leg, float x, float y, float z);

  /**
   * @brief Set body pose by moving all feet relative to body
   * @param x Body shift forward (meters)
   * @param y Body shift left (meters)
   * @param z Body height change (meters, positive = body up/feet down)
   * @param roll Body roll in radians (positive = right side down)
   * @param pitch Body pitch in radians (positive = nose down)
   * @param yaw Body yaw in radians (positive = rotate left)
   * @return true if all positions are reachable
   *
   * Note: This moves feet to achieve the desired body pose.
   * Feet stay planted while body moves.
   */
  bool setBodyPose(float x, float y, float z, float roll, float pitch,
                   float yaw);

  /**
   * @brief Move to standing position
   */
  void stand();

  /**
   * @brief Move to resting position
   */
  void rest();

  /**
   * @brief Start walking with direction and turn control
   */
  void walk();

  /**
   * @brief Set movement heading (direction of travel)
   * @param headingDeg Heading in degrees (0=forward, 90=left, -90=right)
   */
  void setHeading(float headingDeg);

  /**
   * @brief Set turn rate for rotation
   * @param turnRate Rotation rate (-1.0 to 1.0, positive=turn left)
   */
  void setTurnRate(float turnRate);

  /**
   * @brief Set stride length for gait
   * @param length Stride length in meters (clamped to MIN/MAX)
   */
  void setStrideLength(float length);

  /**
   * @brief Get current stride length
   * @return Current stride length in meters
   */
  float getStrideLength() const { return strideLength_; }

  /**
   * @brief Get current maximum stride for current height
   * @return Maximum stride length in meters
   */
  float getMaxStride() const { return maxStride_; }

  /**
   * @brief Set gait speed (cycle frequency)
   * @param speed Gait cycles per second
   */
  void setGaitSpeed(float speed);

  /**
   * @brief Get the maximum stride length for a given body height
   * @param height Current body height (negative, below hip)
   * @return Maximum safe stride length in meters
   *
   * Lower heights (crouched) have shorter max stride to prevent overextension.
   * Higher heights (standing) allow longer strides.
   */
  float getMaxStrideForHeight(float height) const;

  /**
   * @brief Update current body height and recalculate stride limits
   * @param height Body height in meters (negative, below hip)
   *
   * This recalculates the maximum allowed stride based on the new height.
   */
  void updateBodyHeight(float height);

  /**
   * @brief Stop walking (return to standing)
   */
  void stop();

  /**
   * @brief Update hexapod (call in main loop)
   * @param deltaTime Time since last update in seconds
   */
  void update(float deltaTime);

  /**
   * @brief Get the hip mount configuration for a leg
   */
  const HipMount &getHipMount(int leg) const { return hipMounts_[leg]; }

private:
  ServoInterface *servos_;
  float currentAngles_[TOTAL_SERVOS];

  // Inverse kinematics solvers for each leg
  LegIK legIK_[NUM_LEGS];

  // Hip mount positions and angles for each leg
  HipMount hipMounts_[NUM_LEGS];

  // Default standing foot positions in body frame (computed at init)
  struct FootPos {
    float x, y, z;
    FootPos() : x(0), y(0), z(0) {}
    FootPos(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  };
  FootPos defaultFootPos_[NUM_LEGS];

  // Gait control
  GaitState gaitState_ = IDLE;
  float gaitPhase_ = 0.0f; // 0.0 to 1.0, cycles through gait
  float gaitSpeed_ = 1.0f; // Cycles per second

  // Movement control
  float heading_ = 0.0f;  // Direction of travel in radians (0=forward)
  float turnRate_ = 0.0f; // Rotation rate (-1.0 to 1.0, differential steering)
  float strideLength_ = 0.04f; // Stride length in meters
  float currentHeight_ =
      -0.06f;               // Current body height (will be set from geometry)
  float maxStride_ = 0.10f; // Cached max stride (calculated from geometry)

  // Geometry-derived pose values (computed at init from leg dimensions)
  float standingHeight_ = -0.065f; // Computed: -legLength * 0.50
  float standingReach_ = 0.091f;   // Computed: legLength * 0.70
  float restHeight_ = -0.0325f;    // Computed: -legLength * 0.25
  float restReach_ = 0.065f;       // Computed: legLength * 0.50
  float minHeight_ = -0.1105f;     // Computed: -legLength * 0.85
  float maxHeight_ = -0.0325f;     // Computed: -legLength * 0.25

  // Standing pose (used as reference for gait)
  static constexpr float STAND_COXA = 90.0f;
  static constexpr float STAND_FEMUR = 75.0f;
  static constexpr float STAND_TIBIA = 165.0f;

  // Gait parameters
  static constexpr float MIN_STRIDE_LENGTH =
      0.02f;                                    // Minimum stride (safety floor)
  static constexpr float MIN_GAIT_SPEED = 0.5f; // Minimum gait cycles/sec
  static constexpr float MAX_GAIT_SPEED = 2.0f; // Maximum gait cycles/sec
  static constexpr float LIFT_HEIGHT = 0.03f;   // Foot lift height in meters

  // Tripod groups
  static constexpr int GROUP_A[3] = {FRONT_RIGHT, MIDDLE_LEFT, REAR_RIGHT};
  static constexpr int GROUP_B[3] = {FRONT_LEFT, MIDDLE_RIGHT, REAR_LEFT};

  // Convert leg and joint to servo index
  int getServoIndex(int leg, int joint) const {
    return leg * JOINTS_PER_LEG + joint;
  }

  // Apply gait to a single leg
  void applyGaitToLeg(int leg, float phase);

  // Update max stride and re-clamp current stride
  void updateMaxStride_();

  // Update tripod gait
  void updateWalkGait();

  // Initialize hip mount configurations
  void initHipMounts();
};
