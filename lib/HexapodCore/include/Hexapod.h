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
    IDLE,    // Not moving
    WALKING, // Walking forward
    STOPPING // Transitioning to stop
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
   * @brief Start walking forward
   */
  void walk();

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

  // Standing foot position (hip-local)
  static constexpr float STAND_REACH = 0.12f;   // Outward from hip
  static constexpr float STAND_HEIGHT = -0.06f; // Below hip

  // Gait control
  GaitState gaitState_ = IDLE;
  float gaitPhase_ = 0.0f; // 0.0 to 1.0, cycles through gait
  float gaitSpeed_ = 1.0f; // Cycles per second

  // Standing pose (used as reference for gait)
  static constexpr float STAND_COXA = 90.0f;
  static constexpr float STAND_FEMUR = 75.0f;
  static constexpr float STAND_TIBIA = 165.0f;

  // Gait parameters
  static constexpr float STRIDE_LENGTH = 20.0f; // Hip swing in degrees
  static constexpr float LIFT_HEIGHT = 15.0f;   // Femur lift in degrees

  // Tripod groups
  static constexpr int GROUP_A[3] = {FRONT_RIGHT, MIDDLE_LEFT, REAR_RIGHT};
  static constexpr int GROUP_B[3] = {FRONT_LEFT, MIDDLE_RIGHT, REAR_LEFT};

  // Convert leg and joint to servo index
  int getServoIndex(int leg, int joint) const {
    return leg * JOINTS_PER_LEG + joint;
  }

  // Apply gait to a single leg
  void applyGaitToLeg(int leg, float phase);

  // Initialize hip mount configurations
  void initHipMounts();
};
