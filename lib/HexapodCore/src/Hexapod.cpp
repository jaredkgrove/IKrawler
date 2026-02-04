#include "Hexapod.h"
#include "GaitMath.h"
#include "Utils.h"
#include <cmath>

// Define static constexpr arrays
constexpr int Hexapod::GROUP_A[3];
constexpr int Hexapod::GROUP_B[3];

Hexapod::Hexapod(ServoInterface *servoInterface) : servos_(servoInterface) {
  // Initialize all angles to 90 degrees (neutral)
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    currentAngles_[i] = 90.0f;
  }
  // Initialize hip mount positions
  initHipMounts();
}

bool Hexapod::begin(const int *servoPins) {
  // Attach all servos
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    if (!servos_->attach(i, servoPins[i])) {
      return false;
    }
    // Set to neutral position
    servos_->write(i, currentAngles_[i]);
  }
  return true;
}

void Hexapod::setServo(int leg, int joint, float angle) {
  int index = getServoIndex(leg, joint);
  if (index >= 0 && index < TOTAL_SERVOS) {
    currentAngles_[index] = angle;
    servos_->write(index, angle);
  }
}

float Hexapod::getServo(int leg, int joint) {
  int index = getServoIndex(leg, joint);
  if (index >= 0 && index < TOTAL_SERVOS) {
    return currentAngles_[index];
  }
  return 0.0f;
}

bool Hexapod::setFootPosition(int leg, float x, float y, float z) {
  if (leg < 0 || leg >= NUM_LEGS) {
    return false;
  }

  // Create target foot position
  LegIK::FootPosition target;
  target.x = x;
  target.y = y;
  target.z = z;

  // Solve IK
  LegIK::JointAngles angles = legIK_[leg].solve(target);

  if (!angles.valid) {
    return false;
  }

  // Apply joint angles
  setServo(leg, COXA, angles.coxa);
  setServo(leg, FEMUR, angles.femur);
  setServo(leg, TIBIA, angles.tibia);

  return true;
}

bool Hexapod::setFootPositionBody(int leg, float x, float y, float z) {
  if (leg < 0 || leg >= NUM_LEGS) {
    return false;
  }

  const HipMount &hip = hipMounts_[leg];

  // Transform from body coordinates to hip-local coordinates
  // 1. Translate: subtract hip position from target
  float dx = x - hip.x;
  float dy = y - hip.y;

  // 2. Rotate: transform into leg's local frame (rotate by -hip.angle)
  float cosA = std::cos(-hip.angle);
  float sinA = std::sin(-hip.angle);

  // In leg frame: x = outward, y = forward along leg direction
  float legX = dx * cosA - dy * sinA; // Outward from hip
  float legY = dx * sinA + dy * cosA; // Forward in leg's frame

  // Z stays the same (vertical)
  float legZ = z;

  // Now use hip-local IK
  return setFootPosition(leg, legX, legY, legZ);
}

void Hexapod::initHipMounts() {
  // Hip positions and angles for symmetric radial design (60° spacing)
  // Body coordinate system: X = forward, Y = left, Z = up
  // Angles: 0 = pointing forward, positive = counterclockwise
  // All hips at radius 0.05m from body center

  // Front legs: angle = ±30°
  hipMounts_[FRONT_RIGHT] =
      HipMount(0.0433f, -0.025f, -LegIK::PI / 6.0f);                    // -30°
  hipMounts_[FRONT_LEFT] = HipMount(0.0433f, 0.025f, LegIK::PI / 6.0f); // +30°

  // Middle legs: angle = ±90°
  hipMounts_[MIDDLE_RIGHT] = HipMount(0.0f, -0.05f, -LegIK::PI / 2.0f); // -90°
  hipMounts_[MIDDLE_LEFT] = HipMount(0.0f, 0.05f, LegIK::PI / 2.0f);    // +90°

  // Rear legs: angle = ±150°
  hipMounts_[REAR_RIGHT] =
      HipMount(-0.0433f, -0.025f, -5.0f * LegIK::PI / 6.0f); // -150°
  hipMounts_[REAR_LEFT] =
      HipMount(-0.0433f, 0.025f, 5.0f * LegIK::PI / 6.0f); // +150°

  // Compute default foot positions in body frame
  // Each foot extends STAND_REACH from hip at the hip's angle
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    const HipMount &hip = hipMounts_[leg];
    float cosA = std::cos(hip.angle);
    float sinA = std::sin(hip.angle);

    // Foot position = hip position + (reach * direction)
    defaultFootPos_[leg] =
        FootPos(hip.x + STAND_REACH * cosA, // X in body frame
                hip.y + STAND_REACH * sinA, // Y in body frame
                STAND_HEIGHT                // Z (below body)
        );
  }
}

bool Hexapod::setBodyPose(float x, float y, float z, float roll, float pitch,
                          float yaw) {
  // To move the body, we move all feet in the opposite direction
  // Body pose (x, y, z, roll, pitch, yaw) means feet move (-x, -y, -z) and
  // rotate inversely

  // Precompute rotation matrix elements (inverse rotation = negative angles)
  float cr = std::cos(-roll);
  float sr = std::sin(-roll);
  float cp = std::cos(-pitch);
  float sp = std::sin(-pitch);
  float cy = std::cos(-yaw);
  float sy = std::sin(-yaw);

  bool allSuccess = true;

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    // Start with default foot position
    float fx = defaultFootPos_[leg].x;
    float fy = defaultFootPos_[leg].y;
    float fz = defaultFootPos_[leg].z;

    // Apply inverse body translation (move feet opposite to body)
    fx -= x;
    fy -= y;
    fz -= z;

    // Apply inverse rotation (rotate feet around body center)
    // Using ZYX Euler angles (yaw, pitch, roll)
    // Rotation matrix: Rz(yaw) * Ry(pitch) * Rx(roll)

    // Step 1: Rotate around X (roll)
    float y1 = fy * cr - fz * sr;
    float z1 = fy * sr + fz * cr;

    // Step 2: Rotate around Y (pitch)
    float x2 = fx * cp + z1 * sp;
    float z2 = -fx * sp + z1 * cp;

    // Step 3: Rotate around Z (yaw)
    float x3 = x2 * cy - y1 * sy;
    float y3 = x2 * sy + y1 * cy;

    // Set foot position in body frame
    if (!setFootPositionBody(leg, x3, y3, z2)) {
      allSuccess = false;
    }
  }

  return allSuccess;
}

void Hexapod::stand() {
  gaitState_ = IDLE;
  // Standing position
  // Foot position: x = outward reach, y = 0 (centered), z = height below hip
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    setFootPosition(leg, STAND_REACH, 0.0f, STAND_HEIGHT);
  }
}

void Hexapod::rest() {
  gaitState_ = IDLE;
  // Resting position - body lowered
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    setFootPosition(leg, STAND_REACH, 0.0f, REST_HEIGHT);
  }
}

void Hexapod::walk() {
  // Start walking if not already
  if (gaitState_ != TRIPOD_WALK) {
    gaitState_ = TRIPOD_WALK;
    gaitPhase_ = 0.0f;
  }
}

void Hexapod::setHeading(float headingDeg) {
  heading_ = headingDeg * LegIK::PI / 180.0f;
}

void Hexapod::setTurnRate(float turnRate) {
  turnRate_ = Utils::clamp(turnRate, -1.0f, 1.0f);
}

void Hexapod::setStrideLength(float length) {
  // Clamp to valid range
  strideLength_ = Utils::clamp(length, MIN_STRIDE_LENGTH, MAX_STRIDE_LENGTH);
}

void Hexapod::setGaitSpeed(float speed) {
  // Clamp to valid range
  gaitSpeed_ = Utils::clamp(speed, MIN_GAIT_SPEED, MAX_GAIT_SPEED);
}

void Hexapod::stop() {
  if (gaitState_ == TRIPOD_WALK) {
    gaitState_ = STOPPING;
  }
}

void Hexapod::applyGaitToLeg(int leg, float phase) {
  // Calculate phase progress and lift height
  GaitMath::PhaseResult phaseResult =
      GaitMath::calculatePhaseProgress(phase, LIFT_HEIGHT, 0.5f);

  // Calculate XY movement based on heading and turn rate
  GaitMath::FootDelta delta = GaitMath::calculateFootDelta(
      phaseResult.strideProgress, turnRate_, heading_, strideLength_,
      defaultFootPos_[leg].x, defaultFootPos_[leg].y);

  // Apply foot position in body frame
  setFootPositionBody(leg, defaultFootPos_[leg].x + delta.x,
                      defaultFootPos_[leg].y + delta.y,
                      defaultFootPos_[leg].z + phaseResult.liftHeight);
}

void Hexapod::update(float deltaTime) {
  if (gaitState_ == IDLE) {
    return;
  }

  // Update gait phase
  gaitPhase_ += deltaTime * gaitSpeed_;

  // Check if we completed a cycle
  bool cycleCompleted = false;
  if (gaitPhase_ >= 1.0f) {
    gaitPhase_ = Utils::wrapPhase(gaitPhase_);
    cycleCompleted = true;
  }

  // If stopping and we just completed a cycle, return to standing
  if (gaitState_ == STOPPING && cycleCompleted) {
    stand();
    return;
  }

  if (gaitState_ == TRIPOD_WALK) {
    updateWalkGait();
  }
}

void Hexapod::updateWalkGait() {
  // Tripod gait: Group A and Group B are 180 degrees out of phase
  float phaseA = gaitPhase_;
  float phaseB = Utils::wrapPhase(gaitPhase_ + 0.5f);

  // Apply gait to Group A legs
  for (int i = 0; i < 3; i++) {
    applyGaitToLeg(GROUP_A[i], phaseA);
  }

  // Apply gait to Group B legs
  for (int i = 0; i < 3; i++) {
    applyGaitToLeg(GROUP_B[i], phaseB);
  }
}
