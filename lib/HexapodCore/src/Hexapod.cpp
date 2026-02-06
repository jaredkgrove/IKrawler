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

  // Compute geometry-derived pose values from leg dimensions
  const LegIK::LegConfig &config = legIK_[0].getConfig();
  float legLength = config.femurLength + config.tibiaLength;

  standingHeight_ = -legLength * STANDING_HEIGHT_RATIO;
  standingReach_ = legLength * STANDING_REACH_RATIO;
  restHeight_ = -legLength * REST_HEIGHT_RATIO;
  restReach_ = legLength * REST_REACH_RATIO;
  minHeight_ = -legLength * MIN_HEIGHT_RATIO;
  maxHeight_ = -legLength * MAX_HEIGHT_RATIO;

  // Initialize current height to standing
  currentHeight_ = standingHeight_;
  updateMaxStride_();

  // Compute default foot positions in body frame
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    const HipMount &hip = hipMounts_[leg];
    float cosA = std::cos(hip.angle);
    float sinA = std::sin(hip.angle);

    // Foot position = hip position + (reach * direction)
    defaultFootPos_[leg] =
        FootPos(hip.x + standingReach_ * cosA, // X in body frame
                hip.y + standingReach_ * sinA, // Y in body frame
                standingHeight_                // Z (below body)
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

  // Use cached geometry values (computed at init)
  currentHeight_ = standingHeight_;
  updateMaxStride_();

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    setFootPosition(leg, standingReach_, 0.0f, standingHeight_);
  }
}

void Hexapod::rest() {
  gaitState_ = IDLE;

  // Use cached geometry values (computed at init)
  currentHeight_ = restHeight_;
  updateMaxStride_();

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    setFootPosition(leg, restReach_, 0.0f, restHeight_);
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
  // Clamp to valid range using cached max stride (set by updateBodyHeight)
  strideLength_ = Utils::clamp(length, MIN_STRIDE_LENGTH, maxStride_);
}

void Hexapod::setGaitSpeed(float speed) {
  gaitSpeed_ = Utils::clamp(speed, MIN_GAIT_SPEED, MAX_GAIT_SPEED);
}

float Hexapod::getMaxStrideForHeight(float height) const {
  // Geometry-based stride calculation.
  // During a stride, the foot moves forward/back by stride/2.
  // Peak reach = sqrt(stand_reach² + (stride/2)²)
  // This must be <= leg length (femur + tibia).
  //
  // Solving for max stride:
  // max_stride = 2 * sqrt(leg_length² - reach²)

  // Get leg dimensions from the IK config (use leg 0 as reference)
  const LegIK::LegConfig &config = legIK_[0].getConfig();
  float legLength = config.femurLength + config.tibiaLength;

  // Estimate horizontal reach from height using Pythagorean theorem
  // If leg is at angle, reach² + height² ≈ (some extension factor)²
  // Approximate: reach ≈ sqrt(legLength² * extensionFactor² - height²)
  float heightAbs = std::abs(height);
  float reachSquaredEst =
      (legLength * MIN_HEIGHT_RATIO) * (legLength * MIN_HEIGHT_RATIO) -
      heightAbs * heightAbs;

  float currentReach = (reachSquaredEst > 0) ? std::sqrt(reachSquaredEst)
                                             : legLength * REST_REACH_RATIO;
  currentReach = Utils::clamp(currentReach, 0.03f, legLength * 0.9f);

  // Calculate max stride from geometry
  float reachSquared = currentReach * currentReach;
  float legSquared = legLength * legLength;

  if (reachSquared >= legSquared) {
    // Already at or beyond leg limit - return minimum
    return MIN_STRIDE_LENGTH;
  }

  float maxStrideRaw = 2.0f * std::sqrt(legSquared - reachSquared);

  // Apply safety margin (80% of kinematic limit)
  constexpr float SAFETY_MARGIN = 0.8f;
  float maxStride = maxStrideRaw * SAFETY_MARGIN;

  // Clamp to sensible bounds
  return Utils::clamp(maxStride, MIN_STRIDE_LENGTH, 0.15f);
}

void Hexapod::updateBodyHeight(float height) {
  // Use cached height limits (computed at init from geometry)
  currentHeight_ = Utils::clamp(height, minHeight_, maxHeight_);

  // Recalculate max stride and ensure current stride respects it
  updateMaxStride_();
}

void Hexapod::updateMaxStride_() {
  maxStride_ = getMaxStrideForHeight(currentHeight_);
  // Re-clamp current stride to maintain invariant: strideLength_ <= maxStride_
  setStrideLength(strideLength_);
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
