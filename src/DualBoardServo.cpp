#include "DualBoardServo.h"
#include <Preferences.h>

namespace {

// PCA9685 channel assignments per leg (coxa, femur, tibia).
// Leg IDs go clockwise from front-right: 0..2 are right-side, 3..5 left-side.
// Each leg occupies one PCA9685 4-pin bank: front=0-3, middle=4-7, rear=8-11.
// Both boards are wired identically (same position → same channels).
// Right-side legs are on board 2 (A0 jumper, addr 0x41); left-side
// legs are on board 1 (no jumper, addr 0x40). See hardware.md for details.
constexpr uint8_t CHANNEL_MAP[6][3] = {
    {0, 1, 2},   // 0: FRONT_RIGHT  (Bd 2, bank 0-3)
    {4, 5, 6},   // 1: MIDDLE_RIGHT (Bd 2, bank 4-7)
    {8, 9, 10},  // 2: REAR_RIGHT   (Bd 2, bank 8-11)
    {8, 9, 10},  // 3: REAR_LEFT    (Bd 1, bank 8-11)
    {4, 5, 6},   // 4: MIDDLE_LEFT  (Bd 1, bank 4-7)
    {0, 1, 2},   // 5: FRONT_LEFT   (Bd 1, bank 0-3)
};

// Per-servo inversion: when true, the angle is mirrored around 90° before
// being sent to the PCA9685 (and reversed on read). Required because the
// physical servo orientation differs from the IK convention:
//   - Femur and tibia are inverted on every leg.
//   - Coxa is inverted only on right-side legs (mirror-mounted vs left).
constexpr bool INVERT[6][3] = {
    {true,  true, true}, // 0: FRONT_RIGHT
    {true,  true, true}, // 1: MIDDLE_RIGHT
    {true,  true, true}, // 2: REAR_RIGHT
    {false, true, true}, // 3: REAR_LEFT
    {false, true, true}, // 4: MIDDLE_LEFT
    {false, true, true}, // 5: FRONT_LEFT
};

constexpr const char *PREF_NAMESPACE = "hexapod";
constexpr const char *PREF_KEY = "offsets";

} // namespace

DualBoardServo::DualBoardServo() : board1_(0x40), board2_(0x41) {
  for (int i = 0; i < NUM_SERVOS; i++)
    offsets_[i] = 0.0f;
}

bool DualBoardServo::begin() {
  if (!board1_.begin())
    return false;
  if (!board2_.begin())
    return false;
  loadOffsets();
  return true;
}

bool DualBoardServo::attach(int servoId, int /*pin*/) {
  if (servoId < 0 || servoId >= NUM_SERVOS)
    return false;
  Resolved res = resolve(servoId);
  return res.board->attach(res.channel, res.channel);
}

void DualBoardServo::write(int servoId, float angle) {
  if (servoId < 0 || servoId >= NUM_SERVOS)
    return;
  Resolved res = resolve(servoId);
  int leg = servoId / 3;
  int joint = servoId % 3;
  float adjusted = angle + offsets_[servoId];
  if (INVERT[leg][joint])
    adjusted = 180.0f - adjusted;
  // PCA9685ServoImpl clamps to [0, 180], but clamp explicitly for clarity.
  if (adjusted < 0.0f)
    adjusted = 0.0f;
  if (adjusted > 180.0f)
    adjusted = 180.0f;
  res.board->write(res.channel, adjusted);
}

float DualBoardServo::read(int servoId) {
  if (servoId < 0 || servoId >= NUM_SERVOS)
    return 0.0f;
  Resolved res = resolve(servoId);
  int leg = servoId / 3;
  int joint = servoId % 3;
  float raw = res.board->read(res.channel);
  if (INVERT[leg][joint])
    raw = 180.0f - raw;
  return raw - offsets_[servoId];
}

void DualBoardServo::detach(int servoId) {
  if (servoId < 0 || servoId >= NUM_SERVOS)
    return;
  Resolved res = resolve(servoId);
  res.board->detach(res.channel);
}

float DualBoardServo::getOffset(int servoId) const {
  if (servoId < 0 || servoId >= NUM_SERVOS)
    return 0.0f;
  return offsets_[servoId];
}

void DualBoardServo::setOffset(int servoId, float offset) {
  if (servoId < 0 || servoId >= NUM_SERVOS)
    return;
  if (offset < -MAX_OFFSET_DEG)
    offset = -MAX_OFFSET_DEG;
  if (offset > MAX_OFFSET_DEG)
    offset = MAX_OFFSET_DEG;
  offsets_[servoId] = offset;
}

void DualBoardServo::resetOffsets() {
  for (int i = 0; i < NUM_SERVOS; i++)
    offsets_[i] = 0.0f;
}

bool DualBoardServo::saveOffsets() {
  Preferences prefs;
  if (!prefs.begin(PREF_NAMESPACE, /*readOnly=*/false))
    return false;
  size_t written = prefs.putBytes(PREF_KEY, offsets_, sizeof(offsets_));
  prefs.end();
  return written == sizeof(offsets_);
}

void DualBoardServo::loadOffsets() {
  Preferences prefs;
  if (!prefs.begin(PREF_NAMESPACE, /*readOnly=*/true)) {
    resetOffsets();
    return;
  }
  size_t stored = prefs.getBytesLength(PREF_KEY);
  if (stored == sizeof(offsets_)) {
    prefs.getBytes(PREF_KEY, offsets_, sizeof(offsets_));
  } else {
    resetOffsets();
  }
  prefs.end();
}

DualBoardServo::Resolved DualBoardServo::resolve(int servoId) const {
  int leg = servoId / 3;
  int joint = servoId % 3;
  // Right-side legs (0..2) are on board 2 (A0 jumper, 0x41);
  // left-side legs (3..5) are on board 1 (no jumper, 0x40).
  bool isRightSide = (leg < 3);
  return {const_cast<PCA9685ServoImpl *>(isRightSide ? &board2_ : &board1_),
          CHANNEL_MAP[leg][joint]};
}
