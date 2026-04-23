#pragma once

#include "PCA9685ServoImpl.h"
#include "ServoInterface.h"

/**
 * @brief Routes 18 logical servo IDs across two PCA9685 boards.
 *
 * Hexapod uses servo IDs 0–17 (6 legs × 3 joints).
 * This adapter maps each ID to the correct PCA9685 board and channel
 * using the pin tables from hardware.md.
 *
 * Leg numbering and pin mapping:
 *   Board 1 (0x40, no jumper):  Legs 0-2 (right side: front, middle, rear)
 *   Board 2 (0x41, A0 jumper):  Legs 3-5 (left side: rear, middle, front)
 *
 *   Each leg uses 3 channels within a PCA9685 4-pin bank: [coxa, femur, tibia]
 *   Board 1 is mounted rotated 180° from board 2, so its front/rear banks
 *   are flipped relative to board 2:
 *     Board 1 (right):  FR=bank 8-11, MR=bank 4-7,  RR=bank 0-3
 *     Board 2 (left):   FL=bank 0-3,  ML=bank 4-7,  RL=bank 8-11
 *
 * Per-servo neutral-position offsets (degrees) compensate for mechanical
 * mounting imperfections. They are added to the commanded angle before
 * writing to the PCA9685, and subtracted on read so the IK's view is
 * offset-agnostic. Offsets persist in NVS (Preferences).
 */
class DualBoardServo : public ServoInterface {
public:
  static constexpr int NUM_SERVOS = 18;
  static constexpr float MAX_OFFSET_DEG = 30.0f;

  DualBoardServo();

  // Initialize both PCA9685 boards and load any saved offsets from NVS.
  bool begin();

  // ServoInterface
  bool attach(int servoId, int pin) override;
  void write(int servoId, float angle) override;
  float read(int servoId) override;
  void detach(int servoId) override;

  // ── Calibration ──
  float getOffset(int servoId) const;
  void setOffset(int servoId, float offset);
  void resetOffsets();
  bool saveOffsets(); // persist current offsets to NVS
  void loadOffsets(); // load from NVS; missing data → all zeros

private:
  struct Resolved {
    PCA9685ServoImpl *board;
    uint8_t channel;
  };
  Resolved resolve(int servoId) const;

  PCA9685ServoImpl board1_; // 0x40 (no jumper), drives right-side legs
  PCA9685ServoImpl board2_; // 0x41 (A0 jumper),  drives left-side legs
  float offsets_[NUM_SERVOS];
};
