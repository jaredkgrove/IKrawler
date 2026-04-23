# Hexapod Hardware Reference

## Components

| Component | Quantity | Details |
|-----------|----------|---------|
| ESP32 | 1 | Main microcontroller |
| PCA9685 (Adafruit) | 2 | 16-channel PWM servo drivers, I2C |
| MG996R Servos | 18 | 3 per leg (coxa, femur, tibia) × 6 legs |
| UBEC | 2 | 6V output, 10A rated, one per PCA9685 |

## PCA9685 Boards

Leg IDs follow the firmware convention: clockwise from front-right (top-down view).

Each leg occupies one PCA9685 4-pin bank. Channels 3, 7, 11 are spare (one per
bank); bank 12–15 is entirely free.

Board 1 (right side) is mounted rotated 180° from board 2, so its front/rear
bank assignments are flipped. Middle banks line up the same on both boards.

### Board 1 — Address 0x40 (no jumpers)
Controls right-side legs (IDs 0–2).

| Leg ID | Position     | Bank | Coxa | Femur | Tibia |
|--------|--------------|------|------|-------|-------|
| 2      | REAR_RIGHT   | 0-3  | 0    | 1     | 2     |
| 1      | MIDDLE_RIGHT | 4-7  | 4    | 5     | 6     |
| 0      | FRONT_RIGHT  | 8-11 | 8    | 9     | 10    |

### Board 2 — Address 0x41 (A0 jumper soldered)
Controls left-side legs (IDs 3–5).

| Leg ID | Position     | Bank | Coxa | Femur | Tibia |
|--------|--------------|------|------|-------|-------|
| 5      | FRONT_LEFT   | 0-3  | 0    | 1     | 2     |
| 4      | MIDDLE_LEFT  | 4-7  | 4    | 5     | 6     |
| 3      | REAR_LEFT    | 8-11 | 8    | 9     | 10    |

All 18 servos are mounted opposite the IK convention (horn orientation is consistent across sides on this build, not mirrored). `DualBoardServo` inverts every angle in software — see the `INVERT` table.

## Power

- **Servo power (V+):** 6V from UBEC, one UBEC per PCA9685 board
  - Each UBEC rated 10A, handling 9 servos per board
- **Logic power (VCC):** 3.3V from ESP32
- **All grounds tied together** (ESP32, PCA9685s, UBECs)

## I2C Wiring

- ESP32-S3 GPIO 8 (SDA) → both PCA9685 SDA
- ESP32-S3 GPIO 9 (SCL) → both PCA9685 SCL
- Pull-ups: provided on PCA9685 breakout boards

## Known Issues & Notes

- Servo pulse range configured as 500–2500µs in firmware.
