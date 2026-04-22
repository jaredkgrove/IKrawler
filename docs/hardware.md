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

Both boards are wired identically: one PCA9685 4-pin bank per leg, front-to-back.
Channels 3, 7, 11 are spare (one per bank); bank 12–15 is entirely free.

### Board 1 — Address 0x40 (no jumpers)
Controls left-side legs (IDs 3–5).

| Leg ID | Position     | Bank | Coxa | Femur | Tibia |
|--------|--------------|------|------|-------|-------|
| 5      | FRONT_LEFT   | 0-3  | 0    | 1     | 2     |
| 4      | MIDDLE_LEFT  | 4-7  | 4    | 5     | 6     |
| 3      | REAR_LEFT    | 8-11 | 8    | 9     | 10    |

### Board 2 — Address 0x41 (A0 jumper soldered)
Controls right-side legs (IDs 0–2).

| Leg ID | Position     | Bank | Coxa | Femur | Tibia |
|--------|--------------|------|------|-------|-------|
| 0      | FRONT_RIGHT  | 0-3  | 0    | 1     | 2     |
| 1      | MIDDLE_RIGHT | 4-7  | 4    | 5     | 6     |
| 2      | REAR_RIGHT   | 8-11 | 8    | 9     | 10    |

Femur and tibia servos are mounted opposite the IK convention, and right-side coxa servos are mirror-mounted; `DualBoardServo` inverts those angles in software (see `INVERT` table).

## Power

- **Servo power (V+):** 6V from UBEC, one UBEC per PCA9685 board
  - Each UBEC rated 10A, handling 9 servos per board
- **Logic power (VCC):** 3.3V from ESP32
- **All grounds tied together** (ESP32, PCA9685s, UBECs)

## I2C Wiring

- ESP32 GPIO 21 (SDA) → both PCA9685 SDA
- ESP32 GPIO 22 (SCL) → both PCA9685 SCL
- Pull-ups: provided on PCA9685 breakout boards

## Known Issues & Notes

- Servo pulse range configured as 500–2500µs in firmware.
