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
- **ESP32 power:** fed from the servo `V+` rail (6V) through a 5V buck converter
  into the ESP32 `5V` pin
- **All grounds tied together** (ESP32, PCA9685s, UBECs)

### ⚠️ USB back-feed hazard (and the fix)

Because the ESP32 is powered from the `V+` rail via the buck, plugging in USB
with **no battery** energizes the servos. A buck converter is not a one-way
valve: USB 5V lands on the ESP32 `5V` pin (= the buck output) and flows
*backward* through the buck's high-side body diode into `V+`, powering all 18
servos off the USB port. That pushes servo current through the USB connector and
board traces (rated for mA, not amps) and can damage the ESP32 or the host port.
With both USB and battery connected, the two supplies also fight on the shared
rail.

**Fix:** add a Schottky diode (e.g. SS34 or 1N5819) on the **buck output**,
anode at the buck, cathode at the ESP32 `5V` pin.

- Battery on: buck → diode → ESP32 (diode drops ~0.3V, so set the buck to
  ~5.2–5.3V to land at 5V).
- USB only: diode is reverse-biased and blocks; servos stay limp — safe to flash.
- Both connected: safe; the buck only outputs 5V and the diode stops any
  back-feed into `V+`.

Use a Schottky specifically (low forward drop); a silicon diode would lose
~0.7V.

## I2C Wiring

- ESP32-S3 GPIO 8 (SDA) → both PCA9685 SDA
- ESP32-S3 GPIO 9 (SCL) → both PCA9685 SCL
- Pull-ups: provided on PCA9685 breakout boards

## Known Issues & Notes

- Servo pulse range configured as 500–2500µs in firmware.
