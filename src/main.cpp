/**
 * Single Servo Test - PCA9685 Channel 0
 *
 * Sweeps a single servo back and forth on the first PCA9685 board
 * (default I2C address 0x40) to verify wiring before connecting more servos.
 *
 * Wiring:
 *   ESP32 SDA (GPIO 21) -> PCA9685 SDA
 *   ESP32 SCL (GPIO 22) -> PCA9685 SCL
 *   PCA9685 V+ -> Servo power supply (e.g. UBEC 5-6V)
 *   PCA9685 VCC -> 3.3V (logic power)
 *   PCA9685 GND -> Common ground
 */

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

// PCA9685 at default I2C address 0x40
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Servo pulse length limits (in microseconds)
// Typical MG996R range: ~500us to ~2500us
static constexpr uint16_t SERVO_MIN_US = 500;
static constexpr uint16_t SERVO_MAX_US = 2500;

// PCA9685 settings
static constexpr uint8_t SERVO_CHANNEL = 0;
static constexpr uint16_t PWM_FREQ = 50; // 50 Hz for standard servos

// Sweep parameters
static constexpr float SWEEP_MIN_ANGLE = 0.0f;
static constexpr float SWEEP_MAX_ANGLE = 180.0f;
static constexpr float SWEEP_STEP = 1.0f;
static constexpr int SWEEP_DELAY_MS = 15;

// Current sweep state
float currentAngle = SWEEP_MIN_ANGLE;
float sweepDirection = SWEEP_STEP;

/**
 * Convert an angle (0-180) to a PCA9685 pulse tick value.
 */
uint16_t angleToPulseTicks(float angle) {
  // Map angle to pulse width in microseconds
  float pulseUs =
      SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

  // Convert microseconds to PCA9685 ticks (4096 ticks per period at 50Hz =
  // 20ms) tick = pulseUs / 20000 * 4096
  return static_cast<uint16_t>(pulseUs / 20000.0f * 4096.0f);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("==========================================");
  Serial.println("  Single Servo Test - PCA9685 Channel 0");
  Serial.println("==========================================");

  Wire.begin(); // SDA=21, SCL=22 (ESP32 defaults)

  pca.begin();
  pca.setPWMFreq(PWM_FREQ);
  delay(10);

  Serial.println("PCA9685 initialized at 0x40");
  Serial.println("Starting sweep on channel 0...");
  Serial.println();

  // Start at center position
  currentAngle = 90.0f;
  pca.setPWM(SERVO_CHANNEL, 0, angleToPulseTicks(currentAngle));
  Serial.println("Moved to center (90 deg). Starting sweep in 2 seconds...");
  delay(2000);

  currentAngle = SWEEP_MIN_ANGLE;
  sweepDirection = SWEEP_STEP;
}

void loop() {
  // Set the servo position
  uint16_t ticks = angleToPulseTicks(currentAngle);
  pca.setPWM(SERVO_CHANNEL, 0, ticks);

  // Print position every 10 degrees
  if (static_cast<int>(currentAngle) % 10 == 0) {
    Serial.printf("Angle: %6.1f°  |  Ticks: %4u\n", currentAngle, ticks);
  }

  // Update angle for next iteration
  currentAngle += sweepDirection;

  // Reverse direction at limits
  if (currentAngle >= SWEEP_MAX_ANGLE) {
    currentAngle = SWEEP_MAX_ANGLE;
    sweepDirection = -SWEEP_STEP;
    Serial.println("--- Reversing (max reached) ---");
  } else if (currentAngle <= SWEEP_MIN_ANGLE) {
    currentAngle = SWEEP_MIN_ANGLE;
    sweepDirection = SWEEP_STEP;
    Serial.println("--- Reversing (min reached) ---");
  }

  delay(SWEEP_DELAY_MS);
}
