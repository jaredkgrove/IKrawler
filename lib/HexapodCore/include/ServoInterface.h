#pragma once

/**
 * @brief Abstract interface for servo control
 * 
 * This interface allows the hexapod logic to be platform-independent.
 * Different platforms (ESP32, Webots) implement this interface.
 */
class ServoInterface {
public:
    virtual ~ServoInterface() = default;
    
    /**
     * @brief Attach servo to a pin/device
     * @param servoId Servo identifier (0-17 for 18 servos)
     * @param pin Pin number or device name
     * @return true if successful
     */
    virtual bool attach(int servoId, int pin) = 0;
    
    /**
     * @brief Set servo position
     * @param servoId Servo identifier (0-17)
     * @param angle Angle in degrees (0-180)
     */
    virtual void write(int servoId, float angle) = 0;
    
    /**
     * @brief Read current servo position
     * @param servoId Servo identifier (0-17)
     * @return Current angle in degrees
     */
    virtual float read(int servoId) = 0;
    
    /**
     * @brief Detach servo
     * @param servoId Servo identifier (0-17)
     */
    virtual void detach(int servoId) = 0;
};
