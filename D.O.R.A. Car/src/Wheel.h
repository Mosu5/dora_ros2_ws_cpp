#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include <PID_v1.h>

/**
 * @file Wheel.h
 * @brief Definition of the Wheel class for controlling a motorized wheel with PID control.
 */
class Wheel
{
public:
    /**
     * @brief Constructs a new Wheel object.
     *
     * @param motorPin1 The pin number for the first motor control.
     * @param motorPin2 The pin number for the second motor control.
     * @param speedControlPin The pin number for speed control.
     * @param interruptEncoderPin The pin number for the interrupt encoder.
     * @param confermationEncoderPin The pin number for the confirmation encoder.
     * @param Kp The proportional gain for the PID controller.
     * @param Ki The integral gain for the PID controller.
     * @param Kd The derivative gain for the PID controller.
     * @param setpoint The desired setpoint for the PID controller.
     */
    Wheel(int motorPin1, int motorPin2, int speedControlPin,
             int interruptEncoderPin, int confermationEncoderPin,
             double Kp, double Ki, double Kd, double setpoint);

    /**
     * @brief Destroy the Wheel object
     *
     */
    ~Wheel();

    /**
     * @brief Set the Servo object
     *
     * @param activateMotorPin1 Activate the first motor pin.
     * @param activateMotorPin2 Activate the second motor pin.
     */
    void SetServo(bool activateMotorPin1, bool activateMotorPin2);

    /**
     * @brief Update the encoder count.
     *
     */
    static void UpdateEncoder();

private:
    int motorPin1;
    int motorPin2;
    int speedControlPin;

    int interruptEncoderPin;
    int confermationEncoderPin;

    double Kp, Ki, Kd, setpoint;
    volatile long encoderCount;
    double encoderCountAsDouble;
    double PIDOutput;

    PID *moterPID;

    static Wheel *instance; // Static pointer to the instance of the class

    // Accessor for raw encoder count
    static volatile long rawEncoderCount;
};

#endif // WHEEL_H
