#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include <PID_v1.h>

/**
 * @class Wheel
 * @brief Represents a wheel with motor control and encoder feedback.
 *
 * This class provides functionalities to control a wheel's motor using PID control
 * and to read the encoder's position to determine the wheel's movement.
 */
class Wheel
{
public:
    /**
     * @brief Constructs a Wheel object.
     *
     * @param enablePin The pin to enable the motor.
     * @param in1Pin The first pin to control the motor direction.
     * @param in2Pin The second pin to control the motor direction.
     * @param encPinA The first pin for the encoder.
     * @param encPinB The second pin for the encoder.
     * @param kp The proportional gain for the PID controller.
     * @param ki The integral gain for the PID controller.
     * @param kd The derivative gain for the PID controller.
     */
    Wheel(int enablePin, int in1Pin, int in2Pin, int encPinA, int encPinB,
          double kp, double ki, double kd);

    /**
     * @brief Sets the desired setpoint for the PID controller.
     *
     * @param sp The desired setpoint.
     */
    void SetSetpoint(double sp);

    /**
     * @brief Gets the PID controller associated with the wheel.
     *
     * @return A pointer to the PID controller.
     */
    PID *GetPID();

    /**
     * @brief Gets the current position of the encoder.
     *
     * @return The current encoder position.
     */
    long GetEncoderPosition();

    /**
     * @brief Handles the change in encoder state.
     *
     * This function should be called in the encoder interrupt service routine.
     */
    void HandleEncoderChange();

    /**
     * @brief Sets the servo control for the wheel.
     */
    void SetServo();

    /**
     * @brief Calculates the current speed of the wheel.
     */
    void CalculateCurrentSpeed();

private:
    static Wheel *instance; ///< Singleton instance of the Wheel class.

    volatile long encoderPosition = 0; ///< The current position of the encoder.
    long lastEncoderPosition = 0;      ///< The last recorded position of the encoder.
    unsigned long lastTime = 0;        ///< The last time the encoder position was recorded.

    int motorSpeedEnablePin; ///< The pin to enable the motor speed.

    int motorIn1Pin; ///< The first pin to control the motor direction.
    int motorIn2Pin; ///< The second pin to control the motor direction.

    int interruptEncoderPinA;    ///< The first pin for the encoder interrupt.
    int confermationEncoderPinB; ///< The second pin for the encoder confirmation.

    double setpoint=0, PIDinput=0, PIDoutput=0; ///< PID control variables.
    PID pid;                              ///< The PID controller for the wheel.

    // static void isr();
};

#endif // WHEEL_H
