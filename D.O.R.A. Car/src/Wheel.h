#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include <PID_v1.h>

class Wheel
{
public:
    // Constructor
    Wheel(int motorPin1, int motorPin2, int speedControlPin,
          int interruptEncoderPin, int confermationEncoderPin,
          double Kp, double Ki, double Kd, double setpoint);

    // Destructor to clean up dynamic memory
    ~Wheel();

    // Methods
    void InitWheel();
    void SetServo(bool activateMotorPin1, bool activateMotorPin2);

    // Static interrupt service routine
    static void UpdateEncoder();

private:
    int motorPin1, motorPin2, speedControlPin;
    int interruptEncoderPin, confermationEncoderPin;
    double Kp, Ki, Kd, setpoint;
    volatile long encoderCount;  // Change to volatile for ISR use
    double encoderCountAsDouble; // Store encoder count as double for PID
    double PIDOutput;
    PID *moterPID;

    static Wheel *instance; // Static pointer to the instance of the class

    // Accessor for raw encoder count
    static volatile long rawEncoderCount;
};

#endif // WHEEL_H
