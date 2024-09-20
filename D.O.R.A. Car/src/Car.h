#ifndef Car_H
#define Car_H

#include "Wheel.h"

enum Events
{
    KeyUpPressed,
    KeyDownPressed,
    KeyLeftPressed,
    KeyRightPressed,
    KeysReleased,
};

enum CarState
{
    STATIONARY,
    DRIVING_FORWARD,
    REVERSE_DRIVING,
    SIDEWAYS_DRIVING,
    DIAGONAL_DRIVING
};

class Car
{
private:
    // Top left wheel
    const int topLeftWheelMoterPin1 = 24;
    const int topLeftWheelMoterPin2 = 25;

    const int topLeftWheelSpeedControlPin = 5;

    const int topLeftWheelInterruptEncoderPin = 2;
    const int topLeftWheelConfermationEncoderPin = 3;

    // PID constants
    const double topLeftWheelKp = 120;
    const double topLeftWheelKi = 15;
    const double topLeftWheelKd = 117;

    double topLeftWheelSetpoint = 150;

    Wheel topLeftWheel;

    // Top right wheel
    const int topRightWheelMoterPin1 =22;
    const int topRightWheelMoterPin2 = 23;

    const int topRightWheelSpeedControlPin = 4;

    const int topRightWheelInterruptEncoderPin = 0;
    const int topRightWheelConfermationEncoderPin = 0;

    // PID constants
    const double topRightWheelKp = 0;
    const double topRightWheelKi = 0;
    const double topRightWheelKd = 0;

    double topRightWheelSetpoint = 0;

    Wheel topRightWheel;

    // Bottom left wheel
    const int bottomLeftWheelMoterPin1 = 26;
    const int bottomLeftWheelMoterPin2 = 27;

    const int bottomLeftWheelSpeedControlPin = 6;

    const int bottomLeftWheelInterruptEncoderPin = 0;
    const int bottomLeftWheelConfermationEncoderPin = 0;

    // PID constants
    const double bottomLeftWheelKp = 0;
    const double bottomLeftWheelKi = 0;
    const double bottomLeftWheelKd = 0;

    double bottomLeftWheelSetpoint = 0;

    Wheel bottomLeftWheel;

    // Bottom right wheel
    const int bottomRightWheelMoterPin1 = 28;
    const int bottomRightWheelMoterPin2 = 29;

    const int bottomRightWheelSpeedControlPin = 7;

    const int bottomRightWheelInterruptEncoderPin = 0;
    const int bottomRightWheelConfermationEncoderPin = 0;

    // PID constants
    const double bottomRightWheelKp = 0;
    const double bottomRightWheelKi = 0;
    const double bottomRightWheelKd = 0;

    double bottomRightWheelSetpoint = 0;

    Wheel bottomRightWheel;

public:
    Car();
    ~Car();

    CarState CurrentState = STATIONARY;
    void InitCar();
    void ForwardDrive();
    void ReverseDrive();
    void SidewaysDrive();
    void DiagonalDrive();
    void StopCar();
};

#endif