#include "Car.h"


Car::Car()
    : topLeftWheel(topLeftWheelMoterPin1, topLeftWheelMoterPin2, topLeftWheelSpeedControlPin,
                   topLeftWheelInterruptEncoderPin, topLeftWheelConfermationEncoderPin,
                   topLeftWheelKp, topLeftWheelKi, topLeftWheelKd, topLeftWheelSetpoint),

      topRightWheel(topRightWheelMoterPin1, topRightWheelMoterPin2, topRightWheelSpeedControlPin,
                    topRightWheelInterruptEncoderPin, topRightWheelConfermationEncoderPin,
                    topRightWheelKp, topRightWheelKi, topRightWheelKd, topRightWheelSetpoint),

      bottomLeftWheel(bottomLeftWheelMoterPin1, bottomLeftWheelMoterPin2, bottomLeftWheelSpeedControlPin,
                      bottomLeftWheelInterruptEncoderPin, bottomLeftWheelConfermationEncoderPin,
                      bottomLeftWheelKp, bottomLeftWheelKi, bottomLeftWheelKd, bottomLeftWheelSetpoint),

      bottomRightWheel(bottomRightWheelMoterPin1, bottomRightWheelMoterPin2, bottomRightWheelSpeedControlPin,
                       bottomRightWheelInterruptEncoderPin, bottomRightWheelConfermationEncoderPin,
                       bottomRightWheelKp, bottomRightWheelKi, bottomRightWheelKd, bottomRightWheelSetpoint)
{
}

void Car::ForwardDrive()
{
    topLeftWheel.SetServo(1, 0);
    topRightWheel.SetServo(1, 0);
    bottomLeftWheel.SetServo(1, 0);
    bottomRightWheel.SetServo(1, 0);

    Serial.println("Forward Drive");
}

void Car::ReverseDrive()
{
    topLeftWheel.SetServo(0, 1);
    topRightWheel.SetServo(0, 1);
    bottomLeftWheel.SetServo(0, 1);
    bottomRightWheel.SetServo(0, 1);

    Serial.println("Reverse Drive");
}

void Car::SidewaysDrive()
{
    topLeftWheel.SetServo(1, 0);
    topRightWheel.SetServo(0, 1);
    bottomLeftWheel.SetServo(1, 0);
    bottomRightWheel.SetServo(0, 1);

    Serial.println("Sideways Drive");
}

void Car::DiagonalDrive()
{
}

void Car::StopCar()
{
    topLeftWheel.SetServo(0, 0);
    topRightWheel.SetServo(0, 0);
    bottomLeftWheel.SetServo(0, 0);
    bottomRightWheel.SetServo(0, 0);

    Serial.println("Stop Car");
}

Car::~Car()
{
}