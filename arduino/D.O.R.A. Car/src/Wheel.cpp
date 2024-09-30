#include "Wheel.h"

// Initialize static member variables
volatile long Wheel::rawEncoderCount = 0;
Wheel *Wheel::instance = nullptr;


Wheel::Wheel(int motorPin1, int motorPin2, int speedControlPin,
             int interruptEncoderPin, int confermationEncoderPin,
             double Kp, double Ki, double Kd, double setpoint)
    : motorPin1(motorPin1), motorPin2(motorPin2), speedControlPin(speedControlPin),
      interruptEncoderPin(interruptEncoderPin), confermationEncoderPin(confermationEncoderPin),
      Kp(Kp), Ki(Ki), Kd(Kd), setpoint(setpoint),
      encoderCount(0), PIDOutput(0),
      moterPID(nullptr) // Initialize PID object to null
{
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(speedControlPin, OUTPUT);
    pinMode(interruptEncoderPin, INPUT);
    pinMode(confermationEncoderPin, INPUT);

    // Store the instance pointer to the current object
    instance = this;

    attachInterrupt(digitalPinToInterrupt(interruptEncoderPin), Wheel::UpdateEncoder, RISING);

    // Initialize the PID controller now with correct types
    moterPID = new PID(&encoderCountAsDouble, &PIDOutput, &setpoint, Kp, Ki, Kd, DIRECT);
    moterPID->SetMode(AUTOMATIC);
    moterPID->SetSampleTime(100);
    moterPID->SetOutputLimits(0, 255);
}

Wheel::~Wheel()
{
    if (moterPID)
    {
        delete moterPID;
    }
}

void Wheel::UpdateEncoder()
{
    if (instance != nullptr)
    {
        int encoderPinState = digitalRead(instance->confermationEncoderPin);
        instance->encoderCount += (encoderPinState == HIGH) ? 1 : -1;
        instance->encoderCountAsDouble = static_cast<double>(instance->encoderCount); // Convert to double
    }
}

void Wheel::SetServo(bool activateMotorPin1,bool activateMotorPin2)
{
    // Compute the PID output
    moterPID->Compute();

    // Apply PID output to motor speed (PWM control)
    analogWrite(speedControlPin, PIDOutput);

    // Set motor directions (assumed forward for both motors)
    digitalWrite(motorPin1, activateMotorPin1);
    digitalWrite(motorPin2, activateMotorPin2);
}