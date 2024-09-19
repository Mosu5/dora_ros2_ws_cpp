#include <PID_v1.h>

// Pin definitions
const int motorIn1 = 8;
const int motorIn2 = 9;
const int motorIn3 = 7;
const int motorIn4 = 6;
const int motorEnA = 10;   // PWM pin for speed control
const int motorEnB = 11;
const int encoderA = 2;   // Encoder A output to interrupt pin
const int encoderB = 3;   // Encoder B output to digital pin
const int encoderC = 21;
const int encoderD = 23;

// Variables for encoder counts
volatile long encoderCount1 = 0;  // Counts encoder pulses
volatile long encoderCount2 = 0;

// PID control parameters
double setpointA = 150;  // Target for motor A
double inputA = 0;       // Encoder input for motor A
double outputA = 0;      // PID output for motor A
double setpointB = 150;  // Target for motor B
double inputB = 0;       // Encoder input for motor B
double outputB = 0;      // PID output for motor B

// PID constants
double KpA = 120, KiA = 15, KdA = 117;
double KpB = 5, KiB = 9, KdB = 31;

// PID objects for both motors
PID motorA(&inputA, &outputA, &setpointA, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, &setpointB, KpB, KiB, KdB, DIRECT);

void setup() {
  // Motor pins as output
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);
  pinMode(motorEnA, OUTPUT);
  pinMode(motorEnB, OUTPUT);

  // Encoder pins as input
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(encoderC, INPUT);
  pinMode(encoderD, INPUT);

  // Attach interrupt for encoderA and encoderC
  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderC), updateEncoder2, RISING);

  // PID configuration
  motorA.SetMode(AUTOMATIC);
  motorB.SetMode(AUTOMATIC);
  motorA.SetSampleTime(100);  // PID update interval in ms
  motorB.SetSampleTime(100);
  motorA.SetOutputLimits(0, 255);  // PWM range for motor control
  motorB.SetOutputLimits(0, 255);

  // Serial monitor initialization
  Serial.begin(9600);
}

void loop() {
  // Update the PID input with encoder counts
  inputA = encoderCount1;
  inputB = encoderCount2;

  // Compute the PID output
  motorA.Compute();
  motorB.Compute();

  // Apply PID output to motor speed (PWM control)
  analogWrite(motorEnA, outputA);
  analogWrite(motorEnB, outputB);

  // Set motor directions (assumed forward for both motors)
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);

  // Print encoder counts and PID outputs for debugging
  Serial.print("Encoder Count 1: ");
  Serial.println(encoderCount1);
  Serial.print("PID Output A: ");
  Serial.println(outputA);

  Serial.print("Encoder Count 2: ");
  Serial.println(encoderCount2);
  Serial.print("PID Output B: ");
  Serial.println(outputB);

  delay(100);
}

// Interrupt routine to count encoder pulses for motor A
void updateEncoder1() {
  int stateB = digitalRead(encoderB);

  if (stateB == HIGH) {
    encoderCount1++;
  } else {
    encoderCount1--;
  }
}

// Interrupt routine to count encoder pulses for motor B
void updateEncoder2() {
  int stateD = digitalRead(encoderD);

  if (stateD == HIGH) {
    encoderCount2++;
  } else {
    encoderCount2--;
  }
}
