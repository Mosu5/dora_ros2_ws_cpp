#include "Car.h"
Car myCar;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    myCar.ForwardDrive();
    delay(2000);
    myCar.StopCar();
    delay(2000);
    myCar.ReverseDrive();
    delay(2000);
    myCar.StopCar();
    delay(2000);
    myCar.SidewaysDrive();
    delay(2000);
    myCar.StopCar();
    delay(2000);
}