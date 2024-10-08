#include "Communicator.h"

Communicator::Communicator()
{
    // Default constructor
}

void Communicator::init(int baudRate)
{
    Serial.begin(baudRate);
    while (!Serial)
    {
        ; // Wait for serial port to connect. Needed for native USB port only
    }
    Serial.flush(); // Flush the serial buffer
    Serial.println("Communicator initialized.");
}

void Communicator::SendData(const String &data)
{
    Serial.println(data);
}

void Communicator::SendEncoderData(WheelVelocities data)
{
    String encodedData = String(data.topLeft) + "," + String(data.topRight) +
                         "," + String(data.bottomLeft) + "," + String(data.bottomRight);
    SendData(encodedData);
}

Communicator::WheelVelocities Communicator::ReceiveValues()
{
    // Serial.println("Receiving values...");
    if (Serial.available() > 0)
    {
        String readLine = Serial.readStringUntil('\n');
        const char *data = readLine.c_str();
        // Serial.println(data); // Echo back the received data for debugging
        return parseVelocitiesFromData(data);
    }
    return {0, 0, 0, 0};
}

Communicator::WheelVelocities Communicator::parseVelocitiesFromData(const char *data)
{
    // Serial.println("Parsing values...");
    Communicator::WheelVelocities velocities;

    velocities.topLeft = 0;
    velocities.topRight = 0;
    velocities.bottomLeft = 0;
    velocities.bottomRight = 0;
    // if (data == NULL)
    // {
    //     Serial.println("Data is NULL");
    //     return velocities;
    // }

    velocities.topLeft = atof(strtok(data, ","));
    velocities.topRight = atof(strtok(NULL, ","));
    velocities.bottomLeft = atof(strtok(NULL, ","));
    velocities.bottomRight = atof(strtok(NULL, ","));
    // Serial.println("Parsed velocities: TL:" + String(velocities.topLeft) + ",TR:" +
    //                String(velocities.topRight) + ",BL:" + String(velocities.bottomLeft) +
    //                ",BR:" + String(velocities.bottomRight));

    return velocities;
}