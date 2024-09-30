#include "Comunicator.h"

// Constructor
Comunicator::Comunicator() {
    // Constructor body can be empty or used for initialization if needed
}

// Initialize the serial communication
void Comunicator::InitComunicator(long baudRate) {
    Serial.begin(baudRate);
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Comunicator initialized.");
}

// Send data through serial
void Comunicator::SendData(const String &data) {
    Serial.println(data);
}

// Receive data from serial
String Comunicator::ReceiveData() {
    String receivedData = "";
    if (Serial.available() > 0) {
        receivedData = Serial.readStringUntil('\n');
    }
    return receivedData;
}