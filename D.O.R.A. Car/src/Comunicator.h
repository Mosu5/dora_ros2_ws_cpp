#ifndef COMUNICATOR_H
#define COMUNICATOR_H

#include <Arduino.h>

class Comunicator {
public:
    // Constructor
    Comunicator();

    // Methods
    void InitComunicator(long baudRate);
    void SendData(const String &data);
    String ReceiveData();

private:
    // Private methods or variables can be added here if needed
};

#endif // COMUNICATOR_H