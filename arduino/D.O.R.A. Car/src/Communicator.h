#ifndef COMUNICATOR_H
#define COMUNICATOR_H

#include <Arduino.h>

/**
 * @class Communicator
 * @brief Handles communication for sending and receiving data.
 *
 * This class provides functionalities to initialize communication, send data,
 * and receive data, specifically for wheel velocities.
 */
class Communicator
{
public:
    /**
     * @struct WheelVelocities
     * @brief Represents the velocities of all four wheels.
     */
    struct WheelVelocities
    {
        volatile long topLeft;    ///< Velocity of the top left wheel.
        volatile long topRight;   ///< Velocity of the top right wheel.
        volatile long bottomLeft; ///< Velocity of the bottom left wheel.
        volatile long bottomRight;///< Velocity of the bottom right wheel.
    };

    /**
     * @brief Constructs a Communicator object.
     */
    Communicator();

    /**
     * @brief Initializes the communication with the specified baud rate.
     *
     * @param baudRate The baud rate for communication.
     */
    void init(int baudRate);

    /**
     * @brief Sends a string of data.
     *
     * @param data The data to send.
     */
    void SendData(const String &data);

    /**
     * @brief Sends the encoder data for all four wheels.
     *
     * @param data The wheel velocities to send.
     */
    void SendEncoderData(WheelVelocities data);

    /**
     * @brief Receives the wheel velocities from incoming data.
     *
     * @return The received wheel velocities.
     */
    WheelVelocities ReceiveValues();

private:
    /**
     * @brief Parses wheel velocities from a data string.
     *
     * @param data The data string to parse.
     * @return The parsed wheel velocities.
     */
    WheelVelocities parseVelocitiesFromData(const char *data);
};

#endif // COMUNICATOR_H