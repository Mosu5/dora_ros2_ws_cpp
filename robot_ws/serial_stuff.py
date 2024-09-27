import serial

def connect_serial(port='/dev/ttyS0', baudrate=9600):
    try:
        ser = serial.Serial(port, baudrate)
        print(f"Connected to {port} at {baudrate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error: {e}")
        return None

if __name__ == "__main__":
    ser = connect_serial()
    if ser:
        ser.close()