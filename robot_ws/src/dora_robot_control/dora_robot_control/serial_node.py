#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String, Float32MultiArray

class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
        
        # Serial communication settings
        self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=0.1)
                
        # Timer for reading encoder feedback
        self.timer = self.create_timer(0.1, self.encoder_callback)        
        self.encoder_pub = self.create_publisher(Float32MultiArray, 'processed_data', 10)
    
        # Subscriber for wheel_vel
        self.wheel_vel_sub = self.create_subscription(
            String,
            'wheel_vel',
            self.wheel_vel_callback,
            10
        )
        
    def wheel_vel_callback(self, msg: String):
        """ Callback function for receiving wheel velocities """
        self.get_logger().info('Result callback triggered')
        wheel_velocities = msg.data
        self.get_logger().info(f"Sending wheel velocities: {wheel_velocities}")
        self.write_to_serial(wheel_velocities)
        
    def write_to_serial(self, data):
        """ Write data to the serial port """
        try:
            float_data = [float(x) for x in data.strip('[]').split(',')]
            formatted_data = f"[{','.join(map(str, float_data))}]"
            self.serial_port.write(formatted_data.encode())
            self.get_logger().info(f"Data sent: {formatted_data}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")
            
    def encoder_callback(self):
        raw_data = self.read_serial_data()
        if raw_data is None:
            return
        self.get_logger().info(f"Encoder positions: {raw_data}")
        processed_data = self.process_encoder_data(raw_data)
        self.publish_processed_data(processed_data)
    
    def read_serial_data(self) -> str:
        """ Read encoder data from the robot via serial communication """
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Raw serial data: {data}")
                if data.startswith('{') and data.endswith('}'):
                    return data.strip('{}')
                else:
                    return None
            else:
                return None
        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")
            return None

    def process_encoder_data(self, data: str):
        """ Process the encoder data from string to list of floats """
        try:
            int_data = [int(x) for x in data.split(',')]
            if len(int_data) != 4:
                raise ValueError("Invalid number of encoder values")
            float_data = [float(x) / 1000.0 for x in int_data]
            return float_data
        except ValueError as e:
            self.get_logger().warn(f"Invalid data format: {e}")
            return []

    def publish_processed_data(self, data):
        """ Publish the processed encoder data """
        msg = Float32MultiArray()
        msg.data = data
        self.encoder_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()