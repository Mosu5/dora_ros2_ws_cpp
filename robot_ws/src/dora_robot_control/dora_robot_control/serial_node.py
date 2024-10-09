#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import serial
import random
from std_msgs.msg import String


class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
        
        # Serial communication settings
        self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=0.1) # changed timeout from 1 to 0.1 seconds
                
        # Timer for reading encoder feedback
        self.timer = self.create_timer(0.1, self.encoder_callback)        
        self.encoder_pub = self.create_publisher(String, 'encoder_feedback', 10)
    
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
        # Get the wheel velocities from the message
        wheel_velocities = msg.data
        
        # Send wheel velocities to the robot via serial communication
        self.get_logger().info(f"Sending wheel velocities: {wheel_velocities}")
        self.write_to_serial(wheel_velocities)
        
    def write_to_serial(self, data):
        """ Write data to the serial port """
        try:
            self.serial_port.write(data.encode())
            self.get_logger().info(f"Data sent: {data}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")
            
    def encoder_callback(self):
        msg = String()
        
        # Read encoder data from the robot
        msg.data = self.read_serial_data()
        if msg.data is None:
            return
        self.get_logger().info(f"Encoder positions: {msg.data}")
        self.encoder_pub.publish(msg)
    
    def read_serial_data(self):
        """ Read encoder data from the robot via serial communication """
        try:
            if self.serial_port.in_waiting > 0:
                # Read the data from the serial port and decode it after checking the newline character
                data = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Raw serial data: {data}")
                return data
            else:
                return None
        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()