#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import serial
import random
from std_msgs.msg import String

from dora_interfaces.msg import EncoderFeedback    # CHANGE
from dora_interfaces.msg import WheelVel          # CHANGE

class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
        
        # Serial communication settings
        self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=0.1) # changed timeout from 1 to 0.1 seconds
        timer_period = 0.1        
                
        # TODO: Change the callback function to read encoder data from the robot 
        # and publish the last read data to the 'encoder_feedback' topic
        # Timer for reading encoder feedback

        self.timer = self.create_timer(timer_period, self.encoder_callback)        
        # self.publisher_ = self.create_publisher(EncoderFeedback, 'encoder_feedback', 10)     # CHANGE
        self.encoder_pub = self.create_publisher(String, 'encoder_feedback', 10)     # CHANGE
    
        # TODO: add cmd_vel_callback
        # Subscriber for cmd_vel
        # self.cmd_vel_sub = self.create_subscription(
        #     WheelVel,
        #     'wheel_vel',
        #     self.wheel_vel_callback,
        #     10
        # )
        
        self.wheel_vel_sub = self.create_subscription(
            WheelVel,
            'wheel_vel',
            self.wheel_vel_callback,
            10
        )
        
    def wheel_vel_callback(self, msg: WheelVel):
        """ Callback function for receiving wheel velocities """
        self.get_logger().info('Result callback triggered')
        # Get the wheel velocities from the message
        wheel_velocities = msg.wheel_velocities
        
        # Send wheel velocities to the robot via serial communication
        self.get_logger().info(f"Sending wheel velocities: {wheel_velocities}")
        self.serial_port.write(f"{wheel_velocities}\n".encode())
        
    def write_to_serial(self, data):
        """ Write data to the serial port """
        with serial.Serial('/dev/ttyS0', 9600, timeout=0.1) as ser:
            ser.write(data.encode())
            self.get_logger().info(f"Data sent: {data}")
            ser.close()
            
           
    def encoder_callback(self):
        msg = EncoderFeedback()                                           # CHANGE
        
        # send encoder data to the robot
        # if no serial data is available, send random data
        # if self.read_wheel_data() is not None:
        encoder_positions = self.read_wheel_data()
        if encoder_positions is None:
            return
        
        self.get_logger().info(f"Encoder positions: {msg.encoder_positions}")
 
        # else:
            # msg.encoder_positions = random_encoder_positions()         # CHANGE

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing encoder_positions:  %d, %d, %d, %d' % (msg.encoder_positions[0], msg.encoder_positions[1], msg.encoder_positions[2], msg.encoder_positions[3]))  # CHANGE
    
    def read_wheel_data(self):
        """ Read encoder data from the robot via serial communication """
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Raw serial data: {line}")
                # Clean the line by removing unwanted characters (like ':')
                cleaned_data = line.replace(':', '').strip()

                # Split the cleaned data by commas and convert to float
                wheel_data = [float(x) for x in cleaned_data.split(',')]
                
                if len(wheel_data) != 4:
                    self.get_logger().error(f"Invalid data received: {line}")
                    return None 
                
                return wheel_data
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")
            return None

def random_encoder_positions():
    return [random.randint(1000, 3000) for _ in range(4)]

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()