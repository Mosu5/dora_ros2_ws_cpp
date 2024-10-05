#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import serial
import random

from dora_interfaces.msg import EncoderFeedback    # CHANGE


class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
        
        # Serial communication settings
        # self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=1)
        
        # Parameters for wheel base and odometry
        self.wheel_radius = 0.08  # meters (radius of the wheels)
        self.wheel_base_length = 0.153  # meters (length between front and rear wheels)
        self.wheel_base_width = 0.362  # meters (width between left and right wheels)
        
        
        # TODO: Change the callback function to read encoder data from the robot 
        # and publish the last read data to the 'encoder_feedback' topic
        # Timer for reading encoder feedback
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.encoder_callback)        
        self.publisher_ = self.create_publisher(EncoderFeedback, 'encoder_feedback', 10)     # CHANGE
    
        # TODO: add cmd_vel_callback
        # Subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
    def cmd_vel_callback(self, msg: Twist):
        """ Handle incoming cmd_vel messages and send wheel velocities """
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        v_fl, v_fr, v_rl, v_rr = self.compute_wheel_velocities(vx, vy, wz)

        self.get_logger().info(f'FL: {v_fl:.2f}, FR: {v_fr:.2f}, RL: {v_rl:.2f}, RR: {v_rr:.2f} (rad/s)')
        # self.serial_port.write(f'{v_fl:.2f},{v_fr:.2f},{v_rl:.2f},{v_rr:.2f}\n'.encode())
        
    def compute_wheel_velocities(self, vx, vy, wz):
        """ Compute the individual wheel velocities for a mecanum drive """
        L = self.wheel_base_length
        W = self.wheel_base_width
        r = self.wheel_radius

        v_fl = (1 / r) * (vx - vy - (L + W) * wz)
        v_fr = (1 / r) * (vx + vy + (L + W) * wz)
        v_rl = (1 / r) * (vx + vy - (L + W) * wz)
        v_rr = (1 / r) * (vx - vy + (L + W) * wz)

        return v_fl, v_fr, v_rl, v_rr
           
    def encoder_callback(self):
        msg = EncoderFeedback()                                           # CHANGE
        
        # send encoder data to the robot
        # if no serial data is available, send random data
        if self.read_wheel_data() is not None:
            msg.encoder_positions = self.read_wheel_data()
        else:
            msg.encoder_positions = random_encoder_positions()         # CHANGE

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing encoder_positions:  %d, %d, %d, %d' % (msg.encoder_positions[0], msg.encoder_positions[1], msg.encoder_positions[2], msg.encoder_positions[3]))  # CHANGE
    
    def read_wheel_data(self):
        """ Read encoder data from the robot via serial communication """
        # try:
        #     if self.serial_port.in_waiting > 0:
        #         line = self.serial_port.readline().decode('utf-8').strip()
        #         self.get_logger().info(f"Raw serial data: {line}")
        #         wheel_data = [float(x) for x in line.split(',')]
        #         return wheel_data
        #     else:
        #         return None
        # except Exception as e:
        #     self.get_logger().error(f"Error reading serial data: {e}")
        #     return None

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