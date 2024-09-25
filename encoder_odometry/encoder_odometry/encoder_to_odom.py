#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf_transformations import quaternion_from_euler  # Import from tf_transformations for ROS2
import math
from rclpy.time import Time

class HolonomicOdomPublisher(Node):
    def __init__(self):
        super().__init__('holonomic_odom_publisher')

        # Parameters
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baudrate = self.declare_parameter('baudrate', 9600).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.08).value  # 8 cm radius
        self.wheel_base_length = self.declare_parameter('wheel_base_length', 0.153).value  # 15.3 cm base length
        self.wheel_base_width = self.declare_parameter('wheel_base_width', 0.362).value  # 36.2 cm base width

        # Open the serial connection
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)

        # Variables to store position and orientation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_wheel_data = [0, 0, 0, 0]  # For four wheels

        # Time management for velocity calculation
        self.last_time = self.get_clock().now()

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)

        # Timer to regularly call the update function
        self.create_timer(0.1, self.publish_odometry)  # 10 Hz update rate

    def read_wheel_data(self):
        """
        Read the encoder data from all four wheels via serial.
        The Arduino should be sending the encoder values for all four wheels in a specific format.
        """
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                # Expecting data in the format: "wheel1,wheel2,wheel3,wheel4"
                wheel_data = [int(x) for x in line.split(',')]
                return wheel_data
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")
            return None

    def compute_holonomic_odometry(self, wheel_data):
        """
        Compute the robot's odometry based on the wheel encoder data.
        Assumes mecanum wheels or omni-wheels with a known configuration.
        """

        # Update current time and compute delta time
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9

        # Parameters
        ticks_per_revolution = 1440  # Your encoder's CPR
        distance_per_tick = (2 * math.pi * self.wheel_radius) / ticks_per_revolution

        # Calculate the delta encoder values
        delta_wheel = [
            (wheel_data[i] - self.last_wheel_data[i]) * distance_per_tick for i in range(4)
        ]
        self.last_wheel_data = wheel_data

        # Holonomic robot velocity calculations (for mecanum or omni-wheels)
        # Based on inverse kinematics equations for holonomic wheels
        vx = (delta_wheel[0] + delta_wheel[1] + delta_wheel[2] + delta_wheel[3]) / 4.0
        vy = (-delta_wheel[0] + delta_wheel[1] + delta_wheel[2] - delta_wheel[3]) / 4.0
        vth = (-delta_wheel[0] + delta_wheel[1] - delta_wheel[2] + delta_wheel[3]) / (
            4.0 * (self.wheel_base_length + self.wheel_base_width) / 2.0
        )

        # Update position
        delta_x = vx * delta_time
        delta_y = vy * delta_time
        delta_th = vth * delta_time

        self.x += delta_x * math.cos(self.th) - delta_y * math.sin(self.th)
        self.y += delta_x * math.sin(self.th) + delta_y * math.cos(self.th)
        self.th += delta_th

        # Update last time
        self.last_time = current_time

        return self.x, self.y, self.th, vx, vy, vth

    def publish_odometry(self):
        # Read the wheel data
        wheel_data = self.read_wheel_data()
        if wheel_data is None:
            return

        # Compute the odometry
        x, y, th, vx, vy, vth = self.compute_holonomic_odometry(wheel_data)

        # Create the Odometry message
        odom_quat = quaternion_from_euler(0, 0, th)  # Convert euler angles to quaternion
        current_time = self.get_clock().now()

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'

        # Set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        # Set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # Publish the message
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)

    holonomic_odom_publisher = HolonomicOdomPublisher()

    try:
        rclpy.spin(holonomic_odom_publisher)
    except KeyboardInterrupt:
        pass

    holonomic_odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

