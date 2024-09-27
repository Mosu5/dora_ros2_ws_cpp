import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class HolonomicWheelController(Node):
    def __init__(self):
        super().__init__('holonomic_wheel_controller')
        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=1)
        
        # Parameters for wheel base
        self.wheel_radius = 0.1  # meters (radius of the wheels)
        self.L = 0.5  # Length between front and rear wheels
        self.W = 0.3  # Width between left and right wheels

    def cmd_vel_callback(self, msg: Twist):
        # Extract the linear and angular velocity from the Twist message
        vx = msg.linear.x  # forward velocity in m/s
        vy = msg.linear.y  # sideways velocity in m/s
        wz = msg.angular.z  # angular velocity in rad/s

        # Calculate wheel velocities (Mecanum kinematics)
        v_fl, v_fr, v_rl, v_rr = self.compute_wheel_velocities(vx, vy, wz)
        
        # Log the wheel velocities
        self.get_logger().info(f'FL: {v_fl:.2f}, FR: {v_fr:.2f}, RL: {v_rl:.2f}, RR: {v_rr:.2f} (rad/s)')
        self.get_logger().info('Sending wheel velocities to the robot')
        self.serial_port.write(f'{v_fl:.2f},{v_fr:.2f},{v_rl:.2f},{v_rr:.2f}\n'.encode())

    def compute_wheel_velocities(self, vx, vy, wz):
        # Mecanum wheel inverse kinematics formula
        L = self.L
        W = self.W
        r = self.wheel_radius
        
        # Calculate each wheel's velocity
        v_fl = (1 / r) * (vx - vy - (L + W) * wz)
        v_fr = (1 / r) * (vx + vy + (L + W) * wz)
        v_rl = (1 / r) * (vx + vy - (L + W) * wz)
        v_rr = (1 / r) * (vx - vy + (L + W) * wz)

        return v_fl, v_fr, v_rl, v_rr


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicWheelController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

