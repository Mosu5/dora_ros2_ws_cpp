import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from dora_interfaces.action import WheelVelCalc


class WheelVelActionServer(Node):

    def __init__(self):
        super().__init__('wheel_vel_action_server')
        self._action_server = ActionServer(
            self,
            WheelVelCalc,
            'wheel_velocities',
            self.execute_callback)
        
        
         # Parameters for wheel base and odometry
        self.wheel_radius = 0.08  # meters (radius of the wheels)
        self.wheel_base_length = 0.153  # meters (length between front and rear wheels)
        self.wheel_base_width = 0.362  # meters (width between left and right wheels)
        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        wheel_vel = [0, 0, 0, 0]
        
        wheel_vel = compute_wheel_velocities(self, goal_handle.request.vx, goal_handle.request.vy, goal_handle.request.wz)
        
        self.get_logger().info(f'Wheel velocities: {wheel_vel}')
        
        goal_handle.succeed()
        
        self.get_logger().info('Goal succeeded!')
        
        
        result = WheelVelCalc.Result()
        result.wheel_velocities = wheel_vel
        return result

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

def main(args=None):
    rclpy.init(args=args)

    wheel_vel_action_server = WheelVelActionServer()

    rclpy.spin(wheel_vel_action_server)


if __name__ == '__main__':
    main()