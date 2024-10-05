import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from dora_interfaces.action import WheelVelocities


class WheelVelActionClient(Node):

    def __init__(self):
        super().__init__('wheel_vel_action_client')
        self._action_client = ActionClient(self, WheelVelocities, 'wheel_velocities')

    def send_goal(self, vx, vy, wz):
        goal_msg = WheelVelocities.Goal()
        goal_msg.vx = vx
        goal_msg.vy = vy
        goal_msg.wz = wz

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.wheel_velocities))


def main(args=None):
    rclpy.init(args=args)

    action_client = WheelVelActionClient()

    action_client.send_goal(float(100), float(100), float(100))

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()