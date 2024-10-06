import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dora_interfaces.action import WheelVelCalc  # Replace with your actual action definition
from dora_interfaces.msg import WheelVel

class WheelVelActionClient(Node):

    def __init__(self):
        super().__init__('wheel_vel_action_client')
        self._action_client = ActionClient(self, WheelVelCalc, 'wheel_velocities')
        self._result_callback = None  # Placeholder for the result callback
        
        # Subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(WheelVel, 'wheel_vel', 10)

    def cmd_vel_callback(self, msg: Twist):
        """ Callback function for receiving cmd_vel messages """
        self.get_logger().info('Received cmd_vel message')
        self.send_goal(msg.linear.x, msg.linear.y, msg.angular.z)

    def send_goal(self, vx, vy, wz, result_callback=None):
        goal_msg = WheelVelCalc.Goal()
        goal_msg.vx = vx
        goal_msg.vy = vy
        goal_msg.wz = wz

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Publishing result: {0}'.format(result.wheel_velocities))
        
        msg = WheelVel()  
        msg.wheel_velocities = result.wheel_velocities
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    action_client = WheelVelActionClient()
    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()