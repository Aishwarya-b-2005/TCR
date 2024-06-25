import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.action import RotateAbsolute
from rclpy.action import ActionClient

class TurtleMimic(Node):
    def __init__(self):
        super().__init__('turtle_mimic')
        self.subscription = self.create_subscription(Point, 'ballCoordinates', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.last_x = 0.0
        self.last_y = 0.0
        self.ori = 'e'  

        self.client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')

    def change_turtle_orient(self, t):
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = float(t)  

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info('Rotation result: {0}'.format(result))

    def listener_callback(self, msg):
        dx = msg.x - self.last_x
        dy = msg.y - self.last_y

        twist = Twist()

        if abs(dx) > abs(dy):
            if self.ori != 'e':
                self.change_turtle_orient(0)  # Rotate to face east (0 radians)
                self.ori = 'e'
            twist.linear.x = -0.8 if dx > 0 else 0.8
        else:
            if self.ori != 'n':
                self.change_turtle_orient(1.57)  # Rotate to face north (1.57 radians)
                self.ori = 'n'
            twist.linear.x = -0.8 if dy > 0 else 0.8

        self.publisher.publish(twist)
        self.last_x = msg.x
        self.last_y = msg.y

def main(args=None):
    rclpy.init(args=args)
    turtle_mimic = TurtleMimic()
    rclpy.spin(turtle_mimic)
    turtle_mimic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()