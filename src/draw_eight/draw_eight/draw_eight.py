import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from turtlesim.srv import Spawn

class DriverNode(Node):
    def __init__(self):
        super().__init__('driving_node')
        self.publisher1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Define the move message for drawing a circle
        self.move_msg = Twist()
        self.move_msg.linear.x = 1.0  # Move forward with 1 m/s
        self.move_msg.angular.z = 1.0  # Rotate with 1 rad/s


        self.r_move_msg = Twist()
        self.r_move_msg.linear.x = 1.5  # Move forward with 1 m/s
        self.r_move_msg.angular.z = -1.0  # Rotate with 1 rad/s

        # Define the stop message
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0.0
        self.stop_msg.angular.z = 0.0

        # Time to complete a circle
        self.circle_duration = 2 * math.pi / self.move_msg.angular.z

        # Create a timer to publish messages periodically
        self.timer1 = self.create_timer(0.1, self.timer_1_callback)
        self.timer2 = None


        # Start the timer
        self.start_time = self.get_clock().now()


    def timer_1_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # convert nanoseconds to seconds

        if elapsed_time < self.circle_duration:
            self.publisher1.publish(self.move_msg)
            self.get_logger().info('Publishing move message to draw circle')
        else:
            self.publisher1.publish(self.stop_msg)
            self.get_logger().info('Publishing stop message to stop at start point')
            self.destroy_timer(self.timer1)
            self.get_logger().info('time destroyed')
            client = self.create_client(Spawn, 'spawn')

            # Wait for the service to be available
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')

    # Prepare the request
            request = Spawn.Request()
            request.x = 5.54  # X coordinate of the turtle's initial pose
            request.y = 5.54  # Y coordinate of the turtle's initial pose
            request.theta = 0.0  # Orientation (theta) of the turtle's initial pose
            request.name = 'turtle2'  # Name of the turtle to be spawned

    # Call the service
            future = client.call_async(request)
            self.start_time = self.get_clock().now()
            self.timer2 = self.create_timer(0.1, self.timer_2_callback)
            
    def timer_2_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # convert nanoseconds to seconds

        if elapsed_time < self.circle_duration:
            self.publisher2.publish(self.r_move_msg)
            self.get_logger().info('Publishing rev move message to draw circle')
        else:
            self.publisher2.publish(self.stop_msg)
            self.get_logger().info('Publishing rev stop message to stop at start point')
            self.destroy_timer(self.timer2)
            self.get_logger().info('time destroyed')

def main(args=None):
    rclpy.init(args=args)
    driver_node = DriverNode()
    driver_node.should_shutdown = False
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
