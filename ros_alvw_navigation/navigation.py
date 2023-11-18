import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtleBotController:
    def __init__(self):
        self.node = rclpy.create_node('turtlebot_controller')
        self.velocity_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_subscriber = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.cmd_vel = Twist()

    def laser_callback(self, laser_data):
        # Implement your logic based on laser data to decide the linear and angular velocity
        # For simplicity, let's move forward if there are no obstacles in front
        if min(laser_data.ranges) > 0.5:
            self.cmd_vel.linear.x = 0.2  # Set linear velocity
            self.cmd_vel.angular.z = 0.0  # Set angular velocity
        else:
            # If obstacle is detected, stop
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0

    def move_turtlebot(self):
        while rclpy.ok():
            self.velocity_publisher.publish(self.cmd_vel)

if __name__ == '__main__':
    rclpy.init()
    try:
        turtlebot_controller = TurtleBotController()
        turtlebot_controller.move_turtlebot()
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot_controller.node.destroy_node()
        rclpy.shutdown()
