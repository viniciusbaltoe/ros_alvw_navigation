import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class RobotNavigator(Node):

    def __init__(self):
        super().__init__('robot_navigator')
        self.get_logger().info("RobotNavigator node initialized")
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Bool, "stop", self.get_stop, 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.cmd_vel = Twist()

    def laser_callback(self, laser_data):
        ranges_n = laser_data.ranges[:10] + laser_data.ranges[350:]
        ranges_ne = laser_data.ranges[10:45]
        ranges_nw = laser_data.ranges[315:350]
        ranges_e = laser_data.ranges[45:90]
        ranges_w = laser_data.ranges[270:315]
        min_range_n = min(ranges_n)
        min_range_ne = min(ranges_ne)
        min_range_nw = min(ranges_nw)
        min_range_e = min(ranges_e)
        min_range_w = min(ranges_w)

        if min_range_n < 0.6: # se tiver obstáculo próximo a frente
            self.cmd_vel.linear.x = -0.2
            self.cmd_vel.angular.z = -0.5
        elif min_range_ne < 0.8: # se o obstáculo estiver do lado direito
            self.cmd_vel.linear.x = 0.2
            self.cmd_vel.angular.z = -0.6
        elif min_range_nw < 0.8: # se o obstáculo estiver do lado esquerdo
            self.cmd_vel.linear.x = 0.2
            self.cmd_vel.angular.z = 0.6
        elif min_range_e < 0.2:
            self.cmd_vel.linear.x = 0.1
            self.cmd_vel.angular.z = -0.5
        elif min_range_w < 0.2:
            self.cmd_vel.linear.x = 0.1
            self.cmd_vel.angular.z = 0.5
        else:
            self.cmd_vel.linear.x = 0.2
            self.cmd_vel.angular.z = 0.0

        self.velocity_publisher.publish(self.cmd_vel)

    def get_stop(self, msg):
        if msg.data == True:
            self.get_logger().info("Cartographer node has been stopped")
            rclpy.shutdown()


def main():
    rclpy.init()
    robot_navigator = RobotNavigator()
    rclpy.spin(robot_navigator)
    robot_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()