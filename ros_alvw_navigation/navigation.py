import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotNavigator(Node):

    def __init__(self):
        super().__init__('robot_navigator')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.cmd_vel = Twist()

    def laser_callback(self, laser_data):
        self.get_logger().debug("RobotNavigator node initialized")
        
        # Faz a verificação das ditâncias do scan do tópico /scan
        min_range = min(laser_data.ranges)
        if min_range > 0.25:  
            # Navega para frente se a distância é maior que 0.2 metros
            self.cmd_vel.linear.x = 0.4
            self.cmd_vel.angular.z = 0.0
        else:
            # Gira para a esquerda se a distância é menor que 0.2 metros
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.5

        # publica a velocidade no tópico /cmd_vel
        self.velocity_publisher.publish(self.cmd_vel)

def main():
    rclpy.init()
    robot_navigator = RobotNavigator()
    rclpy.spin(robot_navigator)
    robot_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
