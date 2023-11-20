import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import math
import time

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
        #self.cmd_vel.linear.x = 0.0
        #self.cmd_vel.angular.z = 0.0

        self.get_logger().info("RobotNavigator node initialized")

        min_range = min(laser_data.ranges)
        front_range = laser_data.ranges[0]

        if min_range > 0.3: # and front_range > 0.7:
            near_flag = False # Nada próximo 
        else:
            near_flag = True  # Algo próximo

        if not near_flag:
            # Navega para frente
            # self.get_logger().info("Vai para frente")
            self.cmd_vel.linear.x = 0.3
            self.cmd_vel.angular.z = 0.0
        else:
            # Verifica a direção do obstáculo
            angle_min = laser_data.angle_min
            angle_increment = laser_data.angle_increment
            index = laser_data.ranges.index(min_range)
            # O obstáculo está na coordenada polar (angle, distance)
            angle = (angle_min + index * angle_increment) + math.pi/2 # radianos


            if math.sin(angle) > 0: # Obstáculo na frente
                self.cmd_vel.linear.x = - 0.3
                # Como o angulo na frente o robô é zero (0), eu somei pi/2 para ficar igual ao circulo trigonometrico
                # O "/5" foi uma tentativa de reduzir a sensibilidade e fazer ele curvar menos.
                self.cmd_vel.angular.z = -1 * math.sin(angle) /5 #* math.cos(angle)

            else:                   # Obstáculo atras
                self.cmd_vel.linear.x = 0.3
                self.cmd_vel.angular.z = math.sin(angle)  /5#* math.cos(angle)
            

            # # Obstáculo está a frente do robô
            # if  (angle > 2*math.pi * 7/8) or (angle <= 2*math.pi * 1/8):
            #     self.cmd_vel.linear.x = -0.2
            #     self.cmd_vel.angular.z = 0.5 

            # # Obstáculo está atras do robô
            # elif  (angle > 2*math.pi * 3/8) and (angle <= 2*math.pi * 5/8):
            #     self.cmd_vel.linear.x = 0.2
            #     self.cmd_vel.angular.z = 0.5            
            
            # # Obstáculo está a esquerda do robô
            # elif  (angle > 2*math.pi * 1/8) and (angle <= 2*math.pi * 3/8):
            #     self.cmd_vel.linear.x =  0.2
            #     self.cmd_vel.angular.z = -0.5

            # # Obstáculo está a direita do robô
            # else:# (angle > 2*math.pi * 7/8) and (angle <= 2*math.pi * 1/8):
            #     self.cmd_vel.linear.x = 0.2
            #     self.cmd_vel.angular.z = 0.5

                      
        self.velocity_publisher.publish(self.cmd_vel)



def main():
    rclpy.init()
    robot_navigator = RobotNavigator()
    rclpy.spin(robot_navigator)
    robot_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
