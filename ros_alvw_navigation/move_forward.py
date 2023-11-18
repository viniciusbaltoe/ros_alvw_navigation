import rclpy
from geometry_msgs.msg import Twist
import time

class ForwardMotionNode:
    def __init__(self):
        self.node = rclpy.create_node('forward_motion_node')
        self.velocity_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()

    def move_forward(self, linear_speed=0.2, duration=5.0):
        self.cmd_vel.linear.x = linear_speed
        self.cmd_vel.angular.z = 0.0

        start_time = time.time()
        end_time = start_time + duration
        
        while (time.time() < end_time) and rclpy.ok():
            self.velocity_publisher.publish(self.cmd_vel)

    def stop_motion(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.velocity_publisher.publish(self.cmd_vel)

def main():
    rclpy.init()
    forward_motion_node = ForwardMotionNode()

    try:
        forward_motion_node.move_forward()
    except KeyboardInterrupt:
        pass
    finally:
        forward_motion_node.stop_motion()
        forward_motion_node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
