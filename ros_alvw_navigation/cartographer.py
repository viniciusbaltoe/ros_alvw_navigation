import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

from tf_transformations import euler_from_quaternion

import matplotlib.pyplot as plt
import numpy as np
import cv2

class Cartographer(Node):

    def __init__(self, plot=False):
        super().__init__('cartographer')
        self.plot = plot

        #Create publisher
        self.publisher_ = self.create_publisher(OccupancyGrid, 'ownmap', 10)
        self.sync_timer = 2
        self.timer_publisher = self.create_timer(self.sync_timer, self.pub_callback)

        #Create subscribers
        self.pose_subscriber = self.create_subscription(Odometry, "odom", self.get_pose, 10)
        self.pose_subscriber = self.create_subscription(Bool, "stop", self.get_stop, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, "scan", self.get_ranges, 10)

        #Posições
        self.x = None
        self.y = None
        self.yaw = None

        #Grid
        self.height, self.width = 120, 120
        self.grid = np.zeros((self.height, self.width))

        #Plot
        if self.plot:
            self.fig, self.ax = plt.subplots()
            self.im = self.ax.imshow(self.grid, cmap='Greys', origin='lower')
            plt.ion()


    def get_pose(self, msg):
        pose = msg.pose.pose

        self.x =  pose.position.x
        self.y =  pose.position.y
        _, _, self.yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def get_ranges(self, msg):
        self.ranges = msg.ranges

    def _ranges_to_points(self):
        points = []
        for i, r in enumerate(self.ranges):
            if r <10:
                ang_rad = (i * np.pi / 180) + self.yaw
                x = r * np.cos(ang_rad) + self.x
                y = r * np.sin(ang_rad) + self.y
                points.append((x, y))
        return points
    
    def _points_to_grid(self, points):
        points_on_grid = []
        for x,y in points:
            x = int((x+3)*20)
            y = int((y+3)*20)

            x = min(max(0, x), 119)
            y = min(max(0, y), 119)
            
            points_on_grid.append((x,y))
        return points_on_grid
            # self.grid[x, y] = -1

    def _get_beetween_points(self, end, start_x ,start_y):
        # Retorna uma lista de pontos entre o ponto atual e o ponto end
        x0, y0 = start_x, start_y
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        err = dx - dy

        points = []
        while x0 != x1 or y0 != y1:
            points.append((x0, y0))
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points
    
    def pos_process(self, grid):
        kernel = np.ones((5, 5), np.uint8)
        closing_grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, kernel)
        print(closing_grid)
        inverted_grid = cv2.bitwise_not(closing_grid)
        # _, binary_grid = cv2.threshold(closing_grid, 0, 255, cv2.THRESH_BINARY_INV)
        filled_grid = cv2.morphologyEx(inverted_grid, cv2.MORPH_CLOSE, kernel)
        return filled_grid

    def pub_callback(self):
        if self.x is not None and self.y is not None and self.yaw is not None:
            colision_points = self._ranges_to_points()
            objects = self._points_to_grid(colision_points)

            aux_x = int((self.x+3)*20)
            aux_y = int((self.y+3)*20)

            for object_point in objects:
                free_points = self._get_beetween_points(object_point, aux_x, aux_y)
                for free_point in free_points:
                    if self.grid[free_point] != -1:
                        self.grid[free_point] = 1

            for object_point in objects:
                self.grid[object_point] = -1

            # # Update the plot
            if self.plot:
                self.ax.clear()
                self.ax.imshow(self.grid, cmap='gray', interpolation='nearest', origin='lower')
                self.fig.canvas.draw()
                plt.pause(0.01)

            # Publish raw
            msg = OccupancyGrid()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.info.width = self.width
            msg.info.height = self.height
            msg.data = [int(v) for v in self.grid.flatten()]
            self.publisher_.publish(msg)

    def get_stop(self, msg):
        if msg.data == True:
            self.get_logger().info("Cartographer node has been stopped")
            rclpy.shutdown()

def main():
    plot = False

    rclpy.init()
    cartographer = Cartographer(plot=plot)
    rclpy.spin(cartographer)
    cartographer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()