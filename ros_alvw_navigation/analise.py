import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

import matplotlib.pyplot as plt
import numpy as np

import cv2
import numpy as np
from sklearn.cluster import DBSCAN

from tabulate import tabulate
import os
from datetime import datetime

class Analise(Node):

    def __init__(self, plot=False):
        super().__init__('analista')
        self.plot = plot

        #Create publisher
        self.publisher_ = self.create_publisher(Bool, 'stop', 10)
        self.sync_timer = 2
        self.timer_publisher = self.create_timer(self.sync_timer, self.pub_callback)

        # Create subscribers
        self.pose_subscriber = self.create_subscription(OccupancyGrid, 'ownmap', self.get_map, 10)

        #log
        self.get_logger().info("Analista node has been started")

        #Plot
        self.grid = np.zeros((120, 120))

        self.no_update_count = 0
        self.previous_num_clusters = 0

        #Plot
        if self.plot:
            self.fig, self.ax = plt.subplots()
            self.im = self.ax.imshow(self.grid, cmap='Greys', origin='lower')
            plt.ion()
    
    def get_map(self, msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)

        #Binariza os objetos
        self.grid[self.grid==1] = 0
        self.grid[self.grid==-1] = 1

        # Apply closing operation
        kernel = np.ones((3, 3), np.uint8)
        closing_matrix = cv2.morphologyEx(self.grid.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

        #Identifica os objetos
        clusters = self._identify_clusters(closing_matrix, 3, 20)
        # Check if clusters have changed
        if self.previous_num_clusters != len(clusters):
            self.previous_num_clusters = len(clusters)
            self.no_update_count = 0
        else:
            self.no_update_count += 1

        #Retira a borda
        # Calculate the area of each cluster
        cluster_areas = []
        for label, indices in clusters.items():
            area = len(indices)
            cluster_areas.append((label, area))
        # Sort the clusters by area in descending order
        cluster_areas.sort(key=lambda x: x[1], reverse=True)
        # Remove the cluster with the largest area from the list
        if cluster_areas:
            largest_cluster_label = cluster_areas[0][0]
            del clusters[largest_cluster_label]

        # Calculate the centroid of each cluster
        cluster_centroids = []
        for label, indices in clusters.items():
            # Get the x and y coordinates of the points in the cluster
            x_coordinates = [index[1] for index in indices]
            y_coordinates = [index[0] for index in indices]
            
            # Calculate the centroid coordinates
            centroid_x = sum(x_coordinates) / len(x_coordinates)
            centroid_y = sum(y_coordinates) / len(y_coordinates)
            
            # Append the centroid coordinates to the list
            cluster_centroids.append((label, centroid_x, centroid_y))

        # Create a list to store the table data
        self.table_data = []

        # Iterate over the cluster areas and centroids
        cluster_areas.sort(key=lambda x: x[0])
        for label, area in cluster_areas:
            for centroid_label, centroid_x, centroid_y in cluster_centroids:
                if label == centroid_label and label != -1:
                    # Append the data to the table
                    self.table_data.append([label, area, centroid_x, centroid_y])
                    break
        
        # Print the table
        os.system('clear')
        print(tabulate(self.table_data, headers=["label", "area", "x_centroid", "y_centroid"]))
        print("\nNo update count:", self.no_update_count)

        if self.plot:
            image = np.zeros((*self.grid.shape, 3), dtype=np.float32)

            # Generate a list of unique colors for each cluster
            colors = plt.cm.get_cmap('tab20', len(clusters))

            # Iterate over the clusters and assign a unique color to each
            for i, cluster_indices in clusters.items():
                color = colors(i)[:3]  # Get the first three values of the color sequence (R, G, B)
                for index in cluster_indices:
                    image[index[0], index[1]] = color  # Assign the color sequence to the pixel

            self.ax.clear()
            self.ax.imshow(image, cmap='gray', interpolation='nearest', origin='lower')
            
            plt.title("Clusters")
            self.fig.canvas.draw()
            plt.pause(0.01)

    def _identify_clusters(self, binary_matrix, eps, min_samples):
        # Get the indices of the non-zero elements in the binary matrix
        indices = np.argwhere(binary_matrix != 0)
        
        # Apply DBSCAN to the indices
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(indices)
        
        # Get the labels assigned by DBSCAN
        labels = db.labels_
        
        # Get the unique labels
        unique_labels = np.unique(labels)
        
        # Create a dictionary to store the clusters
        clusters = {}
        
        # Iterate over the unique labels
        for label in unique_labels:
            # Get the indices of the points with the current label
            cluster_indices = indices[labels == label]
            
            # Add the cluster to the dictionary
            clusters[label] = cluster_indices
        
        return clusters

    def save_txt(self):
        # Create a text file named "results.txt"
        file_path = "results.txt"
        
        # Open the file in write mode
        with open(file_path, "w") as file:
            # Write the number of clusters found
            num_clusters = len(self.table_data)
            file.write(f"Number of Clusters: {num_clusters}\n")
            
            # Write the table data
            file.write("\n")
            file.write(tabulate(self.table_data, headers=["label", "area", "x_centroid", "y_centroid"], tablefmt="grid"))
        
        self.get_logger().info(f"Results saved to {file_path}")

    def pub_callback(self):
        msg = Bool()
        if self.no_update_count >= 60:
            self.get_logger().info("Stop")
            self.save_txt()

            msg.data = True
            self.publisher_.publish(msg)
            self.timer_publisher.cancel()
            self.get_logger().info("Analista node has been stopped")
            rclpy.shutdown()
        else:
            msg.data = False
            self.publisher_.publish(msg)

def main ():
    plot = False

    rclpy.init(args=None)
    node = Analise(plot=plot)
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':  
    main()