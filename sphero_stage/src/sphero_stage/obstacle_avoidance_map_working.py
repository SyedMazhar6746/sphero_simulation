#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.ndimage import rotate
from nav_msgs.msg import OccupancyGrid
from bresenham import bresenham
from scipy.ndimage import rotate
from std_msgs.msg import Int32MultiArray
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import PoseStamped , PoseArray
from geometry_msgs.msg import Pose

class ClusterAnalyzer:

    def get_neighbors(self, pos):
        row, col = pos
        neighbors = [(row - 1, col - 1), (row - 1, col), (row - 1, col + 1),
                     (row, col - 1),                     (row, col + 1),
                     (row + 1, col - 1), (row + 1, col), (row + 1, col + 1)]
        return [(r, c) for r, c in neighbors if 0 <= r < self.map_data.shape[0] and 0 <= c < self.map_data.shape[1]]

    def dfs_connected_points(self, start_pos, connected_points, visited):
        stack = [start_pos]
        while stack:
            current_pos = stack.pop()
            if current_pos not in visited:
                visited.add(current_pos)
                connected_points.append(current_pos)
                neighbors = self.get_neighbors(current_pos)
                for neighbor in neighbors:
                    if self.map_data[neighbor] == 100 and neighbor not in visited:
                        stack.append(neighbor)

    @staticmethod
    def centroid(cluster):
        # Calculate centroid as the average position of points in the cluster
        centroid_x = int(round(sum(x for x, _ in cluster) / len(cluster)))
        centroid_y = int(round(sum(y for _, y in cluster) / len(cluster)))
        return (centroid_x, centroid_y)

    def pixels_to_meters(self, pixel_coordinates, map_size=200, map_range=10.0):
        # Calculate the middle pixel (center of the map)
        middle_pixel = map_size // 2
        
        # Calculate the meters per pixel
        meters_per_pixel = map_range / map_size
        
        # Convert pixels to meters
        y_pixels, x_pixels = pixel_coordinates
        y_meters = (middle_pixel - y_pixels) * meters_per_pixel
        x_meters = (x_pixels - middle_pixel) * meters_per_pixel
        
        return [x_meters, y_meters]
    




    def find_clusters(self, max_points_per_cluster=10):
        positions_with_100 = np.where(self.map_data == 100)
        position_list = list(zip(positions_with_100[0], positions_with_100[1]))

        cluster_lists = []
        visited = set()

        for pos in position_list:
            if self.map_data[pos] == 100 and pos not in visited:
                connected_points = []
                self.dfs_connected_points(pos, connected_points, visited)
                
                while connected_points:
                    cluster_lists.append(connected_points[:max_points_per_cluster])
                    connected_points = connected_points[max_points_per_cluster:]

        centroids = [self.centroid(cluster) for cluster in cluster_lists]
        centroids = [list(ele) for ele in centroids]
        cluster_lists = [[[int(arr_item) for arr_item in arr] for arr in sublist] for sublist in cluster_lists]

        return centroids, cluster_lists



    def read_map_callback(self, msg):
        """Callback function that is called when a message is received on the `/map` topic."""
        map_data = np.array(msg.data)
        map_data = np.reshape(map_data, (msg.info.height, msg.info.width), 'F')
        map_data = rotate(map_data, 90)
        self.map_data = map_data
        # list_map = list(self.map_data)
        # print('map', list_map[0, 0].shape)
        # rospy.set_param('/correct_map', self.map_data)

        self.map_sub.unregister()  # Unsubscribe after receiving the map data
        centroids, cluster_lists = self.find_clusters()
        
        centroids_meters = [self.pixels_to_meters(centroid_pixel) for centroid_pixel in centroids]
        rospy.set_param('/centroids_meters', centroids_meters)
        
        # print(centroids_meters[0])
        # self.frontier_publisher(centroids_meters)
        # self.publish_markers(centroids_meters)

        # centroids.pop(0)
        # cluster_lists.pop(0)
        # Store centroids and cluster lists as ROS parameters
        rospy.set_param('/centroids', centroids)
        rospy.set_param('/clusters', cluster_lists)







    def __init__(self):
        self.map_data = None
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.read_map_callback)

if __name__ == '__main__':
    try:
        rospy.init_node('analyze_clusters_node', anonymous=True)

        analyzer = ClusterAnalyzer()
        rospy.spin()  # Keeps the node running until shutdown
    except rospy.ROSInterruptException:
        pass
