#!/usr/bin/env python3

import numpy as np
import pdb
 
import math
import rospy
from geometry_msgs.msg import Twist
from helper_function.utils import Vector2, angle_diff, get_agent_position, get_agent_velocity
import rospy
from geometry_msgs.msg import Twist
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from tf.transformations import euler_from_quaternion
import random
 
from helper_function.utils import Vector2
# import sphero_stage.src.sphero_stage.helper_function.utils as utils
from bresenham import bresenham

class Boid(object):

    # use twist type for velocity and pose type message for position

    def __init__(self, map_data, initial_velocity_x, initial_velocity_y, max_speed_mag, slowing_radius): # , wait_count, start_count, frequency
        """Create an empty boid and update parameters."""
        self.position = Vector2()
        self.velocity = Vector2()
        
        self.max_speed_mag = max_speed_mag #
        self.slowing_radius = slowing_radius #

        # Set initial velocity
        # self.initial_velocity = Twist()
        # self.initial_velocity.linear.x = initial_velocity_x
        # self.initial_velocity.linear.y = initial_velocity_y

        self.all_clusters = rospy.get_param('/clusters')
        self.all_centroids = rospy.get_param('/centroids')
        self.map_data = map_data



    # position and velocity vector components
    def set_pose(self, name, variable):
        rospy.set_param(name, variable)

    # position and velocity vector
    def get_pose(self, name):
        pose = rospy.get_param(name)
        new_vector = Vector2(pose[0], pose[1])
        return new_vector

    
    def arrive(self, agent_msg, target):

        target_v = Vector2(target[0], target[1])
        desired_velocity_v = Vector2()
        self.position_v = get_agent_position(agent_msg) # agent position

        target_offset_v = target_v - self.position_v
        distance = target_offset_v.norm() 
        
        ramped_speed = (distance / self.slowing_radius)
        
        if distance < 1e-3:
            
            return Vector2()
        else:
            desired_velocity_v.x = (ramped_speed / distance) * target_offset_v.x
            desired_velocity_v.y = (ramped_speed / distance) * target_offset_v.y
            
            if target_offset_v.norm() > self.max_speed_mag:
                desired_velocity_v.set_mag(self.max_speed_mag)
                
            return desired_velocity_v
        
    def get_cmd_vel(self, agent_msg, target):

        self.steering_force_v = self.arrive(agent_msg, target)

        cmd = Twist()
        cmd.linear.x = self.steering_force_v.x
        cmd.linear.y = self.steering_force_v.y  # Adjust the angular velocity as needed
        return cmd, self.steering_force_v

        
    def meters_to_pixels(self, meter_coordinates, map_size=200, map_range=10.0): # meter_coordinates = [x, y]
        
        # Calculate the middle pixel (center of the map)
        middle_pixel = map_size // 2
        
        # Calculate the pixel scale per meter
        pixels_per_meter = map_size / map_range
        
        # Convert meters to pixels
        x_meters, y_meters = meter_coordinates
        x_pixels = int((x_meters  * pixels_per_meter)+ middle_pixel)
        y_pixels = int(-(y_meters  * pixels_per_meter) + middle_pixel)

        return [y_pixels, x_pixels]
    
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





# take end point from robot and map and return all the cells with obstacles
    def check_front_cylinder_obstacle(self, position_list, end_array, cylinder_len=0.5):

        start_m = [position_list[0], position_list[1]]
        start_pi = self.meters_to_pixels(start_m)
        
        angle_rad = math.atan2(end_array[1], end_array[0]) 
        
        end_m = [start_m[0] + cylinder_len*np.cos(angle_rad), start_m[1] + cylinder_len*np.sin(angle_rad)] # equal lenth 
        
        # Calculate end_pi without clamping 
        end_pi_unclamped = self.meters_to_pixels(end_m)
        
        # Clamp end_pi values to stay within boundary
        end_pi = [np.clip(coord, 0, 200) for coord in end_pi_unclamped]

        # Define the line endpoints
        line_endpoints = [start_pi, end_pi]

        # Get the cells from Bresenham's algorithm
        cells = list(bresenham(line_endpoints[0][0], line_endpoints[0][1], line_endpoints[1][0], line_endpoints[1][1]))
        
        # Expand the list to include adjacent cells
        expanded_cells = cells.copy()
        for cell in cells:
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if (i, j) != (0, 0):  # Exclude the original cells
                        adjacent_cell = (cell[0] + i, cell[1] + j)
                        if 0 <= adjacent_cell[0] < self.map_data.shape[0] and 0 <= adjacent_cell[1] < self.map_data.shape[1]:
                            expanded_cells.append(adjacent_cell)

        # Remove duplicates
        expanded_cells = list(set(expanded_cells))

        # Find cells with a value of 100
        cells_with_100 = [[x, y] for x, y in expanded_cells if 0 <= x < 200 and 0 <= y < 200 and self.map_data[x][y] == 100]
        
        if not cells_with_100:
            
            return None  # no obstacle cells
        
        return cells_with_100 




# take all the cells with obstacles and return clusters and their centroids

    def find_cluster_centroid_infront(self, agent_msg, start_list, end_array): 

        """end is with velocity vector and map_array is a 2D array"""
        
        indices = self.check_front_cylinder_obstacle(start_list, end_array) 

        if not indices:
            return [] # no obstacles found
        
        centroids_of_interest = []
        for point in indices:
            for idx, cluster in enumerate(self.all_clusters):
                if point in cluster:
                    centroid = self.all_centroids[idx]
                    if centroid not in centroids_of_interest:
                        centroids_of_interest.append(centroid)

        # Convert centroids from pixels to meters
        centroids_meters = [self.pixels_to_meters(centroid_pixel) for centroid_pixel in centroids_of_interest]
        
        return centroids_meters   # [[1.0, 1.5], [2.0, 2.5]] cemtroids of the obstacles
        




    def rotate_vector(self, vector, direction, angle_deg):

        angle_rad = np.radians(angle_deg)

        if direction == "clockwise":
            rotation_matrix = np.array([[np.cos(angle_rad), np.sin(angle_rad)],
                                        [-np.sin(angle_rad), np.cos(angle_rad)]])
        else:  # Counterclockwise
            rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                        [np.sin(angle_rad), np.cos(angle_rad)]])
        
        # Perform matrix multiplication to rotate the vector
        rotated_vector = np.dot(rotation_matrix, vector)/np.linalg.norm(vector)
        return rotated_vector

                
    def find_repulsive_vector(self, total_vel_array, adjacent_differences_pos, angle_list_positive_sorted, adjacent_differences_neg, angle_list_negative_sorted):
            
            max_diff_pos = max(adjacent_differences_pos) 
            max_diff_neg = max(adjacent_differences_neg) 

            if max_diff_pos > abs(max_diff_neg):

                direction = "counter-clockwise"
                max_index_pos = adjacent_differences_pos.index(max_diff_pos)
                big_angle_pos = angle_list_positive_sorted[max_index_pos + 1]
                small_angle_pos = angle_list_positive_sorted[max_index_pos]
                desired_orient_pos = small_angle_pos + (big_angle_pos - small_angle_pos)/2
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, desired_orient_pos)

            else:
                direction = "clockwise"
                max_index_neg = adjacent_differences_neg.index(max_diff_neg)
                big_angle_neg = angle_list_negative_sorted[max_index_neg + 1]
                small_angle_neg = angle_list_negative_sorted[max_index_neg]
                desired_orient_neg = small_angle_neg + (big_angle_neg - small_angle_neg)/2
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, desired_orient_neg)

            return repulsive_v_array

    def angle_between_100_and_170(self, CW, rotate_angle, angles_positive, angles_negative, total_vel_array):

        if not angles_positive and not angles_negative:
            
            if CW < 15:

                print('counter-clockwise count', CW)
                direction = "counter-clockwise"
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, rotate_angle)
                CW += 1
            elif CW >= 15:

                print('clockwise count', CW-15)
                direction = "clockwise"
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, rotate_angle)
                CW += 1

        elif not angles_positive:

            print('counter-clockwise arbitrary')
            direction = "counter-clockwise"
            repulsive_v_array = self.rotate_vector(total_vel_array, direction, rotate_angle)
            
        elif not angles_negative:
            print('clockwise arbitrary')
            direction = "clockwise"
            repulsive_v_array = self.rotate_vector(total_vel_array, direction, rotate_angle)
            
        return repulsive_v_array, CW
                



    def get_desired_repulsive_vel(self, agent_msg, total_vel_array, CW, obstacles, max_repulsive_force=1.0):
        

        self.position_v = get_agent_position(agent_msg)

        start_m = [self.position_v.x, self.position_v.y]
        start_m_array = np.array([start_m[0], start_m[1]])
        start_pi = self.meters_to_pixels(start_m)

        adjacent_cells = []

        # Check 10 pixels in all directions around the robot 
        for i in range(-15, 16):
            for j in range(-15, 16):
                cell = [start_pi[0] + i, start_pi[1] + j]

                # Ensure the cell is within the map bounds
                if 0 <= cell[0] < 200 and 0 <= cell[1] < 200:
                    # Check if the cell contains an obstacle
                    if self.map_data[cell[0]][cell[1]] == 100:
                        adjacent_cells.append(cell)

        centroids_of_interest = []
        for point in adjacent_cells:
            for idx, cluster in enumerate(self.all_clusters):
                if point in cluster:
                    centroid = self.all_centroids[idx]
                    if centroid not in centroids_of_interest:
                        centroids_of_interest.append(centroid)
        
        centroids_meters = [self.pixels_to_meters(centroid_pixel) for centroid_pixel in centroids_of_interest]
        

        angle_list_positive = []
        angle_list_negative = []


        for centroid_m in centroids_meters:
            centroid_m_array = np.array([centroid_m[0], centroid_m[1]])

            along_vector = [centroid_m_array[0]-start_m_array[0], centroid_m_array[1]-start_m_array[1]]


            dot_product = np.dot(total_vel_array, along_vector)
            cross_product = np.cross(total_vel_array, along_vector)

            # Calculate angle in radians
            angle_rad = np.arctan2(np.linalg.norm(cross_product), dot_product)
            angle_deg = np.degrees(angle_rad)

            # Convert angle to degrees
            if cross_product > 0:
                angle_list_positive.append(angle_deg)
            else:
                angle_list_negative.append(-angle_deg)

        
        # Check for angles between 45 to 135 degrees 
        angles_45_to_135 = [angle for angle in angle_list_positive if 45 <= angle <= 135]
    
        # Check for angles between -45 to -135 degrees 
        angles_minus_45_to_135 = [angle for angle in angle_list_negative if -135 <= angle <= -45]

        angles_100_to_170 = [angle for angle in angle_list_positive if 100 <= angle <= 170]
    
        # Check for angles between -45 to -135 degrees
        angles_minus_100_to_minus_170 = [angle for angle in angle_list_negative if -170 <= angle <= -100]

        # if not angles_45_to_135 or not angles_minus_45_to_135:
        #     # print(90)
        #     rotate_angle_45_to_135 = 100
        #     repulsive_v_array, CW = self.angle_between_100_and_170(CW, rotate_angle_45_to_135, angles_45_to_135, angles_minus_45_to_135, total_vel_array)

        if not angles_100_to_170 or not angles_minus_100_to_minus_170:
            print(115)

            rotate_angle_100_to_170 = 115
            repulsive_v_array, CW = self.angle_between_100_and_170(CW, rotate_angle_100_to_170, angles_100_to_170, angles_minus_100_to_minus_170, total_vel_array)

        else:
        
            angle_list_positive_sorted = sorted(angle_list_positive)
            adjacent_differences_pos = [angle_list_positive_sorted[i + 1] - angle_list_positive_sorted[i] for i in range(len(angle_list_positive_sorted) - 1)]
            

            angle_list_negative_sorted = sorted(angle_list_negative)
            adjacent_differences_neg = [angle_list_negative_sorted[i + 1] - angle_list_negative_sorted[i] for i in range(len(angle_list_negative_sorted) - 1)] # angle difference

            
            if not adjacent_differences_pos: # if it is empty 
                
                direction = "counter-clockwise"
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, 90)

            elif not adjacent_differences_neg: # if it is empty

                direction = "clockwise"
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, 90)

            else:
                repulsive_v_array = self.find_repulsive_vector(total_vel_array, adjacent_differences_pos, angle_list_positive_sorted, adjacent_differences_neg, angle_list_negative_sorted)




        desired_repulsive_velocity_v = Vector2()
        desired_repulsive_velocity_v.x = repulsive_v_array[0]
        desired_repulsive_velocity_v.y = repulsive_v_array[1]

        return desired_repulsive_velocity_v, CW
                