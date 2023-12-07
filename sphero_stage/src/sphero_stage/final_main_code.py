#!/usr/bin/env python3

import numpy as np
import pdb
 
import rospy
from nav_msgs.msg import Odometry
import nav_msgs.msg
from scipy.ndimage import rotate

from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import math
from helper_function.utils import Vector2
from geometry_msgs.msg import PoseStamped
import time


from final_boid import Boid

class Robot_1_move:

    # take input the whole message from ros and return a vector of class Vector2
    def get_agent_velocity(self, agent):
        """Return agent velocity as Vector2 instance."""
        vel = Vector2()
        vel.x = agent.twist.twist.linear.x
        vel.y = agent.twist.twist.linear.y
        return vel

    # take input the whole message from ros and return a vector of class Vector2
    def get_agent_position(self, agent):
        """Return agent position as Vector2 instance."""
        pos = Vector2()
        pos.x = agent.pose.pose.position.x
        pos.y = agent.pose.pose.position.y
        return pos
    
    def get_goal(self, goal):
            
        print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
        target = [goal.pose.position.x, goal.pose.position.y]
        # print('target', )
        self.targets[1] = [target[0], target[1]]

    def update_target(self):       
        target = rospy.get_param('target')
        self.targets[1] = [target[0], target[1]]

    def rotate_vector(self, vector, direction, angle_rad):

        if direction == "clockwise":
            rotation_matrix = np.array([[np.cos(angle_rad), np.sin(angle_rad)],
                                        [-np.sin(angle_rad), np.cos(angle_rad)]])
        else:  # Counterclockwise
            rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                        [np.sin(angle_rad), np.cos(angle_rad)]])
        
        # Perform matrix multiplication to rotate the vector
        rotated_vector = np.dot(rotation_matrix, vector)
        return rotated_vector
    


    def rotate_repulsive_vector(self, nav_velocity_v, repulsive_velocity_v, angle_rad):

        nav_velocity_array = np.array([nav_velocity_v.x, nav_velocity_v.y])
        repulsive_velocity_array = np.array([repulsive_velocity_v.x, repulsive_velocity_v.y])

        cross_product = np.cross(nav_velocity_array, repulsive_velocity_array)
        if cross_product > 0:  # Greater than 90 degrees, rotate clockwise
            rotated_array = self.rotate_vector(repulsive_velocity_array, "clockwise", angle_rad)
        else:  # Less than or equal to 90 degrees, rotate counterclockwise
            rotated_array = self.rotate_vector(repulsive_velocity_array, "counterclockwise", angle_rad)
        
        rotated_list = rotated_array.tolist()
        return rotated_list


    def send_velocity(self, frame_id, send_velocity_v):
            
            Total_velocity_cmd = Twist()
            Total_velocity_cmd.linear.x = send_velocity_v.x
            Total_velocity_cmd.linear.y = send_velocity_v.y 

            self.cmd_vel_pub[frame_id].publish(Total_velocity_cmd)



    def get_repulsive_vel(self, msg, start_list, total_vel_array):

        obstacle_centroids_points_list = self.agent.find_cluster_centroid_infront(msg, start_list, total_vel_array)
        
        if not obstacle_centroids_points_list:
            repulsive_velocity_v = 0.0
            
        else:
            
            repulsive_velocity_v, self.CW = self.agent.get_desired_repulsive_vel(msg, total_vel_array, self.CW, obstacle_centroids_points_list)
            if self.CW == 20:
                self.CW = 0
            self.repulsive_velocity_v_new = repulsive_velocity_v
            self.stop_repul = True
        return repulsive_velocity_v, self.CW
            
    def odom_callback(self, msg):

        """ General """
        
        if self.count != 40: 
            self.count += 1
        else:
            self.count = 1
            frame_id = msg.header.frame_id 

            if frame_id not in self.cmd_vel_pub:
                self.cmd_vel_pub[frame_id] = rospy.Publisher("{}cmd_vel".format(frame_id[:9]), Twist, queue_size=10)
            id = frame_id[7] # string
            self.agent= self.agents[int(id)]
            
            if self.targets[int(id)] != 0:

                start_list = [msg.pose.pose.position.x, msg.pose.pose.position.y]
                total_vel_array = np.array([self.Total_velocity_list[0], self.Total_velocity_list[1]])
                



                """ ===== Velocities ======"""
                # Repulsive velocity
                repulsive_velocity_v, self.CW = self.get_repulsive_vel(msg, start_list, total_vel_array)

                # Navigation Velocity
                if repulsive_velocity_v == 0.0:
                    _, nav_velocity_v = self.agent.get_cmd_vel(msg, self.targets[int(id)])
                else:
                    nav_velocity_v = Vector2()




                """ ===== Total Velocity ======"""

                Total_velocity = self.nav_weight * nav_velocity_v + self.rep_weight * repulsive_velocity_v

                self.Total_velocity_list = [float(Total_velocity.x), float(Total_velocity.y)]


                """Sending velocities based on obstacle positions """
                if self.stop_repul:
                    self.start_time += 0.1

                    if (self.start_time - self.repulsive_force_duration) < 0:
                        self.send_velocity(frame_id, self.rep_weight * self.repulsive_velocity_v_new)

                    else:
                        self.stop_repul = False
                        self.start_time = 0.1
                        self.send_velocity(frame_id, Total_velocity)

                else:

                    self.send_velocity(frame_id, Total_velocity)
        





    def read_map_callback_2(self, msg):
        """Callback function that is called when a message is received on the `/map` topic."""

        map_data = np.array([msg.data])
        # map_data = map_data.reshape(200, 200)
        map_data = np.reshape(map_data, (200, 200), 'F')
        self.new_map_data = rotate(map_data, 90)
        self.map_subscriber.unregister()

    def __init__(self):

        # initialize
        self.repulsive_velocity_v_new = 0.0
        self.stop_repul = False
        self.count = 1
        self.count_repul = 1
        self.start_time = 0.0
        self.repulsive_force_duration = 1.0 #0.5
        self.pose_list = []
        self.CW = 0
        
        # self.target = rospy.get_param('target')
        self.num_of_robots = rospy.get_param("/num_of_robots")
        self.target_offset = 0.8
        self.targets = [0] * self.num_of_robots
        self.velocity = [0.0, 0.0]  # Initial velocity
        # self.pattern = rospy.get_param('pattern') #'circular_pattern' #'line_pattern'
        self.Total_velocity_list = [0.0, 0.0]
        self.initial_time = rospy.get_time()

        global velocity_history 
        global steering_force_history
        velocity_history = []
        steering_force_history = []
        self.frames = []
        self.cmd_vel_pub = {}

        # Tunable parameters
        self.max_speed = 0.6
        self.slowing_radius = 1.0
        self.search_radius = 2.0

        # Weights
        self.nav_weight = 0.5
        self.rep_weight = 0.5



        self.map_subscriber = rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self.read_map_callback_2)
        self.move_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.get_goal)#None # TODO: subscriber to /move_base_simple/goal published by rviz    
        
        rate = rospy.Rate(10)
        rate.sleep()
        
        self.agents = [Boid(self.new_map_data, initial_velocity_x=0.0, initial_velocity_y=0.0, max_speed_mag=self.max_speed, 
                            slowing_radius=self.slowing_radius) for _ in range(self.num_of_robots)]

        [rospy.Subscriber("robot_1/odom".format(i), Odometry, self.odom_callback) for i in range(self.num_of_robots)]




if __name__ == '__main__':
    try:
        rospy.init_node('rotate_robot_circularly')
        robot = Robot_1_move()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass