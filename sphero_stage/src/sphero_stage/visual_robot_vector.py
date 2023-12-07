#!/usr/bin/env python3

#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped , PoseArray
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry 


import rospy
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Twist
import math 
import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

class VelocityVisualizer:
    def __init__(self):
        rospy.init_node('velocity_visualizer', anonymous=True)
        self.marker_pub = rospy.Publisher('/velocity_arrow_marker', Marker, queue_size=10)
        self.velocity_sub = rospy.Subscriber('/robot_1/cmd_vel', Twist, self.velocity_callback)
        self.rate = rospy.Rate(10)  # Set the publishing rate

    def velocity_callback(self, msg):
        # Create a Marker message

        vel_x = msg.linear.x
        vel_y = msg.linear.y
        
        print(msg)
        # vel_angle = 

        marker = Marker()
        marker.header.frame_id = "robot_1/base_footprint"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "velocity_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0

        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        points = [
            Point(x=0.0, y=0.0, z=0.0),
            # Point(x=1.0, y=1.0, z=0.0)
            Point(x=vel_x, y=vel_y, z=0.0)
        ]

        # point = Point()
        # point.x = vel_x
        # point.y = vel_y
        marker.points = points

        marker.scale.x = 0.05  # Set arrow length based on linear velocity x component
        marker.scale.y = 0.1  # Set arrow width
        marker.scale.z = 1.0  # Set arrow height
        
        marker.color.a = 1.0
        marker.color.r = 1.0  # Arrow color (red)
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the Marker message
        self.marker_pub.publish(marker)
        # self.rate.sleep()

if __name__ == '__main__':
    try:
        VelocityVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
