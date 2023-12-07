#!/usr/bin/env python3

#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped , PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

def publish_point():
    rospy.init_node('point_marker_publisher', anonymous=True)
    # marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    frontier_pub = rospy.Publisher('/frontier', PoseArray, queue_size=10)
    rate = rospy.Rate(1)  # Set publishing rate (1 Hz in this case)
    rate.sleep()
    centroids = rospy.get_param('/centroids_meters')
    j = 0
    # while not rospy.is_shutdown():
    for i in range(4):
        frontiers = PoseArray()
        frontiers.header.frame_id = "map"
        frontiers.header.stamp = rospy.Time.now()
    
        for i in centroids:
            point = Pose()
            point.position.x = i[0]
            point.position.y = i[1]
            
            frontiers.poses.append(point)
        
    
        frontier_pub.publish(frontiers)
        
        rate.sleep()
        j+=1
        print(j)

if __name__ == '__main__':
    try:
        publish_point()
    except rospy.ROSInterruptException:
        pass
