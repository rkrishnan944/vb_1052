#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry


def func_callback(self,msg):   
	self.pose = msg
    rospy.loginfo(pose)


def main():


    rospy.init_node("turtlesim_node",anonymous=True)

    rospy.Subscriber('/turtle1/Odom', Odometry, func_callback)
    current_pose = Odometry()
    print(current_pose)
    rospy.spin()


main()
    
