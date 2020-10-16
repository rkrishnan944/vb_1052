#!/usr/bin/env python
'''
 * File:   VB_1052.py
 * Author: Raj Krishnan , Ayush Mishra , Prashant Singh , Ashish Maurya
 * Created on 15 Oct, 2020
 * Functions Used : func_pose_callback() , func_revolve_in_circle().
 * Variables Used : isGoal_reached, Radius, PI, Speed, Distance
 *
'''
#import necessary modules and Library Required.
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
global isGoal_reached
global Radius
global PI
global Speed
global Distance

Distance = 0.0
PI = 3.141592653589793238

'''
************************************************************
Function :    func_pose_callback
Return type : None
Parameters :  msg  Type : Pose()  
Description : Calculates the total distance covered by the Turtle Using Current Angle(theta) of Turtle and Geometrical Forumala.
              Stops the turtle when distance becomes greater than the circumfrence , using Twist().
************************************************************
'''
def func_pose_callback(msg):
    global Speed
    global isGoal_reached
    global Radius
    global PI
    global Distance
    Circumfrence = 2 * PI * Radius
    Distance = Distance + Circumfrence * (abs((msg.theta)) / 360)
    if Distance >= Circumfrence:
        vel_publisher = rospy.Publisher(
                        "/turtle1/cmd_vel", Twist, queue_size=10)
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        vel_publisher.publish(stop_cmd)
        isGoal_reached = True
        
'''        
*********************************************************************
* Function Name : func_revolve_in_circle
* Input         : None
* Output        : None
* Description   : Initializes Radius and Speed. Publishes velocity using Twist() to Turtle and Calls func_pose_callback
                  for Distance check.
**********************************************************************
'''
def func_revolve_in_circle():

    global isGoal_reached, Radius, PI, Speed, Distance
    # Radius value can be changed , Hits wall at Radius > 3.0.
    Radius = 1.5
    PI = 3.141592653589793238
    Speed = 1.5 * Radius
    isGoal_reached = False
    rospy.init_node("node_turtle_revolve", anonymous=False)
    rospy.Subscriber("/turtle1/pose", Pose, func_pose_callback)
    vel_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rotate_cmd = Twist()
    rotate_cmd.linear.x = Speed
    rotate_cmd.angular.z = Speed / Radius
    while not isGoal_reached:
        vel_publisher.publish(rotate_cmd)


if __name__ == '__main__':
    try:
        func_revolve_in_circle()
    except rospy.ROSInterruptException:
        pass
