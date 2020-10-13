#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
global Goal_reached
global Radius
global PI
global Speed
global Distance
Distance = 0.0
PI = 3.141592653589793238

def pose_callback(msg):
    global Speed
    global Goal_reached
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
        Goal_reached = True
        print("Goal Reached!")
    '''Distance = Distance + abs(msg.theta)
            
                print("Distance {0}".format(Distance))
                
                if Distance >= 2*PI:
                	vel_publisher = rospy.Publisher(
                                    "/turtle1/cmd_vel", Twist, queue_size=10)
                    stop_cmd = Twist()
                    stop_cmd.linear.x = 0.0
                    stop_cmd.angular.z = 0.0
                    vel_publisher.publish(stop_cmd)
                    Goal_reached = True
                    print("Goal Reached!")'''


def func_revolve_circle():

    global Goal_reached, Radius, PI, Speed, Distance
    Radius = 1.5
    PI = 3.141592653589793238
    Speed = 1.5 * Radius
    Goal_reached = False
    rospy.init_node("turtle_revolve", anonymous=False)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    vel_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rotate_cmd = Twist()
    rotate_cmd.linear.x = Speed
    rotate_cmd.angular.z = Speed / Radius
    while not Goal_reached:
        vel_publisher.publish(rotate_cmd)


if __name__ == '__main__':
    try:
        func_revolve_circle()
    except rospy.ROSInterruptException:
        pass
