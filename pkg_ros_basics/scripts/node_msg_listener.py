#!/usr/bin/env python

import rospy
from pkg_ros_basics.msg import myMessage


def func_callback(Data):

    rospy.loginfo("Data Received: (%d, %s, %.2f, %.2f)", Data.id,
                  Data.name, Data.tempreature, Data.humidity)


def main():

    rospy.init_node('node_listener', anonymous=True)


    rospy.Subscriber("my_topic", myMessage, func_callback)

 
    rospy.spin()


# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
