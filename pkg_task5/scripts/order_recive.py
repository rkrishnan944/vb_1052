#!/usr/bin/env python

# ROS Node - Action Client - IoT ROS Bridge

import rospy
import actionlib
import requests


# Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotAction
# Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotGoal
# Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_vb_sim.msg import Model
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode
from pkg_task4.msg import color
global list1
list1 = []



class Iot:

    # Constructor
    def __init__(self):

        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        param_order = rospy.get_param('order')
        self.order_goal_status = param_order['goal_status']
        self.orfer = param_order['goal']
        self.l = []


    def msg_callback(self, mssg):
        print(mssg.message)
        order_msg = mssg.message
        Dict = eval(order_msg)
        self.l.append(Dict)
        rospy.set_param('order/goal', self.l)

        

# Main
def main():
    rospy.init_node('node_detect_pkgs')
    ic = Iot()
    rospy.Subscriber(
        "/ros_iot_bridge/mqtt/sub",
        msgMqttSub,
        ic.msg_callback)
    # while (1):
    #     var_message_handler = rospy.Publisher("eyrc/vb/color_data_to_array_data",color,queue_size = 10) 
    #     message = color()
    #     message.array = list1
    #     var_message_handler.publish(message)
    print("order order order order")
    rospy.spin()

    cv2.destroyAllWindows()




if __name__ == '__main__':
    main()
