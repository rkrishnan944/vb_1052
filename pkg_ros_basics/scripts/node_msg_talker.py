#!/usr/bin/env python
import rospy 
from pkg_ros_basics.msg import myMessage

def main():
    
    var_message_handler = rospy.Publisher("my_topic",myMessage,queue_size = 10) #queue size is no of msgs 
    
    #var_message_handler is like john doe , one which distributes papers

    rospy.init_node("node_talker",anonymous=True)
    
    var_rate = rospy.Rate(1)

    while not rospy.is_shutdown():

    	message = myMessage()
    	message.id = 1 
    	message.name = "Lucknow"
    	message.tempreature = 25.01
    	message.humidity = 35.01

    	rospy.loginfo("Publishing : ")
    	rospy.loginfo(message)

        var_message_handler.publish(message)
        
    	var_rate.sleep()

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException:
        pass
