#!/usr/bin/env python

import rospy 


def main():
    
    rospy.init_node("node_param_get_set",anonymous=True)
    data = rospy.get_param('details')
    
    first_name = data['name']['first']
    last_name = data['name']['last']
    City = data['contact']['address']
    Number = data['contact']['phone']

    rospy.loginfo("-> First Name {}".format(first_name))
    rospy.loginfo("-> Last Name {}".format(last_name))
    rospy.loginfo("-> Residing City {}".format(City)) 
    rospy.loginfo("-> Phone Number {}".format(Number))

    rospy.set_param('/details/contact/phone',11111)
    new_Number = rospy.get_param('/details/contact/phone') 
    rospy.loginfo("-> New Phone Number {}".format(new_Number))
    

if __name__ == "__main__":
    try:
       main()
    except rospy.RosInterruptException:
       pass 
   
