#!/usr/bin/env python

import rospy


def main():    
    
    # 1. Make the script a ROS Node.
    rospy.init_node('hello', anonymous=True)

    # 2. Read from Parameter Server
    rospy.loginfo("Reading from Parameter Server.")

    
    param_config_my = rospy.get_param('order')    # Get all the parameters inside 'details'

    # # Store the parameters in variables
    # first_name = param_config_my['name']['first']
    # last_name = param_config_my['name']['last']
    # address = param_config_my['contact']['address']
    # phone = param_config_my['contact']['phone']

    # # Print the parameters
    # rospy.loginfo(">> First Name: {}".format(first_name))
    # rospy.loginfo(">> Last Name: {}".format(last_name))
    # rospy.loginfo(">> Address: {}".format(address))
    # rospy.loginfo(">> Phone: {}".format(phone))

    # 3. Modify the Phone Number 

    # rospy.set_param('/details/contact/phone', 55555)        # Modify only Phone Number in Parameter Server
    
        # if (rospy.get_param('order/goal_status') == 'True'):
    count = 0
    l = [0]*9
    while(1):
        low_cost_imdex= -1
        high_cost_index = -1
        medium_cost_index = -1
        order_list = rospy.get_param('order/goal')   # Get only Phone Number from Parameter Server
        len_order_list = len(order_list)
        if (len_order_list > count and order_list!="none"):
            for i in range(len_order_list):
                if (l[i]==0):
                    d = order_list[i]
                    if(d["item"] == "Medicine"):
                        high_cost_index = i
                        break
                    elif(d["item"] == "Food"):
                        medium_cost_index = i
                    else:
                        low_cost_imdex = i
            if(high_cost_index != -1):
                l[high_cost_index] = 1
                print(order_list[high_cost_index]["item"])
            elif(medium_cost_index != -1):
                l[medium_cost_index] = 1
                print(order_list[medium_cost_index]["item"])
            else:
                l[low_cost_imdex] = 1
                print(order_list[low_cost_imdex]["item"])
            # print(order_list[count])
            count = count+1
            rospy.sleep(13)
        else:
            print("waiting for order")
            rospy.sleep(.1)




        # rospy.loginfo(">> New Phone: {}".format(new_phone))     # Print the new Phone Number
        # print("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh")
        # rospy.set_param('order/goal_status', 'False')
    # rospy.sleep(1)
    
    


if __name__ == '__main__':
    
    main()

