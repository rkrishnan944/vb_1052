#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty
import copy
from pkg_vb_sim.srv import *
from pkg_vb_sim.msg import *
from pkg_task4.msg import color
global color_sub
global color_matrix
global hashmap
hashmap = {}
color_matrix = []

class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_ur5_2', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        


        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        # Saved Trajectories file inside pkg_task4/config/ur5_2_trajectories
        self._file_path = self._pkg_path + '/config/ur5_2_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        self.vacuum_gripper_state = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
        self.conveyor_belt_power = rospy.ServiceProxy(
            "/eyrc/vb/conveyor/set_power", conveyorBeltPowerMsg)


    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # This function convert a 1D array to 2D array of desired shape
    def reshape(self,the_list, r, c): 

        # Reshape the 1D list into a m row x n column list
        if r*c != len(the_list): 
            raise ValueError('Invalid new shape')  
        return [the_list[tr*c:(tr+1)*c] for tr in range(0,r)] 

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):

        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
          loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

  
    def moveit_hard_play_planned_path_from_file(self, pos1, pos2, arg_max_attempts = 5):

        number_attempts = 0
        flag_success = False
        arg_file_path = self._file_path
        arg_file_name = 'ur5_2_'+ pos1 +'_to_'+ pos2 +'.yaml'
        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
          number_attempts += 1
          flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
          rospy.logwarn("attempts: {}".format(number_attempts) )
          # # self.clear_octomap()
        
        return True

    def color_callback(self,data):

        global color_sub
        global color_matrix
        global hashmap
        color_array = data.array
        # The received list from the topic on which the 1D-matrix was received, is reshaped into a 4*3 matrix (4 row 3 columns in shelf) ,
        # which is converted into a hashmap with pkg_name:color format so UR5_2 can easily search in O(1) time 
        # the color associated with the pkg_name, which can be used to pick-place in respective color bin.
        color_matrix = self.reshape(color_array,4,3)

        for i in range(len(color_matrix)):

            for j in range(len(color_matrix[0])):

                hashmap['packagen'+str(i)+str(j)] = color_matrix[i][j]
        color_sub.unregister()


    def logical_camera_callback(self, msg):
        global hashmap

        if len(msg.models) >= 2:

            data = str(msg.models[1])
            res = data.split()
            package_name = res[1]
            position = [float(res[5]), float(res[7]), float(res[9])]
            orientation = [
                float(
                    res[12]), float(
                    res[14]), float(
                    res[16]), float(
                    res[18])]
        

        if position[1] <= 0.01 :
          
            pkg_name = package_name[1:-1]
            color_detected = hashmap[pkg_name]
            print("my colour is: ",color_detected)
            self.conveyor_belt_power(0)
            self.vacuum_gripper_state(True)
            self.moveit_hard_play_planned_path_from_file('home', color_detected)
            self.vacuum_gripper_state(False)
            self.moveit_hard_play_planned_path_from_file(color_detected, 'home')
            self.conveyor_belt_power(99)


        else:
            self.conveyor_belt_power(99)

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    global color_sub

    ur5 = Ur5Moveit(sys.argv[1])

    ur5.moveit_hard_play_planned_path_from_file('zero','home')
    ur5.conveyor_belt_power(35)

    color_sub = rospy.Subscriber("eyrc/vb/color_data_to_array_data", color, ur5.color_callback,queue_size=10)

    rospy.Subscriber(
        "/eyrc/vb/logical_camera_2",
        LogicalCameraImage,
        ur5.logical_camera_callback,
        queue_size=1)
        
    rospy.spin()

    del ur5

if __name__ == '__main__':
    main()

