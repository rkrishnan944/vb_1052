#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from pkg_vb_sim.srv import *
from pkg_vb_sim.msg import *


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = 'package_box'

        # self.ur5_red_bin_position = geometry_msgs.msg.Pose()
        # self.ur5_red_bin_position.position.x = 0.0636594369028
        # self.ur5_red_bin_position.position.y = 0.54803149259
        # self.ur5_red_bin_position.position.z =  0.992044340223
        # self.ur5_red_bin_position.orientation.x =  -2.64275648683
        # self.ur5_red_bin_position.orientation.y = 1.57016908923
        # self.ur5_red_bin_position.orientation.z = -2.64257958596

        # self.box_length = 0.15               # Length of the Package
        # self.vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        # self.delta = self.vacuum_gripper_width + (self.box_length/2)  # 0.19
        # # Teams may use this info in Tasks
        # # This  is default ur5 position for ideal pick and place.
        # self.ur5_2_home_pose = geometry_msgs.msg.Pose()
        # self.ur5_2_home_pose.position.x = -0.8
        # self.ur5_2_home_pose.position.y = 0
        # self.ur5_2_home_pose.position.z = 1 + self.vacuum_gripper_width + (self.box_length/2)
        # # This to keep EE parallel to Ground Plane
        # self.ur5_2_home_pose.orientation.x = -0.5
        # self.ur5_2_home_pose.orientation.y = -0.5
        # self.ur5_2_home_pose.orientation.z = 0.5
        # self.ur5_2_home_pose.orientation.w = 0.5

        self.ur5_blue_bin_position = geometry_msgs.msg.Pose()
        self.ur5_blue_bin_position.position.x = 0.147515904227
        self.ur5_blue_bin_position.position.y = -0.794079202196
        self.ur5_blue_bin_position.position.z = 0.802800751801
        self.ur5_blue_bin_position.orientation.x = 2.92418580224
        self.ur5_blue_bin_position.orientation.y = 1.57015212667
        self.ur5_blue_bin_position.orientation.z = 2.92372408829



        self.lst_joint_angles_home = [math.radians(172.160516001),
                                      math.radians(-40.0529267276),
                                      math.radians(58.2606911498),
                                      math.radians(-108.167308562),
                                      math.radians(-89.9478601433),
                                      math.radians(-7.82017669014)]

        self.lst_joint_angles_green_box = [math.radians(6.21723913908),
                                           math.radians(-47.5660325559),
                                           math.radians(74.9007236635),
                                           math.radians(-126.775317946),
                                           math.radians(-91.795291338),
                                           math.radians( 1.47214957278)]

        self.lst_joint_angles_red_box = [math.radians(81.1082328036),
                                        math.radians(-50.5093675483),
                                        math.radians(60.2332643044),
                                        math.radians(-99.9580169041),
                                        math.radians(-89.9588518479),
                                        math.radians(171.074097001)]

        self.lst_joint_angles_blue_box = [math.radians(-92.3966858545),
                                          math.radians(-60.5986613821),
                                          math.radians(94.9649209094),
                                          math.radians(-121.333470328),
                                          math.radians(-90.1306535982),
                                          math.radians(-2.42515857036)]

   

        self.vacuum_gripper_state = rospy.ServiceProxy(
            '/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
        self.conveyor_belt_power = rospy.ServiceProxy(
            "/eyrc/vb/conveyor/set_power", conveyorBeltPowerMsg)

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' +
            "Planning Group: {}".format(
                self._planning_frame) +
            '\033[0m')
        rospy.loginfo(
            '\033[94m' +
            "End Effector Link: {}".format(
                self._eef_link) +
            '\033[0m')
        rospy.loginfo(
            '\033[94m' +
            "Group Names: {}".format(
                self._group_names) +
            '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def wait_for_state_update_arg(
            self,
            name,
            box_is_known=False,
            box_is_attached=False,
            timeout=4):

        box_name = name
        scene = self._scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

      # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
        return False

    def add_box_arg(self, name, x, y, z, R, P, Y, timeout=4):

        box_name = name
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.x = x
        box_pose.pose.orientation.y = y
        box_pose.pose.orientation.z = z
        box_pose.pose.position.x = R
        box_pose.pose.position.y = P
        box_pose.pose.position.z = Y
        box_name = name
        scene.add_box(box_name, box_pose, size=(0.186354, 0.153313, 0.168908))

        return self.wait_for_state_update_arg(
            box_name, box_is_known=True, timeout=timeout)

    def attach_box_arg(self, name, timeout=4):

        box_name = name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names

        grasping_group = 'ur5_1_planning_group'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update_arg(
            box_name,
            box_is_attached=True,
            box_is_known=False,
            timeout=timeout)

    def detach_box_arg(self, name, timeout=4):

        box_name = name
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        return self.wait_for_state_update_arg(
            box_name,
            box_is_known=True,
            box_is_attached=False,
            timeout=timeout)

    def logical_camera_callback(self, msg):

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

            if position[1] <= 0.0 and "packagen1" in package_name:
                self.conveyor_belt_power(0)
                self.add_box_arg(
                    "packagen1",
                    position[0],
                    position[1],
                    position[2],
                    orientation[0],
                    orientation[1],
                    orientation[2])
                self.attach_box_arg("packagen1")
                self.vacuum_gripper_state(True)
                # self.ee_cartesian_translation(0.0, 0, 0.5)
                self.set_joint_angles(self.lst_joint_angles_red_box)
                self.detach_box_arg("packagen1")
                self.vacuum_gripper_state(False)
                self.conveyor_belt_power(20)
                self.set_joint_angles(self.lst_joint_angles_home)

            elif position[1] <= 0.0 and "packagen2" in package_name:
                self.conveyor_belt_power(0)
                self.add_box_arg(
                    "packagen2",
                    position[0],
                    position[1],
                    position[2],
                    orientation[0],
                    orientation[1],
                    orientation[2])
                self.attach_box_arg("packagen2")
                self.ee_cartesian_translation(0.15, 0, 0)
                self.vacuum_gripper_state(True)
                # self.ee_cartesian_translation(0.0, 0, 0.5)
                self.set_joint_angles(self.lst_joint_angles_green_box)
                self.detach_box_arg("packagen2")
                self.vacuum_gripper_state(False)
                self.conveyor_belt_power(20)
                self.set_joint_angles(self.lst_joint_angles_home)

            elif position[1] <= 0.0 and "packagen3" in package_name:
                self.conveyor_belt_power(0)
                self.add_box_arg(
                    "packagen3",
                    position[0],
                    position[1],
                    position[2],
                    orientation[0],
                    orientation[1],
                    orientation[2])
                self.attach_box_arg("packagen3")
                self.ee_cartesian_translation(-0.15, 0, 0)
                self.vacuum_gripper_state(True)
                # self.ee_cartesian_translation(0.0, 0, 0.5)
                # self.set_joint_angles(self.lst_joint_angles_blue_box)
                self.set_joint_angles(self.lst_joint_angles_blue_box)
                self.detach_box_arg("packagen3")
                self.vacuum_gripper_state(False)

        else:
            self.conveyor_belt_power(45)

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        # pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        # list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        # if (flag_plan == True):
        #     rospy.loginfo(
        #         '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        # else:
        #     rospy.logerr(
        #         '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' +
            "Going to Pose: {}".format(arg_pose_name) +
            '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo(
            '\033[94m' +
            "Now at Pose: {}".format(arg_pose_name) +
            '\033[0m')

    def go_to_joint_state(self):

        move_group = self._group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -math.pi / 2  # dont change this, for 90 MAKES STRAIGHT
        joint_goal[2] = -math.pi / 4
        joint_goal[3] = -math.pi / 2
        joint_goal[4] = 0
        joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)

        move_group.stop()

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of
        # waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()



    lst_joint_angles_home = [math.radians(172.160516001),
                             math.radians(-40.0529267276),
                             math.radians(58.2606911498),
                             math.radians(-108.167308562),
                             math.radians(-89.9478601433),
                             math.radians(-7.82017669014)]

    lst_joint_angles_green_box = [math.radians(-6.55199936978),
                                  math.radians(-7.28702651406),
                                  math.radians(15.1952554342),
                                  math.radians(-7.89844926019),
                                  math.radians(-7.89844926019),
                                  math.radians(-0.00616903021083)]


    lst_joint_angles_red_box = [math.radians(78.3349824021),
                                math.radians(-52.7170327139),
                                math.radians(87.5682954039),
                                math.radians(-124.534399461),
                                math.radians(-91.7923354083),
                                math.radians(37.5926079827)]

    lst_joint_angles_blue_box = [math.radians(92.0125562124),
                                 math.radians(-126.532325991),
                                 math.radians(-56.0796566166),
                                 math.radians(-178.463797373),
                                 math.radians(89.7046063468),
                                 math.radians(-9.39572429173)]

    
    ur5.set_joint_angles(ur5.lst_joint_angles_home)
    ur5.conveyor_belt_power(35)

    rospy.Subscriber(
        "/eyrc/vb/logical_camera_2",
        LogicalCameraImage,
        ur5.logical_camera_callback,
        queue_size=10)
        
    rospy.spin()

    del ur5


if __name__ == '__main__':
    main()