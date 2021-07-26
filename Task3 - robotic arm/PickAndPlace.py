#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class PickAndPlace:

    # Constructor
    def __init__(self):

        # Initialize the node
        rospy.init_node('node_pick_and_place', anonymous=True)

        # Make the planning groups
        self._planning_group = "ur5_arm_planning_group"
        self._grasping_group = "ur5_gripper_planning_group"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        self._arm_group = moveit_commander.MoveGroupCommander(
            self._planning_group)
        self._gripper_group = moveit_commander.MoveGroupCommander(
            self._grasping_group)

        # Publisher
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        # Action Client
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._arm_group.get_planning_frame()
        self._eef_link = self._arm_group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo('\033[94m' + " >>> PickAndPlace init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        self._arm_group.set_pose_target(arg_pose)
        flag_plan = self._arm_group.go(wait=True)  # wait=False for Async Move

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class PickAndPlace Deleted." + '\033[0m')


def main():

    ur5 = PickAndPlace()

    # Define different poses for the ur5 arm
    ur5_pose_object_1 = geometry_msgs.msg.Pose()
    ur5_pose_object_1.position.x = 0.562050812328
    ur5_pose_object_1.position.y = -0.0176868581916
    ur5_pose_object_1.position.z = 0.896953181537
    ur5_pose_object_1.orientation.x = -0.667717031589
    ur5_pose_object_1.orientation.y = 0.262919510947
    ur5_pose_object_1.orientation.z = -0.266698055582
    ur5_pose_object_1.orientation.w = 0.643350171865

    ur5_pose_container_1 = geometry_msgs.msg.Pose()
    ur5_pose_container_1.position.x = -0.0483484918389
    ur5_pose_container_1.position.y = 0.700293242678
    ur5_pose_container_1.position.z = 1.08912160145
    ur5_pose_container_1.orientation.x = -0.521346110874
    ur5_pose_container_1.orientation.y = -0.477693541919
    ur5_pose_container_1.orientation.z = 0.47772511323
    ur5_pose_container_1.orientation.w = 0.521330824789

    ur5_pose_object_2 = geometry_msgs.msg.Pose()
    ur5_pose_object_2.position.x = 0.450170308722
    ur5_pose_object_2.position.y = 0.218593662625
    ur5_pose_object_2.position.z = 0.905193558423
    ur5_pose_object_2.orientation.x = -0.625650313823
    ur5_pose_object_2.orientation.y = 0.301822448398
    ur5_pose_object_2.orientation.z = -0.312547990759
    ur5_pose_object_2.orientation.w = 0.647903270503

    ur5_pose_container_2 = geometry_msgs.msg.Pose()
    ur5_pose_container_2.position.x = 0.0479443577702
    ur5_pose_container_2.position.y = -0.70477146008
    ur5_pose_container_2.position.z = 1.12306290171
    ur5_pose_container_2.orientation.x = -0.477717995215
    ur5_pose_container_2.orientation.y = 0.521293348242
    ur5_pose_container_2.orientation.z = -0.521396707475
    ur5_pose_container_2.orientation.w = 0.477686335957

    ur5_pose_object_3 = geometry_msgs.msg.Pose()
    ur5_pose_object_3.position.x = 0.536972306585
    ur5_pose_object_3.position.y = -0.256826149519
    ur5_pose_object_3.position.z = 0.979874740752
    ur5_pose_object_3.orientation.x = -0.0608754335372
    ur5_pose_object_3.orientation.y = -0.699073402086
    ur5_pose_object_3.orientation.z = 0.712453564068
    ur5_pose_object_3.orientation.w = 0.000692194564098

    while not rospy.is_shutdown():
        # Go for object 1
        ur5.go_to_pose(ur5_pose_object_1)
        rospy.sleep(0.3)

        # Go Down
        ur5_pose_object_1.position.z = 0.8200
        ur5.go_to_pose(ur5_pose_object_1)
        rospy.sleep(0.3)

        # Pick object 1
        ur5._gripper_group.set_joint_value_target([math.radians(15)])
        ur5._gripper_group.plan()
        flag_plan = ur5._gripper_group.go(wait=True)
        rospy.sleep(0.3)

        # Go up
        ur5_pose_object_1.position.z = 1.100
        ur5.go_to_pose(ur5_pose_object_1)
        rospy.sleep(0.3)

        # Go to container 1
        ur5.go_to_pose(ur5_pose_container_1)
        rospy.sleep(0.3)

        # Drop object 1
        ur5._gripper_group.set_joint_value_target([math.radians(0)])
        ur5._gripper_group.plan()
        flag_plan = ur5._gripper_group.go(wait=True)
        rospy.sleep(0.3)

        # Go for object 2
        ur5.go_to_pose(ur5_pose_object_2)
        rospy.sleep(0.3)

        # Go Down
        ur5_pose_object_2.position.z = 0.8200
        ur5.go_to_pose(ur5_pose_object_2)
        rospy.sleep(0.3)

        # Pick object 2
        ur5._gripper_group.set_joint_value_target([math.radians(13)])
        ur5._gripper_group.plan()
        flag_plan = ur5._gripper_group.go(wait=True)
        rospy.sleep(0.3)

        # Go up
        ur5_pose_object_2.position.z = 1.100
        ur5.go_to_pose(ur5_pose_object_2)
        rospy.sleep(0.3)

        # Go to container 2
        ur5.go_to_pose(ur5_pose_container_2)
        rospy.sleep(0.3)

        # Drop object 2
        ur5._gripper_group.set_joint_value_target([math.radians(0)])
        ur5._gripper_group.plan()
        flag_plan = ur5._gripper_group.go(wait=True)
        rospy.sleep(0.3)

        # Go for object 3
        ur5.go_to_pose(ur5_pose_object_3)
        rospy.sleep(0.3)

        # Go Down
        ur5_pose_object_3.position.z = 0.870
        ur5.go_to_pose(ur5_pose_object_3)
        rospy.sleep(0.3)

        # Pick object 3
        ur5._gripper_group.set_joint_value_target([math.radians(15)])
        ur5._gripper_group.plan()
        flag_plan = ur5._gripper_group.go(wait=True)
        rospy.sleep(0.3)

        # Go up
        ur5_pose_object_3.position.z = 1.100
        ur5.go_to_pose(ur5_pose_object_3)
        rospy.sleep(0.3)

        # Go to container 2
        ur5.go_to_pose(ur5_pose_container_2)
        rospy.sleep(0.3)

        # Drop object 3
        ur5._gripper_group.set_joint_value_target([math.radians(0)])
        ur5._gripper_group.plan()
        flag_plan = ur5._gripper_group.go(wait=True)
        rospy.sleep(0.3)

    del ur5


if __name__ == '__main__':
    main()
