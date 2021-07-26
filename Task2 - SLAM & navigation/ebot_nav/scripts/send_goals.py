#!/usr/bin/env python

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self):

        # Initialize node
        rospy.init_node('send_goals')

        # Sequence of goal points in the format (x, y, z)
        points_seq = [-9.1, -1.1, 0, 10.9, 10.7, 0,
                      12.9, -1.4, 0, 18.2, -1.4, 0, -2, 4, 0]
        # Orientation sequence for corresponding goal points in degrees
        yaweulerangles_seq = [120, -70, 100, 150, 120]

        # List of goal quaternions
        quat_seq = list()
        # List of goal poses
        self.pose_seq = list()
        self.goal_cnt = 0

        for yawangle in yaweulerangles_seq:
            # Unpack the quaternion list and pass it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(
                *(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))

        n = 3
        # Returns a list of lists [[point 1], [point 2],...[point n]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

        for point in points:
            # Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1

        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1) +
                      " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose " +
                      str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1

        # Goal Preempted
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt) +
                          " received a cancel request after it started executing, completed execution!")

        # Goal Succeeded
        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")

            # Send next goal (if exists)
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose " +
                              str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(
                    next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        # Goal Aborted
        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt) +
                          " was aborted by the Action Server")
            rospy.signal_shutdown(
                "Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        # Goal Rejected
        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt) +
                          " has been rejected by the Action Server")
            rospy.signal_shutdown(
                "Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        # Goal Recalled
        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt) +
                          " received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " +
                      str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb,
                              self.active_cb, self.feedback_cb)
        rospy.spin()


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
