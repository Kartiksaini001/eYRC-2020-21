#!/usr/bin/env python

# Necessary imports
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Global variables declaration

# pose of the bot i.e. x-coordinate, y-coordinate and theta/yaw
pose = [0.00, 0.00, 0.00]
# regions for the laser scan
regions = {
    'bright':  	20,
    'fright': 	20,
    'front':  	20,
    'fleft':  	20,
    'bleft':   	20,
}

# Publisher
pub = None

# Defining states and active flags for different functions and conditions
state_wall_follower = 0
state_dict_wall_follower = {
    0: "go to goal",
    1: "turn left",
    2: "follow the wall",
    3: "turn right"
}
state_bug0 = 0
state_dict_bug0 = {
    0: "go to final goal",
    1: "wall follower",
    2: "task finished"
}
active_bug0 = False
active_wall_follower = False
active_goal = False

# Error limits for angle and distance
yaw_precision = math.pi/45
dist_precision = 0.15

# Final goal coordinates
final_goal_x = 12.5
final_goal_y = 0


# Function to generate waypoints for the given function
def Waypoints(waypoint_number):
    mult_factor = 1
    # generating x and y coordinates
    x = waypoint_number*mult_factor/math.pi
    y = 2*(math.sin(x))*(math.sin(x/2))
    return [x, y]


# Function to convert an angle in the range -pi to pi
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))

    return angle


# Function to change the state of the wall follower function
def change_state_wall_follower(state):
    global state_wall_follower, state_dict_wall_follower

    if state is not state_wall_follower:
        rospy.loginfo(
            "Wall follower state - [{}] - {}".format(state, state_dict_wall_follower[state]))

        state_wall_follower = state
    if state == 0:
        change_state_bug0(0)


# Function to change the state of the bug0 function
def change_state_bug0(state):
    global state_bug0, state_dict_bug0, active_wall_follower, active_bug0, final_goal_x, final_goal_y

    if state is not state_bug0:
        state_bug0 = state
        rospy.loginfo(
            "Bug0 state - [{}] - {}".format(state, state_dict_bug0[state]))

    if state_bug0 == 0:
        active_wall_follower = False
        go_ahead(final_goal_x, final_goal_y)
    elif state_bug0 == 1:
        active_wall_follower = True
        wall_follower()
    elif state_bug0 == 2:
        active_bug0 = False


# Function to take the wall following action
def take_action():
    global regions
    state_description = ""

    # Wall detection distance
    d = 1.5

    if regions['front'] > d and regions['fright'] > d and regions['bright'] < d:
        state_description = "case 0 - bright"
        change_state_wall_follower(3)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = "case 1 - nothing"
        change_state_wall_follower(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = "case 2 - front"
        change_state_wall_follower(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = "case 3 - fright"
        change_state_wall_follower(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = "case 4 - fleft"
        change_state_wall_follower(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = "case 5 - front and fright"
        change_state_wall_follower(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = "case 6 - front and fleft"
        change_state_wall_follower(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = "case 7 - front and fleft and fright"
        change_state_wall_follower(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = "case 8 - fleft and fright"
        change_state_wall_follower(0)
    else:
        state_description = "unknown case"

    rospy.loginfo("state description - {}".format(state_description))


# Function for wall follower state 1
def turn_left():
    velocity_msg = Twist()
    velocity_msg.angular.z = 1.5
    return velocity_msg


# Function for wall follower state 2
def follow_the_wall():
    velocity_msg = Twist()
    velocity_msg.linear.x = 1
    return velocity_msg


# Function for wall follower state 3
def turn_right():
    velocity_msg = Twist()
    velocity_msg.linear.x = 0.7
    velocity_msg.angular.z = -1.5
    return velocity_msg


# Function to perform the wall following action
def wall_follower():
    global pub, state_wall_follower
    rate = rospy.Rate(10)
    velocity_msg = Twist()

    # Control statements for the wall follower
    if state_wall_follower == 0:
        pass
    elif state_wall_follower == 1:
        velocity_msg = turn_left()
    elif state_wall_follower == 2:
        velocity_msg = follow_the_wall()
        pass
    elif state_wall_follower == 3:
        velocity_msg = turn_right()
    else:
        rospy.logerr("Unknown state!")

    pub.publish(velocity_msg)
    rate.sleep()


# Function to stop the bot if a particular waypoint/goal is reached
def done():
    global pub, active_bug0
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    if active_bug0:
        change_state_bug0(2)


# Controller Function to move the bot to a particular waypoint
def go_to_goal(x_goal, y_goal):
    global active_goal, active_wall_follower
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if active_wall_follower:
            break
        else:
            go_ahead(x_goal, y_goal)

            if not active_goal:
                break

        rate.sleep()


# Function to assign speed of the bot to move it to a particular location
def go_ahead(x_goal, y_goal):
    global pub, pose, dist_precision, active_bug0, active_wall_follower, active_goal

    # required theta/yaw for the goal
    theta_goal = math.atan2(y_goal-pose[1], x_goal-pose[0])
    # error in the theta/yaw normalised to the range of -pi to pi
    e_theta = normalize_angle(pose[2] - theta_goal)
    # distance of bot from the goal
    e_pos = math.sqrt((x_goal-pose[0])**2 + (y_goal-pose[1])**2)
    # proportional controller for theta/yaw
    p_z = 10

    if e_pos > dist_precision:
        velocity_msg = Twist()
        velocity_msg.angular.z = -p_z*e_theta
        # assigning different speed for bug0 and to trace the given function
        if active_bug0:
            velocity_msg.linear.x = 1
        else:
            velocity_msg.linear.x = 0.35
        pub.publish(velocity_msg)
    else:
        rospy.loginfo("Position error: [{}]".format(e_pos))
        done()
        active_goal = False


# Function for the bug0 algorithm
def bug0():
    global state_bug0, regions, final_goal_x, final_goal_y, pose, active_wall_follower
    active_wall_follower = True
    change_state_bug0(0)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # distance to detect the wall
        d = 1.5
        if state_bug0 == 0:
            # Calling wall follower if a wall is detected upto distance d
            if regions['front'] > 0.5 and regions['front'] < d:
                change_state_bug0(1)
            else:
                change_state_bug0(0)

        elif state_bug0 == 1:
            theta_goal = math.atan2(final_goal_y - pose[1],
                                    final_goal_x - pose[0])
            e_theta = normalize_angle(theta_goal - pose[2])

            # Stoping wall follower if the goal is within 45 degrees of the bot angle and the path is clear
            if math.fabs(e_theta) < (math.pi / 4) \
                    and regions['front'] > d:
                change_state_bug0(0)

            elif e_theta > 0 \
                    and math.fabs(e_theta) > (math.pi / 4) \
                    and math.fabs(e_theta) < (math.pi / 2) \
                    and regions['bleft'] > d \
                    and regions['fleft'] > d:
                change_state_bug0(0)

            elif e_theta < 0 \
                    and math.fabs(e_theta) > (math.pi / 4) \
                    and math.fabs(e_theta) < (math.pi / 2) \
                    and regions['bright'] > d \
                    and regions['fright'] > d:
                change_state_bug0(0)

            else:
                change_state_bug0(1)

        # Stopping the bug0 algorithm when the final goal is reached
        elif state_bug0 == 2:
            break

        rate.sleep()


# Main controller for the task
def control_loop():
    global pub, active_bug0, active_goal, state_bug0
    # Initializing the node, and publisher and subscribers for the node
    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    # waypoint number counter
    waypoint_number = 0

    while not rospy.is_shutdown():
        # if final goal is reached (means state_bug0 is 2), do nothing
        if state_bug0 == 2:
            pass
        # if a goal is active, let other functions do their job
        elif active_goal:
            continue
        else:
            # if bug0 algorithm is not active (the starting condition), trace the function
            if not active_bug0:
                waypoint_number += 1

                if waypoint_number <= 20:
                    goal = Waypoints(waypoint_number)
                    rospy.loginfo(goal)
                    active_goal = True
                    go_to_goal(goal[0], goal[1])
                else:
                    active_bug0 = True
            # when the given function is traced, start the bug0 algorithm
            else:
                active_goal = True
                bug0()

        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


# Callback function for Odometry topic
def odom_callback(data):
    global pose
    # Update pose of the bot
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y,
            euler_from_quaternion([x, y, z, w])[2]]


# Callback function for LaserScan topic
def laser_callback(msg):
    global regions, active_wall_follower
    # Update the regions of the bot
    regions = {
        'bright':  	min(min(msg.ranges[0:143]), msg.range_max),
        'fright': 	min(min(msg.ranges[144:287]), msg.range_max),
        'front':  	min(min(msg.ranges[288:431]), msg.range_max),
        'fleft':  	min(min(msg.ranges[432:575]), msg.range_max),
        'bleft':   	min(min(msg.ranges[576:719]), msg.range_max),
    }

    # If wall follower is active, call take action function for the wall follower
    if active_wall_follower:
        take_action()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
