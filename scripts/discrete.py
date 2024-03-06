#!/usr/bin/env python3

import rospy
import threading
from std_srvs.srv import SetBool, SetBoolRequest
from bug0.srv import UpdateGoal, UpdateGoalRequest
from bug0.msg import NavError
from geometry_msgs.msg import Point32, Twist
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin, fabs

# CONSTANTS
NODE_NAME = "lawnmower_navigation"
# Topics
SCAN_TOPIC = "scan"
ERR_TOPIC = "nav_error"
CMD_VEL_TOPIC = "cmd_vel"  # Command velocity topic name
# Services
FOLLOW_WALL_SRV = "fw/activate"
GO_TO_GOAL_SRV = "gtg/activate"
UPDATE_GOAL_SRV = "update_goal"
# Lawnmower pattern parameters
MOWING_STRIPE_LENGTH = 5.0  # meters
MOWING_STRIPE_WIDTH = 1.0   # meters
intermediate_goal_count = 0
# Navigation thresholds
OBSTACLE_THRESHOLD = 0.5    # meters
GOAL_THRESHOLD = 0.20        # meters
SIDE_CLEARANCE = 0.20       # meters
PAUSE_DURATION = 2.0        # seconds to pause at each intermediate goal

# GLOBALS
current_goal = Point32()
current_state = "MOVE_FORWARD"
stripe_count = 0
following_wall: bool = False
STATE_CHANGE_DELAY = 2.0
last_state_change_time = rospy.Time()
is_stopped_and_waiting = False
state_lock = threading.Lock()
# Services
follow_wall: rospy.ServiceProxy = None
go_to_goal: rospy.ServiceProxy = None
update_goal: rospy.ServiceProxy = None
# Subscribers
scan_sub: rospy.Subscriber = None
err_sub: rospy.Subscriber = None
# Publishers
cmd_vel_pub: rospy.Publisher = None

def map0to2pi(angle: float) -> float:
    """Maps given angles in radians to corresponding angle in range from 0 to 2pi"""
    pi_2 = pi * 2
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle += pi_2
    return angle

def look_for_obstacles(angle: float, scan: LaserScan, fov: float=pi) -> bool:
    """Returns False if any obstacles are found in the given direction"""
    fov /= 2
    end_ind = int(map0to2pi(angle + fov - scan.angle_min) / scan.angle_increment)
    start_ind = int(map0to2pi(angle - fov - scan.angle_min) / scan.angle_increment)
    angle = map0to2pi(-fov)

    if start_ind > end_ind:
        for r in scan.ranges[start_ind:]:
            x = r * cos(angle)
            y = fabs(r * sin(angle))
            if x < OBSTACLE_THRESHOLD and y < SIDE_CLEARANCE:
                return False
            angle += scan.angle_increment

        for r in scan.ranges[:end_ind]:
            x = r * cos(angle)
            y = fabs(r * sin(angle))
            if x < OBSTACLE_THRESHOLD and y < SIDE_CLEARANCE:
                return False
            angle += scan.angle_increment

    for r in scan.ranges[start_ind:end_ind]:
        x = r * cos(angle)
        y = fabs(r * sin(angle))
        if x < OBSTACLE_THRESHOLD and y < SIDE_CLEARANCE:
            return False
        angle += scan.angle_increment

    return True


def stop_and_wait(duration=PAUSE_DURATION):
    global is_stopped_and_waiting
    with state_lock:
    # Indicate that the robot is in the stop-and-wait state
        is_stopped_and_waiting = True
        go_to_goal.call(SetBoolRequest(data=False))
        zero_twist = Twist()
        cmd_vel_pub.publish(zero_twist)
        rospy.loginfo("Stopping at the intermediate goal. Waiting for {} seconds...".format(duration))
        rospy.sleep(duration)
        # Resume navigation
        go_to_goal.call(SetBoolRequest(data=True))
        # Clear the stop-and-wait state
        is_stopped_and_waiting = False


def clbk_err(msg: NavError) -> None:
    global current_goal, current_state, is_stopped_and_waiting
    if fabs(msg.dist_error) < GOAL_THRESHOLD and not is_stopped_and_waiting:
        # Call stop_and_wait here if needed, or ensure it's completed before updating the goal
        stop_and_wait()  # This will now properly manage the stop-and-wait state
        update_stripe_goal()

        

def update_stripe_goal():
    global stripe_count, current_goal, current_state, intermediate_goal_count

    if stripe_count % 2 == 0:
        # Increment the stripe count and set the next goal along the stripe width
        stripe_count += 1
        current_goal.x += MOWING_STRIPE_WIDTH
        intermediate_goal_count = 0  # Reset intermediate goal count
    else:
        # Move along the stripe length in 1-meter increments
        if intermediate_goal_count < MOWING_STRIPE_LENGTH :
            
            intermediate_goal_count += 1
            current_goal.y += 1 if stripe_count % 4 == 1 else -1
        else:
            # Once the end of the stripe is reached, increment the stripe count
            stripe_count += 1
            intermediate_goal_count = 0

    update_goal.call(UpdateGoalRequest(goal=current_goal))
    current_state = "MOVE_FORWARD"
    rospy.loginfo(f"Goal updated: x={current_goal.x}, y={current_goal.y}, state={current_state}")

def clbk_scan(scan: LaserScan) -> None:
    global current_state, following_wall, last_state_change_time
    clear = look_for_obstacles(0, scan)  # Check for obstacles in front

    # Check if enough time has passed since the last state change
    if (rospy.Time.now() - last_state_change_time).to_sec() > STATE_CHANGE_DELAY:
        if clear and following_wall:
            follow_wall.call(SetBoolRequest(data=False))
            go_to_goal.call(SetBoolRequest(data=True))
            following_wall = False
            last_state_change_time = rospy.Time.now()
        elif not clear and not following_wall:
            go_to_goal.call(SetBoolRequest(data=False))
            follow_wall.call(SetBoolRequest(data=True))
            following_wall = True
            last_state_change_time = rospy.Time.now()

def setup() -> None:
    global follow_wall, go_to_goal, update_goal, scan_sub, err_sub, cmd_vel_pub, last_state_change_time

    rospy.init_node(NODE_NAME)

    # Initialize last_state_change_time
    last_state_change_time = rospy.Time.now()

    # Setting up service clients
    rospy.wait_for_service(FOLLOW_WALL_SRV)
    follow_wall = rospy.ServiceProxy(FOLLOW_WALL_SRV, SetBool)
    rospy.wait_for_service(GO_TO_GOAL_SRV)
    go_to_goal = rospy.ServiceProxy(GO_TO_GOAL_SRV, SetBool)
    rospy.wait_for_service(UPDATE_GOAL_SRV)
    update_goal = rospy.ServiceProxy(UPDATE_GOAL_SRV, UpdateGoal)

    cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

    # Setting up goal location
    current_goal.x = float(rospy.get_param("~x_goal", "0.0"))
    current_goal.y = float(rospy.get_param("~y_goal", "0.0"))



    # Check if the initial goal is the same as the starting position
    if current_goal.x == 0.0 and current_goal.y == 0.0:
        rospy.loginfo("Initial goal is at the starting position. Moving to the next goal.")
        update_stripe_goal()  # This will update the goal to the next position in the pattern

    # Setting up subscribers
    err_sub = rospy.Subscriber(ERR_TOPIC, NavError, clbk_err, queue_size=1)
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, clbk_scan, queue_size=1)


def main():
    setup()
    rospy.loginfo(f"Starting lawnmower pattern navigation to x={current_goal.x}, y={current_goal.y}")
    update_goal.call(UpdateGoalRequest(goal=current_goal))
    go_to_goal.call(SetBoolRequest(data=True))
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


this code hits the obstacle sometimes and skips width movement