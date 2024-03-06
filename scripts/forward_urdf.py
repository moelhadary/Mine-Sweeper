#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def move_forward():
    rospy.init_node('robot_mover', anonymous=True)
    
    # Publishers for each wheel
    pubs = [
        rospy.Publisher('/wheel1_velocity_controller/command', Float64, queue_size=1),
        rospy.Publisher('/wheel2_velocity_controller/command', Float64, queue_size=1),
        rospy.Publisher('/wheel3_velocity_controller/command', Float64, queue_size=1),
        rospy.Publisher('/wheel4_velocity_controller/command', Float64, queue_size=1),
        rospy.Publisher('/wheel5_velocity_controller/command', Float64, queue_size=1),
        rospy.Publisher('/wheel6_velocity_controller/command', Float64, queue_size=1),
    ]
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Set a constant speed for all wheels to move the robot forward
        for pub in pubs:
            pub.publish(Float64(-0.75))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
