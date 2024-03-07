#!/usr/bin/env python3
import rospy
from motor_control.msg import PwmArray  # Replace with your actual package name and message

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.pwm_values)

def listener():
    rospy.init_node('pwm_test_listener', anonymous=True)
    rospy.Subscriber('pwm_topic', PwmArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
