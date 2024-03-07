#!/usr/bin/env python3
import rospy
from motor_control.msg import PwmArray  # Replace with your actual package name and message

def talker():
    pub = rospy.Publisher('pwm_topic', PwmArray, queue_size=10)
    rospy.init_node('pwm_test_talker', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        pwm_msg = PwmArray()
        pwm_msg.pwm_values = [0, 20, 40, 60, 70, 80]  # Example values

        rospy.loginfo(pwm_msg)
        pub.publish(pwm_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
