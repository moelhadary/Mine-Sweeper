#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

fgPin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(fgPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

class FGReaderNode:
    def __init__(self):
        rospy.init_node('fg_reader', anonymous=True)
        self.publisher = rospy.Publisher('fg_state', Bool, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def read_and_publish(self):
        while not rospy.is_shutdown():
            fg_state = GPIO.input(fgPin)
            rospy.loginfo("FG Pin State: {}".format(fg_state))
            self.publisher.publish(fg_state)
            self.rate.sleep()

def main():
    node = FGReaderNode()
    try:
        node.read_and_publish()
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
