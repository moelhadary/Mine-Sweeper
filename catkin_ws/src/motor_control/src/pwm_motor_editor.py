#!/usr/bin/env python3
import rospy
from motor_control.msg import PwmArray  # Replace with your message
import RPi.GPIO as GPIO

class PWMSignalController:
    def __init__(self):
        # GPIO setup
        self.pwm_pins = [29, 33, 2, 4, 11, 16]
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        for pin in self.pwm_pins:
            GPIO.setup(pin, GPIO.OUT)

        # Initialize PWM objects
        self.pwm_objects = [GPIO.PWM(pin, 20000) for pin in self.pwm_pins]
        for pwm in self.pwm_objects:
            pwm.start(0)

        # Initialize ROS node and subscriber
        rospy.init_node('pwm_listener', anonymous=True)
        rospy.Subscriber('pwm_topic', PwmArray, self.callback)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.pwm_values)
        for i, value in enumerate(data.pwm_values):
            duty_cycle = value
            self.pwm_objects[i].ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        for pwm in self.pwm_objects:
            pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    controller = PWMSignalController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
