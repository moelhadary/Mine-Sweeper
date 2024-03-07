#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float32

class MotorSpeedSensor:
    def __init__(self):
        self.fgPin = 27  # Define fgPin here
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.fgPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.last_state = GPIO.input(self.fgPin)
        self.pulse_count = 0
        rospy.init_node('motor_speed_sensor', anonymous=True)
        self.last_time = rospy.get_time()
        self.rpm_pub = rospy.Publisher('motor_rpm', Float32, queue_size=10)

        self.t_sample = 0.1  # 100 ms
        self.scalar = 1
        self.gear_ratio = 260
        self.pulses_per_revolution = 6

    def spin(self):
        rate = rospy.Rate(10000)  # Increase rate for more frequent polling
        while not rospy.is_shutdown():
            current_state = GPIO.input(self.fgPin)
            if current_state != self.last_state and current_state == 1:
                self.pulse_count += 1
            self.last_state = current_state

            self.calculate_rpm()
            rate.sleep()

    def calculate_rpm(self):
        current_time = rospy.get_time()
        time_period = current_time - self.last_time
        if time_period >= self.t_sample:
            rpm = (self.pulse_count * self.scalar * 60.0 /
                   self.pulses_per_revolution / self.gear_ratio / time_period)
            self.rpm_pub.publish(rpm)
            rospy.loginfo("RPM: {}".format(rpm))
            self.pulse_count = 0
            self.last_time = current_time

if __name__ == '__main__':
    try:
        sensor = MotorSpeedSensor()
        sensor.spin()
    finally:
        GPIO.cleanup()
