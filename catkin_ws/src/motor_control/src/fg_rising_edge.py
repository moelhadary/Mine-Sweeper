#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float32

class MotorSpeedSensor:
    def __init__(self):
        self.fgPin = 21  # Define fgPin here
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.fgPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.pulse_count = 0
        rospy.init_node('motor_speed_sensor', anonymous=True)
        self.last_time = rospy.get_time()
        self.rpm_pub = rospy.Publisher('motor_rpm', Float32, queue_size=10)

        self.setup_edge_detection()

        self.t_sample = 0.5  # 500 ms
        self.scalar = 2
        self.gear_ratio = 260
        self.pulses_per_revolution = 6
    
    
    def setup_edge_detection(self):
        attempt = 0
        while attempt<50:
            try:
                GPIO.add_event_detect(self.fgPin, GPIO.RISING, callback=self.count_pulse)
                rospy.loginfo("Edge detection setup successful")
                break   
            except RuntimeError as e:
                attempt +=1
                rospy.logwarn("Attempt {} failed: {}".format(attempt,e))
        
    def count_pulse(self, channel):
        self.pulse_count += 1

    def calculate_rpm(self):
        current_time = rospy.get_time()
        if current_time - self.last_time >= self.t_sample:
            rpm = (self.pulse_count * self.scalar * 60.0 /
                   self.pulses_per_revolution / self.gear_ratio)
            self.rpm_pub.publish(rpm)
            rospy.loginfo("RPM: {}".format(rpm))
            self.pulse_count = 0
            self.last_time = current_time

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.calculate_rpm()
            rate.sleep()

if __name__ == '__main__':
    sensor = MotorSpeedSensor()
    try:
        sensor.spin()
    finally:
        GPIO.cleanup()
