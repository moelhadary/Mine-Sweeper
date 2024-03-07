#!/usr/bin/env python3
from gpiozero import Button
import rospy
from std_msgs.msg import Float32

class MotorSpeedSensor:
    def __init__(self):
        self.setup_fg_sensor()
        self.pulse_count = 0
        rospy.init_node('motor_speed_sensor', anonymous=True)
        self.last_time = rospy.get_time()
        self.rpm_pub = rospy.Publisher('motor_rpm', Float32, queue_size=10)

        self.t_sample = 0.5  # 500 ms
        self.scalar = 2
        self.gear_ratio = 260
        self.pulses_per_revolution = 6

    def setup_fg_sensor(self):
        max_attempts = 5
        for attempt in range(max_attempts):
            try:
                self.fg_sensor = Button(27)  # Using GPIO 27
                self.fg_sensor.when_pressed = self.count_pulse
                rospy.loginfo("FG sensor setup successful.")
                break
            except RuntimeError as e:
                rospy.logwarn("Attempt {} failed: {}".format(attempt + 1, e))
                if attempt == max_attempts - 1:
                    rospy.logerr("Failed to set up FG sensor after {} attempts.".format(max_attempts))

    def count_pulse(self):
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
        sensor.fg_sensor.close()
