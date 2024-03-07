#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float32

class MotorSpeedSensor:
    def __init__(self, fg_pins):
        self.fg_pins = fg_pins  # List of FG pins
        self.pulse_counts = {pin: 0 for pin in self.fg_pins}
        self.last_states = {pin: GPIO.LOW for pin in self.fg_pins}

        GPIO.setmode(GPIO.BCM)
        for pin in self.fg_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        rospy.init_node('motor_speed_sensor', anonymous=True)
        self.last_time = rospy.get_time()
        self.rpm_pubs = {pin: rospy.Publisher(f'motor_{pin}_rpm', Float32, queue_size=10) for pin in self.fg_pins}

        self.t_sample = 0.1  # 100 ms
        self.scalar = 1
        self.gear_ratio = 260
        self.pulses_per_revolution = 6

    def spin(self):
        rate = rospy.Rate(10000)  # Increase rate for more frequent polling
        while not rospy.is_shutdown():
            for pin in self.fg_pins:
                current_state = GPIO.input(pin)
                if current_state != self.last_states[pin] and current_state == GPIO.HIGH:
                    self.pulse_counts[pin] += 1
                self.last_states[pin] = current_state

            self.calculate_rpm()
            rate.sleep()

    def calculate_rpm(self):
        current_time = rospy.get_time()
        time_period = cudrrent_time - self.last_time
        if time_period >= self.t_sample:
            for pin in self.fg_pins:
                rpm = (self.pulse_counts[pin] * self.scalar * 60.0 /
                       self.pulses_per_revolution / self.gear_ratio / time_period)
                self.rpm_pubs[pin].publish(rpm)
                rospy.loginfo(f"Pin {pin} RPM: {rpm}")
                self.pulse_counts[pin] = 0
            self.last_time = current_time

if __name__ == '__main__':
    fg_pins = [4, 17, 27, 22, 5, 6]  # Example pin numbers
    try:
        sensor = MotorSpeedSensor(fg_pins)
        sensor.spin()
    finally:
        GPIO.cleanup()
