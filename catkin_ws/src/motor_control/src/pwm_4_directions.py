#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller')
        
        self.pwm_pins = [23, 24, 25, 12, 16, 26]
        self.dir_left_pin = 2
        self.dir_right_pin = 3
        self.frequency = 100  # Frequency in Hz
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_left_pin, GPIO.OUT)
        GPIO.setup(self.dir_right_pin, GPIO.OUT)
        self.pwms = []



        for pin in self.pwm_pins:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, self.frequency)
            pwm.start(0)  # Start with 0% duty cycle
            self.pwms.append(pwm)
        # Command mappings
        self.commands = {
            "F": ([70, 70, 70, 70, 70, 70], GPIO.HIGH, GPIO.LOW),
            "B": ([70, 70, 70, 70, 70, 70], GPIO.LOW, GPIO.HIGH),
            "L": ([70, 70, 70, 70, 70, 70], GPIO.LOW, GPIO.LOW),
            "R": ([70, 70, 70, 70, 70, 70], GPIO.HIGH, GPIO.HIGH),
            "S": ([0, 0, 0, 0, 0, 0], None, None)  # Direction doesn't matter for stop
        }
        self.command_sub = rospy.Subscriber('motor_command', String, self.command_callback)
        
    def command_callback(self, msg):
        command = msg.data
        if command in self.commands:
            pwm_values, dir_right, dir_left = self.commands[command]

            # Set direction pins if necessary
            if dir_left is not None and dir_right is not None:
                GPIO.output(self.dir_left_pin, dir_left)
                GPIO.output(self.dir_right_pin, dir_right)

            # Update PWM values for all motors
            for pwm, value in zip(self.pwms, pwm_values):
                pwm.ChangeDutyCycle(value)
            
            rospy.logwarn(f"Direction implemented: {command}")
            
        else:
            rospy.logwarn(f"Unknown command received: {command}")
        

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MotorController()
        controller.spin()
    finally:
        for pwm in controller.pwms:
            pwm.stop()
        GPIO.cleanup()
