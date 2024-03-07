#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time

class PWMServoConfig:
    def __init__(self, pin=13, frequency=400):
    
        self.pin = pin
        self.frequency = frequency
        
        rospy.init_node('pwm_control_node', anonymous=True)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.frequency)
        self.pwm.start(58)    # neutral position (1500microseconds pulse width)

    def flagger_servo_config(self):

        self.pwm.ChangeDutyCycle(80)
        time.sleep(0.5)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    pwm_node = PWMServoConfig()  # Instantiate with the default pin and frequency
    try:
        pwm_node.flagger_servo_config()
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm_node.cleanup()
