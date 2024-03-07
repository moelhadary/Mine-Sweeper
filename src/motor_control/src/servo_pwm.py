#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time

class PWMServoControl:
    def __init__(self, pin=13, frequency=400):
    
        self.pin = pin
        self.frequency = frequency
        
        rospy.init_node('pwm_control_node', anonymous=True)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.frequency)
        self.pwm.start(58)    # neutral position (1500microseconds pulse width)

    def flagger_servo_motion(self):
        self.pwm.ChangeDutyCycle(36)  # 0 degree (500microsecond pulse width)
        time.sleep(1.5)
        self.pwm.ChangeDutyCycle(58)
        time.sleep(2)
        self.pwm.ChangeDutyCycle(80)
        time.sleep(1.5)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    pwm_node = PWMServoControl()  # Instantiate with the default pin and frequency
    try:
        pwm_node.flagger_servo_motion()
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm_node.cleanup()
