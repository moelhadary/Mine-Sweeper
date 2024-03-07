#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

class StepperMotorController:
    def __init__(self, dir_pin, step_pin):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.delay = 0.001
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        
        
        
    def motor_callback(self):
        GPIO.output(self.dir_pin, GPIO.HIGH)
        for _ in range(200):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(self.delay)
            
        time.sleep(3)
        
        GPIO.output(self.dir_pin, GPIO.LOW)    
        for _ in range(200):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(self.delay)
            
        time.sleep(3)

    def clean_up(self):
        GPIO.cleanup()
    

if __name__ == '__main__':
    rospy.init_node('stepper_motor_node', anonymous=True)
    motor_controller = StepperMotorController(dir_pin=8, step_pin=1)
    for i in range(5):
        motor_controller.motor_callback()
    #test_message = Int32()
    #test_message.data = 200
    #motor_controller.motor_callback(test_message)
    #rospy.Subscriber('/stepper_motor_control', Int32, motor_controller.motor_callback)
    try:
        pass
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor_controller.clean_up()



