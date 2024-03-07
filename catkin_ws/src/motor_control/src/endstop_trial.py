#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy

class EndStop:
    def __init__(self, endstop_1, endstop_2):
        self.endstop_1 = endstop_1
        self.endstop_2 = endstop_2
        
        GPIO.setmode(GPIO.BCM)  # Set GPIO pin numbering
        GPIO.setup(self.endstop_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set endstop pins as input with pull-up
        GPIO.setup(self.endstop_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def read_endstops(self):
        state_1 = GPIO.input(self.endstop_1)
        state_2 = GPIO.input(self.endstop_2)
        return state_1, state_2  # Return the states of the two endstops
        
    def clean_up(self):
        GPIO.cleanup()  # Clean up GPIO when done or restarting

if __name__ == '__main__':
    rospy.init_node('endstop_node', anonymous=True)
    end_stop = EndStop(endstop_1=14, endstop_2=15)
    try:
        while not rospy.is_shutdown():
            state_1, state_2 = end_stop.read_endstops()
            rospy.loginfo(f"Endstop 1: {state_1}, Endstop 2: {state_2}")  # Log the state of the endstops
            rospy.sleep(1)  # Sleep for a bit to avoid flooding the logs
    except rospy.ROSInterruptException:
        pass
    finally:
        end_stop.clean_up()
