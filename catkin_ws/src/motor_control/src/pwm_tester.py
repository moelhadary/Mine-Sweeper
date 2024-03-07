#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Replace 'your_pin' with your GPIO pin number
pwm_pin = 12 
direction_pin = 2

GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(direction_pin, GPIO.OUT)

pwm = GPIO.PWM(pwm_pin, 3000)  # Set frequency to 3kHz
pwm.start(0)  # Start PWM with 0% duty cycle
GPIO.output(direction_pin, GPIO.HIGH)

duty_cycle_values = [0, 30, 40, 50, 60, 80, 90, 100]

try:
    for duty_cycle in duty_cycle_values:
        print(f"Setting duty cycle to: {duty_cycle}%")
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(5)
    
    pwm.ChangeDutyCycle(0)
    GPIO.output(direction_pin, GPIO.LOW)
    print("NOW IN OTHER DIRECTION")
    
    for duty_cycle in duty_cycle_values:
        print(f"Setting duty cycle to: {duty_cycle}%")
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(5)    
        
            

finally:
    pwm.stop()  # Stop the PWM
    GPIO.cleanup()  # Cleanup the GPIO


