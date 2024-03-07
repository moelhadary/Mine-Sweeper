#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Replace 'your_pin' with your GPIO pin number
pwm_pin = 23 
direction_pin = 2

GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(direction_pin, GPIO.OUT)

pwm = GPIO.PWM(pwm_pin, 100)  # Set frequency to 3kHz
pwm.start(0)  # Start PWM with 0% duty cycle
GPIO.output(direction_pin, GPIO.HIGH)

duty_cycle = 70 #set between 0-100

    
try:
    pwm.ChangeDutyCycle(duty_cycle)
    print(f"Setting duty cycle to: {duty_cycle}%")
    print("PWM running. Press Ctrl+C to stop.")
    
    while True:  # Infinite loop
        time.sleep(1)  # Wait for 1 second in the loop

except KeyboardInterrupt:
    print("Stopping PWM...")

finally:
    pwm.stop()  # Stop the PWM
    GPIO.cleanup()  # Cleanup the GPIO
    print("GPIO cleaned up.")
