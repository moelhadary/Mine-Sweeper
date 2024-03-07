#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Replace these with your GPIO pin numbers
pwm_pins = [23, 24, 25, 12, 16, 26]  # Example pin numbers
dir_left_pin = 2
dir_right_pin = 3
frequency = 100  # Frequency in Hz

GPIO.setmode(GPIO.BCM)
GPIO.setup(dir_left_pin, GPIO.OUT)
GPIO.setup(dir_right_pin, GPIO.OUT)
pwms = []

# Setup PWM on each pin and start with 0% duty cycle
for pin in pwm_pins:
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, frequency)
    pwm.start(0)
    pwms.append(pwm)

duty_cycle = 70  # Set duty cycle between 0-100


try:
    # Change duty cycle for all PWMs
    GPIO.output(dir_left_pin, GPIO.HIGH)
    GPIO.output(dir_right_pin, GPIO.HIGH)
    for pwm in pwms:
        pwm.ChangeDutyCycle(duty_cycle)
        print(f"Setting duty cycle on pin {pwm_pins[pwms.index(pwm)]} to: {duty_cycle}%")

    print("PWM running on all pins. Press Ctrl+C to stop.")

    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping PWM on all pins...")

finally:
    # Stop all PWMs and cleanup GPIO
    for pwm in pwms:
        pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up.")

