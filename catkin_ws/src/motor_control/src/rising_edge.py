#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Pin Setup
inputPin = 18  # GPIO pin you want to use (BCM numbering)
GPIO.setmode(GPIO.BCM)
GPIO.setup(inputPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Variables for frequency calculation
pulseCount = 0  # Counter for pulses
lastTime = time.time()  # Time of the last pulse count

# Callback function to increment pulse counter
def handleInterrupt(channel):
    global pulseCount
    pulseCount += 1

# Add event detection and callback function
GPIO.add_event_detect(inputPin, GPIO.RISING, callback=handleInterrupt, bouncetime=200)

try:
    while True:
        # Calculate frequency every second
        currentTime = time.time()
        if currentTime - lastTime >= 1.0:
            # Calculate frequency in Hertz
            frequency = pulseCount / 1.0
            print(f"Frequency: {frequency} Hz")
            
            # Reset the pulse count and update lastTime
            pulseCount = 0
            lastTime = currentTime


except KeyboardInterrupt:
    print("Program stopped")
finally:
    GPIO.cleanup()  # Clean up GPIO on CTRL+C exit
