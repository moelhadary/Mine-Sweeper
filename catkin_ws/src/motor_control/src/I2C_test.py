#!/usr/bin/env python3

from smbus2 import SMBus, i2c_msg
import time

slave_address = 0x12
block_size = 21  # Adjusted to the number of bytes you expect to receive (7 integers * 3 bytes each)
x = 20  # Number of requests per second

interval = 1.0 / x  # Calculate interval time in seconds

def bytes_to_int(byte0, byte1, byte2):
    """Convert 3 bytes to a single integer."""
    return byte0 + (byte1 << 8) + (byte2 << 16)

with SMBus(1) as bus:
    start_time = time.time()
    while True:
        # Using i2c_msg to read
        read = i2c_msg.read(slave_address, block_size)
        bus.i2c_rdwr(read)

        # Convert the received data to a list of integers
        data_received = list(read)
        
        # Ensure the received data list is the expected length
        if len(data_received) != block_size:
            print("Received data is not the expected length:", len(data_received))
            continue  # Skip processing this data and try again

        # Convert bytes to integers
        integers = [bytes_to_int(data_received[i], data_received[i+1], data_received[i+2]) for i in range(0, len(data_received), 3)]
        
        print("Received integers:", integers)

        # Wait for the next interval
        time.sleep(max(0, interval - (time.time() - start_time)))
        start_time += interval
