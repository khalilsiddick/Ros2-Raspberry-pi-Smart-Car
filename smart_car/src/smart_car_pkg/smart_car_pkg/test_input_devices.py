import os
import time

# List of potential device files
device_files = ['/dev/input/event0', '/dev/input/event1', '/dev/input/event2', '/dev/input/event3', '/dev/input/event4', '/dev/input/event5', '/dev/input/event6']

for device_file in device_files:
    try:
        fd = os.open(device_file, os.O_RDONLY | os.O_NONBLOCK)
        print(f"Testing {device_file}...")
        
        while True:
            data = os.read(fd, 1024)
            if data:
                print(f"Received data from {device_file}: {data.decode('utf-8').strip()}")
            time.sleep(0.1)
        
        os.close(fd)
    except Exception as e:
        print(f"Error reading from {device_file}: {e}")