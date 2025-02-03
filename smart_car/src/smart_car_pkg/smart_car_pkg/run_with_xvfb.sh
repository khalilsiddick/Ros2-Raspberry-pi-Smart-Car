#!/bin/bash

# Stop any existing Xvfb process
echo "Stopping any existing Xvfb processes..."
ps aux | grep Xvfb | grep -v grep | awk '{print $2}' | xargs -r kill

# Remove the lock file
echo "Removing any existing Xvfb lock files..."
rm -f /tmp/.X99-lock

# Start Xvfb
echo "Starting Xvfb on display :99..."
Xvfb :99 -ac &

# Wait for Xvfb to start
sleep 2

# Set DISPLAY environment variable
export DISPLAY=:99

# Run the ROS 2 node
echo "Running ROS 2 node..."
ros2 run smart_car_pkg qr_control