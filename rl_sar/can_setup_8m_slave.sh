#!/bin/sh

# Determine repository root (directory containing this script)
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# Define the file name
FILE_NAME="${SCRIPT_DIR}/nohup.out"

# Check if the file exists
if [ -f "$FILE_NAME" ]; then
    echo "The file '$FILE_NAME' exists."
    rm "$FILE_NAME"
        if [ $? -eq 0 ]; then
            echo "The file '$FILE_NAME' has been successfully removed."
        fi
fi

sudo systemctl stop tita-bringup.service
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.80 dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
sudo ifconfig can0 txqueuelen 1000
cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash && source install/setup.bash
nohup ros2 launch titati_bringup quadruped_slave_controller.launch.py &
