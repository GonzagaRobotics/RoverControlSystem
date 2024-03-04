# GPIO 388 is connected to the enable pin of the ESP32

# If we're in host mode, we just want to launch the ROS2 nodes
if [ "$1" =  "jetson" ]; then
    # Pins seem to reset on reboot, so we may need to re-export them
    if [ ! -e /sys/class/gpio/gpio388 ]; then
        echo 388 > /sys/class/gpio/export
    fi

    # Set the pin to output mode and turn it on for 0.25s, enough to enable the ESP32
    echo out > /sys/class/gpio/gpio388/direction
    echo 1 > /sys/class/gpio/gpio388/value 
    sleep 0.25s 
    echo 0 > /sys/class/gpio/gpio388/value
fi

# Now we can launch the ROS2 nodes

# Allow us to launch without the realsense
if [ "$2" = "no_realsense" ]; then
    ros2 launch /app/ros/launch/launch.py disable_realsense:=true
else
    ros2 launch /app/ros/launch/launch.py
fi