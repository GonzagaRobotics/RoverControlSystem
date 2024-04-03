# GPIO 388 is connected to the enable pin of the ESP32

LAUNCH_ARGS=""
JETSON=false

while getopts "rajw" flag; do
    case $flag in
    r) # Enable Realsense
    LAUNCH_ARGS+="use_realsense:=true "
    ;;
    a) # Enable AutoNav
    LAUNCH_ARGS+="use_auto_nav:=true "
    ;;
    j) # Enable Jetson-specific features
    JETSON=true
    ;;
    w) # Enable WebRTC
    LAUNCH_ARGS+="use_rtc:=true "
    ;;
    \?) # Handle invalid options
    echo "Invalid option: $OPTARG" >&2
    exit 1
    ;;
    esac
done

# If we're in host mode, we just want to launch the ROS2 nodes
if [ "$JETSON" = true ]; then
    # Pins seem to reset on reboot, so we may need to re-export them
    if [ ! -e /sys/class/gpio/gpio388 ]; then
        echo 388 > /sys/class/gpio/export
    fi

    # Set the pin to output mode and turn it on for 0.25s, enough to enable the ESP32
    echo out > /sys/class/gpio/gpio388/direction
    echo 1 > /sys/class/gpio/gpio388/value 
    sleep 0.33s 
    echo 0 > /sys/class/gpio/gpio388/value
fi


# Now we can launch the ROS2 nodes
ros2 launch /app/ros/launch/launch.py $LAUNCH_ARGS