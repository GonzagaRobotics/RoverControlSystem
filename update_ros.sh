# 1st argument: host or jetson
# 2nd argument: -l to just update the launch directory

if [ "$2" = "-l" ]; then
    sudo docker run --rm -v $1_ros_vol:/data/ ubuntu /bin/sh -c "rm -rf /data/launch/"
    sudo docker cp ros/launch/ $1-rover_main-1:/app/ros/
    exit 0
fi


sudo docker run --rm -v $1_ros_vol:/data/ ubuntu /bin/sh -c "rm -rf /data/*"
sudo docker cp ros $1-rover_main-1:/app/