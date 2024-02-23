# 1st argument: host or jetson

sudo docker run --rm -v "$1"_ros_vol:/data/ ubuntu /bin/sh -c "rm -rf /data/*"
sudo docker cp ros "$1"-rover_main-1:/app/