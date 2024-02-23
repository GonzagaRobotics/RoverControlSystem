# 1st argument: host or jetson

# 2nd argument: force build
if [ "$2" = "-b" ]; then
    BUILD="--build"
else
    BUILD=""
fi

xhost +local:*
sudo docker compose -f ./docker_scripts/"$1"/docker-compose.yml up -d "$BUILD"
# If we force build, we should make sure that the ros volume is up to date
if [ "$BUILD" = "--build" ]; then
    sh ./update_ros.sh "$1"
fi
sudo docker exec -it "$1"-rover_main-1 bash