docker compose -f ./docker_scripts/jetson/docker-compose.yml up --force-recreate
docker exec -it jetson-ros_echo-1 bash