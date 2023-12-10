sudo docker compose -f ./docker_scripts/jetson/docker-compose.yml up --force-recreate --build -d
sudo docker exec -it jetson-ros_echo-1 bash
