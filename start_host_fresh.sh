sudo docker compose -f ./docker_scripts/host/docker-compose.yml up --force-recreate --build -d
sudo docker exec -it host-ros_echo-1 bash
