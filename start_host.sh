xhost +local:*
sudo docker compose -f ./docker_scripts/host/docker-compose.yml up -d
sudo docker exec -it host-rover_main-1 bash
