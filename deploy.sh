# Open an SSH connection to the rover and remove the ros directory
ssh robotics@192.168.0.2 'cd ~/Documents/RoverControlSystem/ && rm -rf ros'

# Copy the ros directory to the rover
scp -r ros robotics@192.168.0.2:~/Documents/RoverControlSystem/