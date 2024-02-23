# Copy the ros directory to the rover excluding install, log, and build directories and delete the existing ros directory on the rover
rsync -av --exclude='ros/install' --exclude='ros/log' --exclude='ros/build' --delete ros robotics@192.168.0.2:~/Documents/RoverControlSystem/
