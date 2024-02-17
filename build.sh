(cd ros && \
rosdep install -i --from-path src --rosdistro humble -y && \
colcon build --symlink-install)