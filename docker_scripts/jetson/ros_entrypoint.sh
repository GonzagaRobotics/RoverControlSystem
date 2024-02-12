#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /app/install/local_setup.bash

git clone https://github.com/IntelRealSense/librealsense.git \
    && cd librealsense \
    && ./scripts/patch-realsense-ubuntu-L4T.sh \
    && ./scripts/setup_udev_rules.sh \
    && mkdir build \
    && cd build \
    && cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true \
    && make -j$(($(nproc)-1)) \
    && sudo make install
    
exec "$@"
