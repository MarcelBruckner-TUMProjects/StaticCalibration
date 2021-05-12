#!/bin/bash

sudo apt install \
  checkinstall \
  libssl-dev

### OpenCV

sudo apt update && sudo apt install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/master.zip
unzip opencv.zip
unzip opencv_contrib.zip
mkdir -p opencv && cd opencv
cmake \
  -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-master/modules \
  -DCMAKE_BUILD_TYPE=Release \
  ../opencv-master
cpu_cores="$(grep -c ^processor /proc/cpuinfo)"
cmake --build . -j $cpu_cores
#sudo checkinstall
