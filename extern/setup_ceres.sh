#!/bin/bash

working_directory=$PWD

sudo apt install \
  checkinstall \
  libssl-dev

### Ceres
sudo apt-get install libgoogle-glog-dev libgflags-dev -y
sudo apt-get install libatlas-base-dev -y
sudo apt-get install libeigen3-dev -y
sudo apt-get install libsuitesparse-dev -y

cd $working_directory
ceres_version=ceres-solver-2.0.0
ceres_archive=$ceres_version.tar.gz
wget http://ceres-solver.org/$ceres_archive
tar xfv $ceres_archive
cd $ceres_version

cpu_cores="$(grep -c ^processor /proc/cpuinfo)"

mkdir $ceres_version && cd $ceres_version
cmake ..
cmake --build . -j $cpu_cores
sudo checkinstall -y
