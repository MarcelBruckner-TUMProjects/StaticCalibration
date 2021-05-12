#!/bin/bash

working_directory=$PWD

sudo apt install \
  checkinstall \
  libssl-dev

### CMake
cmake_version="3.19.1"
cmake_folder="cmake-$cmake_version"
cmake_archive="$cmake_folder.tar.gz"
wget https://github.com/Kitware/CMake/releases/download/v$cmake_version/$cmake_archive
tar xfv $cmake_archive
cd $cmake_folder
./bootstrap 
make -j 18
sudo checkinstall