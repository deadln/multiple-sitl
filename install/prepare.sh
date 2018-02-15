#!/bin/bash

dir=`pwd`/`dirname $0`/..

cd $dir
git clone --depth 1 --branch v1.7.3 https://github.com/PX4/Firmware.git
git clone --depth 1 --branch 1.7 https://github.com/acsl-mipt/sitl_gazebo.git

#px4

cd $dir/Firmware
make clean
make posix_sitl_default

#sitl_gazebo

cd $dir/sitl_gazebo

git submodule update --init --recursive

rm -rf build
mkdir build
cd build
cmake ..
make
