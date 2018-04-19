#!/bin/bash

dir=`pwd`/`dirname $0`/../

cd $dir
./git-subrepo.sh

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
