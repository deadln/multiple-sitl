#!/bin/bash

dir=`pwd`/`dirname $0`/../

cd $dir
./git-subrepo.sh

#px4
label="default"
if [ "$1" ]; then
  label=$1
fi

cd $dir/Firmware
make clean
make posix_sitl_$label

#sitl_gazebo

cd $dir/sitl_gazebo

git submodule update --init --recursive

rm -rf build
mkdir build
cd build
cmake ..
make
