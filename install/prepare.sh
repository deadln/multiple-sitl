#!/bin/bash

dir=`pwd`/`dirname $0`/..

fw_dir=$dir/../../
sg_dir=$dir/../sitl_gazebo

if [ "$1" ]; then
  fw_dir=$1
fi

if [ "$2" ]; then
  sg_dir=$2
fi

#px4

cd $fw_dir
make clean
make posix_sitl_default

#sitl_gazebo

cd $sg_dir

git submodule update --init --recursive

rm -rf build
mkdir build
cd build
cmake ..
make -j4
