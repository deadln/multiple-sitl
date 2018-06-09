#!/bin/bash

dir=`pwd`/`dirname $0`/..

label="default"
fw_dir=$dir/../../
sg_dir=$dir/../sitl_gazebo

if [ "$1" ]; then
  label=$1
fi

if [ "$2" ]; then
  fw_dir=$2
fi

if [ "$3" ]; then
  sg_dir=$3
fi

#px4

cd $fw_dir
make clean
make posix_sitl_$label

#sitl_gazebo

cd $sg_dir

git submodule update --init --recursive

rm -rf build
mkdir build
cd build
cmake ..
make -j4
