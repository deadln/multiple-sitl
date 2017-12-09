#!/bin/bash

source $1/devel/setup.bash
shift

roslaunch px4_num.launch $@
