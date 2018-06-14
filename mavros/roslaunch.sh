#!/bin/bash

source $1/devel/setup.bash

launch_f=$2

shift 2

roslaunch $launch_f $@
