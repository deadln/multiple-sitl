#!/bin/bash

source /usr/share/gazebo/setup.sh
source config.sh

# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~+/sitl_gazebo/build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~+/sitl_gazebo/models

# Disable online model lookup since this is quite experimental and unstable
#export GAZEBO_MODEL_DATABASE_URI=""

export GAZEBO_MASTER_IP
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
export GAZEBO_IP=$GAZEBO_MASTER_IP

$GAZEBO --verbose $1
