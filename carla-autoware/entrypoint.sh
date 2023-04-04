#!/bin/bash

export CARLA_AUTOWARE_CONTENTS=~/autoware-contents
export PYTHON2_EGG=$(ls /home/autoware/PythonAPI | grep py2.)
export PYTHONPATH=$PYTHONPATH:~/PythonAPI/$PYTHON2_EGG

source ~/carla_ws/devel/setup.bash
source ~/Autoware/install/setup.bash

echo arg1 town: $1, arg2 spawn_point: $2
roslaunch carla_autoware_agent carla_autoware_agent.launch town:=$1 spawn_point:=$2
tail -f /dev/null
