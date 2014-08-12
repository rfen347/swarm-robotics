#!/bin/bash

source ~/indigo_workspace/setup.bash
roscd
roscore &
sleep 5
cd se306_example
rosmake
roscd
rosrun stage_ros stageros world/myworld.world &
rosrun se306_example R0 &
rosrun se306_example R1 &

