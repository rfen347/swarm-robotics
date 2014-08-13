#!/bin/bash

source ../setup.bash
roscd
roscore &
sleep 5
cd project1
rosmake
roscd
rosrun stage_ros stageros project1/world/myworld.world &
rosrun project1 R0 &
rosrun project1 R1 &
rosrun project1 R2 &

read -p "Press any key to continue... " -n1 -s
killall R0
killall R1
killall R2
killall stageros
killall roscore

