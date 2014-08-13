#!/bin/bash

source ../setup.bash
roscd
roscore &
sleep 5
cd project1
rosmake
roscd
rosrun stage_ros stageros project1/world/myworld.world &
rosrun se306_example R0 &
rosrun se306_example R1 &
rosrun se306_example R2 &

read -p "Press any key to continue... " -n1 -s
killall R0
killall R1
killall R2
killall stageros
killall roscore

