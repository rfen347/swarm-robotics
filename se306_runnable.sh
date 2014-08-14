#!/bin/bash

source ../setup.bash
roscd

killall R0
killall R1
killall R2
killall stageros
killall roscore

roscore &
sleep 5
cd project1
rosmake
roscd

rosrun project1 R0 &
rosrun project1 R1 &
rosrun project1 R2 &
rosrun stage_ros stageros project1/world/myworld.world &

read -p "Press any key to exit... " -n1 -s
killall R0
killall R1
killall R2
killall stageros
killall roscore

