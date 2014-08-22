#!/bin/bash

source ../setup.bash
roscd

killall R0
killall R1
killall R2
killall R8
killall R9
killall R10
killall stageros
killall roscore

roscore &
sleep 5
cd project1
rosmake
roscd

rosrun stage_ros stageros project1/world/myworld.world &
sleep 5
rosrun project1 R0 &
rosrun project1 R1 &
rosrun project1 R2 &
rosrun project1 R8 &
rosrun project1 R9 &
rosrun project1 R10 &

read -p "Press any key to exit... " -n1 -s
killall R0
killall R1
killall R2
killall R8
killall R9
killall R10
killall stageros
killall roscore

