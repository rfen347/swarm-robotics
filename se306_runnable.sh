#!/bin/bash

source ../setup.bash
roscd

killall R0
killall R1
killall R2
killall R3
killall R4
killall R5
killall R6
killall R7
killall R8
killall R9
killall R10
killall R11
killall R12
killall stageros
killall roscore

# run roscore in another terminal, it takes so long to run roscore every single time
roscore &
cd project1
rosmake
roscd

rosrun stage_ros stageros project1/world/myworld.world &

sleep 1
rosrun project1 R0 &
rosrun project1 R1 &
rosrun project1 R2 &
rosrun project1 R4 &
rosrun project1 R5 &
rosrun project1 R6 &
rosrun project1 R7 &
rosrun project1 R8 &
rosrun project1 R9 &
rosrun project1 R10 &
rosrun project1 R11 &
rosrun project1 R12 &

cd project1

gnome-terminal -e "./runSchedule.sh"
#rosrun rqt_graph rqt_graph

read -p "Press any key to exit... " -n1 -s
killall R0
killall R1
killall R2
killall R3
killall R4
killall R5
killall R6
killall R7
killall R8
killall R9
killall R10
killall R11
killall R12
killall stageros
killall roscore

