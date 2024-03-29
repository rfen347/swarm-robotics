#!/bin/bash

source ../../setup.bash

CWD=$(pwd)

roscd

#Kill all processes to start fresh
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

rosrun stage_ros stageros project1/world/myworld.world &

#Return back to where the command was run
cd "$CWD"
sleep 5
#rosrun project1 R0 &
#rosrun project1 R1 &
#rosrun project1 R2 &

#Run each robot at a different terminal
gnome-terminal -e "./run_R0.sh"
gnome-terminal -e "./run_R1.sh"
gnome-terminal -e "./run_R2.sh"

read -p "Press any key to exit... " -n1 -s

#Kill all remaining processes
killall R0
killall R1
killall R2
killall stageros
killall roscore
killall gnome-terminal
