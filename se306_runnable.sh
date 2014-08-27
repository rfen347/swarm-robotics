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
killall stageros
killall roscore

roscore &
sleep 3
cd project1
rosmake
roscd

rosrun stage_ros stageros project1/world/myworld.world &
<<<<<<< HEAD
sleep 3
rosrun project1 R0 &
rosrun project1 R1 &
rosrun project1 R2 &
rosrun project1 R3 &
=======
sleep 5
rosrun project1 R0 &
rosrun project1 R1 &
rosrun project1 R2 &
# R3 is not run here because it is run in a seperate terminal (schedule)
>>>>>>> cc7c62e49a5819e4295a696f20228c48d72657fc
rosrun project1 R4 &
rosrun project1 R5 &
rosrun project1 R6 &
rosrun project1 R7 &
rosrun project1 R8 &
rosrun project1 R9 &
rosrun project1 R10 &

cd project1
sleep 5
gnome-terminal -e "./runSchedule.sh"

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
killall stageros
killall roscore

