#!/bin/bash

source ../setup.bash
roscd
printf "\n Schedule     \n\n"
printf "    ---Instruction set--- \n"
printf "Normal day	:	n \n"
printf "Sick day	:	ill	\n"
printf "Emergency day	: 	em \n\n"
rosrun project1 R3
exit