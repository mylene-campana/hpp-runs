#!/bin/bash
# File to launch n times hpp and a script.
#(assuming no need of RViz terminal)

# Command: sh hpp-runs.sh

path="/local/mcampana/devel/hpp"
testPath="/local/mcampana/devel/hpp/src/hpp-runs/scripts"

for i in `seq 1 2`
    do
    echo "Loop begining: "$i
    # In first terminal : server
    xterm -e hppcorbaserver &
    #xterm -e hpp-wholebody-step-server &
    sleep 2s
    # In second terminal : python script
    xterm -e python puzzle_test.py $i /local/mcampana/devel/hpp/src/hpp-runs/scripts/
    #wait until Python script is finished
    echo "Loop done"
    killall xterm
    sleep 2s
done # endFor


# Notes - when writing in a file : > will erase the file, >> will concatenate, 2> will write errors.

# List of available Python scripts :
# robot_2d_test
# potential_test
# puzzle_test
# ur2_test
# ur5_test
# tutorial_2
# room_hand
