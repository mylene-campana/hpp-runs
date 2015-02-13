#!/bin/bash
# File to launch n times hpp and a script.
#(assuming no need of RViz terminal)

# Command: sh hpp-runs.sh

path="/local/mcampana/devel/hpp"
testPath="/local/mcampana/devel/hpp/src/hpp-runs/scripts"

for i in `seq 1 10`
    do
    echo "Loop begining: "$i
    # In first terminal : server
    xterm -e hppcorbaserver &
    sleep 2s
    # In second terminal : python script
    xterm -hold -e python potential_test.py $i /local/mcampana/devel/hpp/src/hpp-runs/scripts/ &
    sleep 25s # finish python script
    killall xterm
    sleep 2s
done # endFor


# Notes - when writing in a file : > will erase the file, >> will concatenate, 2> will write errors.

# List of available Python scripts :
# robot_2d_test
# potential_test
# puzzle
# ur2_test
# ur5_test TODO
# tutorial_2
# gravity TODO
# test-rrt2 TODO
# room TODO
