#!/bin/bash

num=6
model="iris"

./start.sh $model $num gazebo/worlds/race.world --gazebo_ros $@
./bin/path_gen.py iris 6 race/centrals.txt race/test_ws/ 20 w1 w2 w3 w4 &
