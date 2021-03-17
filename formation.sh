#!/bin/bash

num=6
model="iris"

./start.sh $model $num gazebo/worlds/formation.world --ref_point 0,-72,0.1 --gazebo_ros $@
./bin/formations_gen.py $model $num formation/borders.txt formation/test_fs/ --names T E C T &
