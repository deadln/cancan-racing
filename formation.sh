#!/bin/bash

num=6
model="iris"

./start.sh $model $num gazebo/worlds/formation.world --ref_point 0,-72,0.1 $@
