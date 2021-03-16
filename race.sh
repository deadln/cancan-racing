#!/bin/bash

num=6
model="iris"

./start.sh $model $num gazebo/worlds/race.world $@
