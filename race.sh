#!/bin/bash

num=6
model="iris"

if [ "$1" == "prof" ]; then
  arg="--gazebo_ros"
elif [ "$1" == "nonprof" ]; then
  arg=""
else
  echo "$0 [prof | nonprof]"
  exit
fi

./start.sh $model $num gazebo/worlds/race.world $arg $@

if [ "$1" == "prof" ]; then
  ./bin/path_gen.py iris 6 race/centrals.txt race/test_ws/ 20 w1 w2 w3 w4 &
fi
