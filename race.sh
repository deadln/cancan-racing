#!/bin/bash

num=6
model="iris"
world="race.world"

if [ "$1" == "prof" ]; then
  arg="--gazebo_ros"
elif [ "$1" == "nonprof" ]; then
  arg=""
else
  echo "$0 (prof | nonprof) [mc | vtol] [num] [world]"
  exit
fi

if [ "$2" == "vtol" ]; then
  model="standard_vtol"
fi

if [ "$3" != "" ]; then
  num=$3
fi

if [ "$4" != "" ]; then
  world=$4
fi

./start.sh $model $num gazebo/worlds/$world $arg $@

if [ "$1" == "prof" ]; then
  ./bin/path_gen.py $model $num race/centrals.txt race/test_ws/ 20 w1 w2 w3 w4 &
fi
