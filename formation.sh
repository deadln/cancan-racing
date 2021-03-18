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

./start.sh $model $num gazebo/worlds/formation.world --ref_point 0,-72,0.1 $arg $@

if [ "$1" == "prof" ]; then
  ./bin/formations_gen.py $model $num formation/borders.txt formation/test_fs/ --names T E C T &
fi
