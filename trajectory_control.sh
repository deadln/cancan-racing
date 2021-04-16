#!/bin/bash

num=1
model="iris"
mode="mc"

if [ "$1" == "vtol" ]; then
 model="standard_vtol"
 mode="vtol"
fi

./start.sh $model $num --debug --plugin_lists examples/pluginlists.yaml

echo "waiting ..."
sleep 3

./examples/trajectory_control.py $mode

read -p "Press enter to stop ..."

./stop.sh

