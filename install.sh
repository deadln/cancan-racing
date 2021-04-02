#!/bin/bash

dir=`pwd`/`dirname $0`/

cd $dir
./git-subrepo.sh

if [ "$1" == "update" ]; then
 git pull
 exit
fi

f="all"

if [ "$1" == "build" ]; then
 f="prepare"
fi

./multiple-sitl/install/$f.sh default $dir/Firmware

if [ "$1" != "build" ]; then
 sudo apt install -y ros-noetic-gazebo-ros-control
fi


cws="$HOME/catkin_ws"
rosinstall_file=""

if [ ! -d $cws ];then
  ./multiple-sitl/install/catkin_prepare.sh $cws $rosinstall_file
  str="source $cws/devel/setup.bash"
  rcfile=~/.bashrc

  grep "$str" $rcfile >/dev/null
  if [ $? -eq 1 ];then
    echo $str >> $rcfile
    source ~/.bashrc
  fi
fi
