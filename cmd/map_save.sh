#!/bin/bash

maps_path=${HOME}/ros2_ws/src/cylinder/cylinder_ros2/maps
map_path=${HOME}/ros2_ws/src/cylinder/cylinder_ros2/maps/$1

if [ ! -e ${maps_path} ]; then
    mkdir -p $maps_path
fi

if [ ! -e ${map_path} ]; then
    mkdir -p $map_path
fi

ros2 run nav2_map_server map_saver_cli -f $map_path/map

