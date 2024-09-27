#!/bin/bash

# ros2 run my_package my_node --ros-args -p my_param:=my_value
ros2 run csv_editor csv_editor_node --ros-args -p base_path:=$1