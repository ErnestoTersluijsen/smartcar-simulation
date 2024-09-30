#! /bin/bash

xacro smartcar_simulation/urdf/smartcar.urdf.xacro > smartcar_simulation/urdf/smartcar.urdf

colcon build --merge-install

. install/setup.sh

ros2 launch smartcar_simulation smartcar.launch.py
