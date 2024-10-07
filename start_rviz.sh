#! /bin/bash

xacro smartcar_simulation/urdf/smartcar.urdf.xacro > smartcar_simulation/urdf/smartcar.urdf

colcon build --merge-install --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"

. install/setup.sh

ros2 launch smartcar_simulation smartcar.launch.py
