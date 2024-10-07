#! /bin/bash

colcon build --merge-install --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"
