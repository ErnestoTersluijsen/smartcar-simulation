# smartcar-simulation
 
## Guarantees

This repository was build tested on Ubuntu 22.04 LTS with ROS2 Humble, other system configurations's quality can not be guarenteed.

## Compilation Instructions

To build this project a build script has been supplied. 

The `build.sh` script runs `colcon build` with the required arguments.

## Running Instructions

To run this project multiple scripts have been supplied.

Each of the launch files can be called by using the `start_<launch file name>.sh`, these scripts convert the xacro file to an URDF file, build the project, sources the `setup.bash` script, and runs the appropiate launch file.
