#!/bin/bash 
# must run proj2_init.sh in another terminal first

# source the workspace
source devel/setup.bash

chmod +x src/ee4308_bringup/scripts/teleop_turtle.py
rosrun ee4308_bringup teleop_turtle.py
