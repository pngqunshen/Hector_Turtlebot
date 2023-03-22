#!/bin/bash 

# source the workspace
source devel/setup.bash

#rosservice call --wait /uav/enable_motors true # enable motors for uav
chmod +x src/ee4308_bringup/scripts/teleop_hector.py
rosrun ee4308_bringup teleop_hector.py
