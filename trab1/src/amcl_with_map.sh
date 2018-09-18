#!/bin/bash -x
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${1?}
