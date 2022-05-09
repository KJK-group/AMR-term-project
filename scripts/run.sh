#!/bin/bash
px4_wd=${1}
ws_path=${2}

source /opt/share/ros/noetic/setup.bash
source ${ws_path}/devel/setup.bash

cd ${px4_wd} && DONT_RUN=1 make px4_sitl_default gazebo && cd -
source ${px4_wd}/Tools/setup_gazebo.bash ${px4_wd} ${px4_wd}/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${px4_wd}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${px4_wd}/Tools/sitl_gazebo
echo $ROS_PACKAGE_PATH
echo ${ws_path}/src/airlab_gazebo/worlds/airlab_cage.world
roslaunch px4 posix_sitl.launch world:=worlds/airlab_cage.world