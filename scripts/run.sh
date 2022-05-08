#!/bin/bash
DONT_RUN=1 make px4_sitl_default gazebo
source ~/vicon_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
echo $USER/vicon_ws/src/airlab_gazebo/worlds/airlab_cage.world
roscd airlab_gazebo
roslaunch px4 posix_sitl.launch world:=worlds/airlab_cage.world