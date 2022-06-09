#!/bin/bash
px4_wd=${1}
ws_path=${2}

source /opt/ros/noetic/setup.bash
source ${ws_path}/devel/setup.bash

# rosrun amr_term_project run.sh ${px4_wd} ${ws_path} &
terminator --new-tab -e "${ws_path}/src/amr_term_project/scripts/run.sh ${px4_wd} ${ws_path}"
rl_pid=$!
trap "kill -9 $rl_pid" SIGINT
trap "kill -9 $rl_pid" EXIT

until pgrep -i "gz";
do
sleep 1
done
sleep 5

roslaunch amr_term_project sim.launch

kill_sim() {
    killall gzserver
    killall gzclient
}
trap kill_sim SIGINT
trap kill_sim EXIT