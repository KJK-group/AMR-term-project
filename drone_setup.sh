#!/usr/bin/env bash

SESSION=drone

function send_tmux_cmd() {
    tmux send-keys -t ${SESSION} "${1}" C-m
}

function send_cmd() {
    send_tmux_cmd "source ~/amr_ws/devel/setup.bash"
    send_tmux_cmd "${1}"
}

function vsplit-window() {
    tmux split-window -v -t ${SESSION}
}

function hsplit-window() {
    tmux split-window -h -t ${SESSION}
}

function select-window() {
    tmux select-window -t ${SESSION}:${1}
}

tmux new -s ${SESSION} -d

# tmux send-keys -t ${SESSION} "" C-m

send_cmd "./JetsonTX2_drone_start.sh"

vsplit-window

select-window 1

hsplit-window
send_cmd "roscore"
sleep 3

hsplit-window
send_cmd "roslaunch simple_mavros_controller mavros.launch"

hsplit-window
send_cmd "rostopic echo /mavros/local_position/pose"
hsplit-window
send_cmd "rostopic echo /vicon/${DRONE}/${DRONE}"
# select-window 1
#
hsplit-window
send_cmd "rostopic echo /mavros/state"

vsplit-window

# hsplit-window
# hsplit-window

tmux attach -t ${SESSION}
