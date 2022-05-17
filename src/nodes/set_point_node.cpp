#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "utils/utils.hpp"

mavros_msgs::State state;
auto state_cb(const mavros_msgs::State::ConstPtr& msg) { state = *msg; }

auto main(int argc, char** argv) -> int {
    // ros
    ros::init(argc, argv, "amr_set_point");
    auto nh = ros::NodeHandle();
    ros::Rate rate(amr::utils::DEFAULT_LOOP_RATE);

    auto sub_state =
        nh.subscribe<mavros_msgs::State>("/mavros/state", amr::utils::DEFAULT_QUEUE_SIZE, state_cb);
    auto pub_point = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",
                                                              amr::utils::DEFAULT_QUEUE_SIZE);

    auto client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    auto client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    while (ros::ok() && ! state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "CONNECTED" << std::endl;

    auto seq_pose = 0;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = amr::utils::FRAME_WORLD;
    pose.header.seq = seq_pose++;
    pose.header.stamp = ros::Time::now();

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    auto previous_request_time_mode = ros::Time(0);
    auto previous_request_time_arm = ros::Time(0);
    // offboard mode message
    mavros_msgs::SetMode mode_msg{};
    mode_msg.request.custom_mode = "OFFBOARD";
    // arm message
    mavros_msgs::CommandBool srv{};
    srv.request.value = true;

    while (ros::ok()) {
        pub_point.publish(pose);
        // request to set drone mode to OFFBOARD every 5 seconds until drone is in OFFBOARD mode
        if (state.mode != "OFFBOARD" &&
            (ros::Time::now() - previous_request_time_mode > ros::Duration(5.0))) {
            if (client_mode.call(mode_msg) && mode_msg.response.mode_sent) {
                ROS_INFO("mode set: OFFBOARD");
            } else {
                ROS_INFO("mode set: fail");
            }
            previous_request_time_mode = ros::Time::now();
        }
        // request to arm throttle every 5 seconds until drone is armed
        // if (! state.armed && (ros::Time::now() - previous_request_time_arm > ros::Duration(5.0)))
        // {
        //     if (client_arm.call(srv)) {
        //         ROS_INFO("throttle armed: success");
        //     } else {
        //         ROS_INFO("throttle armed: fail");
        //     }
        //     previous_request_time_arm = ros::Time::now();
        // }
    }
}