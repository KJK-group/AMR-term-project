#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <vector>

#include "amr_term_project/MissionStateStamped.h"
#include "amr_term_project/PointNormStamped.h"
//#include "bezier_spline.hpp"

mavros_msgs::State drone_state;

auto drone_state_cb(const mavros_msgs::State::ConstPtr& msg) { drone_state = *msg; }

auto main(int argc, char** argv) -> int {
    // ros
    ros::init(argc, argv, "amr_mission_manager");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);

    auto start_time = ros::Time::now();

    auto hover_point = Eigen::Vector3f(0, 0, 2.5);
    amr_term_project::MissionStateStamped mission_state;
    mission_state.header.frame_id = "map";
    auto seq_mission = 0;

    // subscribers
    auto sub_drone_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, drone_state_cb);
    auto pub_mission_state =
        nh.advertise<amr_term_project::MissionStateStamped>("/amr/mission/state", 10);

    // wait for FCU connection
    while (ros::ok() && ! drone_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "connected" << std::endl;

    while (ros::ok()) {
        auto time = ros::Time::now();
        auto delta_time = start_time - time;

        std::cout << "seq_mission: " << seq_mission << std::endl;

        mission_state.header.seq = seq_mission++;
        mission_state.header.stamp = time;

        mission_state.target.position.x = hover_point.x();
        mission_state.target.position.y = hover_point.y();
        mission_state.target.position.z = hover_point.z();

        pub_mission_state.publish(mission_state);
    }

    return 0;
}