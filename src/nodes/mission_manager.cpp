#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <vector>

#include "amr_term_project/MissionStateStamped.h"
#include "amr_term_project/PointNormStamped.h"
#include "mission.hpp"
#include "utils/utils.hpp"
//#include "bezier_spline.hpp"

using namespace std::string_literals;

mavros_msgs::State drone_state;

auto drone_state_cb(const mavros_msgs::State::ConstPtr& msg) { drone_state = *msg; }

auto main(int argc, char** argv) -> int {
    // ros
    ros::init(argc, argv, "amr_mission_manager");
    auto nh = ros::NodeHandle();
    ros::Rate rate(amr::utils::DEFAULT_LOOP_RATE);

    auto start_time = ros::Time::now();

    // pass in arguments
    auto velocity_target = 1.f;
    auto waypoints_path = ""s;
    if (argc > 1) velocity_target = std::stof(argv[1]);
    if (argc > 2) waypoints_path = argv[2];

    // Points of interest
    std::vector<Eigen::Vector3f> interest_points;
    auto fs = std::ifstream(waypoints_path);
    std::string line, x, y, z, th;

    std::getline(fs, line);
    while (std::getline(fs, line)) {
        std::stringstream s(line);

        std::getline(s, x, ',');
        std::getline(s, y, ',');
        std::getline(s, z, ',');
        // std::getline(s, th, ',');

        auto point = Eigen::Vector3f(std::stof(x), std::stof(y), std::stof(z));
        interest_points.push_back(point);
    }
    interest_points.erase(interest_points.begin());
    interest_points.erase(interest_points.end());

    auto hover_point = Eigen::Vector3f(0, 0, 2);
    interest_points.push_back(hover_point);

    // mission instance
    auto mission = amr::Mission(nh, rate, {0, 0}, velocity_target, hover_point, 5, true);
    for (auto& p : interest_points) {
        std::cout << p.x() << "\t" << p.y() << "\t" << p.z() << std::endl;
        mission.add_interest_point(p);
    }

    // wait for FCU connection
    while (ros::ok() && ! mission.get_drone_state().connected) {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "connected" << std::endl;

    mission.run();

    return 0;
}