#ifndef _MDI_MISSION_MANAGER_HPP_
#define _MDI_MISSION_MANAGER_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Dense>
#include <limits>
#include <optional>
#include <utility>

#include "amr_term_project/MissionStateStamped.h"
#include "amr_term_project/PointNormStamped.h"
#include "amr_term_project/RrtFindPath.h"
#include "compound_trajectory.hpp"
#include "nav_msgs/Odometry.h"
#include "rrt.hpp"
#include "rrt_builder.hpp"
#include "utils/rviz.hpp"
#include "utils/utils.hpp"

namespace amr {
constexpr auto INITIAL_ALTITUDE = 5;
constexpr auto MISSION_TIMEOUT = 40;
class Mission {
   public:
    Mission(ros::NodeHandle& nh, ros::Rate& rate, float velocity_target = 1,
            Eigen::Vector3f home = {0, 0, INITIAL_ALTITUDE}, int timeout_seconds = MISSION_TIMEOUT,
            bool visualise = false);
    enum state { PASSIVE, HOME, EXPLORATION, INSPECTION, LAND };
    static auto state_to_string(enum state s) -> std::string;
    auto add_interest_point(Eigen::Vector3f interest_point) -> void;

    auto get_drone_state() -> mavros_msgs::State;
    auto get_trajectory() -> trajectory::CompoundTrajectory;
    auto drone_takeoff(float altitude = -1) -> bool;
    auto drone_land() -> bool;
    auto drone_arm() -> bool;
    auto run() -> void;
    auto run_step() -> void;
    auto end() -> void;

    auto set_state(state state) -> void;
    auto get_state() -> state;

   private:
    auto find_path(Eigen::Vector3f start, Eigen::Vector3f end) -> std::vector<Eigen::Vector3f>;
    auto fit_trajectory(std::vector<Eigen::Vector3f> path)
        -> std::optional<trajectory::CompoundTrajectory>;
    auto drone_set_mode(std::string mode = "OFFBOARD") -> bool;

    auto go_home() -> void;
    auto trajectory_step() -> bool;
    auto explore() -> bool;
    auto exploration_step() -> bool;
    auto publish() -> void;

    auto state_cb(const mavros_msgs::State::ConstPtr& state) -> void;
    auto error_cb(const amr_term_project::PointNormStamped::ConstPtr& error) -> void;
    auto odom_cb(const nav_msgs::Odometry::ConstPtr& odom) -> void;

    ros::NodeHandle& nh;
    ros::Rate& rate;
    amr_term_project::MissionStateStamped state;

    std::vector<Eigen::Vector3f> interest_points;

    // publishers
    ros::Publisher pub_mission_state;
    ros::Publisher pub_visualise;
    ros::Publisher pub_setpoint;

    // subscribers
    ros::Subscriber sub_drone_state;
    ros::Subscriber sub_position_error;

    // services
    ros::ServiceClient client_arm;
    ros::ServiceClient client_mode;
    ros::ServiceClient client_takeoff;
    ros::ServiceClient client_land;
    ros::ServiceClient client_rrt;

    // msg instances
    mavros_msgs::State drone_state;
    nav_msgs::Odometry drone_odom;
    amr_term_project::PointNormStamped position_error;

    // points
    Eigen::Vector3f home_position;
    Eigen::Vector3f expected_position;

    // time
    ros::Time start_time;
    ros::Time timeout_start_time;
    ros::Duration delta_time;
    ros::Duration timeout_delta_time;
    ros::Duration timeout;

    // path
    trajectory::CompoundTrajectory trajectory;
    int waypoint_idx;

    float velocity_target;

    // state
    int seq_state;
    int seq_point;
    int step_count;
    bool inspection_complete;
    bool exploration_complete;

    // visualisation
    float marker_scale;
    bool visualise;
};
}  // namespace amr
#endif  // _MDI_MISSION_MANAGER_HPP_