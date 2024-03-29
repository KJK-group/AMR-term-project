#include "mission.hpp"

namespace amr {
Mission::Mission(ros::NodeHandle& nh, ros::Rate& rate, Eigen::Vector2f target,
                 float velocity_target, Eigen::Vector3f home, int timeout_seconds, bool visualise)
    : inspection_complete(false),
      exploration_complete(false),
      step_count(0),
      seq_point(0),
      seq_state(0),
      seq_vis(0),
      waypoint_idx(1),
      velocity_target(velocity_target),
      rate(rate),
      nh(nh),
      home_position(std::move(home)),
      marker_scale(0.1),
      should_visualise(visualise),
      trajectory({nh, rate, {{0, 0, 0}, home_position}, visualise}),
      target(target),
      object_center(target) {
    // publishers
    pub_mission_state = nh.advertise<amr_term_project::MissionStateStamped>(
        "/amr/mission/state", utils::DEFAULT_QUEUE_SIZE);
    pub_visualise = nh.advertise<visualization_msgs::Marker>("/amr/visualisation_marker",
                                                             utils::DEFAULT_QUEUE_SIZE);
    pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_local/local",
                                                            utils::DEFAULT_QUEUE_SIZE);

    // subscribers
    sub_drone_state = nh.subscribe<mavros_msgs::State>("/mavros/state", utils::DEFAULT_QUEUE_SIZE,
                                                       &Mission::state_cb, this);
    sub_position_error = nh.subscribe<amr_term_project::ControllerStateStamped>(
        "/amr/controller/state", utils::DEFAULT_QUEUE_SIZE, &Mission::controller_state_cb, this);
    // odom subscriber
    sub_odom = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", amr::utils::DEFAULT_QUEUE_SIZE, &Mission::odom_cb, this);

    // services
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    client_land = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    client_takeoff = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    client_rrt = nh.serviceClient<amr_term_project::RrtFindPath>("/amr/rrt_service/find_path");

    // time
    start_time = ros::Time::now();
    timeout_start_time = ros::Time::now();
    delta_time = ros::Duration(0);
    timeout_delta_time = ros::Duration(0);

    timeout = ros::Duration(timeout_seconds);
    expected_position = Eigen::Vector3f(0, 0, 0);

    // state
    state.header.frame_id = amr::utils::FRAME_WORLD;
    state.state = PASSIVE;

    // path
    interest_points.push_back(home_position);

    // trajectory
    // trajectory = trajectory::CompoundTrajectory(nh, rate, {{0, 0, 0}, home_position});
}

auto Mission::state_cb(const mavros_msgs::State::ConstPtr& state) -> void { drone_state = *state; }
auto Mission::controller_state_cb(const amr_term_project::ControllerStateStamped::ConstPtr& state)
    -> void {
    controller_state = *state;
}
auto Mission::odom_cb(const nav_msgs::Odometry::ConstPtr& odom) -> void { drone_odom = *odom; }

auto Mission::compute_attitude() -> void {
    auto pos = drone_odom.pose.pose.position;

    // std::cout << "target:\n" << target_ << std::endl;
    auto diff_x = target.x() - pos.x;
    auto diff_y = target.y() - pos.y;
    auto expected_yaw = std::atan2(diff_y, diff_x);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, expected_yaw);
    expected_attitude = quat;
}

auto Mission::add_interest_point(Eigen::Vector3f interest_point) -> void {
    interest_points.push_back(interest_point);
}
auto Mission::get_drone_state() -> mavros_msgs::State { return drone_state; }
auto Mission::get_trajectory() -> trajectory::CompoundTrajectory { return trajectory; }

auto Mission::set_state(enum state s) -> void {
    state.state = s;
    publish();
}
auto Mission::get_state() -> enum state { return (enum state)state.state; }

/**
 * @brief requests px4 to takeoff through mavros
 *
 * @param altitude altitude to hover at
 * @return true if request is accepted
 * @return false if request is rejected or a takeoff procedure is in progress
 */
auto Mission::drone_takeoff(float altitude) -> bool {
    if (altitude == -1) {
        altitude = home_position.z();
    }

    ros::Duration(2).sleep();
    // while (ros::ok() && position_error.norm > utils::SMALL_DISTANCE_TOLERANCE) {
    //     expected_position = Eigen::Vector3f(0, 0, utils::DEFAULT_DISTANCE_TOLERANCE);
    //     publish();
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    auto altitude_reached = false;
    start_time = ros::Time::now();

    while (ros::ok() && ! altitude_reached) {
        delta_time = ros::Time::now() - start_time;
        auto altitude_progress = delta_time.toSec() * velocity_target;
        if (altitude_progress > altitude) {
            altitude_progress = altitude;
            altitude_reached = true;
        }
        expected_position = Eigen::Vector3f(0, 0, altitude_progress);
        // std::cout << expected_position << std::endl;
        publish();
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && controller_state.error.position.norm > utils::DEFAULT_DISTANCE_TOLERANCE) {
        publish();
        ros::spinOnce();
        rate.sleep();
    }

    // state.state = EXPLORATION;

    return altitude_reached;
}
/**
 * @brief requests px4 to land through mavros
 *
 * @return true if request is accepted
 * @return false if request is rejected
 */
auto Mission::drone_land() -> bool {
    // std::cout << "Landing drone" << std::endl;
    auto previous_request_time = ros::Time(0);
    auto success = false;

    state.state = LAND;
    mavros_msgs::CommandTOL land_msg;

    while (ros::ok() && drone_state.mode != "AUTO.LAND") {
        success = (client_land.call(land_msg)) ? true : false;
        publish();
        ros::spinOnce();
        rate.sleep();
    }

    return success;
}
auto Mission::drone_set_mode(std::string mode) -> bool {
    auto previous_request_time = ros::Time(0);
    auto success = false;

    mavros_msgs::SetMode mode_msg{};
    mode_msg.request.custom_mode = mode;

    while (ros::ok() && drone_state.mode != mode) {
        success = (client_mode.call(mode_msg) && mode_msg.response.mode_sent) ? true : false;
        publish();
        ros::spinOnce();
        rate.sleep();
    }
    return success;
}
auto Mission::drone_arm() -> bool {
    auto previous_request_time = ros::Time(0);
    auto success = false;

    mavros_msgs::CommandBool srv{};
    srv.request.value = true;

    while (ros::ok() && ! drone_state.armed) {
        success = (client_arm.call(srv)) ? true : false;
        publish();
        ros::spinOnce();
        rate.sleep();
    }
    return success;
}

auto Mission::publish() -> void {
    timeout_start_time = ros::Time::now();
    state.header.seq = seq_state++;
    state.header.stamp = ros::Time::now();
    state.target.position.x = expected_position.x();
    state.target.position.y = expected_position.y();
    state.target.position.z = expected_position.z();
    compute_attitude();
    state.target.orientation.x = expected_attitude.getX();
    state.target.orientation.y = expected_attitude.getY();
    state.target.orientation.z = expected_attitude.getZ();
    state.target.orientation.w = expected_attitude.getW();
    // std::cout << "state.target.position.x: " << state.target.position.x << std::endl;
    // std::cout << "state.target.position.y: " << state.target.position.y << std::endl;
    // std::cout << "state.target.position.z: " << state.target.position.z << std::endl;
    pub_mission_state.publish(state);

    if (should_visualise) {
        visualise();
    }
}

auto Mission::visualise() -> void {
    visualization_msgs::Marker m;
    m.header.frame_id = utils::FRAME_WORLD;
    m.header.seq = seq_vis++;
    m.header.stamp = ros::Time::now();

    m.type = visualization_msgs::Marker::SPHERE;

    m.pose.position.x = target.x();
    m.pose.position.y = target.y();
    m.pose.position.z = home_position.z();

    m.scale.x = marker_scale * 2;
    m.scale.y = marker_scale * 2;
    m.scale.z = marker_scale * 2;

    m.color.a = 1;
    m.color.r = 1;
    m.color.g = 1;
    m.color.b = 1;

    pub_visualise.publish(m);
}

auto Mission::find_path(Eigen::Vector3f start, Eigen::Vector3f end)
    -> std::vector<Eigen::Vector3f> {
    // std::cout << "Finding path from " << start << " to " << end << std::endl;
    const auto goal_tolerance = 2;
    auto rrt_msg = amr_term_project::RrtFindPath{};
    rrt_msg.request.probability_of_testing_full_path_from_new_node_to_goal = 0;
    rrt_msg.request.goal_bias = 0.7;
    rrt_msg.request.goal_tolerance = goal_tolerance;
    rrt_msg.request.start.x = start.x();
    rrt_msg.request.start.y = start.y();
    rrt_msg.request.start.z = start.z();
    rrt_msg.request.goal.x = end.x();
    rrt_msg.request.goal.y = end.y();
    rrt_msg.request.goal.z = end.z();
    rrt_msg.request.max_iterations = 10000;
    rrt_msg.request.step_size = 1.5;

    std::vector<Eigen::Vector3f> path;
    if (client_rrt.call(rrt_msg)) {
        auto& waypoints = rrt_msg.response.waypoints;
        // path = std::vector<Eigen::Vector3f>(waypoints.size());
        // std::cout << "before for loop" << std::endl;
        for (auto& wp : waypoints) {
            // std::cout << wp << std::endl;
            path.emplace_back(wp.x, wp.y, wp.z);
        }
    }

    auto arrow_msg_gen = amr::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.02f)
                             .arrow_length(0.02f)
                             .arrow_width(0.02f)
                             .color({0, 1, 0, 1})
                             .build();
    arrow_msg_gen.header.frame_id = utils::FRAME_WORLD;

    auto sphere_msg_gen = amr::utils::rviz::sphere_msg_gen{};
    sphere_msg_gen.header.frame_id = utils::FRAME_WORLD;

    // ros::Duration(1).sleep();
    // publish starting position
    auto start_msg = sphere_msg_gen(start);
    start_msg.color.r = 0.5;
    start_msg.color.g = 1;
    start_msg.color.b = 0.9;
    start_msg.color.a = 1;
    start_msg.scale.x = 0.2;
    start_msg.scale.y = 0.2;
    start_msg.scale.z = 0.2;
    pub_visualise.publish(start_msg);
    // publish goal position
    auto goal_msg = sphere_msg_gen(end);
    goal_msg.color.r = 0;
    goal_msg.color.g = 1;
    goal_msg.color.b = 0;
    goal_msg.color.a = 1;
    goal_msg.scale.x = 0.2;
    goal_msg.scale.y = 0.2;
    goal_msg.scale.z = 0.2;
    pub_visualise.publish(goal_msg);

    // visualise end tolerance
    auto sphere_tolerance_msg = sphere_msg_gen(end);

    auto msg = sphere_msg_gen(end);
    msg.scale.x = goal_tolerance * 2;
    msg.scale.y = goal_tolerance * 2;
    msg.scale.z = goal_tolerance * 2;
    msg.color.r = 1.f;
    msg.color.g = 1.f;
    msg.color.b = 1.f;
    msg.color.a = 0.2f;
    pub_visualise.publish(msg);

    // visualize result
    arrow_msg_gen.color.r = 0.0f;
    arrow_msg_gen.color.g = 1.0f;
    arrow_msg_gen.color.b = 0.0f;
    arrow_msg_gen.scale.x = 0.05f;
    arrow_msg_gen.scale.y = 0.05f;
    arrow_msg_gen.scale.z = 0.05f;
    int i = 1;
    while (ros::ok() && i < path.size()) {
        auto& p1 = path[i - 1];
        auto& p2 = path[i];
        auto arrow = arrow_msg_gen({p1, p2});
        pub_visualise.publish(arrow);
        ++i;
    }
    // std::reverse(path.begin(), path.end());
    return path;
}

auto Mission::fit_trajectory(std::vector<Eigen::Vector3f> path)
    -> std::optional<trajectory::CompoundTrajectory> {
    // std::cout << "found path" << std::endl;
    // for (auto& p : path) {
    //     std::cout << p << std::endl;
    // }
    if (path.size() > 1) {
        auto valid = false;
        for (int p = 1; p < path.size(); ++p) {
            if ((path[0] - path[1]).norm() > std::numeric_limits<float>::epsilon()) {
                valid = true;
            }
        }
        if (valid) {
            return trajectory::CompoundTrajectory(nh, rate, path, should_visualise);
        }
    }
    return {};
}

auto Mission::exploration_step() -> bool {
    auto success = true;
    if (step_count == 0) {
        // std::cout << "resetting time" << std::endl;
        waypoint_idx = 1;
        start_time = ros::Time::now();
    }
    delta_time = ros::Time::now() - start_time;
    // std::cout << amr::utils::GREEN << "Exploration step at: " << delta_time << amr::utils::RESET
    // << std::endl; std::cout << "step_count: " << step_count << std::endl;

    // std::cout << "Interest points:" << std::endl;
    // for (auto& ip : interest_points) {
    //     std::cout << ip << std::endl;
    // }

    // std::cout << "waypoint_idx: " << waypoint_idx << std::endl;

    auto distance = delta_time.toSec() * velocity_target;
    auto remaining_distance = trajectory.get_length() - distance;
    // std::cout << amr::utils::MAGENTA << "distance: " << distance << amr::utils::RESET <<
    // std::endl; std::cout << amr::utils::MAGENTA << "remaining_distance: " << remaining_distance
    // << amr::utils::RESET << std::endl;

    // std::cout << "before if" << std::endl;
    if (step_count == 0 ||
        (remaining_distance < utils::SMALL_DISTANCE_TOLERANCE &&
         controller_state.error.position.norm < utils::SMALL_DISTANCE_TOLERANCE)) {
        // std::cout << "inside if" << std::endl;
        if (waypoint_idx >= interest_points.size()) {
            // std::cout << waypoint_idx << std::endl;
            // std::cout << interest_points.size() << std::endl;
            // state.state = INSPECTION;
            ros::Duration(0.5).sleep();
            success = false;
        } else {
            auto start = interest_points[waypoint_idx - 1];
            auto end = interest_points[waypoint_idx];
            auto path = find_path(start, end);

            // for (auto& p : path) {
            // // std::cout << p << std::endl;
            // }

            if (auto trajectory_opt = fit_trajectory(path)) {
                trajectory = *trajectory_opt;
            } else {
                success = false;
            }
            start_time = ros::Time::now();
            delta_time = ros::Time::now() - start_time;
            waypoint_idx++;
        }
    }
    // std::cout << "after if" << std::endl;

    // std::cout << amr::utils::GREEN << "delta_time: " << delta_time << amr::utils::RESET <<
    // std::endl;

    // control the drone along the spline path
    // distance = delta_time.toSec() * velocity_target;
    // expected_position = spline.get_point_at_distance(distance);

    // publish();
    if (success) {
        // std::cout << "SUCCESS" << std::endl;
        trajectory_step();
    }

    // step_count++;
    return success;
}

auto Mission::trajectory_step() -> bool {
    auto end_reached = false;
    timeout_start_time = ros::Time::now();

    if (step_count == 0) {
        start_time = ros::Time::now();
    }
    delta_time = ros::Time::now() - start_time;
    auto distance = delta_time.toSec() * velocity_target;
    auto remaining_distance = trajectory.get_length() - distance;
    // std::cout << amr::utils::GREEN << "distance: " << distance << amr::utils::RESET << std::endl;
    // std::cout << amr::utils::GREEN << "remaining_distance: " << remaining_distance <<
    // amr::utils::RESET << std::endl;

    // end of trajectory has been reached if the remaining distance and position error is small
    if (remaining_distance < utils::SMALL_DISTANCE_TOLERANCE &&
        controller_state.error.position.norm < utils::SMALL_DISTANCE_TOLERANCE * 10) {
        std::cout << "end reached!" << std::endl;
        // in this case the heading should be towards the center of the object
        target = object_center;
        end_reached = true;
    } else {  // otherwise the next expected position is computed for the current time step
        expected_position = trajectory.get_point_at_distance(distance);
        // here the heading should be towards the expected position
        target = {expected_position.x(), expected_position.y()};
        end_reached = false;
    }
    publish();
    step_count++;

    return end_reached;
}

auto Mission::go_home() -> void {
    // std::cout << "Going to home position" << std::endl;
    auto start = interest_points[waypoint_idx - 1];
    auto path = find_path(start, home_position);

    auto end_reached = false;

    if (auto trajectory_opt = fit_trajectory(path)) {
        trajectory = *trajectory_opt;
    } else {
        end_reached = true;
    }

    step_count = 0;
    while (ros::ok() && ! end_reached) {
        end_reached = trajectory_step();
        // std::cout << "step_count: " << step_count << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
}

auto Mission::outside() -> bool {
    auto pos = drone_odom.pose.pose.position;
    if (pos.x > 6 || pos.y < -6 || pos.y > 6 || pos.y < -6 || pos.z > 3) {
        return true;
    }
    return false;
}

auto Mission::run_step() -> void {
    timeout_delta_time = ros::Time::now() - timeout_start_time;
    if (timeout_delta_time > timeout) {
        // std::cout << "Mission timed out" << std::endl;
        end();
    }
    if (outside()) {
        set_state(LAND);
    }

    switch (state.state) {
        case PASSIVE:
            drone_set_mode();
            drone_arm();
            set_state(HOME);
            break;
        case HOME:
            if (inspection_complete) {
                go_home();
                set_state(LAND);
                // std::cout << "state: " << state_to_string((enum state)state.state) << std::endl;
            } else {
                drone_takeoff();
                set_state(EXPLORATION);
            }
            break;
        case EXPLORATION:
            if (! exploration_step()) {
                std::cout << std::setw(5) << (timeout - timeout_delta_time).toSec()
                          << " Waiting for new interest point..." << std::endl;
            }
            break;
        case INSPECTION:
            end();
            break;
        case LAND:
            // std::cout << "LANDING?" << std::endl;
            drone_land();
            break;
        default:
            ROS_WARN_STREAM("Unknown state");
            break;
    }
}

auto Mission::run() -> void {
    while (ros::ok()) {
        run_step();
        ros::spinOnce();
        rate.sleep();
    }
}

auto Mission::end() -> void {
    inspection_complete = true;
    if (state.state == LAND) {
        return;
    }
    state.state = HOME;
}

auto Mission::state_to_string(enum state s) -> std::string {
    switch (s) {
        case PASSIVE:
            return "PASSIVE";
            break;
        case HOME:
            return "HOME";
            break;
        case EXPLORATION:
            return "EXPLORATION";
            break;
        case INSPECTION:
            return "INSPECTION";
            break;
        case LAND:
            return "LAND";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

}  // namespace amr
