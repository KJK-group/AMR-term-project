#include "compound_trajectory.hpp"

namespace amr::trajectory {
CompoundTrajectory::CompoundTrajectory(ros::NodeHandle& nh, ros::Rate& rate,
                                       std::vector<Eigen::Vector3f> path, bool visualise,
                                       float marker_scale)
    : seq_marker(0), rate(rate) {
    pub_visualisation = nh.advertise<visualization_msgs::MarkerArray>("/amr/visualisation",
                                                                      utils::DEFAULT_QUEUE_SIZE);

    assert(path.size() > 1);
    // treat difference between each point in path as a vector
    vector<Eigen::Vector3f> vectors;
    std::adjacent_difference(path.begin(), path.end(), std::back_inserter(vectors));
    vectors.erase(vectors.begin());

    std::cout << "VECTORS: " << std::endl;
    for (auto& v : vectors) {
        std::cout << v << "\n" << std::endl;
    }

    // find angle betweeen each ordered pair of vectors
    vector<float> angles;
    std::transform(vectors.begin(), vectors.end() - 1, std::next(vectors.begin()),
                   std::back_inserter(angles), [](auto v1, auto v2) {
                       //    std::cout << "v1: " << v1 << "\n"
                       //              << "v2: " << v2 << "\n"
                       //              << std::endl;
                       auto dot = v1.dot(v2);
                       auto n1 = v1.norm();
                       auto n2 = v2.norm();
                       if (std::isnan(dot)) {
                           return 0.f;
                       }
                       return (dot) / (n1 * n2);
                   });
    angles.push_back(1);
    std::cout << "ANGLES:" << std::endl;
    for (auto& a : angles) {
        std::cout << a << std::endl;
    }

    // find splitting indices for each section of the path
    // splitting one path point before the point where a!=180
    // splitting where the angle returns to a==180 after having been a!=180
    vector<bool> straight;
    vector<int> splits = {0};
    // auto straight = false;
    for (int i = 0; i < angles.size() - 1; i++) {
        auto angle_before = i > 0 ? angles[i - 1] : 1;
        auto angle_current = angles[i];
        auto angle_after = i < angles.size() ? angles[i + 1] : 1;
        std::cout << "angle_before: " << angle_before << std::endl;
        std::cout << "angle_current: " << angle_current << std::endl;
        std::cout << "angle_after: " << angle_after << std::endl;
        static constexpr auto float_eps = std::numeric_limits<float>::epsilon() * 2;
        if (std::abs(angle_current - 1) < float_eps) {
            if (std::abs(angle_before - 1) > float_eps) {
                splits.push_back(i + 1);
                straight.push_back(false);
            } else if (std::abs(angle_before - 1) > float_eps ||
                       std::abs(angle_after - 1) > float_eps) {
                // std::cout << "added idx: " << i << std::endl;
                splits.push_back(i + 1);
                straight.push_back(true);
            }
        }
        // std::cout << std::endl;
    }
    // splits.push_back(angles.size());
    splits.push_back(path.size() - 1);
    if (straight.empty()) {
        straight.push_back(true);
    }

    std::cout << "STRAIGHT:" << std::endl;
    for (int s = 0; s < straight.size(); s++) {
        std::cout << straight[s] << std::endl;
    }

    std::cout << "SPLITS:" << std::endl;
    for (auto& s : splits) {
        std::cout << s << std::endl;
    }

    // uses each pair of splitting indices to slice the path into sections,
    // including points at starting and ending indices
    // vector<vector<Eigen::Vector3f>> sections;
    std::transform(splits.begin(), splits.end() - 1, splits.begin() + 1,
                   std::back_inserter(sections), [&](auto s1, auto s2) {
                       vector<Eigen::Vector3f> section;
                       for (s1; s1 <= s2; s1++) {
                           std::cout << "path[" << s1 << "]" << path[s1] << std::endl;
                           section.push_back(path[s1]);
                       }
                       return section;
                   });

    std::cout << "SECTIONS" << std::endl;
    for (int s = 0; s < sections.size(); s++) {
        std::cout << "section " << s << std::endl;
        std::cout << "straight: " << straight[s] << std::endl;
        for (auto& s : sections[s]) {
            std::cout << s << std::endl;
        }
    }

    // create list of trajectories
    std::transform(sections.begin(), sections.end(), straight.begin(),
                   std::back_inserter(trajectories), [](auto section, auto straight) -> Trajectory {
                       // make check between spline and linear trajectory
                       if (straight) {
                           return LinearTrajectory(section.front(), section.back());
                       } else {
                           return BezierSpline(section);
                       }
                   });

    // generates distance LUT
    auto a = 0.f;
    for (auto& t : trajectories) {
        auto length = std::visit([](auto& t) { return t.get_length(); }, t);  // t.get_length();
        a += length;
        distance_lut.push_back(a);
    }

    // visualise in rviz if specified
    // std::cout << "visualise bool = " << visualise << std::endl;
    if (visualise) {
        this->visualise(marker_scale);
    }
}

auto CompoundTrajectory::visualise(float scale) -> void {
    visualization_msgs::MarkerArray ma;
    for (int s = 0; s < trajectories.size(); s++) {
        float r = 255, g = 255, b = 255;
        auto hue = (float)s / (float)trajectories.size() * 360.f + 180;
        if (hue > 360) {
            hue -= 360;
        }
        std::tie(r, g, b) = utils::hsb_to_rgb(hue, 80, 80);
        r /= 255;
        g /= 255;
        b /= 255;

        // std::cout << "r: " << r << ", g: " << g << ", b: " << b << std::endl;

        visualization_msgs::Marker m;
        m.header.frame_id = utils::FRAME_WORLD;
        m.type = visualization_msgs::Marker::SPHERE;

        m.color.a = 1;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        auto t = trajectories[s];
        auto t_length = std::visit([](auto& t) { return t.get_length(); }, t);

        for (float d = 0; t_length - d > 0; d += 0.05) {
            m.header.seq = seq_marker++;
            m.header.stamp = ros::Time::now();
            m.id = seq_marker++;

            m.scale.x = scale * 0.5;
            m.scale.y = scale * 0.5;
            m.scale.z = scale * 0.5;

            auto point = std::visit([&](auto& t) { return t.get_point_at_distance(d); }, t);
            m.pose.position.x = point.x();
            m.pose.position.y = point.y();
            m.pose.position.z = point.z();
            ma.markers.push_back(m);
        }
        for (int p = 0; p < sections[s].size(); p++) {
            m.header.seq = seq_marker++;
            m.header.stamp = ros::Time::now();
            m.id = seq_marker++;

            if (p == 0 || p == sections[s].size() - 1) {
                m.color.r = 0.8;
                m.color.g = 0.8;
                m.color.b = 0.8;
            } else {
                m.color.r = r;
                m.color.g = g;
                m.color.b = b;
            }
            m.scale.x = scale * 1.5;
            m.scale.y = scale * 1.5;
            m.scale.z = scale * 1.5;

            auto point = sections[s][p];
            m.pose.position.x = point.x();
            m.pose.position.y = point.y();
            m.pose.position.z = point.z();
            ma.markers.push_back(m);
        }
    }

    ros::spinOnce();
    rate.sleep();
    // ros::Duration(2).sleep();
    pub_visualisation.publish(ma);
    ros::spinOnce();
    rate.sleep();
}

auto CompoundTrajectory::get_length() -> float {
    // std::cout << "get_length" << std::endl;
    return distance_lut.back();
}

auto CompoundTrajectory::get_point_at_distance(float distance) -> Eigen::Vector3f {
    // std::cout << "get_point_at_distance" << std::endl;
    assert(distance >= 0);
    int trajectory_idx = 0;
    for (trajectory_idx = 0; trajectory_idx < distance_lut.size(); trajectory_idx++) {
        // std::cout << "distance_lut[" << trajectory_idx << "] = " << distance_lut[trajectory_idx]
        // << std::endl;
        if (distance < distance_lut[trajectory_idx]) {
            break;
        }
    }
    if (trajectory_idx >= distance_lut.size()) {
        trajectory_idx = distance_lut.size() - 1;
    }
    // std::cout << "found index: " << trajectory_idx << std::endl;

    auto t = trajectories[trajectory_idx];
    auto t_length = std::visit([](auto& t) { return t.get_length(); }, t);
    auto distance_in_spline = t_length - (distance - distance_lut[trajectory_idx - 1]);
    return std::visit([&](auto& t) { return t.get_point_at_distance(distance_in_spline); }, t);
}
}  // namespace amr::trajectory