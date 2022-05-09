#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "utils/utils.hpp"

nav_msgs::Odometry odom;
auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void { odom = *msg; }

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "map_to_odom_tf_broadcaster");
    auto nh = ros::NodeHandle();
    auto odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",
                                                     amr::utils::DEFAULT_QUEUE_SIZE, odom_cb);
    auto rate = ros::Rate(amr::utils::DEFAULT_LOOP_RATE);

    while (ros::ok()) {
        static auto map_to_inspection_tf_broadcaster = tf2_ros::TransformBroadcaster{};
        auto tf = geometry_msgs::TransformStamped{};

        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = amr::utils::FRAME_WORLD;
        tf.child_frame_id = amr::utils::FRAME_BODY;

        const auto& position = odom.pose.pose.position;
        tf.transform.translation.x = position.x;
        tf.transform.translation.y = position.y;
        tf.transform.translation.z = position.z;

        const auto& quat = odom.pose.pose.orientation;
        tf.transform.rotation.x = quat.x;
        tf.transform.rotation.y = quat.y;
        tf.transform.rotation.z = quat.z;
        tf.transform.rotation.w = quat.w;

        map_to_inspection_tf_broadcaster.sendTransform(tf);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
