#ifndef _MULTI_DRONE_INSPECTION_ROSPARAM_HPP_
#define _MULTI_DRONE_INSPECTION_ROSPARAM_HPP_

#include <ros/ros.h>

#include <exception>
#include <stdexcept>
#include <string_view>
// #include <variant>

namespace amr::utils::rosparam {

// using RosParameterT = std::variant<std::string, int, double, bool, std::vector<RosParameterT>>;

template <typename T>
auto get(std::string_view key) -> T {
    auto default_value = T{};
    if (ros::param::get(key, default_value)) {
        return default_value;
    }

    const std::string error_msg =
        "key " + std::string(key) + " does not exist in the parameter server.";
    throw std::invalid_argument(error_msg);
}

}  // namespace amr::utils::rosparam

#endif  // _MULTI_DRONE_INSPECTION_ROSPARAM_HPP_
