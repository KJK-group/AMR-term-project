#ifndef _AMR_STATE_HPP_
#define _AMR_STATE_HPP_

#include <cmath>

namespace amr::utils::state {

auto clamp_yaw(float yaw) -> float {
    return yaw > M_PI ? yaw - 2 * M_PI : yaw < -M_PI ? yaw + 2 * M_PI : yaw;
}
}  // namespace amr::utils::state

#endif  // _AMR_STATE_HPP_