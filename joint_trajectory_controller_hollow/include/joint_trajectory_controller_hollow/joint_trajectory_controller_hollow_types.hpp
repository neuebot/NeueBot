#ifndef JOINT_TRAJECTORY_CONTROLLER_TYPES_HPP
#define JOINT_TRAJECTORY_CONTROLLER_TYPES_HPP

#include <string>

namespace RTT {
    struct error_msg {
        int level;
        std::string msg;
    };
}

#endif // JOINT_TRAJECTORY_CONTROLLER_TYPES_HPP