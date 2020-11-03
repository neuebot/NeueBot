#ifndef CARTESIAN_TRAJECTORY_CONTROLLER_TYPES_HPP
#define CARTESIAN_TRAJECTORY_CONTROLLER_TYPES_HPP

#include <kdl/frames.hpp>
#include <string>

namespace RTT {
    struct error_msg {
        int level;
        std::string msg;
    };

    struct posture {
        KDL::Frame frame;
        double psi;
        unsigned int GC;
    };
}

#endif // CARTESIAN_TRAJECTORY_CONTROLLER_TYPES_HPP