#ifndef ROBOT_STATE_TYPES_HPP
#define ROBOT_STATE_TYPES_HPP

#include <kdl/frames.hpp>

namespace RTT {
    struct posture {
        KDL::Frame frame;
        double psi;
        unsigned int GC;
    };
}

#endif // ROBOT_STATE_TYPES_HPP