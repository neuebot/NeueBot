#ifndef TRANSFORMATIONS_TYPES_1027845_HPP
#define TRANSFORMATIONS_TYPES_1027845_HPP

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

#endif // TRANSFORMATIONS_TYPES_1027845_HPP
