#ifndef CONTROL_MODES_INTERFACE_TYPES_HPP
#define CONTROL_MODES_INTERFACE_TYPES_HPP

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

#endif // CONTROL_MODES_INTERFACE_TYPES_HPP
