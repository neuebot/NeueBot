#ifndef KINEMATICS_MODULE_TYPES_HPP
#define KINEMATICS_MODULE_TYPES_HPP

#include <kdl/frames.hpp>

#include <string>
#include <vector>

namespace RTT {
    struct vector_int {
        std::vector< int > data;
    };

    struct vector_frame {
        std::vector< KDL::Frame > data;
    };

    struct vector_vector_double {
        std::vector< std::vector< double > > data;
    };

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


#endif // KINEMATICS_MODULE_TYPES_HPP
