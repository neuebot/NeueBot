#ifndef PROCEDURE_COORDINATOR_TYPES_25023570_HPP
#define PROCEDURE_COORDINATOR_TYPES_25023570_HPP

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

    struct redundancy_scores {
        int res;
        std::vector< int > GC;
        std::vector< std::vector< double > > PsiIntervals;
    };

    struct approach_scores {
        bool possible;
        std::vector< KDL::Frame > Frames;
        std::vector< int > GC;
        std::vector< double > Psi;
        std::vector< std::vector< double > > Scores;
    };

    struct error_msg {
        int level;
        std::string msg;
    };

    //For robot_state
    struct posture {
        KDL::Frame frame;
        double psi;
        unsigned int GC;
    };
}

#endif // PROCEDURE_COORDINATOR_TYPES_25023570_HPP
