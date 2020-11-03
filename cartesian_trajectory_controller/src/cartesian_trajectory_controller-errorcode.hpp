//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_CARTESIAN_TRAJECTORY_CONTROLLER_ERRORCODE_HPP
#define OROCOS_CARTESIAN_TRAJECTORY_CONTROLLER_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x300 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
    VCHECKS = 4001,
    NOTRAJ =   4201,
    NOTVALIDV = 4202,
    MISMATCH = 4401,
    EXCDVLIM = 4402
};

class CTCErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        {VCHECKS,  "[CTC] End-effector velocity in Cartesian space is within the specified limits."},
        {NOTRAJ,   "[CTC] Trying to get a non-existing trajectory. Check trajectory segment construction."},
        {NOTVALIDV,"[CTC] Non-valid value of relative_velocity. Values must be comprehended between [0, 1]."},
        {MISMATCH, "[CTC] Mismatch between the number of samples of Cartesian Frames and associated times."},
        {EXCDVLIM, "[CTC] End-effector velocity in Cartesian space overcomes limits."}
    };
};

#endif //OROCOS_CARTESIAN_TRAJECTORY_CONTROLLER_ERRORCODE_HPP
