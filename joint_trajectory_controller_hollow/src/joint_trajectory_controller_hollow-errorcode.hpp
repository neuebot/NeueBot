//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_JOINT_TRAJECTORY_CONTROLLER_HOLLOW_ERRORCODE_HPP
#define OROCOS_JOINT_TRAJECTORY_CONTROLLER_HOLLOW_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x400 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
    MOVING = 2000,
    COMPLETE = 2001,
    //MTP
    MISMATCH = 2411,
    EXCDJLIM = 2412,
    EXCDVLIM = 2413,
    EXCDALIM = 2414,
    NOVIAPTS = 2415,
    MOTGENER = 2416,
    //PTP errors handled in component (Reflexxes)
};

class JTCErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        //Joint Trajectory Controller
        {MOVING,   "[JTC] Robot is moving. Final state of motion not reached yet."},
        {COMPLETE, "[JTC] Robot finished moving. End state of motion reached."},
        //MTP
        {MISMATCH, "[JTC] Number of received via-points does not match the time-parameterization."},
        {EXCDJLIM, "[JTC] Via-point exceeds the joint limits."},
        {EXCDVLIM, "[JTC] Velocity parameters exceed the maximum value."},
        {EXCDALIM, "[JTC] Acceleration parameters exceed the maximum value."},
        {NOVIAPTS, "[JTC] Not enough via points."},
        {MOTGENER, "[JTC] Error during motion execution after calculation."}
        //PTP

    };
};

#endif //OROCOS_JOINT_TRAJECTORY_CONTROLLER_HOLLOW_ERRORCODE_HPP
