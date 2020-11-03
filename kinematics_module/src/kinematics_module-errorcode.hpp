//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_KINEMATICS_MODULE_ERRORCODE_HPP
#define OROCOS_KINEMATICS_MODULE_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x300 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
    EXISTIKSOL = 3000,
    EXISTTRAJ = 3001,
    CHANGEPSI = 3201,
    OUTWS = 3202,
    IMPPSI = 3203,
    EXCDJLIM = 3402,
    NOPOSSGC = 3403,
    OUTREACH = 3404
};

class KMErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        //Joint Trajectory Controller
        {EXISTIKSOL, "[KM] It exists an Inverse Kinematics solution."},
        {EXISTTRAJ,  "[KM] Trajectory possible from current posture."},
        {CHANGEPSI,  "[KM] Change nullspace parameter to execute the trajectory."},
        {OUTWS,      "[KM] Target frame outside workspace."},
        {IMPPSI,     "[KM] Current psi not in a feasible interval."},
        {EXCDJLIM,   "[KM] Joint limits violated"},
        {NOPOSSGC,   "[KM] Trajectory not feasible for any of the robot configurations."},
        {OUTREACH,   "[KM] Trajectory goes beyond manipulator reach"}
    };
};

#endif //OROCOS_KINEMATICS_MODULE_ERRORCODE_HPP
