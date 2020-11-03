//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_TEST_VREP_FRI_ERRORCODE_HPP
#define OROCOS_TEST_VREP_FRI_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x400 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
    NOTREADEE = 1201,
    NOTATTACH = 1202,
    NOTREADREF = 1203,
    NOTMOVEEE = 1204,
    NOTSENTREF = 1205,
    OUTBJOINT = 1401
};

class VFErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        {NOTREADEE, "[VF] Could not read the current end-effector type."},
        {NOTATTACH, "[VF] Could not attach the end-effector."},
        {NOTREADREF, "[VF] Could read the surgery reference frame."},
        {NOTMOVEEE, "[VF] Could move the tool in the end-effector."},
        {NOTSENTREF, "[VF] Could not send the surgery reference to the transformations component."}
    };
};

#endif //OROCOS_TEST_VREP_FRI_ERRORCODE_HPP
