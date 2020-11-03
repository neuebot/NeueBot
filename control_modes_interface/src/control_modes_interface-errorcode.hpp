//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_CONTROL_MODES_INTERFACE_ERRORCODE_HPP
#define OROCOS_CONTROL_MODES_INTERFACE_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x400 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
    FREEMOVE = 8010,
    POINTLCK = 8020,
    PIERCEMD = 8030
    //PTP errors handled in component (Reflexxes)
};

class CMIErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        //Joint Trajectory Controller
        {FREEMOVE, "[CMI] Haptic - Free Motion."},
        {POINTLCK, "[CMI] Haptic - Point Lock Motion."},
        {PIERCEMD, "[CMI] Haptic - Pierce Motion."}
    };
};

#endif //OROCOS_CONTROL_MODES_INTERFACE_ERRORCODE_HPP
