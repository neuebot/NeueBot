//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_REGISTRATION_ERRORCODE_HPP
#define OROCOS_REGISTRATION_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x400 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
    COMPLETE = 7000,
    LOADWPTS = 7001,
    LOADRPTS = 7002,
    LOADREGS = 7011,
    SAVEREGS = 7012,
    
    REGWPROP = 7201,
    
    MISMATCH = 7401,
    NOWRKPTS = 7402,
    NOREGPTS = 7403,
    REGERROR = 7404,
    UNSUCREG = 7405,
};

class REGErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        {COMPLETE, "[REG] Completed registration process."},
        {LOADWPTS, "[REG] Loaded work piece regitration points."},
        {LOADRPTS, "[REG] Loaded registered points."},
        {LOADREGS, "[REG] Loaded registration matrix from file."},
        {SAVEREGS, "[REG] Saved registration matrix to file."},

        {REGWPROP, "[REG] No work piece points loaded in runtime. Using values defined in property file."},

        {MISMATCH, "[REG] Mismatch in the number of work piece and registered points."},
        {NOWRKPTS, "[REG] Component does not have any work piece points."},
        {NOREGPTS, "[REG] Component does not have any work piece registered points."},
        {REGERROR, "[REG] Registration error above 3mm."},
        {UNSUCREG, "[REG] Registration process not concluded successfully."},
    };
};

#endif //OROCOS_REGISTRATION_ERRORCODE_HPP
