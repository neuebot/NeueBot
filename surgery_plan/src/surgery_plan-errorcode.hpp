//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_SURGERY_PLAN_ERRORCODE_HPP
#define OROCOS_SURGERY_PLAN_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x400 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
    // COMPLETE = 8000,
    // LOADWPTS = 8001,
    // LOADRPTS = 8002,
    // LOADREGS = 8011,
    // SAVEREGS = 8012,
    
     SURGPUPL = 8001,

     NOTSELCT = 8201,

     EMPTYPLN = 8401,
     MISMTRID = 8402,
     MISMTREN = 8403,
     REPEATID = 8404,
     NOTFOUND = 8405,
     NOTFEASB = 8406,

};

class SPErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        {SURGPUPL, "[SP] Surgical Plan uploaded to the controller."},

        {NOTSELCT, "[SP] No surgical trajectory was selected."},

        {EMPTYPLN, "[SP] Empty Surgery Plan."},
        {MISMTRID, "[SP] Mismatch between the number of trajectory IDs and the number of target/entry points."},
        {MISMTREN, "[SP] Mismatch between the number of target and entry points."},
        {REPEATID, "[SP] Repeated trajectory IDs."},
        {NOTFOUND, "[SP] The selected surgical trajectory was not found."},
        {NOTFEASB, "[SP] The selected surgical trajectory is not feasible."},
    };
};

#endif //OROCOS_SURGERY_PLAN_ERRORCODE_HPP
