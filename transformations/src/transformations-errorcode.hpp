//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_TRANSFORMATIONS_ERRORCODE_HPP
#define OROCOS_TRANSFORMATIONS_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x400 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
    WRKREFSET = 5000,
    NONEEESET = 5001,
    EMPTEESET = 5002,
    CAMEEESET = 5003,
    PROBEESET = 5004,
    TREPEESET = 5005,

    EEBASESET = 5006,
    BBUTTNSET = 5007,
    BPROBESET = 5008,
    BTREPNSET = 5009,
    PNTREESET = 5010,

    NOCALIBRS = 5200,
    UNIDENTFR = 5201,

    INEXISTEE = 5400,
    INEXISTRF = 5401,
};

class TErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        {WRKREFSET, "[TRN] Base to work reference frame set."},
        {NONEEESET, "[TRN] No end-effector set"},
        {EMPTEESET, "[TRN] End-effector empty set"},
        {CAMEEESET, "[TRN] End-effector camera set"},
        {PROBEESET, "[TRN] End-effector probe set"},
        {TREPEESET, "[TRN] End-effector trepan set"},

        {EEBASESET, "[TRN] End-effector base set"},
        {BBUTTNSET, "[TRN] End-effector base buttons set"},
        {BPROBESET, "[TRN] End-effector base probe set"},
        {BTREPNSET, "[TRN] End-effector base trepan set"},

        {PNTREESET, "[TRN] End-effector pointer set"},

        {NOCALIBRS, "[TRN] The system was not calibrated yet! Returning the work frame is the Identity Matrix."},
        {UNIDENTFR, "[TRN] Trying to read an undefined type frame from transformations component."},

        {INEXISTEE, "[TRN] End-effector selected does not match any of the possibilities."},
        {INEXISTRF, "[TRN] Reference frame selected does not match any of the possibilities."},
    };
};

#endif //OROCOS_TRANSFORMATIONS_ERRORCODE_HPP
