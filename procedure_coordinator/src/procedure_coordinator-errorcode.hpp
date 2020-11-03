//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_PROCEDURE_COORDINATOR_ERRORCODE_HPP
#define OROCOS_PROCEDURE_COORDINATOR_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x400 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
     FEASIBLE = 9002,
    
    CHNGELBW = 9201,
    CHNGCONF = 9202,
    NOTRJSEL = 9203,
    ATTEEAPP = 9204,
    ATTEEDRI = 9205,
    ATTEEPRB = 9206,
    
    NOTFEASB = 9401,
    ROBOTMVN = 9402,
    SENDSELE = 9403,
    ERRANLYS = 9404
};

class PCErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
        {FEASIBLE, "[PC] Surgical trajectory feasible with current robot configuration and elbow position. Trajectory SET and LOCKED."},

        {CHNGELBW, "[PC] Change elbow position for this surgical trajectory. Trajectory SET and LOCKED."},
        {CHNGCONF, "[PC] Change robot configuration and elbow position for this surgical trajectory. Trajectory SET and LOCKED."},
        {NOTRJSEL, "[PC] No trajectory selected to approach."},
        {ATTEEAPP, "[PC] Attach the 'Acrylic Base Buttons' end-effector to perform the approach step."},
        {ATTEEDRI, "[PC] Attach the 'Acrylic Base Buttons' end-effector to position the robot to perform the drill step."},
        {ATTEEPRB, "[PC] Attach the 'Acrylic Base Buttons' end-effector to position the robot for probe insertion."},

        {NOTFEASB, "[PC] Trajectory not feasible under no configuration. Trajectory NOT set."},
        {ROBOTMVN, "[PC] Cannot execute action while robot is moving."},
        {SENDSELE, "[PC] Error in sending operation 'select surgical trajectory' to the surgical plan component."},
        {ERRANLYS, "[PC] Could not analyse approach trajectory."}
    };
};

#endif //OROCOS_PROCEDURE_COORDINATOR_ERRORCODE_HPP
