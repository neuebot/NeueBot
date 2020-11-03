//Class that lists the ErrorCodes from the other compoments

#ifndef OROCOS_ERROR_CODES_HPP
#define OROCOS_ERROR_CODES_HPP

#include <utility>
#include <map>
#include <string>

enum LOG_LEVEL{INFO=0, WARNING, ERROR, FATAL};

typedef std::map <int, std::pair<LOG_LEVEL,std::string> > dict;

class ErrorCodes {

public:
    dict code_dict = {
        //Default error code - no behaviour
        {   0, std::make_pair(INFO, "[General] ---.")},
        //Joint Trajectory Controller
        //General
        {2000, std::make_pair(INFO, "[JTC] Running.")},
        {2001, std::make_pair(INFO, "[JTC] Completed.")},
        //Received Variables
        {-2000, std::make_pair(ERROR, "[JTC] Number of serialized joints from 'inport_target_joint_positions' are not a multiple of the number of joints.")},
        {-2001, std::make_pair(ERROR, "[JTC] No trajectory points were added.")},
        {-2002, std::make_pair(WARNING, "[JTC] Current joint positions and velocities were not updated.")},
        //ReflexxesAPI
        {-2100, std::make_pair(ERROR, "[JTC] The applied input values are invalid (cf. RMLPositionInputParameters).")},
        {-2101, std::make_pair(ERROR, "[JTC] An error occurred during the first step of the algorithm (i.e., during the calculation of the synchronization time).")},
        {-2102, std::make_pair(ERROR, "[JTC] An error occurred during the second step of the algorithm (i.e., during the synchronization of the trajectory).")},
        {-2103, std::make_pair(ERROR, "[JTC] The number of degree of freedom of the input parameters, the output parameters, and the On-Line Trajectory Generation algorithm do not match.")},
        {-2104, std::make_pair(ERROR, "[JTC] If the input flag RMLFlags::ONLY_PHASE_SYNCHRONIZATION is set and it is not possible to calculate a physically (and mathematically) correct phase-synchronized (i.e., homothetic) trajectory. Please note: Even if this error message is returned, feasible, steady, and continuous output values will be computed in any case.")},
        {-2105, std::make_pair(ERROR, "[JTC] If one of the pointers to objects of the classes: i) RMLPositionInputParameters, ii) RMLPositionOutputParameters, iii) RMLPositionFlags")},
        {-2106, std::make_pair(ERROR, "[JTC] To ensure numerical stability, the value of the minimum trajectory execution time is limited to a value of RML_MAX_EXECUTION_TIME (10¹⁰ s).")},
        {-2107, std::make_pair(ERROR, "[JTC] If either: i) ReflexxesAPI::RMLPositionAtAGivenSampleTime() or ii) ReflexxesAPI::RMLVelocityAtAGivenSampleTime(). was used, the value of the parameter is negative or larger than the value of RML_MAX_EXECUTION_TIME (10¹⁰ s). ")},

        //Kinematics Module
        {3000, std::make_pair(INFO, "[KM] Exists a Inverse Kinematics solutions.")},
        {3001, std::make_pair(INFO, "[KM] Trajectory can be performed..")},
        {3002, std::make_pair(WARNING, "[KM] Change nullspace parameter to perform the trajectory.")},
        {3003, std::make_pair(WARNING, "[KM] Change robot configuration to perform the trajectory.")},
        {3004, std::make_pair(WARNING, "[KM] Target pose outside reachable workspace for one of the robot configurations.")},

        {-3000, std::make_pair(ERROR, "[KM] Target frame outside reachable workspace.")},
        {-3001, std::make_pair(ERROR, "[KM] Joint limits violated.")},
        {-3002, std::make_pair(ERROR, "[KM] No nullspace parameter scores available.")},
        {-3003, std::make_pair(ERROR, "[KM] Trajectory not feasible for any of the robot configurations.")},

        //Cartesian Trajectory Controller
        {4000, std::make_pair(INFO, "[CTC] Computed a Cartesian Linear Trajectory.")},
        {4001, std::make_pair(INFO, "[CTC] Computed a Cartesian Arc Trajectory.")},
        {4002, std::make_pair(INFO, "[CTC] Sending Cartesian Trajectory to Kinematics Module.")},
        {-4000, std::make_pair(ERROR, "[CTC] Linear Cartesian Velocity in Trajectory exceeds maximum velocity limit.")},

        //Transformations
        {5000, std::make_pair(INFO, "[TRN] Base to work reference frame set.")},
        {5001, std::make_pair(INFO, "[TRN] End-effector set")},
        {-5001, std::make_pair(ERROR, "[TRN] Reference frame selected does not match any of the possibilities.")},
        {-5002, std::make_pair(WARNING, "[TRN] The system was not calibrated yet! Returning the work frame is the Identity Matrix.")},
        {-5010, std::make_pair(ERROR, "[TRN] End-effector selected does not match any of the possibilities.")},


        //Test Supervisor
        {-6000, std::make_pair(ERROR, "[TS] Number of target joint values specified do not match with robot's number of joints.")},
        {-6001, std::make_pair(ERROR, "[TS] Target joint values in joint motion violate limits.")},
        {-6002, std::make_pair(ERROR, "[TS] Target frames variable received for composed motion is invalid.")},
        {-6003, std::make_pair(ERROR, "[TS] Cannot initiate joint motion during Cartesian multiple trajectory action.")},

        {-6008, std::make_pair(ERROR, "[TS] Can only initiate Online Joint Trajectory Generation from a STOP state.")},

        {-6409, std::make_pair(ERROR, "[TS] Cannot approach selected trajectory. No feasible solutions.")},

    };
};

#endif //OROCOS_ERROR_CODES_HPP
