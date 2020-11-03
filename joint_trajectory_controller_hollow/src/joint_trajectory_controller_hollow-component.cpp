#include "joint_trajectory_controller_hollow-component.hpp"
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>

#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <cmath>

using namespace KDL;
using namespace RTT;
using namespace std;

JointTrajectoryControllerHollow::JointTrajectoryControllerHollow(const std::string &component_name) :
    TaskContext(component_name, PreOperational)
{
    //PROPERTIES
    this->addProperty("NJoints",m_nj);
    this->addProperty("MaxFreeJointVel", m_max_free_vel);
    this->addProperty("MaxFreeJointAcc", m_max_free_acc);
    this->addProperty("MaxFreeJointJrk", m_max_free_jrk);
    this->addProperty("MaxTFJointVel", m_max_tf_vel);
    this->addProperty("MaxTFJointAcc", m_max_tf_acc);
    this->addProperty("MaxTFJointJrk", m_max_tf_jrk);
    this->addProperty("JointLimits", m_lim);
    this->addProperty("Tolerance", m_tolerance);
    //ATTRIBUTES
    this->addAttribute("m_q_curr", m_q_curr);
    this->addAttribute("m_v_curr", m_v_curr);
    this->addAttribute("m_q_next", m_q_next);

    //PORTS
    this->ports()->addPort("inport_current_joint_positions", inport_current_joint_positions).doc("Input port that receives the current joint positions (each iteration) from the robotic controller.");
    this->ports()->addPort("inport_current_joint_velocities", inport_current_joint_velocities).doc("Input port that receives the current joint velocities (each iteration) from the robotic controller.");
    this->ports()->addPort("outport_next_joint_positions", outport_next_joint_positions).doc("Output port that sends the target joint positions (each iteration) to the robotic controller.");

    this->ports()->addPort("outport_error_msg", outport_error_msg).doc("Output port that sends error/state messages back to the task supervisor.");

    //OPERATIONS
    this->addOperation("StopMotion", &JointTrajectoryControllerHollow::StopMotion, this, ClientThread);

    this->addOperation("InstantMotion", &JointTrajectoryControllerHollow::InstantMotion, this, OwnThread);
    this->addOperation("SetPointToPoint", &JointTrajectoryControllerHollow::SetPointToPoint, this, OwnThread);
    this->addOperation("SetMotionThroughPoints", &JointTrajectoryControllerHollow::SetMotionThroughPoints, this, OwnThread);
}

bool JointTrajectoryControllerHollow::configureHook() {
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *JMP = new char[128], *RSP = new char[128];
        strcpy(JMP, WS);
        strcat(JMP, "/Properties/JOINT_MOTION_PROPERTIES.xml");
        strcpy(RSP, WS);
        strcat(RSP, "/Properties/ROBOT_STRUCTURE_PROPERTIES.xml");

        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(JMP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(RSP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] RSP;
        delete[] JMP;
    }
    else
    {
        log(Error) << "Could not find WORKSPACE environment variable." << endlog();
        return false;
    }

    //Reflexxes cycle_time is the JointTrajectoryControllerHollow component period
    m_dt = this->getActivity()->getPeriod();

    //Construct Trajectory Generators
    m_sem = std::make_shared<RTT::os::Semaphore>(1); //Binary semaphore
    PointToPoint = std::make_shared<PointToPointTrajectoryGenerator>(m_nj, m_dt, m_max_free_vel, m_max_free_acc, m_max_free_jrk);
    MotionThroughPoints = std::make_shared<MotionThroughPointsTrajectoryGenerator>(m_nj, m_dt, m_max_tf_vel, m_max_tf_acc, m_lim);

    //Set port communication
    bool connections = true;
    connections &= inport_current_joint_positions.connected();
    connections &= inport_current_joint_velocities.connected();
    return connections;

    //Reserve variable space for allocation
    m_q_tar.reserve(50);
    m_t_tar.reserve(50);

    //Setup outport data sample
    error_msg e;
    e.level = 9999;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);
}

bool JointTrajectoryControllerHollow::startHook(){
    //Default mode
    m_mode = JTC_MODE::STOP;

    return true;
}

void JointTrajectoryControllerHollow::updateHook(){
    //Read joint state to compute trajectory
    std::vector<double> current_joint_positions, current_joint_velocities;
    std::vector<double> next_joint_positions;
    std::vector<double> online_joint_positions, online_joint_velocities;
    error_msg e;

    if(inport_current_joint_positions.read(current_joint_positions) == NewData) {
        m_q_curr = current_joint_positions;
    }

    if(inport_current_joint_velocities.read(current_joint_velocities) == NewData) {
        m_v_curr = current_joint_velocities;
    }

    //Main robot motion
    switch(m_mode) {
        case JTC_MODE::STOP:
        {
            //Nothing
            break;
        }
        case JTC_MODE::PTP:
        {
            int ResultValue = PointToPoint->GenerateNextPositionMotionState(m_q_next,m_q_curr);
            HandleReflexxesError(ResultValue);
            if(ResultValue < 0) {
                //Error occurred
                StopMotion();
            }
            else {
                if(ResultValue == ReflexxesAPI::RML_WORKING) {
                    next_joint_positions = m_q_next;
                    outport_next_joint_positions.write(next_joint_positions);
                }
                else if(ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
                    StopMotion();
                }
                else {
                    log(Error) << "Should not happen! Check JTCH update() cycle JTC_MODE::PTP." << endlog();
                    StopMotion();
                }
            }
            break;
        }
        case JTC_MODE::MTP:
        {
        int ResultValue = MotionThroughPoints->GenerateNextPositionMotionState(m_q_next);
        next_joint_positions = m_q_next;
        outport_next_joint_positions.write(next_joint_positions);

        switch(ResultValue) {
        case 0:
            //Running;
            break;
        case 1:
            e.level = 2000 + ResultValue;
            e.msg = m_ec.dict[COMPLETE];
            outport_error_msg.write(e);
            StopMotion();
        case 2:
            //Error
            e.level = MOTGENER;
            e.msg = m_ec.dict[MOTGENER];
            outport_error_msg.write(e);
            StopMotion();
        };
        }
    }
}

void JointTrajectoryControllerHollow::stopHook() {
}

void JointTrajectoryControllerHollow::cleanupHook() {
}

void JointTrajectoryControllerHollow::StopMotion()
{
    //! Sets the semaphore free
    m_sem->signal();

    //Switch to a STOP mode - default
    m_mode = JTC_MODE::STOP;

    error_msg e;
    e.level = COMPLETE;
    e.msg = m_ec.dict[COMPLETE];
    outport_error_msg.write(e);
}

bool JointTrajectoryControllerHollow::InstantMotion(const std::vector<double> &target_joints)
{
    if( m_mode == JTC_MODE::STOP ) {
        outport_next_joint_positions.write(target_joints);
        return true;
    }
    return false;
}

bool JointTrajectoryControllerHollow::SetPointToPoint(const vector<double> &target_joints, const double rel_vel) {
    if( m_mode == JTC_MODE::STOP ) {
        m_q_tar.assign(1,target_joints);
        m_q_tar[0] = target_joints;
        if(PointToPoint->InitiateNewPositionMotion(m_q_curr, m_v_curr, m_q_tar[0], rel_vel)) {
            m_mode = JTC_MODE::PTP;
            HandleReflexxesError(ReflexxesAPI::RML_WORKING);

            //! Locks the semaphore. Will not lock anymore until it is signaled back
            m_sem->wait();
            return true;
        }
        else {
            HandleReflexxesError(PointToPoint->get_ResultValue());
        }
    }
    return false;
}

bool JointTrajectoryControllerHollow::SetMotionThroughPoints(const vector<vector<double> > &target_joints, const vector<double> &target_times)
{
    error_msg e;

    if( m_mode == JTC_MODE::STOP ) {
        vector<double> vi(m_nj, 0.0);
        vector<double> vf(m_nj, 0.0);
        vector<double> ai(m_nj, 0.0);
        vector<double> af(m_nj, 0.0);

        m_q_tar.resize(target_joints.size());
        m_q_tar = target_joints;
        m_t_tar = target_times;

        //RecalculateTimes();
        //Cubic Spline Polynomials
        int res = MotionThroughPoints->InitiateNewPositionMotion(m_q_curr, m_v_curr, m_q_tar, m_t_tar, vi, vf, ai, af );
        if(res == 0) {
            m_mode = JTC_MODE::MTP;
            //Inform of the start of motion sequence
            e.level = 2000;
            e.msg = m_ec.dict[MOVING];
            outport_error_msg.write(e);

            //! Locks the semaphore. Will not lock anymore until it is signaled back
            m_sem->wait();
            return true;
        }
        else {
            error_msg e;
            e.level = res + 2410; //Returned values are errors
            e.msg = m_ec.dict[(ERRORCODE)e.level];
            outport_error_msg.write(e);
        }
    }

    return false;
}

void JointTrajectoryControllerHollow::HandleReflexxesError(int code) {
    error_msg e;
    //Setting error code
    if(code < 0) { //-10(n): different errors -> converted to +250(n)
        if(code == -1) {
            e.level = 2510; //avoid conflict with mtp errors
        }
        else {
            e.level = 2400 + std::abs(code);
            PointToPoint->RecoverFromErrorState();
        }
    }
    else { //0: running, 1: finished
        e.level = 2100 + code;
    }
    //Setting error message
    switch(code) {
    case ReflexxesAPI::RML_WORKING:
        e.msg = "[JTC] Robot is moving. Final state of motion not reached yet.";
        break;
    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        e.msg = "[JTC] Robot finished moving. End state of motion reached.";
        break;
    case ReflexxesAPI::RML_ERROR:
        e.msg = "[JTC] Initialization value of error return codes. This should never be returned.";
        break;
    case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
        e.msg = "[JTC] The applied input values are invalid (cf. RMLPositionInputParameters).";
        break;
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
        e.msg = "[JTC] An error occurred during the first step of the algorithm (i.e., during the calculation of the synchronization time).";
        break;
    case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        e.msg = "[JTC] An error occurred during the second step of the algorithm (i.e., during the synchronization of the trajectory).";
        break;
    case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
        e.msg = "[JTC] The number of degree of freedom of the input parameters, the output parameters, and the "
                "On-Line Trajectory Generation algorithm do not match.";
        break;
    case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
        e.msg = "[JTC] If the input flag RMLFlags::ONLY_PHASE_SYNCHRONIZATION is set and it is not possible to calculate a physically "
                "(and mathematically) correct phase-synchronized (i.e., homothetic) trajectory. Please note: Even if this error message "
                "is returned, feasible, steady, and continuous output values will be computed in any case.";
        break;
    case ReflexxesAPI::RML_ERROR_NULL_POINTER:
        e.msg = "[JTC] If one of the pointers to objects of the classes: i) RMLPositionInputParameters, ii) RMLPositionOutputParameters, iii) RMLPositionFlags";
        break;
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
        e.msg = "[JTC] To ensure numerical stability, the value of the minimum trajectory execution time is limited to a value of RML_MAX_EXECUTION_TIME (10¹⁰ s).";
        break;
    case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
        e.msg = "[JTC] If either: i) ReflexxesAPI::RMLPositionAtAGivenSampleTime() or ii) ReflexxesAPI::RMLVelocityAtAGivenSampleTime(). was used, "
                "the value of the parameter is negative or larger than the value of RML_MAX_EXECUTION_TIME (10¹⁰ s). ";
        break;
    default:
        e.level = 2550;
        e.msg = "[JTC] An unknown error code was retrieved from Reflexxes Type II Lib. Please check.";
        break;
    }
    outport_error_msg.write(e);
}

ORO_CREATE_COMPONENT(JointTrajectoryControllerHollow)
