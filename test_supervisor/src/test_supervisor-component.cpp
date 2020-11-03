#include "test_supervisor-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/marsh/Marshalling.hpp>
#include <eigen3/Eigen/Dense>

#include <rtt/transports/corba/TaskContextProxy.hpp>
#include <rtt/transports/corba/CorbaTemplateProtocol.hpp>

#include <cstring>
#include <cstdlib>

#define TODEG(val) (180*val/M_PI)
#define TORAD(val) (M_PI*val/180)

using namespace KDL;
using namespace RTT;
using namespace Eigen;
using namespace std;

//! Signal function - returns signed int value
template <typename T> int sgn(T val) {
    return (T(0) < val + std::numeric_limits<T>::epsilon()) - (val < T(0));
}

TestSupervisor::TestSupervisor(const string &component_name) :
    TaskContext(component_name, PreOperational),
    //VRepFri
    SetHollowVisibility("SetHollowVisibility")
{
    //PROPERTIES AND ATTRIBUTES
    this->addProperty("NJoints", m_nj).doc("Number of robot joints.");
    this->addProperty("JointLimits", m_lim).doc("Maximum allowed joint position");
    this->addProperty("LSampleStep", m_lsample_step).doc("Linear trajectory sample step (distance).");
    this->addProperty("Tolerance", m_tolerance).doc("Calculations tolerance");
    this->addAttribute("m_ext_torque", m_ext_torque);
    this->addAttribute("m_current_force", m_current_force);
    this->addAttribute("m_simulation", m_simulation);

    //PORTS
    this->ports()->addPort("inport_target_online_joint_positions", inport_target_online_joint_positions);
    this->ports()->addPort("inport_target_relative_frame", inport_target_relative_frame);
    this->ports()->addPort("inport_target_online_joint_velocities", inport_target_online_joint_velocities);
    this->ports()->addPort("inport_target_online_robot_posture", inport_target_online_robot_posture);

    this->ports()->addPort("inport_current_robot_posture", inport_current_robot_posture).doc("Inport current robot parameters from robot state.");
    this->ports()->addPort("inport_current_joint_positions", inport_current_joint_positions).doc("Inport current real robot joint positions.");
    this->ports()->addPort("inport_external_joint_torques", inport_external_joint_torques).doc("Inport external real robot joint torques.");
    this->ports()->addPort("inport_current_endeffector_position", inport_current_endeffector_position).doc("Inport current end-effector position.");
    this->ports()->addPort("inport_robot_moving", inport_robot_moving).doc("Continuous input signal to notify if robot is moving.");

    this->ports()->addPort("inport_robot_session_state",inport_robot_session_state);
    this->ports()->addPort("inport_tracking_performance", inport_tracking_performance);
    this->ports()->addPort("outport_robot_session_state",outport_robot_session_state);
    this->ports()->addPort("outport_tracking_performance", outport_tracking_performance);
    this->ports()->addPort("outport_online_frame", outport_online_frame);
    this->ports()->addPort("outport_joint_torque_force", outport_joint_torque_force);
    this->ports()->addPort("outport_online_joint_positions", outport_online_joint_positions);
    this->ports()->addPort("outport_online_joint_velocities", outport_online_joint_velocities);

    this->ports()->addPort("inport_error_msg",inport_error_msg).doc("Inport error messages from peer components.");
    this->ports()->addPort("outport_error_msg", outport_error_msg);

    //OPERATIONS
    this->addOperation("StopRobot", &TestSupervisor::StopRobot, this, OwnThread);
    this->addOperation("ResumeRobot", &TestSupervisor::ResumeRobot, this, OwnThread);
    this->addOperation("ShutdownRobot", &TestSupervisor::ShutdownRobot, this, OwnThread);

    this->addOperation("JointMotion", &TestSupervisor::JointMotion, this, OwnThread);
    this->addOperation("AdjustedCoordinateJointMotion", &TestSupervisor::AdjustedCoordinateJointMotion, this, OwnThread);
    this->addOperation("CoordinateJointMotion", &TestSupervisor::CoordinateJointMotion, this, OwnThread);
    this->addOperation("IncrementMotion", &TestSupervisor::IncrementMotion, this, OwnThread);
    this->addOperation("LinearMotion", &TestSupervisor::LinearMotion, this, OwnThread);
    this->addOperation("ArcMotionFree", &TestSupervisor::ArcMotionFree, this, OwnThread);
    this->addOperation("ArcMotionFrame", &TestSupervisor::ArcMotionFrame, this, OwnThread);
    this->addOperation("ComposedMotion", &TestSupervisor::ComposedMotion, this, OwnThread);

    this->addOperation("ApproachMotion", &TestSupervisor::ApproachMotion, this, OwnThread);
    this->addOperation("ReturnMotion", &TestSupervisor::ReturnMotion, this, OwnThread);
    this->addOperation("SetRelativeVelocity", &TestSupervisor::SetRelativeVelocity, this, OwnThread);
    this->addOperation("AnalyseLinearMotion", &TestSupervisor::AnalyseLinearMotion, this, OwnThread);
    this->addOperation("AnalyseArcMotionFrame", &TestSupervisor::AnalyseArcMotionFrame, this, OwnThread);

    this->addOperation("ScoresTrajectory", &TestSupervisor::ScoresTrajectory, this, OwnThread);
    this->addOperation("ScoresCurrentPoseCurrentGC", &TestSupervisor::ScoresCurrentPoseCurrentGC, this, OwnThread);
    this->addOperation("ScoresCurrentPoseAlternativeGC", &TestSupervisor::ScoresCurrentPoseAlternativeGC, this, OwnThread);

    this->addOperation("AdjustElbow", &TestSupervisor::AdjustElbow, this, OwnThread);
    this->addOperation("InstantJointMotion", &TestSupervisor::InstantJointMotion,this,OwnThread);
    this->addOperation("InstantCoordinateJointMotion", &TestSupervisor::InstantCoordinateJointMotion,this,OwnThread);
    this->addOperation("InstantNullspaceCoordinateJointMotion", &TestSupervisor::InstantNullspaceCoordinateJointMotion,this,OwnThread);
    this->addOperation("InstantAdjustElbow", &TestSupervisor::InstantAdjustElbow,this,OwnThread);
    this->addOperation("InstantShadowReal", &TestSupervisor::InstantShadowReal,this,OwnThread);

    this->addOperation("SetHollowControl", &TestSupervisor::SetHollowControl,this,OwnThread);
    this->addOperation("SetOTGPosition", &TestSupervisor::SetOTGPosition, this, OwnThread);
    this->addOperation("SetOTGPosture", &TestSupervisor::SetOTGPosture, this, OwnThread);
    this->addOperation("SetWorkFrame", &TestSupervisor::SetWorkFrame, this, OwnThread);
    this->addOperation("GetWorkFrame", &TestSupervisor::GetWorkFrame, this, OwnThread);
    this->addOperation("GetToolFrame", &TestSupervisor::GetToolFrame, this, OwnThread);
    this->addOperation("SetToolType", &TestSupervisor::SetToolType, this, OwnThread);
    this->addOperation("GetToolType", &TestSupervisor::GetToolType, this, OwnThread);

    this->provides("TS_PC")->addOperation("SetToolType", &TestSupervisor::SetToolType, this, OwnThread);
    this->provides("TS_PC")->addOperation("GetToolType", &TestSupervisor::GetToolType, this, OwnThread);

    this->provides("TS_SP")->addOperation("CalculateBestTrajectoryApproach", &TestSupervisor::CalculateBestTrajectoryApproach, this, OwnThread);
    this->provides("TS_SP")->addOperation("CollectApproachResults", &TestSupervisor::CollectApproachResults, this, OwnThread);

    //OPERATION CALLERS
    this->requires("VF_TS")->addOperationCaller(SetHollowVisibility);
}

bool TestSupervisor::configureHook() {
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *CMP = new char[128], *RSP = new char[128];
        strcpy(CMP, WS);
        strcat(CMP, "/Properties/CARTESIAN_MOTION_PROPERTIES.xml");
        strcpy(RSP, WS);
        strcat(RSP, "/Properties/ROBOT_STRUCTURE_PROPERTIES.xml");

        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(CMP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(RSP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] RSP;
        delete[] CMP;
    }
    else
    {
        log(Error) << "Could not find WORKSPACE environment variable." << endlog();
        return false;
    }

//    // Manually connect peers through CORBA
//    TaskContext *klf_ptr = corba::TaskContextProxy::Create("kuka_lbr_fri");
//    // Directional connection [supervisor] -> [fri]
//    this->addPeer(klf_ptr);

    if(hasPeer("kuka_lbr_fri")) {
        TaskContext *klf_ptr = getPeer("kuka_lbr_fri");
        CallerRecoverFRI = klf_ptr->getOperation("RecoverFRI");
        CallerStopRobotFRI = klf_ptr->getOperation("StopRobotFRI");
        CallerResumeRobotFRI = klf_ptr->getOperation("ResumeRobotFRI");
        //Set engine caller of send operations (run in component)
        CallerRecoverFRI.setCaller(klf_ptr->engine());
        CallerStopRobotFRI.setCaller(klf_ptr->engine());
        CallerResumeRobotFRI.setCaller(klf_ptr->engine());

        m_simulation = false;
    }
    else {
        m_simulation = true;
    }


    if(hasPeer("joint_trajectory_controller")) {
        TaskContext *jtc_ptr = getPeer("joint_trajectory_controller");
        CallerStopMotion = jtc_ptr->getOperation("StopMotion");
        CallerSetPointToPoint = jtc_ptr->getOperation("SetPointToPoint");
        CallerSetMotionThroughPoints = jtc_ptr->getOperation("SetMotionThroughPoints");
        CallerSetOTGPosition = jtc_ptr->getOperation("SetOTGPosition");
        CallerSetOTGVelocity = jtc_ptr->getOperation("SetOTGVelocity");
        CallerRecoverFromError = jtc_ptr->getOperation("RecoverFromError");
        //Set engine caller of send operations (run in component)
        CallerSetPointToPoint.setCaller(jtc_ptr->engine());
        CallerSetMotionThroughPoints.setCaller(jtc_ptr->engine());
        CallerSetOTGPosition.setCaller(jtc_ptr->engine());
        CallerSetOTGVelocity.setCaller(jtc_ptr->engine());
        CallerRecoverFromError.setCaller(jtc_ptr->engine());
    }
    else {
        log(Error) << "Failed to connect to peer Joint Trajectory Controller." << endlog();
        return false;
    }

    if(hasPeer("joint_trajectory_controller_hollow")) {
        TaskContext *jtch_ptr = getPeer("joint_trajectory_controller_hollow");
        CallerInstantMotion = jtch_ptr->getOperation("InstantMotion");
        CallerSetPointToPointHollow = jtch_ptr->getOperation("SetPointToPoint");
        CallerSetMotionThroughPointsHollow = jtch_ptr->getOperation("SetMotionThroughPoints");
        //Set engine caller of send operations (run in component)
        CallerInstantMotion.setCaller(jtch_ptr->engine());
        CallerSetPointToPointHollow.setCaller(jtch_ptr->engine());
        CallerSetMotionThroughPointsHollow.setCaller(jtch_ptr->engine());
    }
    else {
        log(Error) << "Failed to connect to peer Joint Trajectory Controller." << endlog();
        return false;
    }

    if(hasPeer("kinematics_module")) {
        TaskContext *km_ptr = getPeer("kinematics_module");
        CallerCalculateJointTrajectory = km_ptr->getOperation("CalculateJointTrajectory");
        CallerCalculateTrajectoryApproach = km_ptr->getOperation("CalculateTrajectoryApproach");
        CallerAnalyseTrajectory = km_ptr->getOperation("AnalyseTrajectory");
        CallerAnalyseTrajectoryApproach = km_ptr->getOperation("AnalyseTrajectoryApproach");
        CallerScoresTrajectory = km_ptr->getOperation("ScoresTrajectory");
        CallerCalculateNullspaceJointTrajectory = km_ptr->getOperation("CalculateNullspaceJointTrajectory");
        CallerForwardKinematics = km_ptr->getOperation("ForwardKinematics");
        CallerInverseKinematics = km_ptr->getOperation("InverseKinematics");
        CallerCalculateExternalForces = km_ptr->getOperation("CalculateExternalForces");
        CallerAdjustedInverseKinematics = km_ptr->getOperation("AdjustedInverseKinematics");
        CallerScoresCurrentPoseCurrentGC = km_ptr->getOperation("ScoresCurrentPoseCurrentGC");
        CallerScoresCurrentPoseAlternativeGC = km_ptr->getOperation("ScoresCurrentPoseAlternativeGC");
        CallerSetOnlineTrajectoryGeneration = km_ptr->getOperation("SetOnlineTrajectoryGeneration");
        //Set engine caller of send operations (run in component)
        CallerCalculateJointTrajectory.setCaller(km_ptr->engine());
        CallerCalculateTrajectoryApproach.setCaller(km_ptr->engine());
        CallerAnalyseTrajectory.setCaller(km_ptr->engine());
        CallerAnalyseTrajectoryApproach.setCaller(km_ptr->engine());
        CallerScoresTrajectory.setCaller(km_ptr->engine());
        CallerCalculateNullspaceJointTrajectory.setCaller(km_ptr->engine());
        CallerForwardKinematics.setCaller(km_ptr->engine());
        CallerInverseKinematics.setCaller(km_ptr->engine());
        CallerCalculateExternalForces.setCaller(km_ptr->engine());
        CallerAdjustedInverseKinematics.setCaller(km_ptr->engine());
        CallerScoresCurrentPoseCurrentGC.setCaller(km_ptr->engine());
        CallerScoresCurrentPoseAlternativeGC.setCaller(km_ptr->engine());
    }
    else {
        log(Error) << "Failed to connect to peer Kinematics Module." << endlog();
        return false;
    }

    if(hasPeer("cartesian_trajectory_controller")) {
        TaskContext *ctc_ptr = getPeer("cartesian_trajectory_controller");
        CallerGenerateLinearTrajectory = ctc_ptr->getOperation("GenerateLinearTrajectory");
        CallerGenerateArcTrajectory = ctc_ptr->getOperation("GenerateArcTrajectory");
        CallerGenerateComposedTrajectory = ctc_ptr->getOperation("GenerateComposedTrajectory");
        CallerGenerateNullspaceTrajectory = ctc_ptr->getOperation("GenerateNullspaceTrajectory");
        CallerGetTrajectory = ctc_ptr->getOperation("GetTrajectory");
        CallerSampleLinearTrajectory = ctc_ptr->getOperation("SampleLinearTrajectory");
        //Set engine caller of send operations (run in component)
        CallerGenerateLinearTrajectory.setCaller(ctc_ptr->engine());
        CallerGenerateArcTrajectory.setCaller(ctc_ptr->engine());
        CallerGenerateComposedTrajectory.setCaller(ctc_ptr->engine());
        CallerGenerateNullspaceTrajectory.setCaller(ctc_ptr->engine());
        CallerGetTrajectory.setCaller(ctc_ptr->engine());
        CallerSampleLinearTrajectory.setCaller(ctc_ptr->engine());
    }
    else {
        log(Error) << "Failed to connect to peer Cartesian Trajectory Controller." << endlog();
        return false;
    }

    if(hasPeer("transformations")) {
        TaskContext *tr_ptr = getPeer("transformations");
        CallerSetWorkFrame = tr_ptr->getOperation("SetWorkFrame");
        CallerSetToolType = tr_ptr->getOperation("SetToolType");
        CallerGetToolType = tr_ptr->getOperation("GetToolType");
        CallerGetToolTypeFrame = tr_ptr->getOperation("GetToolTypeFrame");
        CallerGetToolFrame = tr_ptr->getOperation("GetToolFrame");
        CallerGetReferenceFrame = tr_ptr->getOperation("GetReferenceFrame");
        //Set call thread
        CallerSetWorkFrame.setCaller(tr_ptr->engine());
        CallerSetToolType.setCaller(tr_ptr->engine());
    }
    else {
        log(Error) << "Failed to connect to peer Transformations." << endlog();
        return false;
    }

    this->requires("VF_TS")->connectTo( getPeer("test_vrep_fri")->provides("VF_TS") );
    if(!requires("VF_TS")->ready()) {
        log(Error) << "Failed to require vrep_fri sub-service 'VF_TS'." << endlog();
        return false;
    }

    std::vector<double> sample_force(3);
    std::vector<double> sample_vector(m_nj);
    KDL::Frame sample_frame = KDL::Frame::Identity();
    RTT::error_msg sample_error;
    sample_error.level = 9999;
    sample_error.msg = string(256, '\0');

    outport_joint_torque_force.setDataSample(sample_force);
    outport_online_frame.setDataSample(sample_frame);
    outport_error_msg.setDataSample(sample_error);

    outport_online_joint_positions.setDataSample(sample_vector);
    outport_online_joint_velocities.setDataSample(sample_vector);

    bool connections = true;
    connections &= inport_current_robot_posture.connected();
    connections &= inport_current_joint_positions.connected();
    connections &= inport_robot_moving.connected();
    // error channels
    connections &= inport_error_msg.connected();
    // kuka fri state
    if(!m_simulation) {
        connections &= inport_robot_session_state.connected();
        connections &= inport_tracking_performance.connected();
    }

    return connections;
}

bool TestSupervisor::startHook() {
    //! Call simulator to hide hollow robot image
    if(m_simulation) {
        SetHollowVisibility.call(true);
    }
    else {
        SetHollowVisibility.call(false);
    }

    //! Component flags
    m_state = STATE::NORM;
    m_robot_moving = false;
    m_waiting = true;
    m_control_hollow = false;
    m_relative_velocity = 1.0;

    m_start_frame = KDL::Frame::Identity();
    //! Set Haptic Driver simulation variable

    //GUI Report Level
    m_report_level = LOG_LEVEL::INFO;

    m_transition_counter = 0;

    m_transmit = false;
    m_count = 0;

    return true;
}

void TestSupervisor::updateHook(){
    //! Update cycle
    std::vector<double> current_jpos, external_jtor;
    posture current_robot_param;
    bool real_move;
    //! Online mode
    std::vector<double> online_jpos, online_jvel;
    posture online_posture;
    bool new_ojpos, new_ojvel, new_opost;
    //! Error message
    error_msg error;
    KDL::Frame target_frame;

    //! KUKA messages
    int session_state = 0;
    double tracking_performance = 0.0;
    bool new_session_state, new_tracking_performance;
    new_session_state = (inport_robot_session_state.read(session_state) == NewData);
    new_tracking_performance = (inport_tracking_performance.read(tracking_performance) == NewData);

    if(!m_simulation) {
        if(new_session_state) {
            outport_robot_session_state.write(session_state);
        }

        if(new_tracking_performance) {
            outport_tracking_performance.write(new_tracking_performance);
        }

        //SessionState: 0 Idle, 1 MonWait, 2 MonRdy, 3 CommWait, 4 CommRdy
        if(session_state < 3) {
            m_state = STATE::MON; // Monitoring
        }
        else {
            if(m_state == STATE::MON) {
                m_state = STATE::NORM;
            }
        }
    }
    else {
        outport_robot_session_state.write(4); // COMMANDING_READY
        outport_tracking_performance.write(1.0); // PERFECT TRACKING
    }

    //! Read every cycle
    if(inport_current_robot_posture.read(current_robot_param) == NewData) {
        m_current_frame = current_robot_param.frame;
        m_current_psi = current_robot_param.psi;
        m_current_gc = current_robot_param.GC;
    }
    if(inport_current_joint_positions.read(current_jpos) == NewData) {
        m_current_jpos = current_jpos;
    }
    if(inport_external_joint_torques.read(external_jtor) == NewData) {
        m_ext_torque = external_jtor;
        std::vector<double> ext_forc(6);

        m_current_force.assign(ext_forc.begin(), ext_forc.begin()+3);
    }
    if(inport_robot_moving.read(real_move) == NewData) {
        m_robot_moving = real_move;
    }
    //! OTG ports
    new_ojpos = (inport_target_online_joint_positions.read(online_jpos) == NewData);
    new_ojvel = (inport_target_online_joint_velocities.read(online_jvel) == NewData);
    new_opost = (inport_target_online_robot_posture.read(online_posture) == NewData);
    //! Error Ports
    if(inport_error_msg.read(error) == NewData) {
        HandleErrorMsg(error);
    }

    //!TEST HAPTIC
    if(inport_target_relative_frame.read(target_frame) == NewData) {
        if(m_transmit) {
            if(m_count > 5 && session_state > 2) {

                outport_online_frame.write(target_frame);
            }
        }
        m_count++;
    }

    //! Actuation Control States
    switch(m_state) {
        case::STATE::MON:
        {
        }
            break;
        case STATE::NORM:
        {
            //! Task Supervisor is responsible for handling robot motion execution. This means synchronizing motion execution
            //! avoiding overlaps. A motion queue is set to keep motions in order, in case movement with several steps is
            //! required.
            //! To guarantee motion synchronization the supervisor pushes a motion from the queue only when the robot is stopped.
            //! When the robot motion stops, the component waits for an update cycle before starting the next motion. This is
            //! of course for several steps movements, that imply a pause between them. Continuous motions without pauses are
            //! either Composed motion (several points spline interpolation) or Multiple trajectory (several segments of different
            //! types of motions: linear, arc, composed, paused...)
            if(!m_robot_moving) {
                //! Transition cycles between consecutive motions
                if(m_waiting) {
                    m_transition_counter++;
                    if(m_transition_counter % 250 == 0) {
                        m_waiting = false;
                    }
                }
                //If not moving check queue for motions
                if(m_queue.size()>0 && !m_waiting) {
                    ExecuteNextJointMotion();
                    //to enter next when it stops moving
                    m_waiting = true;
                }
            }
        }
            break;
        case STATE::CAR:
        {
            switch(m_motion) {
            case MOTION::LIN:
            {
                SendStatus ssl = sh_lin_motion.collectIfDone();
                if(ssl == SendStatus::SendSuccess) {
                    ProcessCartesianTrajectory();
                }
            }
                break;
            case MOTION::ARC:
            {
                SendStatus ssa = sh_arc_motion.collectIfDone();
                if(ssa == SendStatus::SendSuccess) {
                    ProcessCartesianTrajectory();
                }
            }
                break;
            case MOTION::CMP:
            {
                SendStatus ssc = sh_cmp_motion.collectIfDone();
                if(ssc == SendStatus::SendSuccess) {
                    ProcessCartesianTrajectory();
                }
            }
                break;
            case MOTION::NSP:
            {
                bool res;
                SendStatus ssn = sh_nsp_motion.collectIfDone(res, m_target_psis, m_target_times);
                if(ssn == SendStatus::SendSuccess) {
                    sh_calc_ns_traj = CallerCalculateNullspaceJointTrajectory.send(m_current_frame, m_target_psis, m_calculated_joints);

                    m_state = STATE::KIN;
                }
            }
                break;
            default:
                //Nothing
                ;
            }
        }
            break;
        case STATE::KIN:
        {
            if(m_motion != MOTION::NSP) {
                bool jtres;
                SendStatus ssj = sh_calc_joint_traj.collectIfDone(jtres, m_calculated_joints);
                if(ssj == SendStatus::SendSuccess) {
                    if(jtres) {
                        ProcessJointTrajectory();
                    }
                    else {
                        //Analyze trajectory and return fitting angle

                        error_msg emsg;
                        emsg.level = LOG_LEVEL::ERROR;
                        emsg.msg = "[TS] Cannot perform the required joint trajectory.";
                        outport_error_msg.write(emsg);

                        m_state = STATE::NORM;
                    }
                }
            }
            else {
                bool nsres;
                SendStatus ssn = sh_calc_ns_traj.collectIfDone(nsres, m_target_psis, m_calculated_joints);
                if(ssn == SendStatus::SendSuccess && nsres) {
                    if(nsres) {
                        ProcessJointTrajectory();
                    }
                    else {
                        error_msg emsg;
                        emsg.level = LOG_LEVEL::ERROR;
                        emsg.msg = "Dropped remaining Nullspace Trajectories from queue. Joint trajectory calculation returned false.";
                        outport_error_msg.write(emsg);

                        m_state = STATE::NORM;
                    }
                }
            }
        }
            break;
        case STATE::JTC:
        {
            bool res;
            SendStatus ss = sh_mtp_motion.collectIfDone(res);
            if(ss == SendStatus::SendSuccess) {
                //OK
                if(!m_robot_moving) {
                    m_state = STATE::NORM;
                }
            }
            else {
//                log(Error) << "[TS] Collect sent MTP operation, status: " << ss << endlog();
            }
        }
            break;
        case STATE::OTGP:
        {
            //! Online Joint Positions
            if(new_ojpos) {
                if(!m_control_hollow) {
                    outport_online_joint_positions.write(online_jpos);
                }
            }
            if(new_ojvel) {
                if(!m_control_hollow) {
                    outport_online_joint_velocities.write(online_jvel);
                }
            }
        }
            break;
        case STATE::OTGF:
        {
            //! Online Posture
            if(new_opost) {
                KDL::Frame fr = online_posture.frame;
                // Send for Kinematics Module
                outport_online_frame.write(fr);
            }
        }
            break;
    }
}

void TestSupervisor::stopHook() {
}

void TestSupervisor::cleanupHook() {
}

//! Basic Robot Control Calls
void TestSupervisor::StopRobot() {
    log(Error) << "STOP ROBOT" << endlog();

    while(!m_queue.empty()) {
        m_queue.pop();
    }

    //! Joint Trajectory Controller
    CallerStopMotion.call();
    //! FRI Component
    if(!m_simulation) {
        CallerStopRobotFRI.call();
    }

    m_state = STATE::NORM;
}

void TestSupervisor::ResumeRobot()
{
    log(Warning) << "RESUME ROBOT" << endlog();

    //! Joint Trajectory Controller
    CallerRecoverFromError.call();
    //! FRI Component
    if(!m_simulation) {
        CallerResumeRobotFRI.call();

        //TODO: Recover FRI
        CallerRecoverFRI.call();
    }
}

void TestSupervisor::ShutdownRobot() {
    log(Fatal) << "SHUTDOWN ROBOT" << endlog();

    StopRobot();

    //stops component
    stop();
}

//! Direct Joint Controller Call
void TestSupervisor::JointMotion(std::vector<double> jpos) {
    error_msg emsg;
    if(m_state == STATE::NORM) {
        if(jpos.size() == m_nj) {
            for(size_t i=0; i<jpos.size(); ++i) {
                if(jpos[i] < -m_lim[i] || jpos[i] > m_lim[i]) {
                    emsg.level = LOG_LEVEL::ERROR;
                    emsg.msg = "[TS] Target joint values violate limits.";
                    outport_error_msg.write(emsg);
                    return;
                }
            }

            if(!m_control_hollow) {
                SetHollowVisibility(false);
            }

            PTPMotion point_motion(jpos, m_control_hollow, m_relative_velocity);
            m_queue.push(point_motion);
        }
        else {
            emsg.level = LOG_LEVEL::ERROR;
            emsg.msg = "[TS] Number of target joint values specified do not match the robot's number of joints.";
            outport_error_msg.write(emsg);
            return;
        }
    }
    else {
        emsg.level = LOG_LEVEL::ERROR;
        emsg.msg = "[TS] Can only initiate a joint motion during a NORMAL supervisor control state.";
        outport_error_msg.write(emsg);
    }
}

//! Single Kinematic Module Call
void TestSupervisor::AdjustedCoordinateJointMotion(Frame target, int ref_frame_id, int tool_id) {
    error_msg emsg;
    if(m_state == STATE::NORM) {
        //Not to be used during multiple trajectory
        vector<double> jpos;
        KDL::Frame reference_frame, end_frame, tool_frame, target_ee;

        //Compute new target according to the attached tool
        CallerGetToolTypeFrame.call(tool_id, tool_frame);
        target_ee = target * tool_frame.Inverse();

        if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
            end_frame = reference_frame * target_ee;

            SendHandle<bool(const KDL::Frame,std::vector<double>&)> handle =
                    CallerAdjustedInverseKinematics.send(end_frame, jpos);

            if(handle.collect() == SendSuccess && handle.ret()) {
                if(!m_control_hollow) {
                    SetHollowVisibility(false);
                }

                PTPMotion direct_motion(jpos, m_control_hollow, m_relative_velocity);
                m_queue.push(direct_motion);
            }
        }
    }
    else {
        emsg.level = LOG_LEVEL::ERROR;
        emsg.msg = "[TS] Can only initiate an adjusted coordinate joint motion during a NORMAL supervisor control state.";
        outport_error_msg.write(emsg);
    }
}

void TestSupervisor::CoordinateJointMotion(Frame target, int ref_frame_id, int tool_id) {
    error_msg emsg;
    if(m_state == STATE::NORM) {
        //Not to be used during multiple trajectory
        vector<double> jpos;
        KDL::Frame reference_frame, end_frame, tool_frame, target_ee;

        //Compute new target according to the attached tool
        CallerGetToolTypeFrame.call(tool_id, tool_frame);
        target_ee = target * tool_frame.Inverse();

        if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
            end_frame = reference_frame * target_ee;

            //Blocking
            bool res = CallerInverseKinematics.call(end_frame, m_current_gc, m_current_psi, jpos);
            if(res) {
                if(!m_control_hollow) {
                    SetHollowVisibility(false);
                }
                PTPMotion direct_motion(jpos, m_control_hollow, m_relative_velocity);
                m_queue.push(direct_motion);
            }
            else {
                emsg.level = LOG_LEVEL::ERROR;
                emsg.msg = "[TS] Cannot reach the specified end-effector frame.";
                outport_error_msg.write(emsg);
            }
        }
    }
    else {
        emsg.level = LOG_LEVEL::ERROR;
        emsg.msg = "[TS] Can only initiate a coordinate joint motion during a NORMAL supervisor control state.";
        outport_error_msg.write(emsg);
    }
}

//! Cartesian Space Motions
void TestSupervisor::IncrementMotion(Frame increment, int ref_frame_id, int tool_id) {
    error_msg emsg;
    //! Only makes sense to start these motions from NORM state
    if(m_state == STATE::NORM) {
        KDL::Frame reference_frame, end_frame, tool_frame, increment_ee;
        KDL::Frame current_frame = m_current_frame;

        //Compute new target according to the attached tool
        CallerGetToolTypeFrame.call(tool_id, tool_frame);
        increment_ee = increment * tool_frame.Inverse();


        if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
            end_frame = current_frame * ((current_frame.Inverse() * reference_frame) * increment_ee * (reference_frame.Inverse() * current_frame));
            //! Sending Operation to compute linear motion and enter CAR mode
            sh_lin_motion = CallerGenerateLinearTrajectory.send(current_frame, end_frame, m_relative_velocity);

            if(!m_control_hollow) {
                SetHollowVisibility(false);
            }

            m_motion = MOTION::LIN;
            m_state = STATE::CAR;
        }
    }
    else {
        emsg.level = LOG_LEVEL::WARNING;
        emsg.msg = "Can only initiate an increment motion during a NORMAL supervisor control state";
        outport_error_msg.write(emsg);
    }
}

void TestSupervisor::LinearMotion(Frame target, int ref_frame_id, int tool_id) {
    error_msg emsg;
    //! Only makes sense to start these motions from NORM state
    if(m_state == STATE::NORM) {
        KDL::Frame reference_frame, end_frame, tool_frame, target_ee;
        KDL::Frame current_frame = m_current_frame;

        //Compute new target according to the attached tool
        CallerGetToolTypeFrame.call(tool_id, tool_frame);
        target_ee = target * tool_frame.Inverse();

        if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
            end_frame = reference_frame * target_ee;
            //start_frame = reference_frame * current_frame;
            //! Sending Operation to compute linear motion and enter CAR mode
            sh_lin_motion = CallerGenerateLinearTrajectory.send(current_frame, end_frame, m_relative_velocity);

            if(!m_control_hollow) {
                SetHollowVisibility(false);
            }

            m_motion = MOTION::LIN;
            m_state = STATE::CAR;
        }
    }
    else {
        emsg.level = LOG_LEVEL::WARNING;
        emsg.msg = "[TS] Cannot initialize a linear motion from the current supervisor mode";
        outport_error_msg.write(emsg);
    }
}

void TestSupervisor::ArcMotionFree(Vector center_point, Frame final_frame, double arc_angle, int ref_frame_id, int tool_id) {
    error_msg emsg;
    //! Only makes sense to start these motions from NORM state
    if(m_state == STATE::NORM) {
        KDL::Frame reference_frame, tool_frame;
        KDL::Frame current_frame = m_current_frame;

        //!Get coordinates of current tool frame
        CallerGetToolTypeFrame.call(tool_id, tool_frame);
        //!Get coordinates of current reference frame
        if(CallerGetReferenceFrame.call(ref_frame_id, reference_frame)) {
            //! The idea is to convert the coordinates to the same reference frame, the robot's base frame.
            //! Hence, the current_frame that is already defined in this reference frame remains the same.
            //! As for the final_frame that is expressed in reference_frame (work frame) coordinates, it
            //! needs to be converted to the base frame first (reference_frame * final_frame). Then, and
            //! to consider the attached tool, to the resulting transformation we 'subtract' the tool
            //! transformation (resulting * tool_frame.Inverse()) ending up with the final robot's flange
            //! transformation in the base reference frame.
            KDL::Frame base_trajectory = reference_frame * final_frame * tool_frame.Inverse();
            KDL::Vector base_center = reference_frame * center_point;

            sh_arc_motion = CallerGenerateArcTrajectory.send(current_frame, base_center, base_trajectory, arc_angle, m_relative_velocity);

            if(!m_control_hollow) {
                SetHollowVisibility(false);
            }

            m_motion = MOTION::ARC;
            m_state = STATE::CAR;
        }
    }
    else {
        emsg.level = LOG_LEVEL::WARNING;
        emsg.msg = "[TS] Cannot initialize an arc motion from the current supervisor mode";
        outport_error_msg.write(emsg);
    }
}

void TestSupervisor::ArcMotionFrame(Vector center_point, Frame final_frame, const int ref_frame_id, const int tool_id)
{
    error_msg emsg;
    //! Only makes sense to start these motions from NORM state
    if(m_state == STATE::NORM) {
        KDL::Frame reference_frame, tool_frame;
        KDL::Frame current_frame = m_current_frame;

        //!Get coordinates of current tool frame
        CallerGetToolTypeFrame.call(tool_id, tool_frame);  
        //!Get coordinates of current reference frame
        if(CallerGetReferenceFrame.call(ref_frame_id, reference_frame)) {
            //! The idea is to convert the coordinates to the same reference frame, the robot's base frame.
            //! Hence, the current_frame that is already defined in this reference frame remains the same.
            //! As for the final_frame that is expressed in reference_frame (work frame) coordinates, it
            //! needs to be converted to the base frame first (reference_frame * final_frame). Then, and
            //! to consider the attached tool, to the resulting transformation we 'subtract' the tool
            //! transformation (resulting * tool_frame.Inverse()) ending up with the final robot's flange
            //! transformation in the base reference frame.
            KDL::Frame base_trajectory = reference_frame * final_frame * tool_frame.Inverse();

            //!We have the three positions that form our arc plane
            //! A point = start rotation position in base frame
            //! B point = final rotation position in base frame
            //! C point = center position in base frame (initially defined in the work frame)
            Vector v_a = current_frame.p;
            Vector v_b = base_trajectory.p;
            Vector v_c = reference_frame * center_point;

            //! Create two unit vectors, one from A to B and another from A to C
            Vector v_ab = v_b - v_a;
            double dist_ab = v_ab.Normalize();
            Vector v_ac = v_c - v_a;
            v_ac.Normalize();
            //! Now we determine: np = the vector normal arc plane as the cross product
            //! of the previous vectors
            Vector v_np = v_ab * v_ac;
            v_np.Normalize();
            //! Now we can find the bissection vector to the AB segment
            Vector v_bis = v_np * v_ab;
            v_bis.Normalize();
            //! The bissection segment is then defined by this vector and the starting point
            //! that is the midway point of the AB segment
            Vector v_mp = (v_a + v_b) / 2;
            //! With the pythagorean theorem we calculate the new arc center points (2)
            Vector v_cp1 = v_bis * sqrt(3*dist_ab*dist_ab/4) + v_mp;
            Vector v_cp2 = -v_bis * sqrt(3*dist_ab*dist_ab/4) + v_mp;
            //! Select the solution that is concave to the center_point
            Vector v_cp;
            if((v_c - v_cp1).Norm() < (v_c - v_cp2).Norm()) {
                v_cp = v_cp1;
            }
            else {
                v_cp = v_cp2;
            }
            //! The arc angle is the angle formed by the angle between the vectors of v_cp and the A and B points
            Vector v_na = v_a - v_cp;
            Vector v_nb = v_b - v_cp;
            double angle = acos(dot(v_na, v_nb) / (v_na.Norm() * v_nb.Norm()));

            //const KDL::Frame start_frame, const KDL::Vector center_point, const KDL::Frame direction_rotation, const double arc_angle, const double relative_velocity
            sh_arc_motion = CallerGenerateArcTrajectory.send(current_frame, v_cp, base_trajectory, angle, m_relative_velocity);

            if(!m_control_hollow) {
                SetHollowVisibility(false);
            }

            m_motion = MOTION::ARC;
            m_state = STATE::CAR;
        }
    }
    else {
        emsg.level = LOG_LEVEL::WARNING;
        emsg.msg = "[TS] Cannot initialize an arc motion from the current supervisor mode";
        outport_error_msg.write(emsg);
    }
}

void TestSupervisor::ComposedMotion(RTT::vector_frame frames, int ref_frame_id, int tool_id) {
    error_msg emsg;
    //! Only makes sense to start these motions from NORM state
    if(m_state == STATE::NORM) {
        vector<KDL::Frame> vframes(frames.data.begin(), frames.data.end());
        vector<KDL::Frame> path_frames;
        KDL::Frame reference_frame, tool_frame;

        //Compute new target according to the attached tool
        CallerGetToolTypeFrame.call(tool_id, tool_frame);

        if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
            //N Frames are marshalled into a single vector of N * 12 values
            //1:3 correspond to position (x, y, z)
            //4:12 correspond to rotation (xx, yx, zx, xy, yy, zy, xz, yz, zz)
            if(vframes.size() % 12 == 0) {
                size_t num_frames = vframes.size() / 12;
                for(size_t i=0; i<num_frames; ++i) {
                    KDL::Frame f_ee = vframes[i] * tool_frame.Inverse();
                    path_frames.push_back(reference_frame * f_ee);
                }
                double radius = 0.01; // TODO: 1 cm radius to round
                sh_cmp_motion = CallerGenerateComposedTrajectory.send(path_frames, radius, m_relative_velocity);

                if(!m_control_hollow) {
                    SetHollowVisibility(false);
                }

                m_motion = MOTION::CMP;
                m_state = STATE::CAR;
            }
            else {
                emsg.level = LOG_LEVEL::ERROR;
                emsg.msg = "[TS] Target frames variable received for composed motion are invalid.";
                outport_error_msg.write(emsg);
            }
        }
    }
    else {
        emsg.level = LOG_LEVEL::WARNING;
        emsg.msg = "[TS] Cannot initialize a linear motion from the current supervisor mode";
        outport_error_msg.write(emsg);
    }
}

bool TestSupervisor::AdjustElbow(const double psi, const int gc) {
    error_msg emsg;
    //! Only makes sense to start these motions from NORM state
    if(m_state == STATE::NORM) {
        //! So three scenarios can happen here:
        //! 1) We don't change configuration, and from current psi to target psi we can freely move without crossing avoid regions.
        //! 2) We don't change configuration, but from current psi to target psi, we pass an avoid region
        //! 3) We change configuration
        //! For scenario 1), the robot will adjust its posture without moving the end-effector.
        //! For scenarios 2) and 3) the robot will make large displacements. Thus, the routine is to go to the zero mechanical
        //! position and return to the target position. Since its a PTP, it is not affected by joint limits or singularities.
        //! The later case implies a sequential motion of the hinge, pivot and then again hinge joints.

        if(!m_control_hollow) {
            SetHollowVisibility(false);
        }

        KDL::Frame current_frame = m_current_frame;
        //! Check if target pose is possible above all
        vector<double> jpos(m_nj);
        if(CallerInverseKinematics(current_frame, gc, psi, jpos)) {
            //! Scenario 1) or 2) the configuration does not change
            if(gc == static_cast<int>(m_current_gc)) {
                //! Check intervals around current and reference values.
                std::vector<double> scores;
                CallerScoresCurrentPoseCurrentGC.call(scores);
                //! Get index of current and reference psi - converted to deg
                int ci = (int) std::round(m_current_psi*(180/M_PI)) + 180;
                int ri = (int) std::round(psi*(180/M_PI)) + 180;
                int sign = sgn(ri - ci);
                bool move_possible = true;
                std::vector<std::pair<double, double> > psi_intervals;
                //! Check possible and avoid intervals between current and reference indices
                for(int i=ci; i!=ri; i=i+sign*1) {
                    if(scores[i] == -1) {
                        move_possible = false;
                        break;
                    }
                }
                //! If psi_intervals is empty, means that the whole interval is doable
                //! Scenario 1)
//                if(move_possible) {
                    sh_nsp_motion = CallerGenerateNullspaceTrajectory.send(m_current_psi, psi, m_target_psis, m_target_times);

                    m_motion = MOTION::NSP;
                    m_state = STATE::CAR;
//                }
                //! Scenario 2)
//                else {
//                    //! To change between configurations the robot executes a composed 3 steps motion, to avoid colliding
//                    //! with any equipment that shares the same space. It considers that the space above the robot is free.
//                    //! So it starts by moving upwards to the extended position, and then adjusting the pivot joints.
//                    //! Hinge joints to zero -> Pivot joints to correct position -> Hinge joints to correct position
//                    vector<double> hinge = m_current_jpos;
//                    //! Hinge joint cycle: 1,3,5
//                    for(size_t i=1; i<m_nj; i=i+2) {
//                        hinge[i] = 0.0;
//                    }
//                    PTPMotion hinge_zero_motion(hinge, m_control_hollow, m_relative_velocity);
//                    m_queue.push(hinge_zero_motion);

//                    //! Now the pivot movement to the correct position
//                    vector<double> pivot = hinge;
//                    //! Pivot joint cycle: 0,2,4,6
//                    for(size_t i=0; i<m_nj; i=i+2) {
//                        pivot[i] = jpos[i];
//                    }
//                    PTPMotion pivot_motion(pivot, m_control_hollow, m_relative_velocity);
//                    m_queue.push(pivot_motion);

//                    //! Finally the hinge movement to the correct final position
//                    hinge = pivot;
//                    for(size_t i=1; i<m_nj; i=i+2) {
//                        hinge[i] = jpos[i];
//                    }
//                    PTPMotion hinge_motion(hinge, m_control_hollow, m_relative_velocity);
//                    m_queue.push(hinge_motion);
//                }
            }
            //! Scenario 3)
            else {
                //! To change between configurations the robot executes a composed 3 steps motion, to avoid colliding
                //! with any equipment that shares the same space. It considers that the space above the robot is free.
                //! So it starts by moving upwards to the extended position, and then adjusting the pivot joints.
                //! Hinge joints to zero -> Pivot joints to correct position -> Hinge joints to correct position
                vector<double> hinge = m_current_jpos;
                //! Hinge joint cycle: 1,3,5
                for(size_t i=1; i<m_nj; i=i+2) {
                    hinge[i] = 0.0;
                }
                PTPMotion hinge_zero_motion(hinge, m_control_hollow, m_relative_velocity);
                m_queue.push(hinge_zero_motion);

                //! Now the pivot movement to the correct position
                vector<double> pivot = hinge;
                //! Pivot joint cycle: 0,2,4,6
                for(size_t i=0; i<m_nj; i=i+2) {
                    pivot[i] = jpos[i];
                }
                PTPMotion pivot_motion(pivot, m_control_hollow, m_relative_velocity);
                m_queue.push(pivot_motion);

                //! Finally the hinge movement to the correct final position
                hinge = pivot;
                for(size_t i=1; i<m_nj; i=i+2) {
                    hinge[i] = jpos[i];
                }
                PTPMotion hinge_motion(hinge, m_control_hollow, m_relative_velocity);
                m_queue.push(hinge_motion);
            }
            return true;
        }
        else {
            emsg.level = LOG_LEVEL::ERROR;
            emsg.msg = "[TS] Cannot reach specified frame with the specified configuration and arm angle.";
            outport_error_msg.write(emsg);
            return false;
        }
    }
    else {
        emsg.level = LOG_LEVEL::WARNING;
        emsg.msg = "[TS] Cannot adjust elbow from the current supervisor mode";
        outport_error_msg.write(emsg);
        return false;
    }
}

bool TestSupervisor::ApproachMotion(Frame target, const int conf, const double psi, const int ref_frame_id, const int tool_id)
{
    RTT::error_msg emsg;

    bool res = true;
    //Move the robot to the Adjust elbow position first if there is elbow motion involved
    if( std::abs(m_current_psi - psi) > m_tolerance ) {
        res = AdjustElbow(psi, conf);
    }


    //Then coordinate move to the target frame
    if(res) {
        //Not to be used during multiple trajectory
        vector<double> jpos;
        KDL::Frame reference_frame, end_frame, tool_frame, target_ee;

        //Compute new target according to the attached tool
        CallerGetToolTypeFrame.call(tool_id, tool_frame);
        target_ee = target * tool_frame.Inverse();

        if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
            end_frame = reference_frame * target_ee;

            //Blocking
            bool res = CallerInverseKinematics.call(end_frame, conf, psi, jpos);
            if(res) {
                PTPMotion direct_motion(jpos, m_control_hollow, m_relative_velocity);
                m_queue.push(direct_motion);
            }
            else {
                emsg.level = LOG_LEVEL::ERROR;
                emsg.msg = "[TS] Cannot reach the specified end-effector frame.";
                outport_error_msg.write(emsg);
            }
        }

        return true;
    }
    return false;
}

bool TestSupervisor::ReturnMotion(Frame target, const int ref_frame_id, const int tool_id, KDL::Frame approach_pose,
                                  const double approach_psi)
{
    error_msg emsg;
    //! Only makes sense to start these motions from NORM state
    if(m_state == STATE::NORM) {
        KDL::Frame reference_frame, end_frame, tool_frame, target_ee;
        KDL::Frame current_frame = m_current_frame;

        //Compute new target according to the attached tool
        CallerGetToolTypeFrame.call(tool_id, tool_frame);
        target_ee = target * tool_frame.Inverse();

        if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
            end_frame = reference_frame * target_ee;
            //start_frame = reference_frame * current_frame;
            //! Sending Operation to compute linear motion and enter CAR mode
            CallerGenerateLinearTrajectory.call(current_frame, end_frame, m_relative_velocity);

            if(CallerGetTrajectory.call(m_target_frames, m_target_times)) {
                //! Send operation to calculate the joint space equivalents of the Cartesian trajectory
                if(CallerCalculateJointTrajectory.call(m_target_frames, m_calculated_joints)) {
                    //! If the trajectory is too short, do the PTP instead
                    if(m_target_frames.size() > 3) {
                        CallerSetMotionThroughPoints.call(m_calculated_joints, m_target_times);

                        m_state = STATE::JTC;
                    }

                    //Blocking
                    std::vector<double> jpos;
                    bool res = CallerInverseKinematics.call(approach_pose, m_current_gc, approach_psi, jpos);
                    if(res) {
                        PTPMotion direct_motion(jpos, m_control_hollow, m_relative_velocity);
                        m_queue.push(direct_motion);

                        return true;
                    }
                    else {
                        emsg.level = LOG_LEVEL::ERROR;
                        emsg.msg = "[TS] Cannot reach the specified end-effector frame.";
                        outport_error_msg.write(emsg);
                    }
                }
            }
        }
    }
    else {
        emsg.level = LOG_LEVEL::WARNING;
        emsg.msg = "[TS] Cannot initialize a linear motion from the current supervisor mode";
        outport_error_msg.write(emsg);
    }

    return false;
}

//! Cartesian Space Motion Handling
bool TestSupervisor::ProcessCartesianTrajectory() {
    if(m_state == STATE::CAR) {
        //! Get calculated trajectory variables from Cartesian component
        if(CallerGetTrajectory.call(m_target_frames, m_target_times)) {
            //! Send operation to calculate the joint space equivalents of the Cartesian trajectory
            sh_calc_joint_traj = CallerCalculateJointTrajectory.send(m_target_frames, m_calculated_joints);

            m_state = STATE::KIN;
            return true;
        }
    }
    m_state = STATE::NORM;
    return false;
}

bool TestSupervisor::ProcessJointTrajectory()
{
    if(m_state == STATE::KIN) {
        if(m_control_hollow) {
            //! Set Hollow robot visible
            SetHollowVisibility(true);
            sh_mtp_motion = CallerSetMotionThroughPointsHollow.send(m_calculated_joints, m_target_times);
        }
        else {
            SetHollowVisibility(false);
            sh_mtp_motion = CallerSetMotionThroughPoints.send(m_calculated_joints, m_target_times);
        }
        m_state = STATE::JTC;
        return true;
    }
    m_state = STATE::NORM;
    return false;
}

//! PTP Motion Queue execution
void TestSupervisor::ExecuteNextJointMotion() {
    //Pushes a new action
    // Queue only pops if the command goes through (i.e. if the robot is waiting for new instructions)
    switch(m_queue.front().type_) {
    case PTPMotion::Type::PointToPoint:
        SetHollowVisibility(false);
        if(CallerSetPointToPoint(m_queue.front().joints_, m_queue.front().rel_vel_)) {
            //Current motion possible, pop queue.
            m_queue.pop();
        }
        else {
            //Current motion not possible, clear queue.
//            while(!m_queue.empty()) {
//                m_queue.pop();
//            }
        }
        break;
    case PTPMotion::Type::PointToPointHollow:
        SetHollowVisibility(true);
        if(CallerSetPointToPointHollow(m_queue.front().joints_, m_queue.front().rel_vel_)) {
            //Current motion possible, pop queue.
            m_queue.pop();
        }
        else {
            //Current motion not possible, clear queue.
//            while(!m_queue.empty()) {
//                m_queue.pop();
//            }
        }
        break;
    }
}

//! Shadow Robot Instant Motions - Preview Robot
void TestSupervisor::InstantJointMotion(const std::vector<double> target_joints) {
    error_msg err;
    if(m_state == STATE::NORM) {
        if(target_joints.size() == m_nj) {
            for(size_t i=0; i<target_joints.size(); ++i) {
                if(target_joints[i] < -m_lim[i] || target_joints[i] > m_lim[i]) {
                    err.level = LOG_LEVEL::ERROR;
                    err.msg = "[TS] Target joint values in joint motion violate limits.";
                    outport_error_msg.write(err);
                }
            }
            SetHollowVisibility(true);
            CallerInstantMotion.call(target_joints);
        }
        else {
            err.level = LOG_LEVEL::ERROR;
            err.msg = "[TS] Number of target joint values specified do not match with robot's number of joints.";
            outport_error_msg.write(err);
        }
    }
    else {
        err.level = LOG_LEVEL::WARNING;
        err.msg = "[TS] Cannot initialize a instant motion from the current supervisor mode";
        outport_error_msg.write(err);
    }
}

void TestSupervisor::InstantCoordinateJointMotion(const Frame target, const int ref_frame_id, const int tool_id) {
    vector<double> jpos(m_nj);
    KDL::Frame reference_frame, end_frame, tool_frame, target_ee;

    //Compute new target according to the attached tool
    CallerGetToolTypeFrame.call(tool_id, tool_frame);
    target_ee = target * tool_frame.Inverse();

    if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
        end_frame = reference_frame * target_ee;

        if(CallerAdjustedInverseKinematics.call(end_frame, jpos)) {
            SetHollowVisibility(true);
            CallerInstantMotion.call(jpos);
        }
    }
}

void TestSupervisor::InstantNullspaceCoordinateJointMotion(const Frame target, const int conf, const double psi,
                                                           const int ref_frame_id, const int tool_id)
{
    RTT::error_msg emsg;
    vector<double> jpos(m_nj);
    KDL::Frame reference_frame, end_frame, tool_frame, target_ee;

    //Compute new target according to the attached tool
    CallerGetToolTypeFrame.call(tool_id, tool_frame);
    target_ee = target * tool_frame.Inverse();

    if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
        end_frame = reference_frame * target_ee;

        if(CallerInverseKinematics.call(end_frame, conf, psi, jpos)) {
            SetHollowVisibility(true);
            CallerInstantMotion.call(jpos);

        }
    }
    else {
        emsg.level = LOG_LEVEL::ERROR;
        emsg.msg = "[TS] Could not retrieve reference frame.";
    }
}

void TestSupervisor::InstantAdjustElbow(const double psi, const int gc) {
    KDL::Frame current_frame = m_current_frame;
    vector<double> jpos(m_nj);
    if(CallerInverseKinematics(current_frame, gc, psi, jpos)) {
        SetHollowVisibility(true);
        CallerInstantMotion.call(jpos);
    }
}

void TestSupervisor::InstantShadowReal() {
    SetHollowVisibility(false);
    CallerInstantMotion.call(m_current_jpos);
}

void TestSupervisor::GenerateApproachFrames(const std::vector<double> &target, const std::vector<double> &entry, double approach_step,
                                            double distance, const KDL::Frame &reference, const KDL::Frame &tool, std::vector<Frame> &frames)
{
    //Trajectory frame entry -> target
    double ang_z;
    //Z-axis is the dir vector.frame = KDL::Frame::Identity();
    //Arbitrary vector defined
    //Y-axis is the cross product of Z-axis and the front-vector [1 0 0]
    //X-axis is the complementar vector given by the cross product of Y-axis and Z-axis
    Vector3d front(1,0,0);
    //Trajectory vector (z-axis and position)
    Vector3d zaxis(target[0]-entry[0], target[1]-entry[1], target[2]-entry[2]);
    double d_et = zaxis.norm();
    zaxis.normalize();
    //If z-axis is collinear with front vector, the dot product is 1 and cross product is the zero vector
    Vector3d yaxis(0,1,0);
    double tolerance = 1e-6; //TODO: Import as property
    if(std::abs(zaxis.dot(front)) < (1 - tolerance))
    {
        yaxis = zaxis.cross(front);
        yaxis.normalize();
    }
    Vector3d xaxis = yaxis.cross(zaxis);
    xaxis.normalize();
    Matrix<double,3,3,RowMajor> rot(3,3);
    rot << xaxis, yaxis, zaxis;
    //Position vector
    Vector3d position = Vector3d(target[0], target[1], target[2]) - (zaxis * (d_et + distance));
    //Number of wrist poses around trajectory
    size_t num_poses = std::ceil(2*M_PI / approach_step);
    frames.assign(num_poses, KDL::Frame::Identity());

    for(size_t i=0; i<num_poses; ++i) {
        //angle around trajectory
        ang_z = M_PI;
        if(i<num_poses-1) {
            ang_z = (i+1)*approach_step - M_PI;
        }

        rot = rot * AngleAxisd(ang_z, Vector3d::UnitZ());
        //Orientation
        std::copy(rot.data(), rot.data()+9, frames[i].M.data);
        //Position
        frames[i].p = Vector(position.x(), position.y(), position.z());

        frames[i] = reference * frames[i] * tool.Inverse();
    }
}

bool TestSupervisor::CalculateBestTrajectoryApproach(const std::vector<double> &target, const std::vector<double> &entry, double approach_step,
                                            double distance)
{
    std::vector<KDL::Frame> frames;
    KDL::Frame reference_frame, tool_frame;
    //Compute new target according to the attached tool
    //Should be the Acrylic Buttons Tool
    CallerGetToolFrame.call(tool_frame);
    CallerGetReferenceFrame(REFERENCE_FRAME::WORK, reference_frame);

    GenerateApproachFrames(target, entry, approach_step, distance, reference_frame, tool_frame, frames);

    ///Input
    RTT::vector_frame rtt_frames;
    RTT::vector_vector_double rtt_desired;
    rtt_frames.data = frames;
    //Setup the desired psi intervals for each configuration.
    //These intervals avoid the robot elbow near undesired zones.

//    std::vector<interval_set<double> > conf_desired(8);
    double inLim = M_PI_2;
    double midLim = M_PI_2;
    std::vector<double>
            interval_in({-inLim, inLim}),
            interval_out({-M_PI, -midLim, midLim, M_PI});
    rtt_desired.data.resize(8);
    rtt_desired.data[0] = interval_out;
    rtt_desired.data[1] = interval_out;
    rtt_desired.data[2] = interval_in;
    rtt_desired.data[3] = interval_in;
    rtt_desired.data[4] = interval_out;
    rtt_desired.data[5] = interval_out;
    rtt_desired.data[6] = interval_in;
    rtt_desired.data[7] = interval_in;

    //Output
    RTT::vector_int conf_max;
    std::vector<double> psi_max;
    RTT::vector_frame frames_max;

    // Approach poses are calculated and sorted according to the best fitting manipulability (highest first).
    // Additionally, if the current configuration allows for a robot approach, it becomes the first element.
    bool res = CallerCalculateTrajectoryApproach.call(rtt_desired, rtt_frames, conf_max, psi_max, frames_max);
    if(!res) {
        RTT::error_msg emsg;
        emsg.level = LOG_LEVEL::ERROR;
        emsg.msg = "[TS] Cannot approach selected trajectory. No feasible solutions.";
        outport_error_msg.write(emsg);

        return false;
    }

    //Initialize component approach variables
    m_approach_GC = conf_max.data;
    m_approach_psi = psi_max;
    m_approach_frame = frames_max.data;
    m_approach_scores.resize(conf_max.data.size());

    // Once the best approach frames are calculated according to the manipulability index, we determine if the
    // robot can perform a linear trajectory to the drill / probe position without violating joint limits or
    // passing through singularities.
    RTT::vector_frame trajectory_frames;

    // Store not possible indices
    std::vector<size_t> rm_idx;
    for (size_t i=0; i<conf_max.data.size(); i++) {
        KDL::Frame end_frame = reference_frame.Inverse() * frames_max.data[i] * tool_frame;
        end_frame.p = end_frame.p - end_frame.M.UnitZ() * distance;
        end_frame = reference_frame * end_frame * tool_frame.Inverse();
        // Sample trajectory linear path
        CallerSampleLinearTrajectory.call(m_approach_frame[i], end_frame, m_lsample_step, trajectory_frames.data);

        // Calculate approach score
        bool possible = CallerAnalyseTrajectoryApproach(trajectory_frames, m_approach_GC[i], m_approach_psi[i], m_approach_scores[i]);
        if(!possible) {
            rm_idx.push_back(i);
        }
    }

    // Goes through impossible indices and erases stored values in reverse direction
    // Last to First
    for_each(rm_idx.rbegin(), rm_idx.rend(), [&](size_t &idx){
        m_approach_GC.erase(m_approach_GC.begin()+idx);
        m_approach_psi.erase(m_approach_psi.begin()+idx);
        m_approach_frame.erase(m_approach_frame.begin()+idx);
        m_approach_scores.erase(m_approach_scores.begin()+idx);
    });

    return true;
}

void TestSupervisor::CollectApproachResults(vector_int &conf_max, std::vector<double> &psi_max, vector_frame &frames_max, vector_vector_double &configuration_scores)
{
    conf_max.data = m_approach_GC;
    psi_max = m_approach_psi;
    frames_max.data = m_approach_frame;
    configuration_scores.data = m_approach_scores;
}

bool TestSupervisor::AnalyseTrajectory(vector_frame &target_frames, vector_int &possible, vector_vector_double &initial_interval)
{
    return CallerAnalyseTrajectory.call(target_frames.data, possible.data, initial_interval.data);
}

bool TestSupervisor::AnalyseLinearMotion(Frame target, const int ref_frame_id, const int tool_id, redundancy_scores &solution)
{
    KDL::Frame reference_frame, end_frame, tool_frame, target_ee;
    KDL::Frame current_frame = m_current_frame;

    //Compute new target according to the attached tool
    CallerGetToolTypeFrame.call(tool_id, tool_frame);
    target_ee = target * tool_frame.Inverse();

    if(CallerGetReferenceFrame(ref_frame_id, reference_frame)) {
        end_frame = reference_frame * target_ee;
        //start_frame = reference_frame * current_frame;
        //! Sending Operation to compute linear motion and enter CAR mode
        CallerGenerateLinearTrajectory.call(current_frame, end_frame, m_relative_velocity);

        //!Get Trajectory frames
        std::vector<KDL::Frame> tframes;
        std::vector<double> ttimes;
        CallerGetTrajectory.call(tframes, ttimes);

        //!Call Analyse Trajectory from Kinematics Module directly
        std::vector<int> possible;
        std::vector<std::vector<double> > init_interval;
        bool result = CallerAnalyseTrajectory.call(tframes, possible, init_interval);

        //!Score Trajectory
        if(result) {
            solution.res = CallerScoresTrajectory.call(possible, init_interval, solution.GC, solution.PsiIntervals);
        }
        else {
            solution.res = 3; // NO SOLUTION
        }

        return true;
    }
    return false;
}

bool TestSupervisor::AnalyseArcMotionFrame(Vector center_point, Frame final_frame, const int ref_frame_id, const int tool_id, redundancy_scores &solution)
{
    KDL::Frame reference_frame, tool_frame;
    KDL::Frame current_frame = m_current_frame;

    //!Get coordinates of current tool frame
    CallerGetToolTypeFrame.call(tool_id, tool_frame);
    //!Get coordinates of current reference frame
    if(CallerGetReferenceFrame.call(ref_frame_id, reference_frame)) {
        //! The idea is to convert the coordinates to the same reference frame, the robot's base frame.
        //! Hence, the current_frame that is already defined in this reference frame remains the same.
        //! As for the final_frame that is expressed in reference_frame (work frame) coordinates, it
        //! needs to be converted to the base frame first (reference_frame * final_frame). Then, and
        //! to consider the attached tool, to the resulting transformation we 'subtract' the tool
        //! transformation (resulting * tool_frame.Inverse()) ending up with the final robot's flange
        //! transformation in the base reference frame.
        KDL::Frame base_trajectory = reference_frame * final_frame * tool_frame.Inverse();

        //!We have the three positions that form our arc plane
        //! A point = start rotation position in base frame
        //! B point = final rotation position in base frame
        //! C point = center position in base frame (initially defined in the work frame)
        Vector v_a = current_frame.p;
        Vector v_b = base_trajectory.p;
        Vector v_c = reference_frame * center_point;

        //! Create two unit vectors, one from A to B and another from A to C
        Vector v_ab = v_b - v_a;
        double dist_ab = v_ab.Normalize();
        Vector v_ac = v_c - v_a;
        v_ac.Normalize();
        //! Now we determine: np = the vector normal arc plane as the cross product
        //! of the previous vectors
        Vector v_np = v_ab * v_ac;
        v_np.Normalize();
        //! Now we can find the bissection vector to the AB segment
        Vector v_bis = v_np * v_ab;
        v_bis.Normalize();
        //! The bissection segment is then defined by this vector and the starting point
        //! that is the midway point of the AB segment
        Vector v_mp = (v_a + v_b) / 2;
        //! With the pythagorean theorem we calculate the new arc center points (2)
        Vector v_cp1 = v_bis * sqrt(3*dist_ab*dist_ab/4) + v_mp;
        Vector v_cp2 = -v_bis * sqrt(3*dist_ab*dist_ab/4) + v_mp;
        //! Select the solution that is concave to the center_point
        Vector v_cp;
        if((v_c - v_cp1).Norm() < (v_c - v_cp2).Norm()) {
            v_cp = v_cp1;
        }
        else {
            v_cp = v_cp2;
        }
        //! The arc angle is the angle formed by the angle between the vectors of v_cp and the A and B points
        Vector v_na = v_a - v_cp;
        Vector v_nb = v_b - v_cp;
        double angle = acos(dot(v_na, v_nb) / (v_na.Norm() * v_nb.Norm()));

        //const KDL::Frame start_frame, const KDL::Vector center_point, const KDL::Frame direction_rotation, const double arc_angle, const double relative_velocity
        CallerGenerateArcTrajectory.call(current_frame, v_cp, base_trajectory, angle, m_relative_velocity);

        //!Get Trajectory frames
        std::vector<KDL::Frame> tframes;
        std::vector<double> ttimes;
        CallerGetTrajectory.call(tframes, ttimes);

        //!Call Analyse Trajectory from Kinematics Module directly
        std::vector<int> possible;
        std::vector<std::vector<double> > init_interval;
        bool result = CallerAnalyseTrajectory.call(tframes, possible, init_interval);

        //!Score Trajectory
        if(result) {
            solution.res = CallerScoresTrajectory.call(possible, init_interval, solution.GC, solution.PsiIntervals);
        }
        else {
            solution.res = 3; // NO SOLUTION
        }

        return true;
    }
    return false;
}

int TestSupervisor::ScoresTrajectory(vector_int &possible, vector_vector_double &initial_interval, vector_int &gc_alt, vector_vector_double &scores)
{
    int res = CallerScoresTrajectory.call(possible.data, initial_interval.data, gc_alt.data, scores.data);
    return res;
}

std::vector<double> TestSupervisor::ScoresCurrentPoseCurrentGC() {
    std::vector<double> scores;
    CallerScoresCurrentPoseCurrentGC.call(scores);
    return scores;
}

std::vector<double> TestSupervisor::ScoresCurrentPoseAlternativeGC(const int gc) {
    std::vector<double> scores;
    CallerScoresCurrentPoseAlternativeGC.call(gc, scores);
    return scores;
}

//! Set Get Basic Properties / Tools
void TestSupervisor::SetRelativeVelocity(double relative_velocity) {
    if(relative_velocity > 0.0 && relative_velocity <= 1.0) {
        m_relative_velocity = relative_velocity;
    }
}

void TestSupervisor::SetToolType(const int type)
{
    if(CallerSetToolType.ready()) {
        CallerSetToolType.call(type);
    }
}

void TestSupervisor::SetWorkFrame(const Frame base_work) {
    if(CallerSetWorkFrame.ready())
        CallerSetWorkFrame.send(base_work);
}

int TestSupervisor::GetToolType()
{
    int type = 0;
    if(CallerGetToolType.ready()) {
        type = CallerGetToolType.call();
    }
    else {
        log(Error) << "Could not return the current tool type" << endlog();
    }
    return type;
}

void TestSupervisor::GetToolFrame(Frame &tool)
{
    if(CallerGetToolFrame.ready())
        CallerGetToolFrame.call(tool);
}

void TestSupervisor::GetWorkFrame(Frame &ref)
{
    if(CallerGetReferenceFrame.ready())
        CallerGetReferenceFrame.call(REFERENCE_FRAME::WORK, ref);
}

//! Set Alternative Supervisor Control Modes
void TestSupervisor::SetHollowControl(const bool hollow_control) {
    m_control_hollow = hollow_control;

    SetHollowVisibility(hollow_control);
}

bool TestSupervisor::SetOTGPosition(bool otgp)
{
    //! Can only enter Online Trajectory Generation Mode from STOP mode
    //! and turn off Online Trajectory Generation Mode from STOP or OTGP mode
    error_msg emsg;
    if((otgp && m_state == STATE::NORM) ||
            (!otgp && (m_state == STATE::OTGP || m_state == STATE::NORM))) {
        //! Set OTGPosition in JTC component
        bool res = CallerSetOTGPosition.call(otgp);
        if(res) {
            if(otgp) {
                m_state = STATE::OTGP;

                emsg.level = LOG_LEVEL::INFO;
                emsg.msg = "[TS] Initiated Online Joint Trajectory Generation.";
            } else {
                m_state = STATE::NORM;

                emsg.level = LOG_LEVEL::INFO;
                emsg.msg = "[TS] Stopped Online Joint Trajectory Generation.";
            }
            outport_error_msg.write(emsg);
            return true;
        }
        else {
            emsg.level = LOG_LEVEL::WARNING;
            emsg.msg = "[TS] Could not change Online Joint Trajectory Generation state in Joint Trajectory Controller.";
        }
    }
    else {
        emsg.level = LOG_LEVEL::ERROR;
        emsg.msg = "[TS] Can only switch Online Joint Trajectory Generation from a STOP state of OTGP state.";
    }
    outport_error_msg.write(emsg);
    return false;
}

bool TestSupervisor::SetOTGPosture(bool otgf)
{
    //! Can only enter Online Trajectory Generation Mode from STOP mode
    //! and turn off Online Trajectory Generation Mode from STOP or OTGP mode
    error_msg emsg;
    if((otgf && m_state == STATE::NORM) ||
            (!otgf && (m_state == STATE::OTGF || m_state == STATE::NORM))) {
        //! Set OTGPosition in JTC component
        bool res = CallerSetOTGPosition.call(otgf);
        if(res) {
            //! Signal Kinematics Module to start processing continuous info
            CallerSetOnlineTrajectoryGeneration.call(otgf);

            if(otgf) {
                m_state = STATE::OTGF;

                emsg.level = LOG_LEVEL::INFO;
                emsg.msg = "[TS] Initiated Online Posture Trajectory Generation.";
            } else {
                m_state = STATE::NORM;

                emsg.level = LOG_LEVEL::INFO;
                emsg.msg = "[TS] Stopped Online Posture Trajectory Generation.";
            }
            outport_error_msg.write(emsg);
            return true;
        }
        else {
            emsg.level = LOG_LEVEL::WARNING;
            emsg.msg = "[TS] Could not change Online Posture Trajectory Generation state in Joint Trajectory Controller.";
        }
    }
    else {
        emsg.level = LOG_LEVEL::ERROR;
        emsg.msg = "[TS] Can only switch Online Posture Trajectory Generation from a STOP state of OTGP state.";
    }
    outport_error_msg.write(emsg);
    return false;
}

//! Error Handling
void TestSupervisor::HandleErrorMsg(error_msg e) {
    RTT::error_msg e_msg;
    //Figure error level
    if(e.level % 1000 > 600) {
        //Fatal - always delivered
        e_msg.level = LOG_LEVEL::FATAL;
        e_msg.msg = e.msg;
        outport_error_msg.write(e_msg);

        ShutdownRobot();
    }
    else if(e.level % 1000 > 400 && m_report_level <= LOG_LEVEL::ERROR) {
        //Clean motion queue & stop robot
        e_msg.level = LOG_LEVEL::ERROR;
        e_msg.msg = e.msg;
        outport_error_msg.write(e_msg);

        StopRobot();
    }
    else if(e.level % 1000 > 200 && m_report_level <= LOG_LEVEL::WARNING) {
        //Warning
        e_msg.level = LOG_LEVEL::WARNING;
        e_msg.msg = e.msg;
        outport_error_msg.write(e_msg);
        return;
    }
    else if(m_report_level <= LOG_LEVEL::INFO) {
        //Info
        e_msg.level = LOG_LEVEL::INFO;
        e_msg.msg = e.msg;
        outport_error_msg.write(e_msg);
        return;
    }
}

void TestSupervisor::print(string name, Frame frame)
{
    log(Warning) << name << endlog();
    log(Warning) << frame.M.data[0] << ", "  << frame.M.data[1] << ", " << frame.M.data[2] << ", " << frame.p.data[0] << endlog();
    log(Warning) << frame.M.data[3] << ", "  << frame.M.data[4] << ", " << frame.M.data[5] << ", " << frame.p.data[1] << endlog();
    log(Warning) << frame.M.data[6] << ", "  << frame.M.data[7] << ", " << frame.M.data[8] << ", " << frame.p.data[2] << endlog();
}

ORO_CREATE_COMPONENT(TestSupervisor)
