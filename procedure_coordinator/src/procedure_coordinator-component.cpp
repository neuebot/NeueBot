#include "procedure_coordinator-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/marsh/Marshalling.hpp>

#include <eigen3/Eigen/Dense>
#include <cstring>
#include <cstdlib>

using namespace KDL;
using namespace RTT;
using namespace Eigen;
using namespace std;

ProcedureCoordinator::ProcedureCoordinator(const string &component_name) :
    TaskContext(component_name, PreOperational),
    //TASK_SUPERVISOR
    CallSetToolType("SetToolType"),
    CallGetToolType("GetToolType"),
    //VREP_FRI
    CallGetSurgeryReference("GetSurgeryReference"),
    CallMoveEndEffectorTool("MoveEndEffectorTool")
{
    //PROPERTIES AND ATTRIBUTES
    this->addProperty("ApproachDistance", m_approach_distance);
    this->addProperty("DrillDistance", m_drill_distance);
    this->addProperty("RetractDistance", m_retract_distance);
    this->addProperty("ProbeDistance", m_probe_distance);
    this->addProperty("SafeDistance", m_safe_distance);
    this->addProperty("TrajectoryStep", m_trajectory_step);
    this->addProperty("NullspaceStep", m_nullspace_step);

    this->addAttribute("CurrentConfiguration", m_conf);

    //PORTS
    this->ports()->addEventPort("inport_robot_moving", inport_robot_moving);
    this->ports()->addEventPort("inport_robot_posture", inport_robot_posture);
    this->ports()->addEventPort("inport_error_msg", inport_error_msg);
    this->ports()->addPort("outport_error_msg", outport_error_msg);

    //OPERATIONS
    this->addOperation("SelectSurgicalTrajectory", &ProcedureCoordinator::SelectSurgicalTrajectory, this, OwnThread);
    this->addOperation("LockSurgicalTrajectory", &ProcedureCoordinator::LockSurgicalTrajectory, this, OwnThread);
    this->addOperation("ApproachSurgicalTrajectory", &ProcedureCoordinator::ApproachSurgicalTrajectory, this, OwnThread);
    this->addOperation("SkullDrillStep", &ProcedureCoordinator::SkullDrillStep, this, OwnThread);
    this->addOperation("RetractStep", &ProcedureCoordinator::RetractStep, this, OwnThread);
    this->addOperation("InsertProbeStep", &ProcedureCoordinator::InsertProbeStep, this, OwnThread);
    this->addOperation("ReturnBasePose", &ProcedureCoordinator::ReturnBasePose, this, OwnThread);

    this->addOperation("ConfirmSurgeryPlan", &ProcedureCoordinator::ConfirmSurgeryPlan , this, ClientThread);
    this->addOperation("RetrieveSurgeryPlan", &ProcedureCoordinator::RetrieveSurgeryPlan , this, ClientThread);
    this->addOperation("RemoveAllSurgeryTrajectories", &ProcedureCoordinator::RemoveAllSurgeryTrajectories, this, ClientThread);
    this->addOperation("ShowSurgeryTrajectory", &ProcedureCoordinator::ShowSurgeryTrajectory, this, ClientThread);
    this->addOperation("ShowAllSurgeryTrajectories", &ProcedureCoordinator::ShowAllSurgeryTrajectories, this, ClientThread);
    this->addOperation("LoadSurgeryPlanFile", &ProcedureCoordinator::LoadSurgeryPlanFile, this, ClientThread);
    this->addOperation("SaveSurgeryPlanFile", &ProcedureCoordinator::SaveSurgeryPlanFile, this, ClientThread);

    this->addOperation("LoadWorkPiecePoints", &ProcedureCoordinator::LoadWorkPiecePoints, this, ClientThread);
    this->addOperation("LoadRegistrationPoints", &ProcedureCoordinator::LoadRegistrationPoints, this, ClientThread);
    this->addOperation("ComputeRegistrationMatrix", &ProcedureCoordinator::ComputeRegistrationMatrix, this, ClientThread);
    this->addOperation("SaveInterlockRegistrationMatrix", &ProcedureCoordinator::SaveInterlockRegistrationMatrix, this, ClientThread);
    this->addOperation("LoadCalibrationFile", &ProcedureCoordinator::LoadCalibrationFile, this, ClientThread);
    this->addOperation("SaveCalibrationFile", &ProcedureCoordinator::SaveCalibrationFile, this, ClientThread);
    this->addOperation("IsRegistrationComplete", &ProcedureCoordinator::IsRegistrationComplete, this, OwnThread);
    this->addOperation("IsSurgeryPlanSetup", &ProcedureCoordinator::IsSurgeryPlanSetup, this, OwnThread);

    //REMOTE OPERATION CALLERS
    this->requires("VF_GUI")->addOperationCaller(CallGetSurgeryReference);
    this->requires("VF_GUI")->addOperationCaller(CallMoveEndEffectorTool);

    this->requires("TS_PC")->addOperationCaller(CallSetToolType);
    this->requires("TS_PC")->addOperationCaller(CallGetToolType);
}

bool ProcedureCoordinator::configureHook() {
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *PC = new char[128];
        strcpy(PC, WS);
        strcat(PC, "/Properties/PROCEDURE_CONSTANTS.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(PC)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] PC;
    }

    /// How to connect peers through CORBA
//    TaskContext *tvf_ptr = corba::TaskContextProxy::Create("test_vrep_fri");
//    TaskContext *ts_ptr = corba::TaskContextProxy::Create("test_supervisor");
//    this->addPeer(tvf_ptr);
//    this->addPeer(ts_ptr);

    if(hasPeer("test_supervisor")) {
        TaskContext *ts_ptr = getPeer("test_supervisor");
        CallSetRelativeVelocity = ts_ptr->getOperation("SetRelativeVelocity");
        CallJointMotion = ts_ptr->getOperation("JointMotion");
        CallCoordinateJointMotion = ts_ptr->getOperation("CoordinateJointMotion");
        CallAdjustedCoordinateJointMotion = ts_ptr->getOperation("AdjustedCoordinateJointMotion");
        CallLinearMotion = ts_ptr->getOperation("LinearMotion");
        CallArcMotionFrame = ts_ptr->getOperation("ArcMotionFrame");
        CallComposedMotion = ts_ptr->getOperation("ComposedMotion");

        CallApproachMotion = ts_ptr->getOperation("ApproachMotion");
        CallReturnMotion = ts_ptr->getOperation("ReturnMotion");
        CallIncrementMotion = ts_ptr->getOperation("IncrementMotion");
        CallAnalyseLinearMotion = ts_ptr->getOperation("AnalyseLinearMotion");
        CallAnalyseArcMotionFrame = ts_ptr->getOperation("AnalyseArcMotionFrame");
        //To send and collect data (run in transformations enging 'OwnThread')
        CallSetRelativeVelocity.setCaller(ts_ptr->engine());
        CallJointMotion.setCaller(ts_ptr->engine());
        CallCoordinateJointMotion.setCaller(ts_ptr->engine());
        CallAdjustedCoordinateJointMotion.setCaller(ts_ptr->engine());
        CallLinearMotion.setCaller(ts_ptr->engine());
        CallArcMotionFrame.setCaller(ts_ptr->engine());
        CallIncrementMotion.setCaller(ts_ptr->engine());
        CallApproachMotion.setCaller(ts_ptr->engine());
        CallReturnMotion.setCaller(ts_ptr->engine());
        CallAnalyseLinearMotion.setCaller(ts_ptr->engine());
        CallAnalyseArcMotionFrame.setCaller(ts_ptr->engine());

        /// Just wasted an entire morning to understand that the framework does not allow remote calls
        /// to proxy methods that return an int and have no parameters.
        /// Says that it Expects 0 arguments and receives 0 arguments... go figure
        /// Anyway, if the method is bundled in a Service, this no longer happens.
        this->requires("TS_PC")->connectTo( ts_ptr->provides("TS_PC") );
        if(!requires("TS_PC")->ready()) {
            log(Error) << "Failed to require test_supervisor sub-service 'TS_PC'." << endlog();
            return false;
        }
    }
    else {
        log(Error) << "Could not connect procedure_coordinator to test_supervisor" << endlog();
    }

    if(hasPeer("test_vrep_fri")) {
        TaskContext *tvf_ptr = getPeer("test_vrep_fri");
        CallGetSurgeryReference = tvf_ptr->getOperation("GetSurgeryReference");
        CallMoveEndEffectorTool = tvf_ptr->getOperation("MoveEndEffectorTool");
    }

    if(hasPeer("registration")) {
        TaskContext *reg_ptr = getPeer("registration");
        CallLoadWorkPiecePoints = reg_ptr->getOperation("LoadWorkPiecePoints");
        CallLoadRegistrationPoints = reg_ptr->getOperation("LoadRegistrationPoints");
        CallComputeRegistrationMatrix = reg_ptr->getOperation("ComputeRegistrationMatrix");
        CallSaveInterlockRegistrationMatrix = reg_ptr->getOperation("SaveInterlockRegistrationMatrix");
        CallLoadCalibrationFile = reg_ptr->getOperation("LoadCalibrationFile");
        CallSaveCalibrationFile = reg_ptr->getOperation("SaveCalibrationFile");
        CallGetRegistrationMatrix = reg_ptr->getOperation("GetRegistrationMatrix");
        //To send and collect data (run in transformations enging 'OwnThread')
        CallComputeRegistrationMatrix.setCaller(reg_ptr->engine());
        CallSaveInterlockRegistrationMatrix.setCaller(reg_ptr->engine());
        CallLoadCalibrationFile.setCaller(reg_ptr->engine());
        CallSaveCalibrationFile.setCaller(reg_ptr->engine());
        CallGetRegistrationMatrix.setCaller(reg_ptr->engine());

        a_registration_successful = reg_ptr->attributes()->getAttribute("registration_successful");
    }
    else {
        log(Error) << "Could not connect to Registration component." << endlog();
        return false;
    }

    if(hasPeer("surgery_plan")) {
        TaskContext *sp_ptr = getPeer("surgery_plan");
        CallConfirmSurgeryPlan = sp_ptr->getOperation("ConfirmSurgeryPlan");
        CallRetrieveSurgeryPlan = sp_ptr->getOperation("RetrieveSurgeryPlan");
        CallRemoveAllTrajectories = sp_ptr->getOperation("RemoveAllTrajectories");
        CallShowSurgeryTrajectory = sp_ptr->getOperation("ShowSurgeryTrajectory");
        CallShowAllTrajectories = sp_ptr->getOperation("ShowAllTrajectories");
        CallLoadSurgeryPlanFile = sp_ptr->getOperation("LoadSurgeryPlanFile");
        CallSaveSurgeryPlanFile = sp_ptr->getOperation("SaveSurgeryPlanFile");
        CallSelectSurgeryTrajectory = sp_ptr->getOperation("SelectSurgeryTrajectory");
        CallVerifyTrajectory = sp_ptr->getOperation("VerifyTrajectory");
        CallGetSurgeryTrajectoryParameters = sp_ptr->getOperation("GetSurgeryTrajectoryParameters");
        //To send and collect data (run in transformations enging 'OwnThread')
        CallConfirmSurgeryPlan.setCaller(sp_ptr->engine());
        CallRetrieveSurgeryPlan.setCaller(sp_ptr->engine());
        CallRemoveAllTrajectories.setCaller(sp_ptr->engine());
        CallShowSurgeryTrajectory.setCaller(sp_ptr->engine());
        CallShowAllTrajectories.setCaller(sp_ptr->engine());
        CallLoadSurgeryPlanFile.setCaller(sp_ptr->engine());
        CallSaveSurgeryPlanFile.setCaller(sp_ptr->engine());
        CallSelectSurgeryTrajectory.setCaller(sp_ptr->engine());
        CallVerifyTrajectory.setCaller(sp_ptr->engine());
        CallGetSurgeryTrajectoryParameters.setCaller(sp_ptr->engine());

        a_surgery_plan_setup = sp_ptr->attributes()->getAttribute("surgery_plan_setup");
    }
    else {
        log(Error) << "Could not connect to Surgery Plan component." << endlog();
        return false;
    }

    //Set Data Sample
    error_msg e;
    e.level = 0;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);

    //Custom Variables
    m_moving = false;
    m_trajectory_selected = -1;

    bool connections = true;
    connections &= inport_robot_moving.connected();
    connections &= inport_robot_posture.connected();
    connections &= inport_error_msg.connected();

    return connections;
}

bool ProcedureCoordinator::startHook(){
    //GUI Report Level
    m_report_level = LOG_LEVEL::INFO;

    return true;
}

void ProcedureCoordinator::updateHook(){
    bool moving;
    RTT::posture post;
    error_msg e;

    if(inport_robot_moving.read(moving) == NewData) {
        m_moving = moving;
    }

    if(inport_robot_posture.read(post) == NewData) {
        m_pose = post.frame;
        m_conf = post.GC;
        m_psi = post.psi;
    }

    if(inport_error_msg.read(e) == NewData) {
        HandleErrorMsg(e);
    }
}

void ProcedureCoordinator::stopHook() {
}

void ProcedureCoordinator::cleanupHook() {
}

void ProcedureCoordinator::SelectSurgicalTrajectory(const int id)
{
    sh_select_surgery_trajectory = CallSelectSurgeryTrajectory.send(id);
}

RTT::approach_scores ProcedureCoordinator::LockSurgicalTrajectory(int id) {
    error_msg emsg;
    bool select, verify;
    RTT::approach_scores ascores;
    ascores.possible = false;

    if(sh_select_surgery_trajectory.collect(select) == SendStatus::SendSuccess) {
        if(select) {
            RTT::vector_frame frames;
            RTT::vector_int conf;
            std::vector<double> psi;
            RTT::vector_vector_double scores;

            verify = CallVerifyTrajectory.call(id, frames, conf, psi, scores);

            if(conf.data.empty()) {
                m_trajectory_selected = -1;

                ascores.possible = false;
                emsg.level = NOTFEASB;
                emsg.msg = m_ec.dict[NOTFEASB];
                HandleErrorMsg(emsg);
            }
            else {
                m_trajectory_selected = id;

                ascores.possible = true;
                ascores.Frames = frames.data;
                ascores.GC = conf.data;
                ascores.Psi = psi;
                ascores.Scores = scores.data;

                emsg.level = FEASIBLE;
                emsg.msg = m_ec.dict[FEASIBLE];
                HandleErrorMsg(emsg);
            }
        }
    }
    else {
        m_trajectory_selected = -1;

        ascores.possible = false;
        emsg.level = SENDSELE;
        emsg.msg = m_ec.dict[SENDSELE];
        HandleErrorMsg(emsg);
    }

    return ascores;
}

int ProcedureCoordinator::ApproachSurgicalTrajectory(const Frame &frame, const int conf, const double psi, const double rel_vel,
                                                     redundancy_scores &red_scrs)
{
    //TODO: Get frame from surgical plan according to Current GC and psi!
    // Check if arc motion to there is possible
    int method_out = 2; // impossible
    if(!m_moving) {
        int ref_id = 3; //Surgery Frame
        CallSetRelativeVelocity.call(rel_vel);
        //Check if empty is attached
        int tool_id = CallGetToolType.call();
        //TODO: Enums must be xml loaded files
        if(tool_id == 6) { //BaseButtons end-effector: 6
            //Check if a trajectory is selected
            if(m_trajectory_selected >= 0) {
                KDL::Frame reg_mat;
                bool success = CallGetRegistrationMatrix(reg_mat);

                m_current_trajectory = frame;

                //Frame from base to flange
                if(success) {
                    approach_pose = m_pose;
                    approach_psi = psi;

                    bool res = CallApproachMotion(frame, conf, psi, ref_id, tool_id);
                    if(res)
                        method_out = 0;
                }
            }
            else {
                error_msg emsg;
                emsg.level = NOTRJSEL;
                emsg.msg = m_ec.dict[NOTRJSEL];
                HandleErrorMsg(emsg);
            }
        }
        else {
            error_msg emsg;
            emsg.level = ATTEEAPP;
            emsg.msg = m_ec.dict[ATTEEAPP];
            HandleErrorMsg(emsg);
        }
    }
    else {
        error_msg emsg;
        emsg.level = ROBOTMVN;
        emsg.msg = m_ec.dict[ROBOTMVN];
        HandleErrorMsg(emsg);
    }

    return method_out;
}

int ProcedureCoordinator::ArcApproachSurgicalTrajectory(const Frame &frame, const double rel_vel, RTT::redundancy_scores &red_scrs) {
    //TODO: Get frame from surgical plan according to Current GC and psi!
    // Check if arc motion to there is possible
    int method_out = 2; // impossible
    if(!m_moving) {
        int ref_id = 3; //Surgery Frame
        CallSetRelativeVelocity.call(rel_vel);
        //Check if empty is attached
        int tool_id = CallGetToolType.call();
        //TODO: Enums must be xml loaded files
        if(tool_id == 6) { //BaseButtons end-effector: 6
            //Check if a trajectory is selected
            if(m_trajectory_selected >= 0) {
                KDL::Frame reg_mat;
                bool success = CallGetRegistrationMatrix(reg_mat);

                m_current_trajectory = frame;


                //Frame from base to flange
                if(success) {
                    RTT::redundancy_scores rs;
                    bool analyse = CallAnalyseArcMotionFrame(Vector(0.0, 0.0, 0.0), frame, ref_id, tool_id, rs);
                    if(analyse) {
                        //Checks nullspace solutions
                        switch (rs.res) {
                        case 0: //Feasible from current configuration and arm angle
                            approach_pose = m_pose;
                            approach_psi = m_psi;

                            CallArcMotionFrame.call(Vector(0.0, 0.0, 0.0), frame, ref_id, tool_id);
                            method_out = 0;
                            break;
                        case 1: //Require arm angle adjustment - Prompt user to adjust the initial arm angle
                            red_scrs.res = rs.res;
                            red_scrs.GC = rs.GC;
                            red_scrs.PsiIntervals = rs.PsiIntervals;
                            method_out = 1; //User-input
                            break;
                        case 2: //Not feasible as a task space trajectory.
                        case 3: //Executed as a joint space motion instead.
                            approach_pose = m_pose;
                            approach_psi = m_psi;

                            CallAdjustedCoordinateJointMotion.call(frame, ref_id, tool_id);
                            method_out = 0;
                            break;
                        }
                    }
                    else {
                        //!TODO: Message could not analyse arc motion
                        error_msg emsg;
                        emsg.level = ERRANLYS;
                        emsg.msg = m_ec.dict[NOTRJSEL];
                        HandleErrorMsg(emsg);
                    }
                }
            }
            else {
                error_msg emsg;
                emsg.level = NOTRJSEL;
                emsg.msg = m_ec.dict[NOTRJSEL];
                HandleErrorMsg(emsg);
            }
        }
        else {
            error_msg emsg;
            emsg.level = ATTEEAPP;
            emsg.msg = m_ec.dict[ATTEEAPP];
            HandleErrorMsg(emsg);
        }
    }
    else {
        error_msg emsg;
        emsg.level = ROBOTMVN;
        emsg.msg = m_ec.dict[ROBOTMVN];
        HandleErrorMsg(emsg);
    }

    return method_out;
}

void ProcedureCoordinator::SkullDrillStep(const double rel_vel) {
    if(!m_moving) {
        int ref_id = 3; //Surgery Frame
        CallSetRelativeVelocity.call(rel_vel);
        //Check if empty is attached
        int tool_id = CallGetToolType.call();
        //TODO: Enums must be xml loaded files
        if(tool_id == 6) { //BaseButtons end-effector: 6
            //Check if a trajectory is selected
            if(m_trajectory_selected >= 0) {
                //Get current trajectory info
                std::vector<double> target, entry;
                CallGetSurgeryTrajectoryParameters.call(m_trajectory_selected, target, entry);
                Vector kdl_tar = Vector(target[0],target[1],target[2]);
                Vector kdl_ent = Vector(entry[0],entry[1],entry[2]);
                double d_et = (kdl_tar - kdl_ent).Norm();
                //Apply end-effector transformation
                KDL::Frame tr_drill;
                tr_drill = m_current_trajectory;
                tr_drill.p = kdl_tar - tr_drill.M.UnitZ() * ( m_drill_distance + d_et );

                auto handle = CallLinearMotion.send(tr_drill, ref_id, tool_id);
            }
            else {
                error_msg emsg;
                emsg.level = NOTRJSEL;
                emsg.msg = m_ec.dict[NOTRJSEL];
                HandleErrorMsg(emsg);
            }
        }
        else {
            error_msg emsg;
            emsg.level = ATTEEDRI;
            emsg.msg = m_ec.dict[ATTEEDRI];
            HandleErrorMsg(emsg);
        }
    }
    else {
        error_msg emsg;
        emsg.level = ROBOTMVN;
        emsg.msg = m_ec.dict[ROBOTMVN];
        HandleErrorMsg(emsg);
    }
}

void ProcedureCoordinator::RetractStep(const double rel_vel) {
    if(!m_moving) {
        int ref_id = 3; //Surgery Frame
        CallSetRelativeVelocity.call(rel_vel);
        //Check if empty is attached
        int tool_id = CallGetToolType.call();
        //Check if a trajectory is selected
        if(m_trajectory_selected >= 0) {
            //Get current trajectory info
            std::vector<double> target, entry;
            CallGetSurgeryTrajectoryParameters.call(m_trajectory_selected, target, entry);
            Vector kdl_tar = Vector(target[0],target[1],target[2]);
            Vector kdl_ent = Vector(entry[0],entry[1],entry[2]);
            double d_et = (kdl_tar - kdl_ent).Norm();
            //Apply end-effector transformation
            KDL::Frame tr_retract;
            tr_retract = m_current_trajectory;
            tr_retract.p = kdl_tar - tr_retract.M.UnitZ() * ( m_retract_distance + d_et );

            CallLinearMotion.call(tr_retract, ref_id, tool_id);
        }
        else {
            error_msg emsg;
            emsg.level = NOTRJSEL;
            emsg.msg = m_ec.dict[NOTRJSEL];
            HandleErrorMsg(emsg);
        }
    }
    else {
        error_msg emsg;
        emsg.level = ROBOTMVN;
        emsg.msg = m_ec.dict[ROBOTMVN];
        HandleErrorMsg(emsg);
    }
}

void ProcedureCoordinator::InsertProbeStep(const double rel_vel) {
    if(!m_moving) {
        int ref_id = 3; //Surgery Frame
        CallSetRelativeVelocity.call(rel_vel);
        //Check if empty is attached
        int tool_id = CallGetToolType.call();
        //TODO: Enums must be xml loaded files
        if(tool_id == 6) { //BaseButtons end-effector: 6
            //Check if a trajectory is selected
            if(m_trajectory_selected >= 0) {
                //Get current trajectory info
                std::vector<double> target, entry;
                CallGetSurgeryTrajectoryParameters.call(m_trajectory_selected, target, entry);
                Vector kdl_tar = Vector(target[0],target[1],target[2]);
                Vector kdl_ent = Vector(entry[0],entry[1],entry[2]);
                double d_et = (kdl_tar - kdl_ent).Norm();
                //Apply end-effector transformation
                KDL::Frame tr_probe;
                tr_probe = m_current_trajectory;
                /// !!! The target-entry distance is not included in the probe position calculation
                /// as this position is computed from the target distance directly.
                tr_probe.p = kdl_tar - tr_probe.M.UnitZ() * ( m_probe_distance );

                CallLinearMotion.call(tr_probe, ref_id, tool_id);
            }
            else {
                error_msg emsg;
                emsg.level = NOTRJSEL;
                emsg.msg = m_ec.dict[NOTRJSEL];
                HandleErrorMsg(emsg);
            }
        }
        else {
            error_msg emsg;
            emsg.level = ATTEEPRB;
            emsg.msg = m_ec.dict[ATTEEPRB];
            HandleErrorMsg(emsg);
        }
    }
    else {
        error_msg emsg;
        emsg.level = ROBOTMVN;
        emsg.msg = m_ec.dict[ROBOTMVN];
        HandleErrorMsg(emsg);
    }
}

void ProcedureCoordinator::ReturnBasePose(const double rel_vel)
{
    if(!m_moving) {
        int ref_id = 3; //Surgery Frame
        CallSetRelativeVelocity.call(rel_vel);
        //Check if empty is attached
        int tool_id = CallGetToolType.call();
        //TODO: Enums must be xml loaded files
        if(tool_id == 6) { //BaseButtons end-effector: 6
            //Check if a trajectory is selected
            if(m_trajectory_selected >= 0) {
                //Get current trajectory info
                std::vector<double> target, entry;
                CallGetSurgeryTrajectoryParameters.call(m_trajectory_selected, target, entry);
                Vector kdl_tar = Vector(target[0],target[1],target[2]);
                Vector kdl_ent = Vector(entry[0],entry[1],entry[2]);
                double d_et = (kdl_tar - kdl_ent).Norm();
                //Apply end-effector transformation
                KDL::Frame tr_retract;
                tr_retract = m_current_trajectory;
                tr_retract.p = kdl_tar - tr_retract.M.UnitZ() * ( m_safe_distance + d_et );

                CallReturnMotion.call(tr_retract, ref_id, tool_id, approach_pose, approach_psi);
            }
            else {
                error_msg emsg;
                emsg.level = NOTRJSEL;
                emsg.msg = m_ec.dict[NOTRJSEL];
                HandleErrorMsg(emsg);
            }
        }
        else {
            error_msg emsg;
            emsg.level = ATTEEAPP;
            emsg.msg = m_ec.dict[ATTEEAPP];
            HandleErrorMsg(emsg);
        }
    }
    else {
        error_msg emsg;
        emsg.level = ROBOTMVN;
        emsg.msg = m_ec.dict[ROBOTMVN];
        HandleErrorMsg(emsg);
    }
}

bool ProcedureCoordinator::IsRegistrationComplete()
{
    return a_registration_successful.get();
}

bool ProcedureCoordinator::IsSurgeryPlanSetup()
{
    return a_surgery_plan_setup.get();
}

/// Surgery Plan proxy methods
bool ProcedureCoordinator::ConfirmSurgeryPlan(const vector_int &id, const vector_vector_double &target,
                                              const vector_vector_double &entry) {
    bool success = CallConfirmSurgeryPlan.call(id,target,entry);
    return success;
}

bool ProcedureCoordinator::RetrieveSurgeryPlan(vector_int &id, vector_vector_double &target, vector_vector_double &entry)
{
    bool success = CallRetrieveSurgeryPlan.call(id,target,entry);
    return success;
}

void ProcedureCoordinator::RemoveAllSurgeryTrajectories() {
    CallRemoveAllTrajectories.send();
}

bool ProcedureCoordinator::ShowSurgeryTrajectory(const int id) {
    return CallShowSurgeryTrajectory.call(id);
}

void ProcedureCoordinator::ShowAllSurgeryTrajectories() {
    CallShowAllTrajectories.send();
}

bool ProcedureCoordinator::LoadSurgeryPlanFile(const string &filename, vector_int &id, vector_vector_double &target, vector_vector_double &entry)
{
    return CallLoadSurgeryPlanFile.call(filename, id, target, entry);
}

bool ProcedureCoordinator::SaveSurgeryPlanFile(const string &filename)
{
    return CallSaveSurgeryPlanFile.call(filename);
}

/// Registration component proxy methods
void ProcedureCoordinator::LoadWorkPiecePoints(const vector_vector_double &wpoints) {
    CallLoadWorkPiecePoints.send(wpoints);
}

void ProcedureCoordinator::LoadRegistrationPoints(const vector_vector_double &rpoints) {
    CallLoadRegistrationPoints.send(rpoints);
}

bool ProcedureCoordinator::ComputeRegistrationMatrix(Frame &transformation, double &error) {
    bool success = CallComputeRegistrationMatrix.call(transformation, error);
    return success;
}

bool ProcedureCoordinator::SaveInterlockRegistrationMatrix(const Frame &base_tool, Frame &transformation)
{
    bool success = CallSaveInterlockRegistrationMatrix.call(base_tool, transformation);
    return success;
}

bool ProcedureCoordinator::LoadCalibrationFile(const string &filename, Frame &transformation, double &cum_err)
{
    return CallLoadCalibrationFile.call(filename, transformation, cum_err);
}

bool ProcedureCoordinator::SaveCalibrationFile(const string &filename)
{
    return CallSaveCalibrationFile.call(filename);
}

/// Error Handling
void ProcedureCoordinator::HandleErrorMsg(error_msg e) {
    RTT::error_msg e_msg;
    //Figure error level
    if(e.level % 1000 > 600) {
        //Fatal - always delivered
        e_msg.level = LOG_LEVEL::FATAL;
        e_msg.msg = e.msg;
        outport_error_msg.write(e_msg);

        CallShutdownRobot.call();
    }
    else if(e.level % 1000 > 400 && m_report_level <= LOG_LEVEL::ERROR) {
        //Clean motion queue & stop robot
        e_msg.level = LOG_LEVEL::ERROR;
        e_msg.msg = e.msg;
        outport_error_msg.write(e_msg);

        CallStopRobot.call();
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

ORO_CREATE_COMPONENT(ProcedureCoordinator)
