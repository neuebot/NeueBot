#include "robot_state-component.hpp"
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>

#include <cstring>
#include <cstdlib>

using namespace RTT;
using namespace std;

RobotState::RobotState(const string &component_name) : 
    TaskContext(component_name, PreOperational)
{
    this->addAttribute("m_cx", m_cx);
    this->addAttribute("m_cv", m_cv);
    this->addAttribute("m_ctv", m_ctv);
    this->addAttribute("m_diff", m_diff);

    this->addAttribute("m_nsparam", m_nsparam);
    this->addAttribute("m_frame_pos", m_frame_pos);
    this->addAttribute("m_frame_rot", m_frame_rot);

    this->addProperty("NJoints", m_nj).doc("Number of joints.");
    this->addProperty("Tolerance", m_tol).doc("Calculations tolerance");
    this->addProperty("LinkLength", m_lnk).doc("Robot Link Length");
    this->addProperty("JointLimits", m_lim).doc("Maximum allowed joint position.");

    this->addProperty("Denavit-Hartenberg-A", m_dh_a).doc("Displacement from one Frame to the next along x-axis.");
    this->addProperty("Denavit-Hartenberg-Alpha", m_dh_alpha).doc("Rotation from one Frame to the next in turn of the previous x-axis.");
    this->addProperty("Denavit-Hartenberg-D", m_dh_d).doc("Displacement from one Frame to the next along z-axis.");
    this->addProperty("Denavit-Hartenberg-Theta", m_dh_theta).doc("Rotation from one Frame to the next in turn of the previous z-axis.");

    this->addOperation("SetFlangeToolFrame", &RobotState::SetFlangeToolFrame, this, OwnThread);
    this->addOperation("SetWorkBaseFrame", &RobotState::SetWorkBaseFrame, this, OwnThread);
    this->addOperation("GetFlangeToolFrame", &RobotState::GetFlangeToolFrame, this, OwnThread);
    this->addOperation("GetWorkBaseFrame", &RobotState::GetWorkBaseFrame, this, OwnThread);

    this->ports()->addPort("inport_current_joint_positions", inport_current_joint_positions).doc("Input of the current joint positions (rad)");
    this->ports()->addPort("inport_current_joint_velocities", inport_current_joint_velocities).doc("Input of the current joint velocities (rad/s)");
    this->ports()->addPort("inport_current_joint_positions_hollow", inport_current_joint_positions_hollow).doc("Input of the current hollow joint positions (rad)");
    this->ports()->addPort("inport_robot_moving", inport_robot_moving).doc("Input of current motion state (boolean)");
    this->ports()->addPort("outport_robot_posture", outport_robot_posture).doc("Output of the robot parameters (frame, nsparam, rconf) - FK");

    this->ports()->addPort("outport_current_joint_positions", outport_current_joint_positions).doc("Output of the current joint positions (rad)");
    this->ports()->addPort("outport_current_joint_velocities", outport_current_joint_velocities).doc("Output of the current joint velocities (rad/s)");
    this->ports()->addPort("outport_current_joint_positions_hollow", outport_current_joint_positions_hollow).doc("Output of the current hollow joint positions (rad)");
    this->ports()->addPort("outport_robot_moving", outport_robot_moving);
}

bool RobotState::configureHook(){
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *RSP = new char[128];
        strcpy(RSP, WS);
        strcat(RSP, "/Properties/ROBOT_STRUCTURE_PROPERTIES.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(RSP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] RSP;
    }

    //Marshall Denavit-Hartenberg parameters
    m_dh.assign(m_nj, vector<double>(4,0.0));
    for(size_t j=0; j<m_nj; ++j) {
        m_dh[j][0] = m_dh_a[j];
        m_dh[j][1] = m_dh_alpha[j];
        m_dh[j][2] = m_dh_d[j];;
        m_dh[j][3] = m_dh_theta[j];;
    }
    m_frame_pos.assign(3,0.0);
    m_frame_rot.assign(3,0.0);
    m_kinematic_solver = std::make_shared<ForwardKinematicSolver>(m_nj, m_tol, m_lnk, m_lim, m_dh);

    bool connections = true;
    connections &= inport_current_joint_positions.connected();
    connections &= inport_current_joint_velocities.connected();
    connections &= inport_robot_moving.connected();
    return connections;
}

bool RobotState::startHook(){
    m_diff = std::vector<double>(7, 0.0);

    return true;
}

void RobotState::updateHook(){
    bool moving;
    vector<double> current_joint_positions, current_joint_velocities, current_joint_positions_hollow;
    bool new_cjpos, new_cjvel, new_cjpos_hlw;

    //! Read every cycle
    new_cjpos = (inport_current_joint_positions.read(current_joint_positions) == NewData);
    new_cjvel = (inport_current_joint_velocities.read(current_joint_velocities) == NewData);
    new_cjpos_hlw = (inport_current_joint_positions_hollow.connected() &&
                     inport_current_joint_positions_hollow.read(current_joint_positions_hollow) == NewData);

    if(inport_robot_moving.read(moving) == NewData) {
        outport_robot_moving.write(moving);
    }

    if(new_cjpos && new_cjvel) {
        m_cx = current_joint_positions;
        m_cv = current_joint_velocities;

        posture msg_robot_param;
        m_kinematic_solver->ForwardKinematics(m_cx, m_rconf, m_nsparam, m_frame);
        m_frame_pos[0] = m_frame.p.x();
        m_frame_pos[1] = m_frame.p.y();
        m_frame_pos[2] = m_frame.p.z();
        m_frame.M.GetRPY(m_frame_rot[0],m_frame_rot[1],m_frame_rot[2]);
        msg_robot_param.frame = KDL::Frame(m_frame);
        msg_robot_param.psi = m_nsparam;
        msg_robot_param.GC = m_rconf;

        outport_current_joint_positions.write(m_cx);
        outport_current_joint_velocities.write(m_cv);
        outport_robot_posture.write(msg_robot_param);
    }

    if(new_cjpos_hlw) {
        outport_current_joint_positions_hollow.write(current_joint_positions_hollow);
    }
}

void RobotState::stopHook() {
}

void RobotState::cleanupHook() {
}

void RobotState::SetFlangeToolFrame(const KDL::Frame ft)
{
    lock_guard<mutex> lk(m_mutex_ft);
    m_flange_tool_frame = ft;
}

void RobotState::SetWorkBaseFrame(const KDL::Frame wb)
{
    lock_guard<mutex> lk(m_mutex_wb);
    m_work_base_frame = wb;
}

KDL::Frame RobotState::GetFlangeToolFrame()
{
    lock_guard<mutex> lk(m_mutex_ft);
    return m_flange_tool_frame;
}

KDL::Frame RobotState::GetWorkBaseFrame()
{
    lock_guard<mutex> lk(m_mutex_wb);
    return m_work_base_frame;
}

ORO_CREATE_COMPONENT(RobotState)
