#include "test_vrep_fri-component.hpp"

#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>

#include <cstring>
#include <cstdlib>

using namespace RTT;
using namespace std;

TestVrepFri::TestVrepFri(const string &component_name) :
    TaskContext(component_name, PreOperational)
{
    //PROPERTIES
    this->addProperty("MaxTFJointVel", m_maxv).doc("Maximum allowed joint velocity");
    this->addProperty("MaxTFJointAcc", m_maxa).doc("Maximum allowed joint acceleration");

    //OPERATIONS
    this->addOperation("GetSurgeryReference", &TestVrepFri::GetSurgeryReference, this, ClientThread);
    this->addOperation("GetCurrentEndEffector", &TestVrepFri::GetCurrentEndEffector, this, ClientThread);
    this->addOperation("AttachEndEffector", &TestVrepFri::AttachEndEffector, this, ClientThread);
    this->addOperation("MoveEndEffectorTool", &TestVrepFri::MoveEndEffectorTool, this, ClientThread);

    //PORTS
    this->ports()->addPort("arch_target_joint_positions", arch_target_joint_positions).doc("Inport with target KUKA Lbr iiwa joint positions.");
    this->ports()->addPort("arch_current_joint_positions", arch_current_joint_positions).doc("Outport with current KUKA Lbr iiwa joint positions.");
    this->ports()->addPort("arch_current_joint_velocities", arch_current_joint_velocities).doc("Outport with current KUKA Lbr iiwa joint velocities.");
    this->ports()->addPort("arch_target_joint_positions_hollow", arch_target_joint_positions_hollow).doc("Inport with target KUKA hollow robot joint positions");
    this->ports()->addPort("arch_current_joint_positions_hollow", arch_current_joint_positions_hollow).doc("Outport with current KUKA hollow robot joint positions.");
    this->ports()->addPort("arch_current_joint_velocities_hollow", arch_current_joint_velocities_hollow).doc("Outport with current KUKA hollow robot joint velocities.");
    this->ports()->addPort("arch_current_endeffector_position", arch_current_endeffector_position).doc("Outport of the current end-effector position relative to flange to architecture.");

    this->ports()->addPort("vrep_target_joint_positions",vrep_target_joint_positions).doc("Outport of target KUKA Lbr iiwa joint positions to VRep");
    this->ports()->addPort("vrep_current_joint_positions",vrep_current_joint_positions).doc("Inport of target KUKA Lbr iiwa joint positions from VRep");
    this->ports()->addPort("vrep_current_joint_velocities",vrep_current_joint_velocities).doc("Inport of target KUKA Lbr iiwa joint velocities from VRep");
    this->ports()->addPort("vrep_target_joint_positions_hollow",vrep_target_joint_positions_hollow).doc("Outport of target KUKA Lbr iiwa hollow joint positions to VRep");
    this->ports()->addPort("vrep_current_joint_positions_hollow",vrep_current_joint_positions_hollow).doc("Inport of target KUKA Lbr iiwa hollow joint positions from VRep");
    this->ports()->addPort("vrep_current_joint_velocities_hollow",vrep_current_joint_velocities_hollow).doc("Inport of target KUKA Lbr iiwa hollow joint velocities from VRep");
    this->ports()->addPort("vrep_current_endeffector_position", vrep_current_endeffector_position).doc("Inport of the current end-effector position relative to flange from VRep.");

    this->ports()->addPort("outport_error_msg", outport_error_msg).doc("Output the result value and error codes of the component to task supervisor.");

    //EXTERNAL OPERATIONS
    this->provides("VF_TS")->addOperation("SetHollowVisibility",&TestVrepFri::SetHollowVisibility, this, ClientThread);
    this->provides("VF_SP")->addOperation("AddTrajectories", &TestVrepFri::AddTrajectories , this, ClientThread);
    this->provides("VF_SP")->addOperation("RemoveTrajectories", &TestVrepFri::RemoveTrajectories, this, ClientThread);
    this->provides("VF_SP")->addOperation("ShowTrajectory", &TestVrepFri::ShowTrajectory, this, ClientThread);
    this->provides("VF_SP")->addOperation("ShowTrajectories", &TestVrepFri::ShowTrajectories, this, ClientThread);
}

bool TestVrepFri::configureHook(){
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *JMP = new char[128];
        strcpy(JMP, WS);
        strcat(JMP, "/Properties/JOINT_MOTION_PROPERTIES.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(JMP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] JMP;
    }
    else
    {
        log(Error) << "Could not find WORKSPACE environment variable." << endlog();
        return false;
    }

    //Get activity period
    m_dt = this->getActivity()->getPeriod();

    //Operation Caller Interface
    if(hasPeer("robot_interface")) {
        TaskContext *kcp_ptr = getPeer("robot_interface");
        CallerSetHollowVisibility = kcp_ptr->getOperation("SetHollowVisibility");
    }
    else {
        log(Error) << "Could not connect to robot_interface over CORBA." << endlog();
        return false;
    }

    if(hasPeer("endeffector_interface")) {
        TaskContext *eep_ptr = getPeer("endeffector_interface");
        CallerGetCurrentEndEffector = eep_ptr->getOperation("GetCurrentEndEffector");
        CallerAttachEndEffector = eep_ptr->getOperation("AttachEndEffector");
        CallerMoveEndEffector = eep_ptr->getOperation("MoveEndEffector");
    }
    else {
        log(Error) << "Could not connect to endeffector_interface over CORBA." << endlog();
        return false;
    }

    if(hasPeer("trajectories_interface")) {
        TaskContext *tp_ptr = getPeer("trajectories_interface");
        CallerGetSurgeryReference = tp_ptr->getOperation("GetSurgeryReference");
        CallerAddTrajectory = tp_ptr->getOperation("AddTrajectory");
        CallerRemoveTrajectories = tp_ptr->getOperation("RemoveTrajectories");
        CallerShowTrajectory = tp_ptr->getOperation("ShowTrajectory");
        CallerShowTrajectories = tp_ptr->getOperation("ShowTrajectories");
    }
    else {
        log(Error) << "Could not connect to trajectories_interface over CORBA." << endlog();
        return false;
    }

    if(hasPeer("transformations")) {
        TaskContext *tr_ptr = getPeer("transformations");
        CallerSetWorkFrame = tr_ptr->getOperation("SetWorkFrame");
        //To send and collect data (run in transformations enging 'OwnThread')
        CallerSetWorkFrame.setCaller(tr_ptr->engine());
    }
    else {
        log(Error) << "Could not connect to transformations." << endlog();
        return false;
    }

    if(hasPeer("test_supervisor")) {
        TaskContext *ts_ptr = getPeer("test_supervisor");
        a_simulation = ts_ptr->getAttribute("m_simulation");
    }
    else {
        log(Error) << "Could not connect test_vrep_fri to test_supervisor component." << endlog();
        return false;
    }

    //Set data samples for vector ports
    std::vector<double> joints(7,0.0);
    arch_current_joint_positions.setDataSample(joints);
    arch_current_joint_velocities.setDataSample(joints);
    vrep_target_joint_positions.setDataSample(joints);

    arch_current_joint_positions_hollow.setDataSample(joints);
    arch_current_joint_velocities_hollow.setDataSample(joints);
    vrep_target_joint_positions_hollow.setDataSample(joints);

    //Setup error message data sample
    RTT::error_msg e;
    e.level = 9999;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);

    bool connections = true;
    connections &= arch_target_joint_positions.connected();
    connections &= vrep_current_joint_positions.connected();
    connections &= vrep_current_joint_velocities.connected();
    return connections;
}

bool TestVrepFri::startHook() {
    m_cjppos.assign(7,0.0);
    m_cjtvel.assign(7,0.0);

    m_cee = 0; //None
    m_brakes = false;

    m_missed_packs=0;
    last_read = false;

    m_has_trajectories = false;
    m_cycle_count = 0;

    return true;
}

void TestVrepFri::updateHook() {
    bool new_cjpos, new_cjvel, new_ceepos;

    //! Read real robot joint positions and velocities from simulator
    new_cjpos = (vrep_current_joint_positions.read(m_cjpos) == NewData);
    new_cjvel = (vrep_current_joint_velocities.read(m_cjvel) == NewData);

    if(new_cjpos && new_cjvel)
    {
        arch_current_joint_positions.write(m_cjpos);
        arch_current_joint_velocities.write(m_cjvel);
    }
    else {
        m_missed_packs++;
    }

    //! Read, real and hollow robot, target joint positions and send to simulator
    if(arch_target_joint_positions.read(m_tjpos) == NewData) {
        vrep_target_joint_positions.write(m_tjpos);
        // If a simulation run
        if(a_simulation.get()) {
            if(CheckTargetPositions(m_tjpos)) {
                vrep_target_joint_positions.write(m_tjpos);
            }
            else {
                error_msg emsg;
                emsg.level = OUTBJOINT;
                emsg.msg = "[VREPFRI] Received a target position out of limits!";
                outport_error_msg.write(emsg);

                stop();
            }
        }
        else {
            vrep_target_joint_positions.write(m_tjpos);
        }

    }
    if(arch_target_joint_positions_hollow.read(m_tjpos_hlw) == NewData) {
        vrep_target_joint_positions_hollow.write(m_tjpos_hlw);
    }

    //! Read hollow robot joint positions and velocities from simulator
    if(vrep_current_joint_positions_hollow.read(m_cjpos_hlw) == NewData) {
        arch_current_joint_positions_hollow.write(m_cjpos_hlw);
    }
    if(vrep_current_joint_velocities_hollow.read(m_cjvel_hlw) == NewData) {
        arch_current_joint_velocities_hollow.write(m_cjvel_hlw);
    }

    //! Read current end-effector position from simulator and send to architecture
    //! Probe: 3 and Trepan: 4, BaseProbe: 7 and BaseTrepan: 8
    new_ceepos = (vrep_current_endeffector_position.read(m_ceepos) == NewData);
    if(m_cee == 3 || m_cee == 4 || m_cee == 7 || m_cee == 8) {
        if(new_ceepos) {
            arch_current_endeffector_position.write(m_ceepos);
        }
    }
    else {
        arch_current_endeffector_position.write(0.0);
    }

    //! Handle the drawing of multiple trajectories
    int half_sec_count = std::max(20, static_cast<int>(std::ceil(0.5 / m_dt)));
    if(m_has_trajectories && m_cycle_count%half_sec_count==0) {
        SendSingleTrajectory();
    }

    m_cycle_count++;
}

void TestVrepFri::stopHook() {

}

void TestVrepFri::cleanupHook() {
}

void TestVrepFri::SetHollowVisibility(bool visible)
{
    CallerSetHollowVisibility.send(visible);
}

KDL::Frame TestVrepFri::GetSurgeryReference()
{
    KDL::Frame transf = KDL::Frame::Identity();
    std::vector<double> vref;
    // Obtain transformation from V-Rep
    if(CallerGetSurgeryReference.ready()) {
        CallerGetSurgeryReference.call(vref);

        KDL::Vector pos(vref[3], vref[7], vref[11]);
        KDL::Rotation rot(vref[0], vref[1], vref[2], vref[4], vref[5], vref[6], vref[8], vref[9], vref[10]);
        transf.p = pos;
        transf.M = rot;

        if(CallerSetWorkFrame.ready()) {
            CallerSetWorkFrame.call(transf);
            return transf;
        }
        else {
            error_msg e;
            e.level = NOTSENTREF;
            e.msg = m_ec.dict[NOTSENTREF];
            outport_error_msg.write(e);
            return KDL::Frame::Identity();
        }
    }
    else {
        error_msg e;
        e.level = NOTREADREF;
        e.msg = m_ec.dict[NOTREADREF];
        outport_error_msg.write(e);
        return transf;
    }
}

int TestVrepFri::GetCurrentEndEffector()
{
    int current_tool = -1;
    if(CallerGetCurrentEndEffector.ready()) {
        current_tool = CallerGetCurrentEndEffector.call();
        if(current_tool == -1) {
            error_msg e;
            e.level = NOTREADEE;
            e.msg = m_ec.dict[NOTREADEE];
            outport_error_msg.write(e);
        }
    }
    else {
        error_msg e;
        e.level = NOTREADEE;
        e.msg = m_ec.dict[NOTREADEE];
        outport_error_msg.write(e);
    }

    return current_tool;
}

bool TestVrepFri::AttachEndEffector(const int tool)
{
    if(CallerAttachEndEffector.ready()) {
        // Send information to V-Rep
        CallerAttachEndEffector.call(tool);

        m_cee = tool;
    }
    else {
        error_msg e;
        e.level = NOTATTACH;
        e.msg = m_ec.dict[NOTATTACH];
        outport_error_msg.write(e);
        return false;
    }
}

bool TestVrepFri::MoveEndEffectorTool(const double dist)
{
    if(CallerMoveEndEffector.ready()) {
        CallerMoveEndEffector.send(dist);
        return true;
    }
    else {
        error_msg e;
        e.level = NOTMOVEEE;
        e.msg = m_ec.dict[NOTMOVEEE];
        outport_error_msg.write(e);
        return false;
    }
}

void TestVrepFri::AddTrajectories(const vector_int &id, const vector_vector_double &target,
                                  const vector_vector_double &entry)
{
    m_trajectory_ids = id;
    m_trajectory_targets = target;
    m_trajectory_entries = entry;

    m_has_trajectories = true;
}

void TestVrepFri::RemoveTrajectories() {
    SendHandle<void()> handle = CallerRemoveTrajectories.send();
}

void TestVrepFri::ShowTrajectory(const int id) {
    SendHandle<void(const int)> handle = CallerShowTrajectory.send(id);
}

void TestVrepFri::ShowTrajectories() {
    SendHandle<void()> handle = CallerShowTrajectories.send();
}

bool TestVrepFri::CheckTargetPositions(const std::vector<double> &tar)
{
    for(size_t i=0; i<7; ++i) {
        double dq = std::abs(tar[i] - m_cjpos[i]);
        double max_dq = m_maxv * m_dt * 512;
        if( dq > max_dq ) {
            log(Error) << "[VREP] Prevented Motion!!!" << endlog();
            log(Error) << "Joint " << i << " moving " << dq << endlog();
            log(Error) << "Maximum allowed " << max_dq << endlog();
            log(Error) << "Maximum velocity " << m_maxv << " and dt " << m_dt << endlog();
            log(Error) << "Component stopped. Call RecoverFRI() or Relaunch the component." << endlog();

            return false;
        }
    }
    return true;
}

void TestVrepFri::PrintJointCoordinates(const char *name, std::vector<double> &var) const
{
    log(Warning) << name << ": [";
    for(size_t i=0; i<var.size()-1; ++i) {
        log(Warning) << var[i] << ", ";
    }
    log(Warning) << var.back() << "]" << endlog();
}

void TestVrepFri::SendSingleTrajectory()
{
    int id = -1;
    std::vector<double> target, entry;

    if(!m_trajectory_ids.data.empty() && !m_trajectory_targets.data.empty() && !m_trajectory_entries.data.empty())
    {
        id = m_trajectory_ids.data.front();
        target = m_trajectory_targets.data.front();
        entry = m_trajectory_entries.data.front();

        m_trajectory_ids.data.erase(m_trajectory_ids.data.begin());
        m_trajectory_targets.data.erase(m_trajectory_targets.data.begin());
        m_trajectory_entries.data.erase(m_trajectory_entries.data.begin());

        CallerAddTrajectory.call(id, target, entry);
    }
    else {
        m_has_trajectories = false;
    }
}

ORO_CREATE_COMPONENT(TestVrepFri)
