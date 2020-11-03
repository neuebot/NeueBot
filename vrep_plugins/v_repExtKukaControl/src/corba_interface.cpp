#include "corba_interface.h"
#include "robot_handler.h"

#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>

#include <iostream>

using namespace RTT;
using namespace std;

CorbaInterface::CorbaInterface(const string &name) : TaskContext(name) {
    //Attributes and Properties
//    this->addProperty("prop", m_prop).doc("THIS IS PROPERTY");

    //PORTS
    this->ports()->addEventPort("inport_joint_positions", inport_joint_positions);
    this->ports()->addPort("outport_joint_positions", outport_joint_positions);
    this->ports()->addPort("outport_joint_velocities", outport_joint_velocities);

    this->ports()->addEventPort("inport_joint_positions_hollow", inport_joint_positions_hollow);
    this->ports()->addPort("outport_joint_positions_hollow", outport_joint_positions_hollow);
    this->ports()->addPort("outport_joint_velocities_hollow", outport_joint_velocities_hollow);

    //OPERATIONS
    this->addOperation("SetHollowVisibility", &CorbaInterface::SetHollowVisibility, this, ClientThread);
    this->addOperation("GetSurgeryReference", &CorbaInterface::GetSurgeryReference, this, ClientThread);

    m_rh = std::make_shared<RobotHandler>();
}

bool CorbaInterface::configureHook() {
    m_rh->InitRobots();

    std::vector<double> joints(7,0.0);
    outport_joint_positions.setDataSample(joints);
    outport_joint_positions_hollow.setDataSample(joints);
    outport_joint_velocities.setDataSample(joints);
    outport_joint_velocities_hollow.setDataSample(joints);

    return true;
}

bool CorbaInterface::startHook() {
    return true;
}

void CorbaInterface::updateHook() {
    std::vector<double> dpos, dhpos;
    std::vector<double> cpos, cvel, chpos, chvel;

    if(inport_joint_positions.read(dpos) == NewData) {
        m_rh->SetJointPositions(dpos);
        //log(Warning) << "Read: " << dpos[0] << endlog();
    }

    if(inport_joint_positions_hollow.read(dhpos) == NewData) {
        m_rh->SetJointPositionsHollow(dhpos);
    }

    m_rh->GetJointPositions(cpos);
    m_rh->GetJointVelocities(cvel);
    m_rh->GetJointPositionsHollow(chpos);
    m_rh->GetJointVelocitiesHollow(chvel);

    outport_joint_positions.write(cpos);
    outport_joint_velocities.write(cvel);
    outport_joint_positions_hollow.write(chpos);
    outport_joint_velocities_hollow.write(chvel);
}

void CorbaInterface::stopHook() {
}

void CorbaInterface::cleanupHook() {
}

void CorbaInterface::SetHollowVisibility(const bool vis) {
    m_rh->SetHollowVisibility(vis);
}

/*
 *	KDL::Rotation (double Xx, double Yx, double Zx, double Xy, double Yy, double Zy, double Xz, double Yz, double Zz)
 *
 *  matrix: pointer to 12 simFloat values (the last row of the 4x4 matrix (0,0,0,1) is not needed)
 *    The x-axis of the orientation component is (matrix[0],matrix[4],matrix[8])
 *    The y-axis of the orientation component is (matrix[1],matrix[5],matrix[9])
 *    The z-axis of the orientation component is (matrix[2],matrix[6],matrix[10])
 *    The translation component is (matrix[3],matrix[7],matrix[11])
*/
void CorbaInterface::GetSurgeryReference(std::vector<double> &surg_ref) {
    std::vector<double> matrix;
    if(m_rh->GetSurgeryReference(matrix))
    {
        surg_ref = matrix;
//        KDL::Vector pos(matrix[3], matrix[7], matrix[11]);
//        KDL::Rotation rot(matrix[0], matrix[1], matrix[2], matrix[4], matrix[5], matrix[6], matrix[8], matrix[9], matrix[10]);
//        surg_ref.p = pos;
//        surg_ref.M = rot;
    }
}


