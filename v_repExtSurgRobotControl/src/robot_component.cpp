#include "robot_component.h"
#include "../v_repExtSurgRobotControl.h"

#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>
#include <cstring>
#include <cstdlib>
#include <iostream>

using namespace RTT;
using namespace std;

RobotComponent::RobotComponent(const string &name) : TaskContext(name) {
    this->addProperty("RobotPeriod", m_period);

    this->ports()->addPort("inport_joint_positions", inport_joint_positions);
    this->ports()->addPort("outport_joint_positions", outport_joint_positions);
    this->ports()->addPort("outport_joint_velocities", outport_joint_velocities);

    this->ports()->addPort("inport_joint_positions_hollow", inport_joint_positions_hollow);
    this->ports()->addPort("outport_joint_positions_hollow", outport_joint_positions_hollow);
    this->ports()->addPort("outport_joint_velocities_hollow", outport_joint_velocities_hollow);

    //OPERATIONS
    this->addOperation("ActivateBrakes", &RobotComponent::ActivateBrakes, this, ClientThread);
    this->addOperation("SetHollowVisibility", &RobotComponent::SetHollowVisibility, this, ClientThread);
}

bool RobotComponent::configureHook() {
    // Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *VPP = new char[128];
        strcpy(VPP, WS);
        strcat(VPP, "/Properties/VREP_PLUGIN_PROPERTIES.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(VPP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] VPP;
    }

    // Set component activity
    this->setActivity( new Activity(os::LowestPriority, m_period) );

    std::vector<double> joints(7,0.0);
    outport_joint_positions.setDataSample(joints);
    outport_joint_positions_hollow.setDataSample(joints);
    outport_joint_velocities.setDataSample(joints);
    outport_joint_velocities_hollow.setDataSample(joints);

    return true;
}

bool RobotComponent::startHook() {
    return true;
}

void RobotComponent::updateHook() {
    std::vector<double> dpos, dhpos;
    std::vector<double> cpos, cvel, chpos, chvel;

    if(inport_joint_positions.read(dpos) == NewData) {
        v_repExtPQSetJointPositions(dpos);
    }

    if(inport_joint_positions_hollow.read(dhpos) == NewData) {
        v_repExtPQSetJointPositionsHollow(dhpos);
    }

    v_repExtPQGetJointPositions(cpos);
    v_repExtPQGetJointVelocities(cvel);
    v_repExtPQGetJointPositionsHollow(chpos);
    v_repExtPQGetJointVelocitiesHollow(chvel);

    outport_joint_positions.write(cpos);
    outport_joint_velocities.write(cvel);
    outport_joint_positions_hollow.write(chpos);
    outport_joint_velocities_hollow.write(chvel);
}

void RobotComponent::stopHook() {
}

void RobotComponent::cleanupHook() {
}

void RobotComponent::ActivateBrakes(const bool lock) {
    v_repExtPQActivateBrakes(lock);
}

void RobotComponent::SetHollowVisibility(const bool vis) {
    v_repExtPQSetHollowVisibility(vis);
}


