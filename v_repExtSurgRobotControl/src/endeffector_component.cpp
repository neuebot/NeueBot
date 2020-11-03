#include "endeffector_component.h"
#include "../v_repExtSurgRobotControl.h"

#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>
#include <cstring>
#include <cstdlib>
#include <iostream>

using namespace RTT;
using namespace std;

EndEffectorComponent::EndEffectorComponent(const string &name) : TaskContext(name) {
    this->addProperty("EndEffectorPeriod", m_period);

    this->ports()->addPort("outport_endeffector_position", outport_endeffector_position);

    this->addOperation("GetCurrentEndEffector", &EndEffectorComponent::GetCurrentEndEffector, this, ClientThread);
    this->addOperation("AttachEndEffector", &EndEffectorComponent::AttachEndEffector, this, ClientThread);
    this->addOperation("MoveEndEffector", &EndEffectorComponent::MoveEndEffector, this, ClientThread);
}

bool EndEffectorComponent::configureHook() {
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

    this->setActivity( new Activity(os::LowestPriority, m_period) );
}

bool EndEffectorComponent::startHook() {
    return true;
}

void EndEffectorComponent::updateHook() {
    double tpos = v_repExtPQGetToolPosition();
    outport_endeffector_position.write(tpos);
}

void EndEffectorComponent::stopHook() {
}

void EndEffectorComponent::cleanupHook() {
}

int EndEffectorComponent::GetCurrentEndEffector()
{
    return v_repExtPQGetTool();
}

void EndEffectorComponent::AttachEndEffector(const int ee) {
    v_repExtPQSetTool(static_cast<TOOL_TYPES>(ee));
}

void EndEffectorComponent::MoveEndEffector(const double dist) {
    v_repExtPQMoveTool(dist);
}
