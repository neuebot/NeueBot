#include "corba_interface.h"
#include "tool_handler.h"

#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using namespace std;

CorbaInterface::CorbaInterface(const string &name) : TaskContext(name) {
    //Attributes and Properties
//    this->addProperty("prop", m_prop).doc("THIS IS PROPERTY");

    //Ports
    this->ports()->addPort("outport_alive", outport_alive);
    this->ports()->addPort("outport_endeffector_distance", outport_endeffector_distance);

    //Operations
    this->addOperation("AttachEndEffector", &CorbaInterface::AttachEndEffector, this, ClientThread);
    this->addOperation("MoveEndEffector", &CorbaInterface::MoveEndEffector, this, ClientThread);

    m_th = std::make_shared<ToolHandler>();
}

bool CorbaInterface::configureHook() {
    return m_th->InitTools();
}

bool CorbaInterface::startHook() {
    return true;
}

void CorbaInterface::updateHook() {
    double cpos;
    m_th->GetToolPos(cpos);

    outport_alive.write(true);
    outport_endeffector_distance.write(cpos);
}

void CorbaInterface::stopHook() {
}

void CorbaInterface::cleanupHook() {
}

void CorbaInterface::AttachEndEffector(const int ee) {
    m_th->SetTool(static_cast<TOOL_TYPES>(ee));
}

void CorbaInterface::MoveEndEffector(const double dist) {
    m_th->MoveTool(dist);
}
