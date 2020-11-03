#ifndef V_REPEXT_ENDEFFECTOR_CORBA_INTERFACE
#define V_REPEXT_ENDEFFECTOR_CORBA_INTERFACE

#include "tool_handler.h"

#include <rtt/RTT.hpp>

class CorbaInterface : public RTT::TaskContext {
public:
    CorbaInterface(const std::string &name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    std::shared_ptr<ToolHandler> m_th;

protected: //Ports
    RTT::OutputPort<bool> outport_alive;
    RTT::OutputPort<double> outport_endeffector_distance;

protected: //Operations
    void AttachEndEffector(const int ee);
    void MoveEndEffector(const double dist);
};

#endif // V_REPEXT_ENDEFFECTOR_CORBA_INTERFACE
