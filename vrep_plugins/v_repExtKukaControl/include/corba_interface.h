#ifndef V_REPEXT_KUKACONTROL_CORBA_INTERFACE
#define V_REPEXT_KUKACONTROL_CORBA_INTERFACE

#include "robot_handler.h"

#include <kdl/frames.hpp>
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
    std::shared_ptr<RobotHandler> m_rh;

protected: //Ports
    RTT::InputPort< std::vector<double> > inport_joint_positions;
    RTT::OutputPort< std::vector<double> > outport_joint_positions;
    RTT::OutputPort< std::vector<double> > outport_joint_velocities;

    RTT::InputPort< std::vector<double> > inport_joint_positions_hollow;
    RTT::OutputPort< std::vector<double> > outport_joint_positions_hollow;
    RTT::OutputPort< std::vector<double> > outport_joint_velocities_hollow;

protected: //Operations
    void SetHollowVisibility(const bool vis);
    void GetSurgeryReference(std::vector<double> &surg_ref);
};

#endif // V_REPEXT_KUKACONTROL_CORBA_INTERFACE
