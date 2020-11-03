#include "control_modes_interface-component.hpp"

#include <kdl/frames.hpp>
#include <rtt/Component.hpp>
#include <rtt/marsh/Marshalling.hpp>

#include <iostream>

using namespace KDL;
using namespace RTT;
using namespace std;

ControlModesInterface::ControlModesInterface(const string & component_name) : TaskContext(component_name) {
    // PROPERTIES
    this->addProperty("FreeModePosScaling", m_free_pos_scaling).doc("Free motion mode position scaling factor.");
    this->addProperty("FreeModeRotScaling", m_free_rot_scaling).doc("Free motion mode rotation scaling factor.");
    this->addProperty("PointLockModePosScaling", m_pointlock_pos_scaling).doc("Point lock motion mode position scaling factor.");
    this->addProperty("PointLockModeRotScaling", m_pointlock_rot_scaling).doc("Point lock motion mode rotation scaling factor.");
    this->addProperty("PierceModePosScaling", m_pierce_pos_scaling).doc("Pierce motion mode position scaling factor.");
    this->addProperty("PierceModeRotScaling", m_pierce_rot_scaling).doc("Pierce motion mode rotation scaling factor.");
    this->addProperty("HapticZAng", m_haptic_z_ang).doc("Rotation around z-axis of haptic to match the robot's base frame.");

    // PORTS
    this->ports()->addPort("inport_current_robot_posture",inport_current_robot_posture);
    this->ports()->addPort("inport_haptic_relative_frame",inport_haptic_relative_frame);
    this->ports()->addPort("outport_target_relative_frame",outport_target_relative_frame);
    this->ports()->addPort("inport_current_robot_force", inport_current_robot_force);
    this->ports()->addPort("outport_current_robot_force", outport_current_robot_force);
    this->ports()->addPort("outport_error_msg",outport_error_msg);

    // OPERATIONS
    this->addOperation("HapticGripperCloseEventHandler", &ControlModesInterface::HapticGripperCloseEventHandler, this, ClientThread);
    this->addOperation("SetControlMode", &ControlModesInterface::SetControlMode, this, OwnThread);
    this->addOperation("SetPositionScaling", &ControlModesInterface::SetPositionScaling, this, OwnThread);
    this->addOperation("SetRotationScaling", &ControlModesInterface::SetRotationScaling, this, OwnThread);
}

bool ControlModesInterface::configureHook() {
    // Needs to be connected to peer 'haptic_driver' to set scaling and control modes attributes
    if(hasPeer("haptic_driver")) {
        TaskContext *hd_ptr = getPeer("haptic_driver");
        a_pos_scaling = hd_ptr->attributes()->getAttribute("pos_scaling");
        a_rot_scaling = hd_ptr->attributes()->getAttribute("rot_scaling");
        a_control_mode = hd_ptr->attributes()->getAttribute("control_mode");
    }
    else {
        log(Error) << "Failed to connect to peer Haptic Driver." << endlog();
        return false;
    }

    // Properties Marshalling
    if(const char* WS = std::getenv("HAPTIC_WORKSPACE")) {
        m_ws_path = WS;

        char *RSP = new char[128];
        char *NP = new char[128];
        strcpy(RSP, WS);
        strcat(RSP, "/Properties/TELEOPERATION_PROPERTIES.xml");
        strcpy(NP, WS);
        strcat(NP, "/Properties/TELEOPERATION_PROPERTIES.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(RSP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(NP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] NP;
        delete[] RSP;
    }
    else
    {
        log(Error) << "Could not find HAPTIC_WORKSPACE environment variable." << endlog();
        return false;
    }

    // Initialize with NONE teleoperation control mode to avoid undesired actions
    m_control_mode = NONE;

    // Initial tool frame just 25cm along flange z-axis.
    m_tool_frame = KDL::Frame::Identity();
    m_tool_frame.p[2] = 0.25;

    // Output ports data sample
    outport_current_robot_force.setDataSample(std::vector<double>(3));

    // Setup error message data sample
    error_msg e;
    e.level = 9999;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);

    // All Input ports need to be connected for the component to start()
    bool connections = true;
    connections &= inport_current_robot_force.connected();
    connections &= inport_current_robot_posture.connected();
    connections &= inport_haptic_relative_frame.connected();
    return connections;

    // Debug variable
    count = 0;

    return true;
}

bool ControlModesInterface::startHook(){
    return true;
}

void ControlModesInterface::updateHook(){
    RTT::posture posture;
    KDL::Frame input, output;
    std::vector<double> force;

    // Sets the current robot frame at each update() cycle
    if(inport_current_robot_posture.read(posture) == NewData) {
        m_current_frame = posture.frame;
    }

    // Receives input from 'haptic_driver'
    if(inport_haptic_relative_frame.read(input) == NewData) {
        /// The output frame sent for the kinematics_module results from the manipulation of the input frame.
        /// Whenever the gripper closes a new start_frame is stored in memory, also from the haptic_driver,
        /// the transformation is zeroed (input becomes idendity matrix). Thus, the transformation received
        /// in the input is always relative to the start frame.
        switch(m_control_mode) {
        case FREE:
        {
            /// In FREE mode the input position adds to the position of the start frame, given both reference
            /// frames are at the origin.
            output.p = m_start_frame.p + input.p;
            /// The rotation results from an inverse input rotation (from the input to the origin frame) and
            /// then from the origin to the start. It works!
            output.M = input.M.Inverse() * m_start_frame.M;

            outport_target_relative_frame.write(output);
        }
        break;
        case POINT_LOCK:
        {
            /// In this mode the output position is always that of the start_frame (start_frame includes tool in this mode!)
            output.p = m_start_frame.p;
            /// Rotation is processed the same as for the FREE mdoe
            output.M = input.M.Inverse() * m_start_frame.M;

            outport_target_relative_frame.write(output * m_tool_frame.Inverse());
        }
        break;
        case PIERCE:
        {
            /// Rotate around start frame z-axis
            KDL::Rotation rot;
            std::vector<double> rpy(3);
            input.M.GetRPY(rpy[0],rpy[1],rpy[2]);
            /// Translates only along the start_frame z-axis
            /// Consider different Haptic Z Angle in relation to the robot
            switch(m_haptic_z_ang) {
            case 0: {
                output.p = m_start_frame.p + m_start_frame.M.UnitZ() * -input.p.x();
                rot = KDL::Rotation::RotZ(rpy[0]);
            }
                break;
            case 90: {
                output.p = m_start_frame.p + m_start_frame.M.UnitZ() * -input.p.y();
                rot = KDL::Rotation::RotZ(rpy[1]);
            }
                break;
            case 180: {
                /// TODO CHECK
                output.p = m_start_frame.p + m_start_frame.M.UnitZ() * input.p.x();
                rot = KDL::Rotation::RotZ(-rpy[0]);
            }
                break;
            case 270: {
                /// TODO CHECK
                output.p = m_start_frame.p + m_start_frame.M.UnitZ() * input.p.y();
                rot = KDL::Rotation::RotZ(-rpy[1]);
            }
                break;
            default: {
                output.p = m_start_frame.p + m_start_frame.M.UnitZ() * -input.p.x();
                rot = KDL::Rotation::RotZ(rpy[0]);
            }
            }

            output.M = m_start_frame.M * rot; // Maybe pitch or yaw too

            outport_target_relative_frame.write(output * m_tool_frame.Inverse());
        }
        break;
        }
    }

    if(inport_current_robot_force.read(force) == NewData) {
        switch(m_control_mode) {
        case FREE:
        case POINT_LOCK:
        {
            /**
             * @brief force
             * Force felt in Free mode should match the force computed by the Jacobian at the base frame.
             * Thus, the output current robot force is equal to the input.
             */
            outport_current_robot_force.write(force);
        }
        break;
        case PIERCE:
        {
            /**
             * @brief force
             * Force felt in Point Lock or Pierce mode is relative to the end-effector frame.
             * Thus, the input current force needs to change reference frames.
             */
            Vector vForce(force[0], force[1], force[2]);
            Vector vOutForce = m_current_frame.M * vForce;

            std::vector<double> out_force = {vOutForce.x(), vOutForce.y(), vOutForce.z()};

//            double r,p,y;
//            m_current_frame.M .GetRPY(r,p,y);
//            log(Warning) << "FForce: " << r << ", " << p << ", " << y << endlog();
//            if(count%10==0) {
//                log(Warning) << force[0] << ", " << force[1] << ", " << force[2] << endlog();
//                log(Warning) << "FForce: " << out_force[0] << ", " << out_force[1] << ", " << out_force[2] << endlog();
//            }

            outport_current_robot_force.write(out_force);
        }
        break;
        }
        count++;
    }
}

void ControlModesInterface::stopHook() {
}

void ControlModesInterface::cleanupHook() {
}

void ControlModesInterface::SetPositionScaling(double pscale) {
    a_pos_scaling.set(pscale);
}

void ControlModesInterface::SetRotationScaling(double rscale) {
    a_rot_scaling.set(rscale);
}

/// For each mode set a position and rotation scaling
/// Set a force overlay so the haptic feels different, for example, when piercing,
/// impose a force in y and z such that one can only move in x
void ControlModesInterface::SetControlMode(int mode)
{
    error_msg emsg;

    switch (mode) {
    case 0: {
        m_control_mode = FREE;
        a_control_mode.set((int)FREE);

        a_pos_scaling.set(m_free_pos_scaling);
        a_rot_scaling.set(m_free_rot_scaling);

        emsg.level = ERRORCODE::FREEMOVE;
        emsg.msg = m_ec.dict[FREEMOVE];
        outport_error_msg.write(emsg);
    }
        break;
    case 1: {
        m_control_mode = POINT_LOCK;
        a_control_mode.set((int)POINT_LOCK);

        a_pos_scaling.set(m_pointlock_pos_scaling);
        a_rot_scaling.set(m_pointlock_rot_scaling);

        emsg.level = ERRORCODE::POINTLCK;
        emsg.msg = m_ec.dict[POINTLCK];
        outport_error_msg.write(emsg);
    }
        break;
    case 2: {
        m_control_mode = PIERCE;
        a_control_mode.set((int)PIERCE);

        a_pos_scaling.set(m_pierce_pos_scaling);
        a_rot_scaling.set(m_pierce_rot_scaling);

        emsg.level = ERRORCODE::PIERCEMD;
        emsg.msg = m_ec.dict[PIERCEMD];
        outport_error_msg.write(emsg);
    }
        break;
    }
}

bool ControlModesInterface::HapticGripperCloseEventHandler()
{
    /// The stored start_frame depends on the mode selected.
    /// Only makes sense to add the tool to the POINT_LOCK and PIERCE modes.
    switch (m_control_mode) {
    case FREE:
    {
        m_start_frame = m_current_frame;
    }
        break;
    case POINT_LOCK:
    case PIERCE:
    {
        m_start_frame = m_current_frame * m_tool_frame;
    }
    }

    return true;
}

ORO_CREATE_COMPONENT(ControlModesInterface)
