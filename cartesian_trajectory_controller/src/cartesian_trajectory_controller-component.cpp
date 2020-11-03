#include "cartesian_trajectory_controller-component.hpp"
#include <kdl/utilities/error.h>
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>

#include <cstring>
#include <cstdlib>

using namespace KDL;
using namespace RTT;
using namespace std;

CartesianTrajectoryController::CartesianTrajectoryController(const string &component_name) :
    TaskContext(component_name, PreOperational)
{
    this->addProperty("MaxCartVel",m_max_cart_vel).doc("");
    this->addProperty("MaxCartAcc",m_max_cart_acc).doc("");
    this->addProperty("EqRadius", m_eq_radius).doc("");
    this->addProperty("MaxNSVel",m_max_ns_vel).doc("");
    this->addProperty("MaxNSAcc",m_max_ns_acc).doc("");

    this->ports()->addPort("outport_error_msg", outport_error_msg).doc("Output of the resulting value of the operations taking place in the component.");

    this->addOperation("GenerateLinearTrajectory", &CartesianTrajectoryController::GenerateLinearTrajectory, this, OwnThread);
    this->addOperation("GenerateArcTrajectory", &CartesianTrajectoryController::GenerateArcTrajectory, this, OwnThread);
    this->addOperation("GenerateComposedTrajectory", &CartesianTrajectoryController::GenerateComposedTrajectory, this, OwnThread);
    this->addOperation("GenerateNullspaceTrajectory", &CartesianTrajectoryController::GenerateNullspaceTrajectory, this, OwnThread);
    this->addOperation("StartMultipleTrajectory", &CartesianTrajectoryController::StartMultipleTrajectory, this, ClientThread);
    this->addOperation("FinishMultipleTrajectory", &CartesianTrajectoryController::FinishMultipleTrajectory, this, ClientThread);
    this->addOperation("AddPauseSegment", &CartesianTrajectoryController::AddPauseSegment, this, OwnThread);
    this->addOperation("GetTrajectory", &CartesianTrajectoryController::GetTrajectory, this, ClientThread);
    this->addOperation("SampleLinearTrajectory", &CartesianTrajectoryController::SampleLinearTrajectory, this, OwnThread);
}

bool CartesianTrajectoryController::configureHook(){
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *CMP = new char[128];
        char *NP = new char[128];
        strcpy(CMP, WS);
        strcat(CMP, "/Properties/CARTESIAN_MOTION_PROPERTIES.xml");
        strcpy(NP, WS);
        strcat(NP, "/Properties/NULLSPACE_PROPERTIES.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(CMP)) {
            log(RTT::Error) << "Properties not loaded" << endlog();
            return false;
        }
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(NP)) {
            log(RTT::Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] NP;
        delete[] CMP;
    }

    // Period times 8 -> Have a sensible number of intermediate frames!
    double dt = this->getActivity()->getPeriod();
    double adjusted_sampling_rate = dt * (1 / (m_max_cart_vel * 3));
//    double adjusted_sampling_rate = dt * 8;
    m_cartesian_trajectory_generator = std::make_shared<CartesianTrajectoryGenerator>(adjusted_sampling_rate, m_max_cart_vel,
                                                                                      m_max_cart_acc, m_eq_radius, m_max_ns_vel,
                                                                                      m_max_ns_acc);
    m_multiple_trajectory = false;

    error_msg e;
    e.level = 9999;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);

    return true;
}

bool CartesianTrajectoryController::startHook(){
    return true;
}

void CartesianTrajectoryController::updateHook(){
}

void CartesianTrajectoryController::stopHook() {
}

void CartesianTrajectoryController::cleanupHook() {
}

void CartesianTrajectoryController::errorHook() {
}

void CartesianTrajectoryController::GenerateLinearTrajectory(const Frame start_frame, const Frame end_frame, const double relative_velocity) {
    VELOCITY_PROFILE vp;
    (m_multiple_trajectory ? vp = VELOCITY_PROFILE::TRAP_HALF : vp = VELOCITY_PROFILE::TRAP );
    if(relative_velocity > 0 && relative_velocity <= 1) {
        try {
            if(relative_velocity == 1) {
                m_cartesian_trajectory_generator->GenerateLinearMotion(start_frame, end_frame, vp);
            }
            else {
                m_cartesian_trajectory_generator->GenerateLinearMotion(start_frame, end_frame, (relative_velocity * m_max_cart_vel), m_max_cart_acc, vp);
            }
        }
        catch (KDL::Error &err) {
            error_msg e;
            e.level = 4500;
            e.msg = "[CTC] ";
            e.msg.append(err.Description());
            outport_error_msg.write(e);
        }
    }
    else {
        error_msg e;
        e.level = NOTVALIDV; // ERROR Level
        e.msg = m_ec.dict[NOTVALIDV];
        outport_error_msg.write(e); //Non-valid value of relative velocity
    }
}

void CartesianTrajectoryController::GenerateArcTrajectory(const Frame start_frame, const Vector center_point, const Frame direction_rotation,
                                                          const double arc_angle, const double relative_velocity) {
    VELOCITY_PROFILE vp;
    (m_multiple_trajectory ? vp = VELOCITY_PROFILE::TRAP_HALF : vp = VELOCITY_PROFILE::TRAP );
    if(relative_velocity > 0 && relative_velocity <= 1) {
        try {
            if(relative_velocity == 1) {
                m_cartesian_trajectory_generator->GenerateArcMotion(start_frame, center_point, direction_rotation.p, direction_rotation.M, arc_angle, vp);
            }
            else {
                m_cartesian_trajectory_generator->GenerateArcMotion(start_frame, center_point, direction_rotation.p, direction_rotation.M, arc_angle,
                                                                    (relative_velocity * m_max_cart_vel), m_max_cart_acc, vp);
            }
        }
        catch (KDL::Error &err) {
            error_msg e;
            e.level = 4501;
            e.msg = "[CTC] ";
            e.msg.append(err.Description());
            outport_error_msg.write(e);
        }
    }
    else {
        error_msg e;
        e.level = NOTVALIDV; // ERROR Level
        e.msg = m_ec.dict[NOTVALIDV];
        outport_error_msg.write(e); //Non-valid value of relative velocity
    }
}

void CartesianTrajectoryController::GenerateComposedTrajectory(const std::vector<Frame> path_frames, const double radius, const double relative_velocity) {
    VELOCITY_PROFILE vp;
    (m_multiple_trajectory ? vp = VELOCITY_PROFILE::TRAP_HALF : vp = VELOCITY_PROFILE::TRAP );
    if(relative_velocity > 0 && relative_velocity <= 1) {
        try {
            if(relative_velocity == 1) {
                m_cartesian_trajectory_generator->GenerateComposedMotion(path_frames, radius, vp);
            }
            else {
                m_cartesian_trajectory_generator->GenerateComposedMotion(path_frames, radius, (relative_velocity * m_max_cart_vel), m_max_cart_acc, vp);
            }
        }
        catch (KDL::Error &err) {
            error_msg e;
            e.level = 4502;
            e.msg = "[CTC] ";
            e.msg.append(err.Description());
            outport_error_msg.write(e);
        }
    }
    else {
        error_msg e;
        e.level = NOTVALIDV; // ERROR Level
        e.msg = m_ec.dict[NOTVALIDV];
        outport_error_msg.write(e); //Non-valid value of relative velocity
    }
}

bool CartesianTrajectoryController::GenerateNullspaceTrajectory(const double cur_psi, const double ref_psi, std::vector<double> &psi_vec, std::vector<double> &time_vec) {
    if(!m_multiple_trajectory) {
        m_cartesian_trajectory_generator->GenerateNullspaceMotion(cur_psi, ref_psi, psi_vec, time_vec);
        return true;
    }
    return false;
}

void CartesianTrajectoryController::StartMultipleTrajectory() {
    m_multiple_trajectory = true;
}

void CartesianTrajectoryController::FinishMultipleTrajectory() {
    m_multiple_trajectory = false;
}

void CartesianTrajectoryController::AddPauseSegment(const double pause_time, const Frame &current_frame) {
    m_cartesian_trajectory_generator->AddPauseSegment(pause_time, current_frame);
}

//TODO: Only measuring Position Velocity
bool CartesianTrajectoryController::VerifyVelocities(const vector< Frame > &target_frames, const vector< double > &target_times) {
    if(target_frames.size() > 0 && target_frames.size() == target_times.size()) {
        for(unsigned int i=0; i<target_frames.size()-1; i++) {
            Vector lvel(0.0,0.0,0.0);
            lvel[0] = (target_frames[i+1].p.x() - target_frames[i].p.x()) / (target_times[i+1] - target_times[i]);
            lvel[1] = (target_frames[i+1].p.y() - target_frames[i].p.y()) / (target_times[i+1] - target_times[i]);
            lvel[2] = (target_frames[i+1].p.z() - target_frames[i].p.z()) / (target_times[i+1] - target_times[i]);
            if(lvel.Norm() > m_max_cart_vel * 1.1) {
                error_msg e;
                e.level = EXCDVLIM; // ERROR Level
                e.msg = m_ec.dict[EXCDVLIM];
                outport_error_msg.write(e); //Non-valid value of relative velocity
                return false;
            }
        }
        error_msg e;
        e.level = VCHECKS; // INFO Level
        e.msg = m_ec.dict[VCHECKS];
        outport_error_msg.write(e); //Non-valid value of relative velocity
        return true;
    }
    else {
        error_msg e;
        e.level = MISMATCH; // ERROR Level
        e.msg = m_ec.dict[MISMATCH];
        outport_error_msg.write(e); //Non-valid value of relative velocity
        return false;
    }
}

bool CartesianTrajectoryController::GetTrajectory(std::vector<Frame> &target_frames, std::vector<double> &target_times) {
    if(m_cartesian_trajectory_generator->GetTargetTrajectory(target_frames, target_times)) {
        m_cartesian_trajectory_generator->PrintToFile(target_frames, "/home/carlos/MATLAB/Trajectory/Cartesian/LastCartesian.txt");
        return VerifyVelocities(target_frames, target_times);
    }
    else {
        error_msg e;
        e.level = NOTRAJ; // ERROR Level
        e.msg = m_ec.dict[NOTRAJ];
        outport_error_msg.write(e); //Non-valid value of relative velocity
        return false;
    }
}

void CartesianTrajectoryController::SampleLinearTrajectory(Frame start_frame, Frame end_frame, const double step, std::vector<Frame> &frames)
{
    if(step > 0.0) {
        m_cartesian_trajectory_generator->SampleLinearTrajectory(start_frame, end_frame, step, frames);
    }
}

ORO_CREATE_COMPONENT(CartesianTrajectoryController)
