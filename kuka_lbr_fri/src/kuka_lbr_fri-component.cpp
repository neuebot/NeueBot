/*****************************************************************************
  File: kuka_lbr_fri-component.hpp

  Version: 1.0
  Author: Carlos Faria <carlosfaria89@gmail.com>
  Maintainer: Carlos Faria <carlosfaria89@gmail.com>

  Copyright (C) 2017 Carlos André de Oliveira Faria. All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/


#include "kuka_lbr_fri-component.hpp"
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>

#include <rtt/rt_string.hpp>

#include <cstring>
#include <cstdlib>

#include <iostream>

//#define _LOGGING_ 1

using namespace OCL::logging;
using namespace RTT;
using namespace std;

static const char* parentCategory = "org.orocos.ocl";

/*!
 * \brief KukaLbrFri::KukaLbrFri Component that interfaces the control architecture with the
 * KUKA Controller FRI mode.
 *
 * This component operates at real-time (200Hz), communicates target joint positions and receives
 * current joint positions and external torques.
 * Operating at real-time, it includes a real-time logger based on log4cpp.
 * Logging properties are defined in the appender_properties.cpf and logging_properties.cpf that
 * are located in the same folder as the deployer launcher script.
 *
 * \param component_name
 */
KukaLbrFri::KukaLbrFri(const std::string &component_name) :
    TaskContext(component_name, PreOperational),
    categoryName(parentCategory + std::string(".") + component_name),
            logger(dynamic_cast<OCL::logging::Category*>(
                       &log4cpp::Category::getInstance(categoryName)))
{
    this->addProperty("RobotType", m_robot_type).doc("Robot type");
    this->addProperty("LocalIP", m_local_addr).doc("Local IP address (this machine)");
    this->addProperty("LocalPort", m_local_port).doc("Local port");
    this->addProperty("RemoteIP", m_remote_addr).doc("Remote IP address (controller KONI port)");
    this->addProperty("RemotePort", m_remote_port).doc("Remote port");

    this->addProperty("MaxTFJointVel", m_maxv).doc("Maximum allowed joint velocity");
    this->addProperty("MaxTFJointAcc", m_maxa).doc("Maximum allowed joint acceleration");

    this->ports()->addPort("inport_target_joint_positions", inport_target_joint_positions).doc("Inport with target KUKA LBR iiwa joint positions.");
    this->ports()->addPort("outport_current_joint_positions", outport_current_joint_positions).doc("Outport with current KUKA LBR iiwa joint positions.");
    this->ports()->addPort("outport_current_joint_velocities", outport_current_joint_velocities).doc("Outport with calculated KUKA LBR iiwa joint velocities.");
    this->ports()->addPort("outport_external_joint_torques", outport_external_joint_torques).doc("Outport with external KUKA LBR iiwa joint torques.");
    this->ports()->addPort("outport_session_state", outport_session_state).doc("Outport with current KUKA LBR iiwa controller communication session state.");
    this->ports()->addPort("outport_tracking_performance", outport_tracking_performance).doc("Outport with KUKA LBR iiwa tracking performance.");

    this->ports()->addPort("outport_error_msg", outport_error_msg).doc("Output the result value and error codes of the component to task supervisor.");

    this->addOperation("RecoverFRI", &KukaLbrFri::RecoverFRI, this, OwnThread);
}

bool KukaLbrFri::configureHook() {
    //! Component configuration
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *JMP = new char[128];
        char *RC = new char[128];
        strcpy(JMP, WS);
        strcat(JMP, "/Properties/JOINT_MOTION_PROPERTIES.xml");
        strcpy(RC, WS);
        strcat(RC, "/Properties/ROBOT_CONNECTION.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(JMP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(RC)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] RC;
        delete[] JMP;
    }

    //! FRI configuration
    m_driver = std::make_shared<FriDriver>();
    bool fri_connect = m_driver->FriConnect();

    //Get activity period
    m_dt = this->getActivity()->getPeriod();
    //Set data samples for vector ports
    std::vector<double> joints(7,0.0);
    outport_current_joint_positions.setDataSample(joints);
    outport_current_joint_velocities.setDataSample(joints);
    outport_external_joint_torques.setDataSample(joints);

    //Setup error message data sample
    RTT::error_msg e;
    e.level = 9999;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);

    return fri_connect;
}

bool KukaLbrFri::startHook(){
    if(m_driver->isFriConnected()) {
        /// Executes an initial step to initialize the variables, and populate queues
        m_driver->FriStep();
    }

    //Make sure to read current joint positions
    while(!m_driver->GetCurrentJointPositions(m_curpos)) {}
    //Assigning Circular Buffers for each joint, 5 points
    m_prvpos = std::make_shared< CircularBuffers<double> > (7, 5);
    //Assign by default robot starts stationary
    m_curvel.assign(7, 0.0);
    //Last position value read
    last_valid = false;

    //Get Starting Connection indicators
    m_session_state = m_driver->getSessionState();
    m_tracking_performance = m_driver->getTrackingPerformance();

    mov_n = 0;

    return true;
}

void KukaLbrFri::updateHook() {
    std::vector<double> cur_jpos(7, 0.0),
            cur_jvel(7, 0.0),
            ext_jtor(7, 0.0);

    // Connection parameters and Tracking Performance
    m_session_state = m_driver->getSessionState();
    m_tracking_performance = m_driver->getTrackingPerformance();

    outport_session_state.write(m_session_state);
    outport_tracking_performance.write(m_tracking_performance);

    // Read joint positions / 'velocities'
    if(m_driver->GetCurrentJointPositions(cur_jpos)) {
        m_curpos = cur_jpos;

        //Push read positions to circular buffer used to compute
        // the current joint velocity using a 5-point numerical
        // differentiation.
        m_prvpos->push(cur_jpos);
        if(m_prvpos->full()) {
            for(size_t i=0; i<7; ++i) {
                cur_jvel[i] = (3*m_prvpos->at(i)[4] - 16*m_prvpos->at(i)[3] + 36*m_prvpos->at(i)[2]
                        - 48*m_prvpos->at(i)[1] + 25*m_prvpos->at(i)[0]) / (12*m_dt);
            }
            m_curvel = cur_jvel;

            //Current joint velocities
            outport_current_joint_velocities.write(cur_jvel);
        }
        //Current positions
        outport_current_joint_positions.write(cur_jpos);
    }

    // Current joint external torques
    if(m_driver->GetExternalJointTorques(ext_jtor)) {
        outport_external_joint_torques.write(ext_jtor);
    }

    // Set target joint positions
    // Requires a commanding session state (3 or 4).
    std::vector<double> tar_jpos;
    bool new_tjpos = (inport_target_joint_positions.read(tar_jpos) == NewData);

    if(m_session_state == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
            m_session_state == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {

        if(new_tjpos) {
            //TOREMOVE - LOGGING
            if(new_tjpos != last_mov) {
                mov_n++;
            }

            //! Check if target positions are within a doable range
            if(CheckTargetPositions(tar_jpos)) {
                m_driver->SetTargetJointPositions(tar_jpos);

                // cur_jpos[7], tar_jpos[7], mov_n, track_perf;
                RTT::rt_ostringstream msg;
                msg << cur_jpos[0] << ", " << cur_jpos[1] << ", " << cur_jpos[2] << ", " << cur_jpos[3] << ", "
                                   << cur_jpos[4] << ", " << cur_jpos[5] << ", " << cur_jpos[6] << ", " <<
                       tar_jpos[0] << ", " << tar_jpos[1] << ", " << tar_jpos[2] << ", " << tar_jpos[3] << ", "
                                   << tar_jpos[4] << ", " << tar_jpos[5] << ", " << tar_jpos[6] << ", " <<
                       mov_n << ", " <<
                       m_driver->getTrackingPerformance();

#ifdef _LOGGING_
                logger->warn(RTT::rt_string(msg.str().c_str()));
#endif
            }
            else {
                error_msg emsg;
                emsg.level = Error;
                emsg.msg = "Received a target position out of limits!";
                outport_error_msg.write(emsg);

                stop();
            }
        }
    }

    last_mov = new_tjpos;

    // Run a FRI step - guarantees synchronism
    if(m_driver->isFriConnected()) {
        m_driver->FriStep();
    }
}

void KukaLbrFri::stopHook() {
}

void KukaLbrFri::cleanupHook() {
    if(m_driver->isFriConnected()) {
        m_driver->FriDisconnect();
    }
}

void KukaLbrFri::RecoverFRI()
{
    recover();
}

bool KukaLbrFri::CheckTargetPositions(const std::vector<double> &tar)
{
    for(size_t i=0; i<7; ++i) {
        double dq = std::abs(tar[i] - m_curpos[i]);
        double max_dq = m_maxv * m_dt * 16;
        if( dq > max_dq ) {
            log(Error) << "[KUKA] Prevented Motion!!!" << endlog();
            log(Error) << "Joint " << i << " moving " << dq << endlog();
            log(Error) << "Maximum Allowed " << max_dq << endlog();
            log(Error) << "Component stopped. Call RecoverFRI() or Relaunch the component." << endlog();

            return false;
        }
    }
    return true;
}

ORO_CREATE_COMPONENT(KukaLbrFri)
