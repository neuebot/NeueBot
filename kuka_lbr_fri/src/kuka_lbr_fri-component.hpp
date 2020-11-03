/*****************************************************************************
  File: kuka_lbr_fri-component.hpp

  Version: 1.0
  Author: Carlos Faria <carlosfaria89@gmail.com>
  Maintainer: Carlos Faria <carlosfaria89@gmail.com>

  Copyright (C) 2018 Carlos André de Oliveira Faria. All rights reserved.

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

#ifndef OROCOS_KUKA_LBR_FRI_COMPONENT_HPP
#define OROCOS_KUKA_LBR_FRI_COMPONENT_HPP

#include "kuka_lbr_fri-types.hpp"
#include "kuka_lbr_fri-errorcode.hpp"
#include "circular_vector_buffer.hpp"

// Logging
#include <ocl/LoggingService.hpp>
#include <ocl/Category.hpp>

// FRI Client Driver
#include "fri_driver.hpp"

#include <rtt/RTT.hpp>

/*!
 * \brief The KukaLbrFri class
 * This component connects the control architecture to the Kuka Controller via Fast Research Interface (FRI).
 * At the moment, the component only works in Position control mode. It receives the current joint positions,
 * and sends target joint positions at the set frequency rate.
 *
 * This component is expected to run in real-time (check real-time log).
 */
class KukaLbrFri : public RTT::TaskContext {

public:
    KukaLbrFri(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    /*!
     *  \brief RecoverFRI Recover from an error/stop state to normal functioning.
     */
    void RecoverFRI();
    /*!
     *  \brief CheckTargetPositions Checks sent target positions against the current joint position.
     *  The joint coordinates differential must be within a specified limit otherwise the component stops the robot motion.
     *  This function is called whenever new target joint coordinates are set and stands as the last protection resort before sending
     *  the coordinates to the robot's controller.
     *  \param tar Target joint coordinates
     *  \return True if target joint coordinates are within the feasible range.
     */
    bool CheckTargetPositions(const std::vector<double> &tar);

private:
    std::shared_ptr<FriDriver> m_driver; //!< FRI driver pointer

private:
    int m_missed_packs; //!< Communication lost packages
    KLFErrorCode m_ec;   //!< Error codes object
    int m_session_state; //!< Session state whether it is MONITORING or COMMANDING
    double m_tracking_performance; //!< Tracking Performance of robot to the target joint positions

    double m_dt;
    double m_maxv;
    double m_maxa;
    std::vector<double> m_curpos;

    std::shared_ptr< CircularBuffers<double> > m_prvpos;
    std::vector<double> m_curvel;
    std::vector<double> m_tarpos;

    bool last_valid;

    // _DEBUG_
    bool last_mov;
    int mov_n;

    /// Name of our category
    std::string categoryName;
    /// Our logging category
    OCL::logging::Category* logger;

protected:
    std::string m_robot_type;
    std::string m_local_addr;
    int m_local_port;
    std::string m_remote_addr;
    int m_remote_port;

protected:
    RTT::InputPort < std::vector<double> > inport_target_joint_positions;
    RTT::OutputPort< std::vector<double> > outport_current_joint_positions;
    RTT::OutputPort< std::vector<double> > outport_current_joint_velocities;
    RTT::OutputPort< std::vector<double> > outport_external_joint_torques;

    RTT::OutputPort< int > outport_session_state;
    RTT::OutputPort< double > outport_tracking_performance;

    RTT::OutputPort< RTT::error_msg > outport_error_msg;
};
#endif
