/*****************************************************************************
  File: robot_state-component.hpp

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

#ifndef OROCOS_ROBOT_STATE_COMPONENT_HPP
#define OROCOS_ROBOT_STATE_COMPONENT_HPP

#include "forward_kinematics.hpp"
#include "robot_state_types.hpp"

#include <kdl/frames.hpp>
#include <rtt/RTT.hpp>
#include <memory>

#include <mutex>

class RobotState : public RTT::TaskContext{
  public:
    RobotState(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    std::shared_ptr <ForwardKinematicSolver> m_kinematic_solver;

    std::vector<double> m_cx;
    std::vector<double> m_cv;
    std::vector<double> m_ctv;

    std::vector<double> m_diff;

    std::vector<double> m_frame_pos;
    std::vector<double> m_frame_rot;
    double m_nsparam;
    unsigned int m_rconf;
    KDL::Frame m_frame;

    std::mutex m_mutex_ft;
    KDL::Frame m_flange_tool_frame;
    std::mutex m_mutex_wb;
    KDL::Frame m_work_base_frame;

private:
    /*!
     * \brief SetFlangeToolFrame Sets the Transformation frame that describes the Tool position and orientation in
     * the flange reference system.
     * \param ft Transformation matrix from the Flange to the Tool reference systems.
     */
    void SetFlangeToolFrame(const KDL::Frame ft);
    /*!
     * \brief SetWorkBaseFrameTool Sets the Transformation frame that describes the Robot Base position and orientation in
     * the defined Work reference system.
     *
     * TODO: Consider having this as a port that continuously reads the transformation between robot and work frame.
     * This should be used to instantly correct the robot movement in case there is a displacement between both
     * reference systems. This will imply online corrections to the robot motion path, even mid motion.
     * \param wb Transformation matrix from the Work to the Robot Base reference systems.
     */
    void SetWorkBaseFrame(const KDL::Frame wb);

    /*!
     * \brief GetFlangeToolFrame returns the Transformation frame that describes the Tool position and orientation in
     * the flange reference system.
     * \return Transformation matrix from the Flange to the Tool reference systems.
     */
    KDL::Frame GetFlangeToolFrame();
    /*!
     * \brief GetWorkBaseFrame returns the Transformation frame that describes the Robot Base position and orientation in
     * the defined Work reference system. If no work reference system is defined, the Identity matrix is returned instead.
     * \return Transformation matrix from the Work to the Robot Base reference systems.
     */
    KDL::Frame GetWorkBaseFrame();   

protected:
    //Properties
    unsigned int m_nj;
    double m_tol;
    std::vector<double> m_lnk;
    std::vector<double> m_lim;

    std::vector<double> m_dh_a;
    std::vector<double> m_dh_alpha;
    std::vector<double> m_dh_d;
    std::vector<double> m_dh_theta;
    std::vector<std::vector<double> > m_dh; //dh[joint_index][Denavit-Hartenberg params]

protected:
    RTT::InputPort< std::vector<double> > inport_current_joint_positions;   // DataPort containing the input current joint positions
    RTT::InputPort< std::vector<double> > inport_current_joint_velocities;  // DataPort containing the input current joint velocities
    RTT::InputPort< std::vector<double> > inport_request_joint_positions;
    RTT::InputPort< std::vector<double> > inport_current_joint_positions_hollow;

    RTT::InputPort < bool > inport_robot_moving;
    RTT::OutputPort< RTT::posture > outport_robot_posture; // DataPort containing the robot param output (frame, nullspace and rconf)

    RTT::OutputPort< std::vector<double> > outport_current_joint_positions;
    RTT::OutputPort< std::vector<double> > outport_current_joint_velocities;
    RTT::OutputPort< std::vector<double> > outport_current_joint_positions_hollow;
    RTT::OutputPort < bool > outport_robot_moving;
};

#endif //OROCOS_ROBOT_STATE_COMPONENT_HPP
