/*****************************************************************************
  File: test_vrep_fri-component.hpp

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

#ifndef OROCOS_TEST_VREP_FRI_COMPONENT_HPP
#define OROCOS_TEST_VREP_FRI_COMPONENT_HPP

#include "test_vrep_fri_types.hpp"
#include "test_vrep_fri-errorcode.hpp"

#include <kdl/frames.hpp>
#include <rtt/RTT.hpp>


/*!
 * \brief The VrepFri class is an orocos component that connects the control architecture to the VRep simulator.
 * This component connects to two components (Servers) spawned by a VRep plugin. The communication is set through CORBA.
 *
 * This component is expected to run in real-time (check real-time log).
 */
class TestVrepFri : public RTT::TaskContext {
public:
    TestVrepFri(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    int m_missed_packs; //!< Communication lost packages
    VFErrorCode m_ec;   //!< Error codes object

    bool last_read;
    int m_cee;
    double m_ceepos;
    bool m_brakes;
    double m_dt;
    double dt;

    double m_maxv;
    double m_maxa;

    std::vector<double> m_cjpos;
    std::vector<double> m_cjppos;
    std::vector<double> m_cjvel;
    std::vector<double> m_cjtvel;
    std::vector<double> m_tjpos;

    std::vector<double> m_cjpos_hlw;
    std::vector<double> m_cjvel_hlw;
    std::vector<double> m_tjpos_hlw;

    RTT::vector_int m_trajectory_ids;
    RTT::vector_vector_double m_trajectory_targets;
    RTT::vector_vector_double m_trajectory_entries;
    bool m_has_trajectories;

    RTT::Attribute<bool> a_simulation;

    int m_cycle_count;

private:
    /*!
     * \brief SetRobotHollowVisibility controls the visibility of the hollow robot.
     * \param visible Boolean variable that codes the desired state for the hollow robot.
     */
    void SetHollowVisibility(const bool visible);
    /*!
     * \brief GetSurgeryReference gets the surgery reference frame relative to the robot base frame, in the simulation environment.
     * \return success of retrieving value
     */
    KDL::Frame GetSurgeryReference();
    /*!
     * \brief GetCurrentEndEffector gets the current end-effector attached to the robot. Check \enum END_EFFECTORS.
     * \return Identifier of the tool attached.
     */
    int GetCurrentEndEffector();
    /*!
     * \brief AttachEndEffector attaches the selected end-effector. Check \enum END_EFFECTORS in
     * the transformation component.
     * \param tool Identifier of the tool to attach.
     * \return success
     */
    bool AttachEndEffector(const int tool);
    /*!
     * \brief MoveEndEffectorTool moves the selected end-effector tool (when possible) the specified distance from the current position.
     * \param dist Distance in milimeters to move the end-effector according to its current position.
     * \return success
     */
    bool MoveEndEffectorTool(const double dist);

    /*!
     * \brief AddTrajectories Adds all surgery trajectories to simulation environment in VRep.
     * \param id Vector of identity keys of the surgery trajectory.
     * \param target Vector of position vectors of target points relative to surgical frame.
     * \param entry Vector of position vectors of entry points relative to surgical frame.
     */
    void AddTrajectories(const RTT::vector_int &id, const RTT::vector_vector_double &target, const RTT::vector_vector_double &entry);
    /*!
     * \brief RemoveTrajectories Removes all trajectories in the surgical plan.
     */
    void RemoveTrajectories();
    /*!
     * \brief ShowTrajectory Shows the ID specified trajectory from the plan and hides the rest.
     * \param id Identity key of the surgery trajectory.
     */
    void ShowTrajectory(const int id);
    /*!
     * \brief ShowTrajectories Shows all trajectories contained in the plan.
     */
    void ShowTrajectories();

    /*!
     * \brief CheckTargetPositions Checks sent target positions against the current joint position.
     * The joint coordinates differential must be within a specified limit otherwise the component stops the robot motion.
     * This function is called whenever new target joint coordinates are set and stands as the last protection resort before sending
     * the coordinates to the robot's controller.
     * \param tar Target joint coordinates
     * \return True if target joint coordinates are within the feasible range.
     */
    bool CheckTargetPositions(const std::vector<double> &tar);
    /*!
     * \brief PrintJointCoordinates Prints joint coordinates in log
     * \param name Name of variable to print.
     * \param var Joint Coordinates to print.
     */
    void PrintJointCoordinates(const char *name, std::vector<double> &var) const;

    /*!
     * \brief SendSingleTrajectory Allows component to send a single trajectory at a time
     * Cannot send all trajectories at once to VREP
     */
    void SendSingleTrajectory();

protected:
    RTT::OperationCaller<void(bool)> CallerSetHollowVisibility;
    RTT::OperationCaller<void(std::vector<double> &)> CallerGetSurgeryReference;

    RTT::OperationCaller<int(void)> CallerGetCurrentEndEffector;
    RTT::OperationCaller<void(const int)> CallerAttachEndEffector;
    RTT::OperationCaller<void(const double)> CallerMoveEndEffector;

    RTT::OperationCaller<void(const int, const std::vector<double> &, const std::vector<double> &)> CallerAddTrajectory;
    RTT::OperationCaller<void()> CallerRemoveTrajectories;
    RTT::OperationCaller<void(const int)> CallerShowTrajectory;
    RTT::OperationCaller<void()> CallerShowTrajectories;

    RTT::OperationCaller<void(const KDL::Frame)> CallerSetWorkFrame;
//    RTT::OperationCaller<bool(const int)> CallerSetToolType;

protected: //Ports
    //ARCH
    RTT::InputPort < std::vector<double> > arch_target_joint_positions;
    RTT::OutputPort< std::vector<double> > arch_current_joint_positions;
    RTT::OutputPort< std::vector<double> > arch_current_joint_velocities;

    RTT::InputPort < std::vector<double> > arch_target_joint_positions_hollow;
    RTT::OutputPort< std::vector<double> > arch_current_joint_positions_hollow;
    RTT::OutputPort< std::vector<double> > arch_current_joint_velocities_hollow;

    //VREP
    RTT::OutputPort< std::vector<double> > vrep_target_joint_positions;
    RTT::InputPort < std::vector<double> > vrep_current_joint_positions;
    RTT::InputPort < std::vector<double> > vrep_current_joint_velocities;

    RTT::OutputPort< std::vector<double> > vrep_target_joint_positions_hollow;
    RTT::InputPort < std::vector<double> > vrep_current_joint_positions_hollow;
    RTT::InputPort < std::vector<double> > vrep_current_joint_velocities_hollow;

    //End-effector
    RTT::OutputPort<double> arch_current_endeffector_position;
    RTT::InputPort<double> vrep_current_endeffector_position;

    RTT::OutputPort< RTT::error_msg > outport_error_msg;
};

#endif //OROCOS_TEST_VREP_FRI_COMPONENT_HPP
