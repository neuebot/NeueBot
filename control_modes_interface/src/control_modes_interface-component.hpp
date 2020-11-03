/*****************************************************************************
  File: control_modes_interface-component.hpp

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

#ifndef OROCOS_CONTROL_MODES_INTERFACE_COMPONENT_HPP
#define OROCOS_CONTROL_MODES_INTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include "kdl/frames.hpp"

#include "control_modes_interface-types.hpp"
#include "control_modes_interface-errorcode.hpp"

/*!
 * \brief The ControlModesInterface class
 * Component responsible for handling the different operation modes when teleoperating.
 * Different modes involve position/rotation scaling, new force reference frames and new haptic actuation.
 */
class ControlModesInterface : public RTT::TaskContext {

    /*!
     * \brief The CONTROL_MODE enum
     * Sets the control mode of the robot when teleoperated.
     * NONE = 0, no motion;
     * FREE = 1, free motion in all axes position and orientation;
     * POINT_LOCK = 2, motion restricted to the tip of the end-effector position. Only rotation is applied
     * PIERCE = 3, motion restricted to the path collinear to the z-axis of the tool
     */
enum CONTROL_MODE {
    NONE = 0,
    FREE,
    POINT_LOCK,
    PIERCE,
};

public:
    ControlModesInterface(std::string const& name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    /*!
     * \brief SetControlMode Sets the teleoperating control mode. Check #CONTROL_MODE.
     * \param mode Control modes available: NONE(0), FREE(1), POINT_LOCK(2), PIERCE(3)
     */
    void SetControlMode(int mode);

    /*!
     * \brief SetPositionScaling Sets the scaling applied to the input reference frame position displacement from the Haptic.
     * \param pscale Position scaling factor [0, 1].
     */
    void SetPositionScaling(double pscale);
    /*!
     * \brief SetRotationScaling Sets the scaling applied to the input reference frame rotation displacement from the Haptic.
     * \param rscale Rotation scaling factor [0, 1].
     */
    void SetRotationScaling(double rscale);

    /*!
     * \brief HapticGripperCloseEventHandler EventHandler to handle Haptic gripper close event.
     * Sets the \a start_frame variable to the current_frame whenever the haptic closes.
     * \return TODO: always returns true now.
     */
    bool HapticGripperCloseEventHandler();

private:
    /// Current tool: needle tip at [0, 0, 0.25]
    KDL::Frame m_tool_frame;
    /// Current robot frame directly from port inport_current_robot_frame
    KDL::Frame m_current_frame;
    /// Start mode frame -> Set when the gripper closes
    KDL::Frame m_start_frame;

    /// Control mode flag
    CONTROL_MODE m_control_mode;

    int count;
    std::string m_ws_path;
    CMIErrorCode m_ec;

protected:
    RTT::Attribute<double> a_pos_scaling;
    RTT::Attribute<double> a_rot_scaling;
    RTT::Attribute<int> a_control_mode;

protected:
    /// Properties
    double m_free_pos_scaling;
    double m_free_rot_scaling;
    double m_pointlock_pos_scaling;
    double m_pointlock_rot_scaling;
    double m_pierce_pos_scaling;
    double m_pierce_rot_scaling;
    int m_haptic_z_ang; /// Easier to check ints than doubles

protected:
    /// Transformation from robot Base to robot Flange
    /// (Need to multiply by transformation of tool)
    RTT::InputPort< RTT::posture > inport_current_robot_posture;

    /// Manipulates the input relative frame from haptic to match the behavior
    /// expected from the different control modes.
    RTT::InputPort< KDL::Frame > inport_haptic_relative_frame;
    RTT::OutputPort< KDL::Frame > outport_target_relative_frame;

    /// Manipulate forces relative frame
    /// Always receive forces at the base reference frame.
    /// If mode FREE, it remains the same.
    /// If mode PIERCE, the force changes reference to the end-effector.
    RTT::InputPort < std::vector<double> > inport_current_robot_force;
    RTT::OutputPort < std::vector<double> > outport_current_robot_force;

    /// Error port message - Handled by Supervisor
    RTT::OutputPort < RTT::error_msg > outport_error_msg;
};

#endif
