/*****************************************************************************
  File: cartesian_trajectory_controller-component.hpp

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

#ifndef OROCOS_CARTESIAN_TRAJECTORY_CONTROLLER_COMPONENT_HPP
#define OROCOS_CARTESIAN_TRAJECTORY_CONTROLLER_COMPONENT_HPP

#include "cartesian_trajectory_generator.hpp"
#include "cartesian_trajectory_controller_types.hpp"
#include "cartesian_trajectory_controller-errorcode.hpp"

#include <rtt/RTT.hpp>

class CartesianTrajectoryController : public RTT::TaskContext
{

public:
    CartesianTrajectoryController(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void errorHook();

private:
    CTCErrorCode m_ec;
    unsigned int m_num_poses;
    std::vector< KDL::Frame > m_target_frames;
    std::vector< double > m_target_times;
    std::shared_ptr<CartesianTrajectoryGenerator> m_cartesian_trajectory_generator;

protected:
    //! Boolean variable that specifies whether the trajectory is multiple or not
    //! When a new multiple trajectory is started (\ref StartMultipleTrajectory) it is assigned true.
    //! When multiple trajectory is finished (\ref FinishMultipleTrajectory) it is assigned false.
    bool m_multiple_trajectory;
    double m_max_cart_vel; //!< Maximum allowed end-effector speed in Cartesian space.
    double m_max_cart_acc; //!< Maximum allowed end-effector modular acceleration in Cartesian space.
    double m_eq_radius; //!< Rotation motion measured by the arc formed by this radius and an angle.
    double m_max_ns_vel; //!< Maximum allowed nullspace velocity
    double m_max_ns_acc; //!< Maximum allowed nullspace acceleration

private:
    //For some odd reason, operations can only take on 5 parameters.
    /*!
     * \brief GenerateLinearTrajectory Generates a linear trajectory in Cartesian space. It interpolates between initial and final frame,
     * respecting the position and orientation variations. The positions along the trajectory respect a trapezoid velocity profile, with a
     * constant acceleration and a maximum velocity.
     * \param start_frame Initial frame.
     * \param end_frame Final frame.
     * \param relative_velocity Relative velocity to the default value.
     */
    void GenerateLinearTrajectory(const KDL::Frame start_frame, const KDL::Frame end_frame, const double relative_velocity);
    /*!
     * \brief GenerateArcTrajectory Generates an arc trajectory in Cartesian space. The arc starts at the initial frame, relative
     * to the center point and following the arc angle specified in the correct direction. The positions along the trajectory
     * respect a trapezoid velocity profile, with a constant acceleration and a maximum velocity.
     * \param start_frame Initial frame.
     * \param center_point Position of the center of the arc relative to the same reference as the start_frame.
     * \param direction_rotation Combination of: 1) Unit vector at the base of the arc, which indicates the direction of the arc. For instance,
     * if the vector [1,0,0] is specified, the arc will be described in the xz-plane, towards the positive x-axis; and 2) Desired Rotation of the target frame.
     * \param arc_angle Angle of the arc to be described in the trajectory.
     * \param relative_velocity Relative velocity to the default value.
     */
    void GenerateArcTrajectory(const KDL::Frame start_frame, const KDL::Vector center_point, const KDL::Frame direction_rotation, const double arc_angle, const double relative_velocity);
    /*!
     * \brief GenerateComposedTrajectory Generates a path with several segments and smooths the transition between them.
     * \param path_frames Set of frames the robot should pass through.
     * \param radius Radius of transition between segments.
     * \param relative_velocity Relative velocity to the default value.
     */
    void GenerateComposedTrajectory(const std::vector< KDL::Frame > path_frames, const double radius, const double relative_velocity);

    /*!
     * \brief GenerateNullspaceTrajectory Generates a timed trajectory profile for a nullspace trajectory accounting for the target velocity and acceleration.
     * NOTE: It is assumed there is a continuous and possible nullspace path between current and reference point.
     * \param cur_psi Input starting psi value.
     * \param ref_psi Input reference psi value to reach.
     * \param psi_vec Output vector of psi values.
     * \param time_vec Output vector of time values corresponding to each psi.
     */
    bool GenerateNullspaceTrajectory(const double cur_psi, const double ref_psi, std::vector<double> &psi_vec, std::vector<double> &time_vec);

    /*!
     * \brief StartMultipleTrajectory
     */
    void StartMultipleTrajectory();
    /*!
     * \brief FinishMultipleTrajectory
     */
    void FinishMultipleTrajectory();
    /*!
     * \brief AddPauseSegment adds a segment to a composed trajectory where the robot holds still in the current frame for a specified duration of time.
     * \param pause_time double time in seconds, to pause the robot motion during a composed trajectory.
     * \param current_frame KDL:Frame current frame where to hold still.
     */
    void AddPauseSegment(const double pause_time, const KDL::Frame &current_frame);

    /*!
     * \brief VerifyVelocities Checks whether the Cartesian velocities set for the trajectory are within the specified limits.
     * \param target_frames Vector of frames that compose the trajectory.
     * \param target_times Vector of times assigned to the trajectory. It specifies the robot motion law.
     * \return True if the velocities are within the limits defined.
     */
    bool VerifyVelocities(const std::vector< KDL::Frame > &target_frames, const std::vector< double > &target_times);
    /*!
     * \brief GetTrajectory Returns the trajectory computed and the associated times.
     * \param target_frames Vector of frames that compose the trajectory.
     * \param target_times Vector of times assigned to the trajectory. It specifies the robot motion law.
     * \return True if the velocities are within the limits defined.
     */
    bool GetTrajectory(std::vector< KDL::Frame > &target_frames, std::vector< double > &target_times);

    /*!
     * \brief SampleLinearTrajectory Samples the linear path returning a vector of 'step-spaced' trajectory frames.
     * \param start_frame Initial trajectory frame.
     * \param end_frame Final trajectory frame.
     * \param step Distance step that defines how the trajectory will be sampled.
     * \param frames Vector of trajectory samples.
     */
    void SampleLinearTrajectory(KDL::Frame start_frame, KDL::Frame end_frame, const double step, std::vector< KDL::Frame > &frames);

protected:
    RTT::OutputPort< RTT::error_msg > outport_error_msg;
};

#endif
