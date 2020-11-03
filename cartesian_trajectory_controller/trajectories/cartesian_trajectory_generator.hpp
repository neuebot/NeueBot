/*****************************************************************************
  File: cartesian_trajectory_generator.hpp

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

#ifndef CARTESIAN_TRAJECTORY_GENERATOR
#define CARTESIAN_TRAJECTORY_GENERATOR

#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>

#include <kdl/velocityprofile_dirac.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>

#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_stationary.hpp>

#include <fstream>
#include <memory>
#include <vector>

/*!
 * \brief The VELOCITY_PROFILE enum lists the possible velocity profiles
 */
enum VELOCITY_PROFILE {TRAP, TRAP_HALF, RECT};

/*!
 * \brief The CartesianTrajectoryGenerator class is responsible for generating trajectories
 * for the robot end-effector in Cartesian space.
 *
 * It allows the generation of:
 * - Linear
 * - Arc
 * - Composite motion paths (given a set of Cartesian poses)
 *
 * following a timing law given by a selected velocity profile.
 *
 * This class is a wrapper of the [KDL::Trajectory](http://docs.ros.org/hydro/api/orocos_kdl/html/classKDL_1_1Trajectory.html) class.
 * Each trajectory contains a trajectory path [KDL::Path](http://docs.ros.org/hydro/api/orocos_kdl/html/classKDL_1_1Path.html) and a velocity profile [KDL::VelocityProfile](http://docs.ros.org/hydro/api/orocos_kdl/html/classKDL_1_1VelocityProfile.html).
 *
 * \note Don't try to use smart pointers. Trajectory segment deallocation triggers SEG FAULT.
 */

class CartesianTrajectoryGenerator {

public:
    /*!
     * \brief CartesianTrajectoryGenerator constructs a CartesianTrajectoryGenerator object with three required parameters.
     * \param cycle_time Sample time between each target pose.
     * The time differential of the trajectory controller, does not need to match the time differential of the joint controller.
     * This variable is usually defined by the component update rate.
     *
     * The trajectory is generated based on trapezoidal velocity profiles. That is, the end-effector accelerates at a constant rate
     * until reaching the maximum velocity for the segment, and then one of two option may occur. Either the final velocity is different,
     * and so the end-effector deaccelerates at a constant rate until reaching the target velocity, or keeps the velocity if the segment
     * transitions into another segment.
     * By the last segment the robot should stop, so a constant deacceleration is required until stop.
     *
     * Thus the trajectory segments follow 3 types of velocity profiles:
     * - TRAP_HALF, when accelerating or deaccelerating to reach a target velocity only in one end (start or finish)
     *  This is applied at the start or end of full-trajectories.
     * - RECT, when the robot is between segments of trajectory and should keep the same end-effector velocity.
     * - TRAP, when the trajectory segment requires the robot to accelerate and deaccelerate in the same segment.
     *
     * All 'trajectory segment generation' methods have a polymorphed alternative to specify velocity and acceleration limits.
     * If no limits are explicitly specified, the global maximum values for velocity and acceleration are considered.
     *
     * This class only generates the trajectories with <b>explicit</b> paths and velocity profiles. It is up to the caller to correctly
     * specify these parameters.
     *
     * \param max_cart_vel Maximum allowed end-effector speed in Cartesian space.
     * \param max_cart_acc Maximum allowed end-effector modular acceleration in Cartesian space.
     * \param eq_radius Equivalent radius, used to convert rotations to a relatable distance.
     * \param max_ns_vel Maximum allowed nullspace velocity.
     * \param max_ns_acc Maximum allowed nullspace acceleration.
     * \note Do not forget that KDL::Trajectory takes as arguments pointers to KDL::Path and KDL::VelocityProfile and is the owner of the
     * passed pointers, being responsible for their destruction.
     */
    CartesianTrajectoryGenerator(const double cycle_time, const double max_cart_vel, const double max_cart_acc, const double eq_radius,
                                 const double max_ns_vel, const double max_ns_acc);
    ~CartesianTrajectoryGenerator();

private:
    const double m_max_cart_vel; //!< Maximum allowed end-effector speed in Cartesian space.
    const double m_max_cart_acc; //!< Maximum allowed end-effector modular acceleration in Cartesian space.
    const double m_eq_radius; //!< Rotation motion measured by the arc formed by this radius and an angle.
    const double m_max_ns_vel; //!< Maximum allowed nullspace velocity
    const double m_max_ns_acc; //!< Maximum allowed nullspace acceleration
    const double m_dt; //!< Time differential of frame interpolation.

    std::vector< KDL::Trajectory* > m_segments;
    std::shared_ptr<KDL::Trajectory_Composite> m_trajectory;

    unsigned int m_num_it; //!< Number of frames generated for the given trajectory.
    std::vector< KDL::Frame > m_target_frames; //!< Vector containing the target frames.
    std::vector<double> m_target_times; //!< Vector containing the target times to reach the corresponding frames

public:
    /*!
     * \brief GenerateLinearMotion creates a linear path trajectory from the start_frame to the end_frame in Cartesian space.
     * \param start_frame Initial frame of the end-effector (usually related to the base frame).
     * \param end_frame Target (final) frame of the end-effector, with respect to the same reference of the start_frame.
     * \param vel_prof Velocity profile for the generated trajectory segment.
     */
    void GenerateLinearMotion(const KDL::Frame &start_frame, const KDL::Frame &end_frame, const VELOCITY_PROFILE vel_prof=TRAP);
    /*!
     * \copydoc GenerateLinearMotion
     */
    void GenerateLinearMotion(const KDL::Frame &start_frame, const KDL::Frame &end_frame, const double max_vel, const double max_acc,
                              const VELOCITY_PROFILE vel_prof=TRAP);

    /*!
     * \brief GenerateArcMotion Generates an arc path from the initial frame to the target frame.
     * The target frame is computed based on the arc center position (relative to base), the arc angle to be described,
     * the final orientation and the direction of the arc.
     * \param start_frame Initial frame of the end-effector (usually related to the base frame).
     * \param arc_center Position of the center of the arc relative to the same reference as the start_frame.
     * \param arc_dir Unit vector at the base of the arc, which indicates the direction of the arc. For instance,
     * if the vector [1,0,0] is specified, the arc will be described in the xz-plane, towards the positive x-axis.
     * \param end_rot Desired Rotation of the target frame.
     * \param angle Angle of the arc to be described in the trajectory
     * \param vel_prof Velocity profile for the generated trajectory segment.
     */
    void GenerateArcMotion(const KDL::Frame &start_frame, const KDL::Vector &arc_center, const KDL::Vector &arc_dir,
                           const KDL::Rotation &end_rot, const double angle, const VELOCITY_PROFILE vel_prof=TRAP);
    /*!
     * \copydoc GenerateArcMotion
     */
    void GenerateArcMotion(const KDL::Frame &start_frame, const KDL::Vector &arc_center, const KDL::Vector &arc_dir,
                           const KDL::Rotation &end_rot, const double angle, const double max_vel, const double max_acc,
                           const VELOCITY_PROFILE vel_prof=TRAP);

    /*!
     * \brief GenerateComposedMotion Generates a path composed by several segments, and rounds the transition between segments.
     *
     *  \note The routines are now robust against segments that are parallel.
     *  When the routines are parallel, no rounding is needed, and no attempt is made add constructing a rounding arc.
     *  (It is still not possible when the segments are on top of each other)
     *  Note that you can only rotate in a deterministic way over an angle less then M_PI!
     *  With an angle == M_PI, you cannot predict over which side will be rotated.
     *  With an angle > M_PI, the routine will rotate over 2*M_PI-angle.
     *  If you need to rotate over a larger angle, you need to introduce intermediate points.
     *  So, there is a common use case for using parallel segments.
     *
     * \param path_frames Set of frames (relative to robot base), through which the robot should pass.
     * \param radius Radius of transition between segments.
     * \param vp Velocity profile for the generated trajectory segment.
     */
    void GenerateComposedMotion(const std::vector< KDL::Frame > &path_frames, const double radius, const VELOCITY_PROFILE vp=TRAP);
    /*!
     * \copydoc GenerateComposedMotion
     */
    void GenerateComposedMotion(const std::vector< KDL::Frame > &path_frames, const double radius, const double max_vel,
                                const double max_acc, const VELOCITY_PROFILE vp=TRAP);

    /*!
     * \brief GenerateNullspaceMotion
     * \param cur_psi
     * \param ref_psi
     * \param psi_vec
     * \param time_vec
     */
    void GenerateNullspaceMotion(const double cur_psi, const double ref_psi, std::vector<double> &psi_vec, std::vector<double> &time_vec);

    /*!
     * \brief AddPauseSegment
     * \param pause_time
     * \param current_frame
     */
    void AddPauseSegment(const double pause_time, const KDL::Frame &current_frame);

    /*!
     * \brief GetTargetTrajectory Returns the target frames and associated times.
     * Compiles the KDL::Trajectory_Segment(s) into a KDL::Trajectory, and samples the total trajectory in frames and associated times,
     * according to the defined time differential.
     *
     * \param target_frames Vector containing the frames of the last generated trajectory.
     * \param target_frames_times Vector containing the time associated with the target_frames.
     * \param destroy Boolean variable that specifies whether the vector of trajectory segments is to be destroyed (default true).
     */
    bool GetTargetTrajectory(std::vector< KDL::Frame > &target_frames, std::vector<double> &target_frames_times, const bool destroy=true);

    /*!
     * \brief SampleLinearTrajectory samples the linear path returning a vector of 'step-spaced' transformations on this trajectory.
     * \param start_frame Initial frame of the end-effector (usually related to the base frame).
     * \param end_frame Target (final) frame of the end-effector, with respect to the same reference of the start_frame.
     * \param step Position path step-size.
     * \param frames Output vector of frames.
     */
    void SampleLinearTrajectory(const KDL::Frame &start_frame, const KDL::Frame &end_frame, const double step, std::vector<KDL::Frame> &frames);

    /*!
     * \brief PrintToFile stores a vector of frames in a file, following the CSV format.
     * \param target_frames Vector of frames to be stored in a file.
     * \param path Path to file.
     */
    void PrintToFile(const std::vector< KDL::Frame > &target_frames, const std::string &path);

private:
    /*!
     * \brief SelectVelocityProfile generates a new velocity profile object from the selected option, enum [VELOCITY_PROFILE](\ref VELOCITY_PROFILE).
     * Velocity profiles Dirac and Rectangular are disabled due to infinite velocity and accelerations involved.
     * \param vel_prof Enum [VELOCITY_PROFILE](\ref VELOCITY_PROFILE), selects the desired velocity profile.
     * \param starting If the trajectory segment is the first one.
     * \return KDL::VelocityProfile object returned by reference.
     */
    KDL::VelocityProfile* SelectVelocityProfile(const VELOCITY_PROFILE vel_prof, const bool starting=true);
    KDL::VelocityProfile* SelectVelocityProfile(const VELOCITY_PROFILE vel_prof, const double max_vel, const double max_acc, const bool starting=true);

};

#endif //CARTESIAN_TRAJECTORY_GENERATOR
