/*****************************************************************************
  File: point_to_point_trajectory_generator.hpp

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

#ifndef POINT_TO_POINT_TRAJECTORY_GENERATOR_HPP
#define POINT_TO_POINT_TRAJECTORY_GENERATOR_HPP

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>

#include <memory>
#include <vector>

using namespace std;

/*!
 * \brief The PointToPointTrajectoryGenerator class Joint Motion Point to Point Online Trajectory Generation
 * Implemented using Reflexxes Type II library.
 * - Position-based control, with phase synchronization when possible.
 * - Offline loop updating next joint positions in point-to-point.
 * - Online loop updating position/velocity based on updated inputs.
 */
class PointToPointTrajectoryGenerator {

public:
    /*!
     * \brief PointToPointTrajectoryGenerator Initializes Reflexxes API, Input and Output parameters.
     * Setup Max Velocity, Acceleration and Jerk, and select to work with all joints at the same time.
     * Setup flags (default):
     * - PHASE_SYNCHRONIZATION_IF_POSSIBLE
     * - KEEP_TARGET_VELOCITY
     * - EnableTheCalculationOfTheExtremumMotionStates = true
     * \param n_joints Number of joints
     * \param cycle_time Iteration time interval
     * \param max_vel Vector with maximum joint velocities
     * \param max_acc Vector with maximum joint accelerations
     * \param max_jerk Vector with maximum joint jerk
     */
    PointToPointTrajectoryGenerator(const unsigned int n_joints, const double cycle_time, const double max_vel, const double max_acc, const double max_jerk);
    ~PointToPointTrajectoryGenerator();

private:
    const unsigned int m_nj;
    double m_dt;
    const double m_v_max;
    const double m_a_max;
    const double m_j_max;

    std::shared_ptr<ReflexxesAPI> RML;
    std::shared_ptr<RMLPositionInputParameters> IPpos;
    std::shared_ptr<RMLPositionOutputParameters> OPpos;
    std::shared_ptr<RMLVelocityInputParameters> IPvel;
    std::shared_ptr<RMLVelocityOutputParameters> OPvel;

    RMLPositionFlags Flagspos;
    RMLVelocityFlags Flagsvel;
    int ResultValue;

public:
    int get_ResultValue();

public:
    /*!
     * \brief InitiateNewPositionMotion Creates first segment of a position controlled joint trajectory.
     * \param current_joint_positions Current joint positions.
     * \param current_joint_velocities Current joint velocities.
     * \param target_joint_positions Target joint positions.
     * \param max_rel_vel Maximum relative velocity
     * \return True if ResultValue was RML_FINAL_STATE_REACHED
     */
    bool InitiateNewPositionMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                           const vector<double> &target_joint_positions, const double max_rel_vel = 1.0);

    /*!
     * \brief InitiateNewVelocityMotion Creates first segment of a velocity controlled joint trajectory.
     * \param current_joint_positions Current joint positions.
     * \param current_joint_velocities Current joint velocities.
     * \param target_joint_velocities Target joint positions.
     * \param max_rel_vel Maximum relative velocity
     * \return
     */
    bool InitiateNewVelocityMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                           const vector<double> &target_joint_velocities, const double max_rel_vel = 1.0);

    /*!
     * \brief GenerateNextPositionMotionState Generate next position iteration from current segment based on current and target positions.
     * \param next_joint_positions Next joint positions of the current segment.
     * \param current_joint_positions Current joint positions
     * \return Return ResultValue from Reflexxes API:
     * - During trajectory execution should return RML_WORKING;
     * - At reaching the final pose, it returns RML_FINAL_STATE_REACHED;
     * - Else returns an error code.
     */
    int GenerateNextPositionMotionState(vector<double> &next_joint_positions, const vector<double> &current_joint_positions);

    int GenerateNextVelocityMotionState(vector<double> &next_joint_positions, vector<double> &next_joint_velocities,
                                        const vector<double> &current_joint_positions); //should pass as parameters cjp and cjv

    /*!
     * \brief RecoverFromErrorState If Reflexxes goes into an error state, it needs to recover to continue normal functioning.
     */
    void RecoverFromErrorState();

public:
    /*!
     * \brief AdaptPositionMotion Generates a new motion segment based on the new inputs for target position and velocity.
     * This method is called upon the event of a new sensory-input for position
     * \param current_joint_positions Current joint positions.
     * \param current_joint_velocities Current joint velocities.
     * \param target_joint_positions Target joint positions.
     * \param max_rel_vel Maximum relative velocity.
     */
    void AdaptPositionMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                     const vector<double> &target_joint_positions, const double max_rel_vel = 1.0);

    /*!
     * \brief AdaptVelocityMotion Generates a new motion segment based on the new inputs for target velocity.
     * \param current_joint_positions Current joint positions.
     * \param current_joint_velocities Current joint velocities.
     * \param target_joint_velocities Target joint velocities.
     */
    void AdaptVelocityMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                     const vector<double> &target_joint_velocities);

};

#endif //POINT_TO_POINT_TRAJECTORY_GENERATOR_HPP
