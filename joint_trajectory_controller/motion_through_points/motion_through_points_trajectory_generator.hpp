/*****************************************************************************
  File: motion_through_points_trajectory_generator.hpp

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

#ifndef MOTION_THROUGH_POINTS_TRAJECTORY_GENERATOR_HPP
#define MOTION_THROUGH_POINTS_TRAJECTORY_GENERATOR_HPP

#include <eigen3/Eigen/Dense>
#include <vector>

/*!
 * \brief The MotionThroughPointsTrajectoryGenerator class responsible for handling multi-point, multi-axis trajectories with time-synchronization.
 */
class MotionThroughPointsTrajectoryGenerator {

public:
    /*!
     * \brief MotionThroughPointsTrajectoryGenerator Initializes number of axis, iteration times, motion limits, etc.
     * \param n_joints Number of joints to move.
     * \param cycle_time Algorithm sample period.
     * \param max_vel Maximum joint velocity allowed.
     * \param max_acc Maximum joint acceleration allowed.
     * \param limits Joint limits.
     */
    MotionThroughPointsTrajectoryGenerator(const unsigned int &n_joints, const double &cycle_time, const double &max_vel, const double &max_acc,
                                           const std::vector<double> &limits);

private:
    const double m_dt;
    const double m_v_max;
    const double m_a_max;
    const unsigned int m_nj;
    std::vector<double> m_lim;
    std::vector<double> v_max;
    std::vector<double> a_max;

private:
    unsigned int m_num_it; //!< Sample index
    std::vector<std::vector<double> > m_q_it; //!< Sample positions
    std::vector<std::vector<double> > m_v_it; //!< Sample velocities
    std::vector<std::vector<double> > m_a_it; //!< Sample accelerations
    std::vector<double> m_t_it;  //!< Sample times
    unsigned int m_index_it;   //!< Path segment index

public:
    /*!
     * \brief InitiateNewPositionMotion Initiates a new motion through points, computing the polynomials that represent the path segment between points.
     * These segments are then sampled at the defined sample rate, for a smooth and finer motion.
     * \param current_joint_positions Current joint positions.
     * \param current_joint_velocities Current joint velocities.
     * \param via_points Vector of vector of joint positions corresponding to the end of a path segment (via-points).
     * \param time_via_points Vector with the same size of via_points, with the time associated with each via-point.
     * \param init_vel Initial joint velocities for path.
     * \param end_vel Final joint velocities for path.
     * \param init_acc Initial joint accelerations for path.
     * \param end_acc Final joint accelerations for path.
     * \return Returned codes:
     * Code  | Meaning
       ----- | -------------
       0     | New position motion initiated.
       1     | Number of received via-points does not match the time-parameterization.
       2     | Via-point exceeds the joint limits.
       3     | Velocity parameters exceed the maximum value.
       4     | Acceleration parameters exceed the maximum value.
     *
     */
    int InitiateNewPositionMotion(const std::vector<double> &current_joint_positions, const std::vector<double> &current_joint_velocities,
                           const std::vector<std::vector<double> > &via_points, const std::vector<double> &time_via_points,
                           const std::vector<double> &init_vel, const std::vector<double> &end_vel, const std::vector<double> &init_acc,
                           const std::vector<double> &end_acc);

    /*!
     * \brief GenerateNextPositionMotionState Returns the next set of joint positions (next time sample) according to the current time index and path segment.
     * \param next_joint_positions Set of target joint positions.
     * \return Returned codes:
     * Code  | Meaning
       ----- | -------------
       0     | Iterating through path segments.
       1     | Final motion state reached.
     */
    int GenerateNextPositionMotionState(std::vector<double> &next_joint_positions);

private:
    /*!
     * \brief ComputePolynomials Computes the polynomial's coefficients that connect the via-points for a single joint.
     * \param via_points 'Coarse' path points.
     * \param time_via_points Time law of path motion.
     * \param initial_velocity Initial path velocity.
     * \param end_velocity Final path velocity.
     * \param initial_acceleration Initial path acceleration.
     * \param end_acceleration Final path acceleration.
     * \param poly Vector of vector of polynomial coefficients. Each vector index contains a vector of 4 coefficients (cubic polynomial), where each index
     * corresponds to a path segment.
     */
    bool ComputePolynomials(const std::vector<double> &via_points, const std::vector<double> &time_via_points, const double &initial_velocity,
                                   const double &end_velocity, const double &initial_acceleration, const double &end_acceleration,
                                   std::vector<std::vector<double> > &poly);

    /*!
     * \brief PrintToFile Auxiliary function to print path via-points and finer positions inbetween (according to polynomials).
     * The saving directory is hard-coded in the function: "/home/carlos/MATLAB/Debug/01_jtc.csv" ...
     * \param qs Vector of starting joint positions.
     * \param cq Vector of vector of joint positions.
     * \param t Vector of time samples according to qv.
     */
    void PrintToFile(const std::vector<double> &qs, const std::vector<std::vector<double> > &cq, const std::vector<double> &t);
};

#endif //MOTION_THROUGH_POINTS_TRAJECTORY_GENERATOR_HPP
