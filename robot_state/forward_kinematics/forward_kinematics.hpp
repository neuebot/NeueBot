/*****************************************************************************
  File: forward_kinematics-component.hpp

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

#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP

#include <eigen3/Eigen/Dense>
#include <kdl/frames.hpp>

#include <vector>

//! Signal function - returns signed int value
template <typename T> int sgn(T val) {
    return (T(0) < val + std::numeric_limits<T>::epsilon()) - (val < T(0));
}

/**
 * @brief The ForwardKinematicSolver class Solves Forward kinematics.
 */
class ForwardKinematicSolver {

public:
    /**
     * \brief ForwardKinematicSolver Kinematics solver for a 7 DOF SRS non-offset robotic manipulator [KUKA LBR IIWA].
     * This constructor is responsible for assigning the robot specifications, needed to develop the kinematic expressions.
     * \param nj Number of joints.
     * \param tol Calculations tolerance.
     * \param links Vector with link lengths.
     * \param limits Vector with maximum joint limits (mirrored).
     * \param dh Denavit-Hartenberg parameters for the arm.
     */
    ForwardKinematicSolver(const unsigned int nj, const double tol, const std::vector<double> &links,
                    const std::vector<double> &limits, const std::vector<std::vector<double> > &dh);
    ~ForwardKinematicSolver();

//Interface KDL // Inside Eigen
public:
    /**
     * \brief ForwardKinematics Transforms a set of joint positions in a transformation that describes the end-effector position
     * and orientation in the robot base frame.
     * \param joints Joint positions.
     * \param gc Global configuration (output)
     * \param psi Robot arm angle (output)
     * \param pose Transformation of end-effector in the robot base frame (output)
     */
    void ForwardKinematics(const std::vector<double> &joints, unsigned int &rconf, double &psi, KDL::Frame &pose);

private:
    /**
     * \brief ReferencePlane Calculate the arm angle (psi), such that it is always defined.
     *
       The arm plane is defined by the plane formed by the xs, xe and xw points
       The reference plane is defined by the xs and xw and a xe0 (explained below)
       - A virtual robotic manipulator is created from the KUKA LBR iiwa manipulator
         structure. Equal in everything least the 3rd joint, which is fixed as 0.
       - Now we compute the Inverse Kinematics to place the virtual robot in the
         same pose as the real robot.
       - The elbow position of this virtual robot is xe0. Thus, with xs, xw and
         xe0 we form the reference plane
     * \param pose Target frame.
     * \param elbow Boolean variable that indicates the sign of the elbow joint (4th).
     * \param ref_plane Vector of normal to plane.
     * \param rot_elbow Rotation matrix that represents the elbow in relation to the robot base.
     * \return True if pose is within workspace.
     */
    bool ReferencePlane(const Eigen::Matrix4d &pose, const int elbow, Eigen::Vector3d &ref_plane, Eigen::Matrix3d &rot_elbow);

    /**
     * \brief denavit_hartenberg Computes the Denavit-Hartenberg standard notation matrix from the input parameters.
     * T = [cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a*cos(theta)]
           [sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a*sin(theta)]
           [       0.0,               sin(alpha),               cos(alpha),            d]
           [       0.0,                      0.0,                      0.0,          1.0]
     * \param a The length of the common normal
     * \param alpha The angle around the common normal to between the previous z-axis and current z-axis.
     * \param d The distance between the previous x-axis and the current x-axis, along the common normal
     * \param theta The angle around the z-axis between the previous x-axis and current x-axis.
     * \return Corresponding DH matrix.
     */
    Eigen::Matrix4d denavit_hartenberg(const double a, const double alpha, const double d, const double theta);

    /**
     * @brief eigen2kdlframe Converts Eigen::Matrix4d to KDL::Frame.
     * @param mat Input Eigen::Matrix4d.
     * @return frame Output KDL::Frame.
     */
    KDL::Frame eigen2kdlframe(const Eigen::Matrix4d &mat);

private:
    const unsigned int m_nj;
    const double m_tol;
    std::vector<double> m_lnk;
    std::vector<double> m_lim;
    std::vector<std::vector<double> > m_dh;
};

#endif //FORWARD_KINEMATICS_HPP
