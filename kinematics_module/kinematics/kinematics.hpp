/*****************************************************************************
  File: kinematics.hpp

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

#ifndef OROCOS_KINEMATICS_MODULE_KINEMATICS_HPP
#define OROCOS_KINEMATICS_MODULE_KINEMATICS_HPP

#include <kdl/frames.hpp>
#include <eigen3/Eigen/Dense>
#include <boost/icl/interval_set.hpp>
#include <vector>

//! Signal function - returns signed int value
template <typename T> int sgn(T val) {
    return (T(0) < val + std::numeric_limits<T>::epsilon()) - (val < T(0));
}

/*!
 * \brief The KinematicSolver class Solves Forward and Inverse kinematics, and deals with the manipulator redundancy.
 */
class KinematicSolver {

public:
    enum MANIP {AugmentedTranslational, AugmentedRotational, OriginalTranslational, OriginalRotational};

public:
    /*!
     * \brief KinematicsSolver Kinematics solver for a 7 DOF SRS non-offset robotic manipulator [KUKA LBR IIWA].
     * This constructor is responsible for assigning the robot specifications, needed to develop the kinematic expressions.
     * \param nj Number of joints.
     * \param tol Calculations tolerance.
     * \param links Vector with link lengths.
     * \param limits Vector with maximum joint limits (mirrored).
     * \param dh Denavit-Hartenberg parameters for the arm.
     */
    KinematicSolver(const unsigned int nj, const double tol, const std::vector<double> &links,
                    const std::vector<double> &limits, const std::vector<std::vector<double> > &dh,
                    const double Kappa, const double alpha, const double max_change,
                    const double sing_interval, const double detect_proxim);
    ~KinematicSolver();

private:
    unsigned int m_nj;
    double m_tol;
    double m_K;
    double m_alpha;
    double m_max_change_ns;
    double m_sing_interval;
    double m_detect_proxim;
    std::vector<double> m_lnk;
    std::vector<double> m_lim;
    // dh[joint_index][Denavit-Hartenberg params]
    // [a, alpha, d, theta]
    std::vector<std::vector<double> > m_dh;

/////////////////////////////////////////////////  SOLVE KINEMATICS  /////////////////////////////////////////////////
public:
    /*!
     * \brief ForwardKinematics Transforms a set of joint positions in a transformation that describes the end-effector position
     * and orientation in the robot base frame.
     * \param joints Joint positions.
     * \param gc Global configuration (output)
     * \param psi Robot arm angle (output)
     * \param pose Transformation of end-effector in the robot base frame (output)
     */
    void ForwardKinematics(const std::vector<double> &joints, unsigned int &gc, double &psi, KDL::Frame &pose);

    /*!
     * \brief SingleInverseKinematics Returns the joint positions vector that solves the IK for the given manipulator.
     * Requires a target end-effector pose, the robot global configuration, and the arm angle (psi)
     * \param pose End-effector pose in the robot base frame.
     * \param gc Robot global configuration.
     * \param psi Robot arm angle.
     * \param joints Joint positions (output).
     * \return Returns true if a solution is possible, false if a solution is outside workspace or if it violates joint limits.
     */
    bool SingleInverseKinematics(const KDL::Frame &pose, const unsigned int gc, const double psi, std::vector<double> &joints);

    /*!
     * \brief IterationInverseKinematics Calculates position Inverse Kinematics solutions every loop changing the arm angle on the fly.
     * Current variables are used to calculate next joint positions only.
     * \param pose Target transformation pose relative to robot base
     * \param gc Current robot configuration parameter (GC)
     * \param st_psi Current robot arm angle (psi)
     * \param joints Next joint positions (output)
     * \param new_psi New arm angle (psi) (output)
     * \return Return code:
     * Code  | Meaning
     * ----- | -------------
     * 0     | Inverse Kinematics solution found.
     * 1     | Current PSI not in a feasible interval.
     * 2     | Trajectory goes beyond manipulator reach.
     */
    int IterationInverseKinematics(const KDL::Frame &pose, const unsigned int gc, const double st_psi, std::vector<double> &joints, double &new_psi);

    /*!
     * \brief BatchInverseKinematics Calculates position Inverse Kinematics solutions for a batch of poses.
     * Analyses the target poses to follow to compute the best Nullspace path to follow
     * \param poses Target transformation poses relative to robot base
     * \param gc Initial robot configuration (kept during trajectory)
     * \param st_psi Initial robot arm angle (changes during trajectory)
     * \param joints Output joint positions vector
     * \return Return code:
     * Code  | Meaning
     * ----- | -------------
     * 0     | Inverse Kinematics solution found.
     * 1     | Current PSI not in a feasible interval.
     * 2     | Trajectory goes beyond manipulator reach.
     */
    int BatchInverseKinematics(const std::vector<KDL::Frame> &poses, const unsigned int gc, const double st_psi, std::vector<std::vector<double> > &joints);

    /*!
     * \brief BatchInverseKinematicsIntervalAvoidance
     * \param poses
     * \param gc
     * \param st_psi
     * \param joints
     * \return Return code:
     * Code  | Meaning
     * ----- | -------------
     * 0     | Inverse Kinematics solution found.
     * 1     | Current PSI not in a feasible interval.
     * 2     | Trajectory goes beyond manipulator reach.
     * 3     | No solutions in current configuration.
     */
    int BatchInverseKinematicsIntervalAvoidance(const std::vector<KDL::Frame> &poses, const unsigned int gc, const double st_psi, std::vector<std::vector<double> > &joints);

    /*!
     * \brief SinglePoseIntervals
     * \param pose
     * \param gc
     * \param intervals
     * \return
     */
    bool SinglePoseIntervals(const KDL::Frame &pose, const int gc, boost::icl::interval_set<double> &interval);

    /*!
     * \brief SinglePoseScores Computes the arm angle scores for a single pose and GC.
     * Useful when one needs to adjust the arm position without going beyond limits / singularities.
     * \param pose End-effector pose in the robot base frame.
     * \param gc Robot global configuration.
     * \param psi Robot arm angle.
     * \param scores Scores for the passed paramters (output).
     * \return Returns true if possible, false if all poses are outside working range.
     */
    bool SinglePoseScores(const KDL::Frame &pose, const unsigned int gc, const double psi, std::vector<double> &scores);

    /*!
     * \brief BatchNullspaceInverseKinematics Calculates Inverse Kinematics solution within Nullspace.
     * The algorithm generates IK solutions for the same pose, with different psi values.
     * \param pose constant pose to maintain during nullspace motion
     * \param psi_vec vector of psi values for the nullspace
     * \param joints vector of vector of joint positions that correspond to the trajectory
     * \return true if all values in trajectory are possible, false otherwise
     */
    bool BatchNullspaceInverseKinematics(const KDL::Frame &pose, const unsigned int gc, const std::vector<double> &psi_vec, std::vector<std::vector<double> > &joints);

    /*!
     * \brief GeometricJacobian Returns the current Geometric Jacobian matrix of the manipulator. For more information about the method consult
     * the Robotics: Modelling, Planning and Control.
     * \param joints current joint positions
     * \param pose current pose
     * \param jacobian jacobian matrix
     */
    void GeometricJacobian(const std::vector<double> &joints, Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor, 6, 7> &jacobian);

    /*!
     * \brief Manipulability
     * \param joints
     * \param opt
     * \return
     */
    double Manipulability(const std::vector<double> &joints, MANIP opt);

/////////////////////////////////////////////////  SOLVE NULLSPACE  /////////////////////////////////////////////////
public:
    /*!
     * \brief CheckNullspace For a list of reference frames, computes the possible global configurations and nullspace intervals for the given manipulator kinematics.
     * \param target_frames List of end-effector frames relative to the robot base.
     * \param max_nschange Maximum allowed psi (elbow) change between iterations.
     * \param gc_order List of global configuration with possible solutions. (output)
     * \param initial_psi Vector of vectors of intervals with initial possible elbow positions (psi). (output)
     * \return True if there is at least on global configuration that allows the motion to be performed.
     */
    bool CheckNullspace(const std::vector<KDL::Frame> &target_frames, std::vector<unsigned int> &gc_order, std::vector<std::vector<double> > &initial_psi);

    bool CheckConfigurationNullspace(const std::vector<KDL::Frame> &target_frames, const int conf, std::vector<double> &initial_psi);

    /*!
     * \brief ArmAngleScore Calculates the nullpace sample scores and possible values according to current psi value and nullspace interval received.
     * Value calculated for a single global configuration.
     * \param interval Nullspace possible interval.
     * \param current_psi Current elbow position.
     * \param max_nschange Maximum allowed psi (elbow) change between iterations.
     * \param ns_scores Neighbor elbow position scores according to current position. (output)
     */
    void ArmAngleScore(const boost::icl::interval_set<double> &interval, const double current_psi, std::vector<double> &ns_scores);

    /*!
     * \brief GlobConfOrder Creates a ordered vector of robot configurations closer to current configuration.
     * Only possible global configurations are considered.
     * \param gc Current global configuration.
     * \param possible Vector of possible global configurations.
     * \param gc_order Ordered vector of possible global configurations.
     * Wrist changes are preferrable to elbow changes.
     * Elbow changes are preferrable to shoulder changes. (output)
     */
    void GlobConfOrder(const int gc, std::vector<int> &possible, std::vector<int> &gc_index, std::vector<int> &gc_order);

private:
    /*!
     * \brief SingleInverseKinematics Internal function that executes SingleInverseKinematics using Eigen libraries.
     */
    bool SingleInverseKinematics(const Eigen::Matrix4d &pose, const unsigned int gc, const double psi, std::vector<double> &joints);
    /*!
     * \brief IterationInverseKinematics Internal function that executes IterationInverseKinematics using Eigen libraries.
     */
    int IterationInverseKinematics(const Eigen::Matrix4d &pose, const unsigned int gc, const double st_psi, std::vector<double> &joints, double &new_psi);
    /*!
     * \brief BatchInverseKinematics Internal function that executes BatchInverseKinematics using Eigen libraries.
     */
    int BatchInverseKinematics(const std::vector<Eigen::Matrix4d> &poses, const unsigned int gc, const double st_psi, std::vector<std::vector<double> > &joints);
    /*!
     * \brief BatchInverseKinematicsIntervalAvoidance Internal function that executes BatchInverseKinematics using Eigen libraries.
     */
    int BatchInverseKinematicsIntervalAvoidance(const std::vector<Eigen::Matrix4d> &poses, const unsigned int gc, const double st_psi,
                                                const std::vector<boost::icl::interval_set<double> > &shadow, std::vector<std::vector<double> > &joints);
    /*!
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
    /*!
     * \brief CoeffCalculation Calculates the joint matrices coefficients required to compute the joint position as a function of the elbow position.
     * For more information check: Position-Based Kinematics for 7-DoF Serial Manipulators with Global Configuration Control, Joint Limit and Singularity Avoidance.
     * \param pose Target end-effector pose from robot base frame.
     * \param gc Target global configuration.
     * \param coeffs Coefficients calculated. (output)
     * \param j4 Elbow joint value (4th joint). (output)
     * \return True if pose is within workspace.
     */
    bool CoeffCalculation(const Eigen::Matrix4d &pose, const unsigned int gc, std::vector<std::vector<double> > &coeffs, double &j4);
    /*!
     * \brief InverseKinematics The complementary method to calculate Inverse Kinematics once the coefficients are calculated.
     * It basically returns the values of the joint positions as a function of these coefficients, the elbow position and the global configuration.
     * \param coeffs Coefficients.
     * \param j4 Elbow joint position.
     * \param gc Global configuration.
     * \param psi Elbow position (psi).
     * \param joints Target joint positions. (output)
     */
    void InverseKinematics(const std::vector<std::vector<double> > &coeffs, const double j4, const unsigned int gc, const double psi, std::vector<double> &joints);
    /*!
     * \brief BatchNullspaceInverseKinematics Internal function that executes BatchNullspaceInverseKinematics using Eigen libraries.
     */
    bool BatchNullspaceInverseKinematics(const Eigen::Matrix4d &pose, unsigned int gc, const std::vector<double> &psi_vec, std::vector<std::vector<double> > &joints);

//////////////////////////////////////////////  NULLSPACE CALCULATION  //////////////////////////////////////////////
private:
    /*!
     * \brief PsiFunction Solves the psi(theta) function for a joint.
     * \param coeff Coefficients of the joint. Takes either 3 (hinge) or 6 (pivot) parameters depending on the type of joint.
     * \param theta Joint position, usually the joint limit.
     * \param psi Related elbow position. (output)
     */
    void PsiFunction(const std::vector<double> &coeff, const double theta, std::vector<double> &psi);
    /*!
     * \brief PivotLimits Computes the arm angle values that correspond to the current pivot joint position limits.
     *
     * Stationary points not always map the theta max or min. Specially when we
       have configurations with negative shoulder, elbow or wrist values, we
       tend to have discontinuities in the function theta(psi) eq.23.
       These discontinuities are not contemplated in Shimizu paper, due to the
       values reported and the joint limits of his setup.
       These discontinuities are not singularities, but are related to the
       domain of tan. Thus, when a monotonic function reaches -pi or pi a
       discontinuity happens, and the theta value shifts 2*pi or -2*pi.
       Since it crosses the joint limit values, they create a new interval where
       psi values lead to joint limit violation.
     * \param coeff Coefficients of the current joint.
     * \param limit Joint angle limit.
     * \param intervals Elbow position corresponding values. Psi interval.
     */
    void PivotLimits(const std::vector<double> &coeff, const double limit, std::vector<double> &intervals);
    /*!
     * \brief HingeLimits Computes the arm angle values that correspond to the current hinge joint position limits.
     * \param coeff Coefficients of the current joint.
     * \param conf Sign of current hinge joint position.
     * \param limit Joint angle limit.
     * \param intervals Elbow position corresponding values. Psi interval.
     */
    void HingeLimits(const std::vector<double> &coeff, const int conf, const double limit, std::vector<double> &intervals);
    /*!
     * \brief PsiIntervals Computes the possible elbow position intervals for each joint (least the elbow) given the coefficients and global configuration.
     * \param coeffs Coefficients for each joint.
     * \param gc Global configuration.
     * \param feasible Intersected feasible interval of possible elbow positions for each joint.
     */
    void PsiIntervals(const std::vector<std::vector<double> > &coeffs, const unsigned int gc,
                      boost::icl::interval_set<double> &feasible);
    /*!
     * \brief CheckSolution In the event that two solutions are returned, it checks which one is valid.
     * \param coeff Coefficients of the current joint. It takes either 3 or 6 parameters depending on the type of joint.
     * \param jl Current joint limits.
     * \param psi Current elbow position.
     * \return True if solution is valid.
     */
    bool CheckSolution(const std::vector<double> &coeff, const double jl, const double psi);
    /*!
     * \brief BatchPsiIntervals Computes a vector of possible elbow position intervals from a vector of target poses.
     * \param poses Target (trajectory) poses.
     * \param gc Current global configuration.
     * \param intrvls Computed possible elbow position intervals. (output)
     * \return True if poses are within workspace.
     */
    bool BatchPsiIntervals(const std::vector<KDL::Frame> &poses, const unsigned int gc, std::vector<boost::icl::interval_set<double> > &intrvls);
    /*!
     * \brief IterationNewPsi Calculates the new elbow position online, granted the current interval and previous elbow position.
     * \param feasible Interval of feasible elbow positions.
     * \param cur_psi Current psi.
     * \param new_psi Computed new psi. Moving away from singularities and joint limits. (output)
     * \return True if psi is in a feasible interval.
     */
    bool IterationNewPsi(const boost::icl::interval_set<double> &feasible, const double cur_psi, double &new_psi);

    //! \brief GlobConfOrder Creates a ordered vector of robot configurations closer to current configuration.
    void GlobConfOrder(const unsigned int gc, std::vector<unsigned int> &gc_order);
    /*! \brief CreateShadowNS Creates the shadow of nullspace obstacles using a backtrack scheme.
     * All obstacles are avoidable moving at most the max nschange allowed.
     * \return True if there is a possible path. If any of the intervals is empty than the trajectory is impossible.
     */
    bool ShadowIntervals(const std::vector<boost::icl::interval_set<double> > &intervals, const double max_nschange, std::vector<boost::icl::interval_set<double> > &shadow);

    /*!
     * \brief pseudoInverse Template function to calculate the pseudo inverse of an Eigen matrix.
     * \param a Input matrix.
     * \param epsilon
     * \return
     */
    template<typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }

/////////////////////////////////////////////  PSI INTERVAL MANIPULATION  /////////////////////////////////////////////
private:
    /*!
     * \brief IntersectIntervals Intersects sets of intervals.
     * \param intervals Input vector of intervals.
     * \param interval_out Output vector of intersected intervals.
     */
    void IntersectIntervals(const std::vector<std::vector<double> > &intervals, boost::icl::interval_set<double> &interval_out);
    /*!
     * \brief IntersectIntervals Internal function that executes IntersectIntervals using Boost::icl libraries (intervals).
     */
    void IntersectIntervals(const std::vector<boost::icl::interval_set<double> > &intervals, boost::icl::interval_set<double> &interval_out);

/////////////////////////////////////////////  AUXILIARY AND CONVERTIONS  /////////////////////////////////////////////
public:
    /*!
     * \brief skew_symmetric Computes the skew symmetric matrix from a vector.
     * \param vec Input vector.
     * \return Returns the skew symmetric matrix.
     */
    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &vec);
    /*!
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
    /*!
     * \brief eigen2kdlframe Converts a Eigen::Matrix4d into a KDL::Frame.
     * \param mat Input Eigen::Matrix4d.
     * \return Output KDL::Frame.
     */
    KDL::Frame eigen2kdlframe(const Eigen::Matrix4d &mat);
    /*!
     * \brief kdlframe2eigen Converts a KDL::Frame into a Eigen::Matrix4d.
     * \param T Input KDL::Frame.
     * \return Output Eigen::Matrix4d.
     */
    Eigen::Matrix4d kdlframe2eigen(const KDL::Frame &T);
    /*!
     * \brief iclinterval2vector Converts a Boost::icl::interval_set into a std::vector.
     * \param interval Input Boost::icl::interval_set.
     * \return Output std::vector.
     */
    std::vector<double> iclinterval2vector(const boost::icl::interval_set<double> &interval);
    /*!
     * \brief iclinterval2vector Converts a std::vector into a Boost::icl::interval_set.
     * \param interval Input std::vector.
     * \return Output Boost::icl::interval_set.
     */
    boost::icl::interval_set<double> vector2iclinterval(const std::vector<double> &interval);

    void printtofile(const char *file, std::vector<double> &vec);
};

#endif // OROCOS_KINEMATICS_MODULE_KINEMATICS_HPP
