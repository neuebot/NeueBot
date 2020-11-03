/*****************************************************************************
  File: kinematics_module-component.hpp

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

#ifndef OROCOS_KINEMATICS_MODULE_COMPONENT_HPP
#define OROCOS_KINEMATICS_MODULE_COMPONENT_HPP

#include "kinematics.hpp"
#include "kinematics_module-errorcode.hpp"
#include "kinematics_module_types.hpp"

#include <kdl/frames.hpp>
#include <boost/icl/interval_set.hpp>
#include <rtt/RTT.hpp>

#include <utility>
#include <memory>

//To sort the conf, psi and frames together based on the augmented manipulability factorized index
struct Approach{
    int _conf;
    double _psi;
    KDL::Frame _frame;
    double _manip;

    Approach(int conf, double psi, KDL::Frame &frame, double manip) :
        _conf(conf),
        _psi(psi),
        _frame(frame),
        _manip(manip)
    {}
};

class KinematicsModule : public RTT::TaskContext {

    /*!
     * \brief The KIN_MODE enum Specifies the working mode of the component.
     */
    enum KIN_MODE {
        TFC = 0, //!< Trajectory Following Control
        OTG = 1, //!< Online Trajectory Generation
    };

public:
    KinematicsModule(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

protected:
    unsigned int m_nj;
    double m_tol;
    std::vector<double> m_lim;
    std::vector<double> m_lnk;
    double m_K;
    double m_alpha;
    double m_max_change_ns;
    double m_sing_interval;
    double m_detect_proxim;

    std::vector<double> m_dh_a;
    std::vector<double> m_dh_alpha;
    std::vector<double> m_dh_d;
    std::vector<double> m_dh_theta;
    std::vector<std::vector<double> > m_dh; //dh[joint_index][Denavit-Hartenberg params]

private:
    /*!
     * \note Correct thing would be to use std::unique_ptr instead of std::shared_ptr,
     * However the compiler we are using is gcc-4.8 that supports std::c++11 and only partially std::c++14.
     * The std::make_unique<T> is not implemented in c++11 standard. For convenience and to avoid using
     * std::unique_ptr \a new command, we use std::shared_ptr instead.
     */
    std::shared_ptr<KinematicSolver> m_kinematic_solver; //!< Smart ponter to [KinematicSolver](\ref KinematicSolver)
    KIN_MODE m_mode;    //!< Component operating mode
    KMErrorCode m_ec;   //!< Error codes object
    KDL::Frame m_current_frame; //!< Current end-effector frame
    double m_current_psi;       //!< Current elbow position (psi)
    unsigned int m_current_gc;  //!< Current global configuration
    std::vector<double> m_current_joint_positions; //!< Current joint positions

private:
    /*!
     * \brief SetOnlineTrajectoryGeneration Set the component working mode to Online Trajectory Generation. The component receives target frames
     * for the robot end-effector to reach, and generates in run-time solutions that use redundancy to avoid limits and singularities.
     * \param otg True to ser working mode to Online Trajectory Generation
     */
    void SetOnlineTrajectoryGeneration(bool otg);

    /*!
     * \brief CalculateJointTrajectory Receives a set of frames (end-effector from base) and computes the respective joint values, adjusting the robot
     * pose (redundancy) to move away from limits and singularities. Once a trajectory is assigned, the robot maintains the configuration.
     * \param target_frames Trajectory frames in Cartesian space.
     * \param output_joints Calculated output joint positions.
     * \return Returned codes:
     * Code  | Meaning
       ----- | -------------
       0     | Joint trajectory calculated.
       1     | At least one of the frames is outside the workspace.
       2     | Current arm angle (psi) not in a feasible interval to execute trajectory.
       3     | Output solution violates joint limits (should not happen).
     */
    bool CalculateJointTrajectory(const std::vector<KDL::Frame> &target_frames, std::vector<std::vector<double> > &output_joints);

    /*!
     * \brief CalculateNullspaceJointTrajectory Generates a coordinated joint trajectory keeping the same end-effector frame, by moving
     * strictly in the robot Nullspace. Changing robot posture, without changing its pose.
     * \param current_frame Current end-effector pose
     * \param psi_vec Vector of target psi values to reach, starting in current
     * \param output_joints Output vector of target joint coordinates
     */
    bool CalculateNullspaceJointTrajectory(KDL::Frame current_frame, std::vector<double> &psi_vec,
                                           std::vector<std::vector<double> > &output_joints);

    /*!
     * \brief CalculateTrajectoryApproach Calculates the maximum augmented manipulability factorized score for the received possible
     * approach frames. The algorithm computes the highest score frame and nullspace parameter to reach the specified trajectory.
     * \param conf_desired Artificially imposed limits to the psi interval. Avoid elbow to reach undesired zones.
     * \param frames Vector of input frames with different approaches to a given trajectory.
     * \param conf_max Sorted vector of possible global configurations from the highest to the lowest score. Only the possible configurations
     * are numbered, others appear as a -1.
     * \param psi_max Sorted vector of best psi values.
     * \param frames_max Sorted vector of best approach frames.
     * \return True if there is at least one possible solution.
     */
    bool CalculateTrajectoryApproach(const RTT::vector_vector_double &conf_desired, const RTT::vector_frame &frames,
                                     RTT::vector_int &conf_max, std::vector<double> &psi_max, RTT::vector_frame &frames_max);

    /*!
     * \brief AnalyseTrajectory checks the Cartesian trajectory received against joint limits and singularities.
     * For each global configuration, the feasible intervals are calculated for the received trajectory and a final score found.
     * These scores are passed to the TaskSupervisor who handles this information to the user.
     * \param target_frames Sampled trajectory path.
     * \param max_nschange Maximum allowed psi (elbow) change between iterations.
     * \param possible Returns a vector of non-ordered global configurations. Only possible are listed. (output)
     * \param initial_interval Returns a vector of nullspace scores for each robot configuration. It is returned the first psi intervals
     * possible for each global configuration after shadowing previous values. Scoring is later. (output)
     * \return True if there is at least on global configuration that allows the motion to be performed.
     */
    bool AnalyseTrajectory(std::vector<KDL::Frame> &target_frames, std::vector<int> &possible, std::vector<std::vector<double> > &initial_interval);

    /*!
     * \brief AnalyseTrajectoryApproach Checks a branch (configuration) of the possible approach trajectories (w/ frame and psi).
     * The function checks the possible nullspace solution for the rest of the specified linear trajectory and returns a score
     * based on the distance to limits or singularities.
     * \param frames
     * \param gc Current configuration.
     * \param psi Current arm angle.
     * \param scores Output score vector for the initial interval (see CheckNullspace() to understand how the initial interval
     * reflects the trajectory nullspace navigation).
     * \return
     */
    bool AnalyseTrajectoryApproach(const RTT::vector_frame &frames, const int gc, const double psi, std::vector<double> &scores);

    /*!
     * \brief ScoresTrajectory Takes the output of the trajectory analysis and generates the scores for new psi and global configuration
     *  based on the current values.
     *  Test: Try to include the interval available for the robot to move to the new configuration
     * \param possible Possible Global Configurations.
     * \param initial_interval Initial interval for psi (nullspace).
     * \param scores Vector of values relating to the psi scores for the current interval. (output)
     * \return Returned codes:
     * Code  | Meaning
       ----- | -------------
       0     | Current psi is fitting to perform the trajectory.
       1     | Change psi to perform the trajectory.
       2     | Change gc to perform the trajectory.
       3     | No psi in any gc allows to perform trajectory.
     */
    int ScoresTrajectory(std::vector<int> &possible, std::vector<std::vector<double> > &initial_interval,
                        std::vector<int> &gc_alt, std::vector<std::vector<double> > &scores);

    /*!
     * \brief ScoresCurrentPoseCurrentGC Compute the NS interval for the specified end-effector frame, for a single robot configuration.
     * \param max_nschange Maximum allowed psi (elbow) change between iterations.
     * \param ns_scores Vector of values relating to the psi scores for the current interval. (output)
     * \return success True if the current arm configuration is valid.
     */
    bool ScoresCurrentPoseCurrentGC(std::vector<double> &ns_scores);
    bool ScoresCurrentPoseAlternativeGC(const int gc, std::vector<double> &ns_scores);

private:
    /*!
     * \brief CheckWaypointsInWorkspace checks whether the frames of the waypoints are within the robot workspace before any further calculations.
     * \param target_frames Vector of waypoint frames.
     * \return True if all waypoints are within the robot workspace.
     */
    bool CheckWaypointsInWorkspace(const std::vector<KDL::Frame> &target_frames);

    /*!
     * \brief ForwardKinematics Calculates the Forward Kinematics solution for the specified frame, psi and global configuration.
     * \param joints Current joint values.
     * \param frame Current end-effector frame. (output)
     * \param gc Current global configuration. (output)
     * \param psi Current elbow position. (output)
     */
    void ForwardKinematics(std::vector<double> &joints, KDL::Frame& frame, unsigned int& gc, double& psi);
    /*!
     * \brief InverseKinematics Calculates the Inverse Kinematics solution for the specified frame, psi and global configuration.
     * \param target Target frame with relation to robot base.
     * \param gc Target Robot configuration.
     * \param psi Target elbow position.
     * \param joints Calculated joint positions. (output)
     * \return Returns true if position is possible within robot limits.
     */
    bool InverseKinematics(const KDL::Frame target, const unsigned int gc, const double psi, std::vector<double> &joints);

    /*!
     * \brief AdjustedInverseKinematics Calculates the best fitting Inverse Kinematics solution. Tries to keep the values of
     * psi and GC unchanged. If that is not possible, it makes the smallest adjustment to make it happen.
     * \param target Target frame with relation to robot base.
     * \param joints Calculated joint positions. (output)
     * \return Returns true if any position is possible within robot limits.
     */
    bool AdjustedInverseKinematics(const KDL::Frame target, std::vector<double> &joints);

    /*!
     * \brief SortBestApproach
     * \param conf_max
     * \param psi_max
     * \param frames_max
     * \param manip_max
     */
    void SortBestApproach(std::vector<int> &conf_max, std::vector<double> &psi_max, std::vector<KDL::Frame> &frames_max,
                          std::vector<double> &manip_max);

    /*!
     * \brief ScoreApproachFrames
     * \param frame
     * \param conf
     * \param interval
     * \param best_psi
     * \param best_manip
     * \return
     */
    bool ScoreApproachFrames(const KDL::Frame &frame, int conf, boost::icl::interval_set<double> &interval,
                             double &best_psi, double &best_manip);

    /**
     * @brief CalculateExternalForces Calculates the external forces at the end-effector, from the external torques felt
     * at each of the robot joints.
     * @param ext_torques External torques retrieved from the FRI component.
     * @param ext_forces External forces felt at the tip of the end-effector.
     */
    void CalculateExternalForces(const std::vector<double> &ext_torques, std::vector<double> &ext_forces);

    /*!
     * \brief PrintToFile Stores interval variables into a CSV file.
     * \param file_path path and name of file (*.csv)
     * \param joints Vector of vectors of joints with the coarse path defined.
     */
    void PrintToFile(const std::vector<std::vector<double> > &joints, const std::vector<double> &psis);

    void PrintTESTS(const std::vector<double> &j1t, const std::vector<double> &j3t, const std::vector<double> &j5t, const std::vector<double> &j7t);

protected:
    RTT::InputPort< RTT::posture > inport_current_robot_posture;
    RTT::InputPort< std::vector<double> > inport_current_joint_positions;
    RTT::InputPort< KDL::Frame > inport_online_frame;
    RTT::OutputPort< std::vector<double> > outport_online_joint_positions;

    RTT::InputPort< std::vector<double> > inport_external_joint_torques;
    RTT::OutputPort< std::vector<double> > outport_current_force;
    RTT::OutputPort< RTT::error_msg > outport_error_msg;
};

#endif // OROCOS_KINEMATICS_MODULE_COMPONENT_HPP
