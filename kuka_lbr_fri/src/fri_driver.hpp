/*****************************************************************************
  File: fri_driver.hpp

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

#ifndef OROCOS_FRI_DRIVER_HPP
#define OROCOS_FRI_DRIVER_HPP

///Persistent Thread-safe queue
#include "persistent_vector_queue.hpp"

/// Kuka Fast Robot Inteface (FRI) C++ SDK interface include files
/// The FRI headers below depend on code that can't be shared
/// This header helps the Java API work when FRI is not available.
/// See grl documentation for details on how to activate FRI.
#include "fri_joint_overlay_client.hpp"
#include "friUdpConnection.h"
#include "friClientApplication.h"

/// System includes
#include <memory>
#include <atomic>
#include <vector>

#define NJ 7

struct ConnectionProperties {
    std::string robot_type;
    std::string local_address;
    int local_port;
    std::string remote_address;
    int remote_port;
};

struct KukaState {
    std::atomic<KUKA::FRI::EClientCommandMode> client_mode;
    std::atomic<KUKA::FRI::EConnectionQuality> connection_quality;
    std::atomic<KUKA::FRI::EControlMode> control_mode;
    std::atomic<KUKA::FRI::EDriveState> drive_state;
    std::atomic<KUKA::FRI::EOperationMode> operation_mode;
    std::atomic<KUKA::FRI::EOverlayType> overlay_type;
    std::atomic<KUKA::FRI::ESafetyState> safety_state;
    std::atomic<KUKA::FRI::ESessionState> session_state;
};

//Client interface to FRI - to create real-time functionalities
//fri_driver should derive from LBRClient
class FriDriver : KUKA::FRI::LBRState{

public:
    FriDriver();

    bool FriConnect();
    void FriDisconnect();
    bool FriStep();
    bool isFriConnected();

    void SetTargetJointPositions(const std::vector<double> &tar);

    bool GetCurrentJointPositions(std::vector<double> &cur);
    bool GetInterpolatedJointPositions(std::vector<double> &ipo);
    bool GetCommandedJointPositions(std::vector<double> &com);
    bool GetCurrentJointTorques(std::vector<double> &curt);
    bool GetExternalJointTorques(std::vector<double> &extt);

    /*!
     * \brief getConnectionQuality Returns the client FRI module connection quality.
     * This parameter should be checked each iteration, with a returning value of (3) Excellent or (2) Good.
     * If the connection quality fallse below that value (1) Fair, (0) Poor, Operation should be terminated or
     * halted until the connection quality resumes to (3) or (2).
     * \return Quality of the connection internally calculated by FRI module:
     * Code  | Meaning
       ----- | -------------
       0     | POOR
       1     | FAIR
       2     | GOOD
       3     | EXCELLENT
     */
    int getConnectionQuality();

    /*!
     * \brief getSessionState Returns the current session state from the FRI module.
     * This parameter should be checked whenever any robot action is to be performed.
     * It can change between MONITORING and COMMANDING states, with the possibility of commanding the joint overlay
     * only during COMMANDING states. The SessionState is also dependent on the connection quality.
     * If the connection quality falls below (2) Good, the SessionState returns to Monitoring until quality improves.
     * \return Session state indicative handled internally by the FRI module:
     * Code  | Meaning
       ----- | -------------
       0     | IDLE
       1     | MONITORING_WAIT
       2     | MONITORING_ACTIVE
       3     | COMMANING_WAIT
       4     | COMMANDING_ACTIVE
     */
    int getSessionState();

    /*!
     * \brief Get an indicator for the current tracking performance of the commanded robot.
     *
     * The tracking performance is an indicator on how well the commanded robot
     * is able to follow the commands of the FRI client. The best possible value
     * 1.0 is reached when the robot executes the given commands instantaneously.
     * The tracking performance drops towards 0 when latencies are induced,
     * e.g. when the commanded velocity, acceleration or jerk exceeds the
     * capabilities of the robot.
     * The tracking performance will always be 0 when the session state does
     * not equal COMMANDING_ACTIVE.
     *
     * \return current tracking performance
     */
    double getTrackingPerformance();

private:
    void CopyState(const KUKA::FRI::LBRState &state);

private:
    std::shared_ptr<KUKA::FRI::UdpConnection> fri_connection;
    std::shared_ptr<FRIJointOverlayClient> fri_client;
    std::shared_ptr<KUKA::FRI::ClientApplication> fri_client_application;

    KUKA::FRI::LBRState fri_state;
    std::shared_ptr<KUKA::FRI::LBRCommand> fri_command;

    bool fri_connected;

private:
    ConnectionProperties conn_prop;
    KukaState kuka_state;
    //Joint Position
    PersistentVectorQueue<double> pvq_mjpos; //measured
    PersistentVectorQueue<double> pvq_ijpos; //interpolator
    PersistentVectorQueue<double> pvq_cjpos; //commanded
//    PersistentVectorQueue<double> pvq_tjpos; //target

    //Joint Torques
    PersistentVectorQueue<double> pvq_mjtor; //measured
    PersistentVectorQueue<double> pvq_ejtor; //external
    PersistentVectorQueue<double> pvq_tjtor; //target

public:
    std::atomic<int> number_joints;
    std::atomic<double> sample_time; //cycle time
    std::atomic<double> time_stamp_sec;
    std::atomic<double> time_stamp_nano;

    std::atomic<double> performance;
};
#endif // OROCOS_FRI_DRIVER_HPP
