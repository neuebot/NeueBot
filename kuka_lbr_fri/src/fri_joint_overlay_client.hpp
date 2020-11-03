/*****************************************************************************
  File: fri_joint_overlay_client.hpp

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

#ifndef OROCOS_FRI_JOINT_OVERLAY_CLIENT_HPP
#define OROCOS_FRI_JOINT_OVERLAY_CLIENT_HPP

///Persistent Thread-safe queue
#include "persistent_vector_queue.hpp"

#include "friLBRClient.h"
#include <atomic>

/*!
 * \brief The FRIJointOverlayClient class
 * Derives from LBRClient to implement real-time functionalities.
 * Implements virtual onStateChange, functions monitor, waitForCommand and command from LBRClient.
 *
 * The Overlay Client is setup so that the queue keeps the last reference joint data even when
 * no information is received from the joint controller.
 *
 * This intermediate queue object is needed for two reasons.
 * 1) as a concurrent access buffer for the joint controller to send data, and to send that data
 * to the KUKA controller;
 * 2) as a mechanism to keep the last sent joint data so that the robot stays immobile even when
 * no new reference positions are being sent.
 *
 * About the last point, I tried shifting to a:
 *      robotCommand().setJointPosition(robotState().getIpoJointPosition())
 * This causes the robot so suddently jump to first joint positions received from the joint controller
 * at the end of any movement (when Persistent buffer is empty).
 *
 * I also tried shifting to a:
 *      robotCommand().setJointPosition(robotState().getCommandedJointPosition())
 * Which no longer jumps to the first position as the motion ends, but, if we continuously set the
 * joint positions to the later, the robot will slowly drift away as the set positions are not static
 * and change with slight errors.
 *
 * It is important to refer that any of these methods do not cause the robot to jump to the last FRI
 * sent position after switching to any other control mode (Hands-on, Position Control KUKA Controller).
 *
 * To cope with these problems, the Persistent buffer that stores the commanded joint positions is
 * continuously used to set the joint reference points, even after switching modes.
 * It keeps the last received set of positions, which prevents the robot from drifting away.
 * Also, since we update these positions any time we switch to COMMANDING_WAIT mode (from any other control
 * mode) with the current measured positions, we prevent to robot from jumping to the last stored
 * coordinates.
 */
class FRIJointOverlayClient : public KUKA::FRI::LBRClient
{

public: 
    FRIJointOverlayClient();
    ~FRIJointOverlayClient();

    /**
     * \brief Callback that is called whenever the FRI session state changes.
     *
     * @param oldState previous FRI session state
     * @param newState current FRI session state
     */
    void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

    /**
    * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
    */
    void monitor();

    /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    */
    void waitForCommand();

    /**
    * \brief Callback for the FRI session state 'Commanding'.
    */
    void command();

public:
    void SetTargetJointPositions(const std::vector<double> &tjp);

private:
    /*!
     * \brief safe Flag that controls when the class keeps or drops the received target positions.
     * It is initially set to true, but if at any point in time, the robot is requested to move
     * one of its joints more than the safe motion threshold, the flag is set to false and the
     * robot no longer is able to from the received joint positions
     */
    std::atomic<bool> safe;

    /*!
     * \brief pvq_tjpos Buffer that keeps the last received joint positions.
     * Interface between the architecture sent target joint data, and the requested joint positions
     * sent to the KUKA controller.
     */
    PersistentVectorQueue<double> pvq_tjpos; //target

};

#endif // OROCOS_FRI_JOINT_OVERLAY_CLIENT_HPP
