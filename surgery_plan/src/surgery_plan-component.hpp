/*****************************************************************************
  File: surgery_plan-component.hpp

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

#ifndef OROCOS_SURGERY_PLAN_COMPONENT_HPP
#define OROCOS_SURGERY_PLAN_COMPONENT_HPP

#include "surgery_plan_types.hpp"
#include "surgery_plan-errorcode.hpp"

#include "marshaller_surgery_plan.h"

#include <rtt/RTT.hpp>

enum LOG_LEVEL{INFO=0, WARNING, ERROR, FATAL};

/*!
 * \brief The Approach struct stores information regarding possible approach solutions to the selected linear trajectory.
 * Two degrees of redundancy exist when operating with a 7-DOF manipulator for a 5-DOF task (linear trajectory).
 * Thus, the manipulability index is applied to determine the best z-axis angle (around the trajectory).
 * With that being determined, for each configuration, we determine the frame and psi value that leads to the highest
 * manipulability, as well as a 360 degree score set with possible solutions that quantify the manipulability, proximity
 * to limits/singularities and distance to current pose.
 */
struct Approach {
    KDL::Frame _frame;
    int _conf;
    double _psi;
    std::vector<double> _scores;


    Approach(const KDL::Frame &frame, const int conf, const double psi, const std::vector<double> &scores) :
        _frame(frame),
        _conf(conf),
        _psi(psi),
        _scores(scores)
    {}
};

/*!
 * \brief The Trajectory struct stores information regarding a single surgery plan trajectory.
 * Not only does it store the immediate information about this trajectory (entry, target and id), it contains
 * flags that code whether the trajectory is visible or is selected.
 *
 * When the redundancy is solved for the current trajectory (returns Approach from Task Supervisor), the trajectory
 * is considered 'set up'. It should then contain a list of Approaches that span the range of possible configurations
 * (configurations with at least one possible solution).
 */
struct Trajectory {
    Trajectory(const int id, const std::vector<double> target, std::vector<double> entry) :
        id_(id),
        entry_(entry),
        target_(target),
        visible_(true),
        selected_(false),

        setup_(false)
    {}

    void SetApproaches(const std::vector<KDL::Frame> &frame, const std::vector<int> conf, const std::vector<double> psi,
                       const std::vector<std::vector<double> > &scores)
    {
        approaches_.clear();

        for(size_t i=0; i<frame.size(); ++i) {
            approaches_.emplace_back(frame[i], conf[i], psi[i], scores[i]);
        }

        setup_ = true;
    }

    void GetApproaches(std::vector<KDL::Frame> &frame, std::vector<int> &conf, std::vector<double> &psi,
                       std::vector<std::vector<double> > &scores) const
    {
        frame.resize(approaches_.size());
        conf.resize(approaches_.size());
        psi.resize(approaches_.size());
        scores.resize(approaches_.size());

        for(size_t i=0; i<frame.size(); ++i) {
            frame[i]  = approaches_[i]._frame;
            conf[i]   = approaches_[i]._conf;
            psi[i]    = approaches_[i]._psi;
            scores[i] = approaches_[i]._scores;
        }
    }

    /*!
     * \brief OrderApproaches Reorders the list of approaches to place the current current robot configuration at the top.
     * If the list does not contain the current robot configuration, it returns false.
     * \param current_conf Current Global Configuration (GC).
     * \return True if list contains the current configuration
     */
    bool OrderApproaches(const int current_conf)
    {
        //if the current configuration has solutions we want it to be the first
        auto it = std::find_if(approaches_.begin(), approaches_.end(), [&](Approach& a){
            return (a._conf == current_conf);
        });
        if(it != approaches_.end()) {
            //Rotates the iterator that matches the current global configuration to the start of the order vector
            std::rotate(approaches_.begin(), it, it+1);

            return true;
        }
        else {
            return false;
        }
    }

    int id_;
    std::vector<double> entry_;
    std::vector<double> target_;
    bool visible_;
    bool selected_;
    bool setup_;

    std::vector<Approach> approaches_;
};

/*!
 * \brief The Surgery Plan class handles and stores the surgery trajectories.
 * The idea behind this component is to work separately from the Procedure coordinator.
 * Although there is a certain level of dependency between both components, it is easier and cleaner to maintain
 * a component that only keeps track of the current trajectory parameters.
 * Besides storing information regarding the surgical plan, the component is responsible for interacting with
 * the Task Supervisor peer component in the Core Layer that computes feasible approach poses.
 */
class SurgeryPlan : public RTT::TaskContext {
public:
    SurgeryPlan(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private: //Member Variables
    double m_tolerance;
    double m_safe_distance;
    double m_trajectory_approach_step;
    double m_nullspace_step;
    std::vector<Trajectory> m_trajectories;
    bool m_surgery_plan_setup;

    RTT::Attribute<int> a_current_conf;

    KDL::MarshallerSurgeryPlan m_marshaller;
    SPErrorCode m_ec;

private: //Member Functions
    /*!
     * \brief ConfirmSurgeryPlan Receives a surgery plan sent by the user and stores the information internally.
     * \param id Vector of each trajectory identifier.
     * \param target Vector of each trajectory target point.
     * \param entry Vector of each trajectory entry point.
     * \return Success
     */
    bool ConfirmSurgeryPlan(const RTT::vector_int &id, const RTT::vector_vector_double &target,
                            const RTT::vector_vector_double &entry);

    /*!
     * \brief RetrieveSurgeryPlan Retrieves the current internal surgery plan.
     * \param id Vector of each trajectory identifier.
     * \param target Vector of each trajectory target point.
     * \param entry Vector of each trajectory entry point.
     * \return Success
     */
    bool RetrieveSurgeryPlan(RTT::vector_int &id, RTT::vector_vector_double &target,
                             RTT::vector_vector_double &entry);

    /*!
     * \brief RemoveAllTrajectories Removes all stored trajectories.
     */
    void RemoveAllTrajectories();

    /*!
     * \brief ShowSurgeryTrajectory Highlights the specified trajectory, occulting the remaining ones.
     * \param id Key identifier of the trajectory.
     * \return True if the id specified trajectory existed and was highlighted.
     */
    bool ShowSurgeryTrajectory(const int id);

    /*!
     * \brief ShowAllTrajectories Shows again all stored trajectories.
     */
    void ShowAllTrajectories();

    /*!
     * \brief LoadSurgeryPlanFile Loads a saved surgery plan and returns by reference the id, target and entry points
     * of each stored trajectory.
     * \param filename Path of the file that contains the surgery plan.
     * \param id Vector of each trajectory identifier.
     * \param target Vector of each trajectory target point.
     * \param entry Vector of each trajectory entry point.
     * \return Success
     */
    bool LoadSurgeryPlanFile(const std::string &filename, RTT::vector_int &id, RTT::vector_vector_double &target, RTT::vector_vector_double &entry);

    /*!
     * \brief SaveSurgeryPlanFile Saves the surgery plan with the internal list of trajectories (id, target and entry points).
     * \param filename ath of the file to save the surgery plan.
     * \return Success
     */
    bool SaveSurgeryPlanFile(const std::string &filename);

    /*!
     * \brief SelectSurgeryTrajectory Selects one of the surgery plan trajectories, identified by the key 'id' and forwards its
     * information to the task supervisor component.
     * The task supervior is responsible for solving both redundancies relative to the linear trajectory (elbow and z-axis angle
     * around the trajectory). A cost function is solved to determine the possible pose solutions that allow the robot to reach
     * the specified trajectory and approach the surgery positions (drill and probe step).
     * Only poses that lead to possible solutions are stored.
     * SelectSurgeryTrajectory() is a non-blocking function, so the function immediately returns after sending the information to
     * the task supervisor.
     * \param id Key identifier of surgical trajectory.
     * \return True if surgical trajectory exists and information was sent to the task supervisor.
     */
    bool SelectSurgeryTrajectory(const int id);

    /*!
     * \brief VerifyTrajectory Collects the results from the operation sent when SelectSurgeryTrajectory() was called.
     * The function requires the selected trajectory id, and returns by parameter the set of Approaches (frames, configurations,
     * psi and scores). If any solution was possible, this information is stored internally as the vector of Trajectories is updated.
     *
     * The each element of the sets 'frame', 'conf', 'psi' and 'scores' correspond to an Approach, and thus, their size must match.
     * \param id Key identifier of surgical trajectory.
     * \param frames Set of Approach possible frames.
     * \param conf Set of Approach possible configurations
     * \param psi Set of Approach ideal psi.
     * \param scores Set of Approach possible scores.
     * \return
     */
    bool VerifyTrajectory(const int id, RTT::vector_frame &frames, RTT::vector_int& conf, std::vector<double> &psi,
                          RTT::vector_vector_double &scores);

    /*!
     * \brief GetSurgeryTrajectoryParameters Function available to the Application layer. Allows the procedure coordinator
     * to access the data of a particular trajectory.
     * \param id Key identifier of surgical trajectory.
     * \param target Target position of the surgical trajectory regarding the surgical reference frame.
     * \param entry Entry position of the surgical trajectory regarding the surgical reference frame.
     * \return True if selected trajectory exists.
     */
    bool GetSurgeryTrajectoryParameters(const int id, std::vector<double> &target, std::vector<double> &entry);

protected: //OperationCallers
    //VRepFRI
    RTT::OperationCaller<void(const RTT::vector_int &, const RTT::vector_vector_double &,
                               const RTT::vector_vector_double &)> AddTrajectories;
    RTT::OperationCaller<void()> RemoveTrajectories;
    RTT::OperationCaller<void(const int)> ShowTrajectory;
    RTT::OperationCaller<void()> ShowTrajectories;
    //TaskSupervisor
    RTT::OperationCaller< bool(const std::vector<double> &,const std::vector<double> &,double,double) > CallCalculateBestTrajectoryApproach;
    RTT::OperationCaller<void(RTT::vector_int &, std::vector<double> &, RTT::vector_frame &, RTT::vector_vector_double &)> CallCollectApproachResults;
//    RTT::OperationCaller< void( KDL::Frame, KDL::Frame, const int, const int, RTT::vector_frame &) > CallSampleLinearPath;
//    RTT::OperationCaller< bool( RTT::vector_frame&, RTT::vector_int&, RTT::vector_vector_double &)> CallAnalyseTrajectory;

protected: //Ports
    RTT::OutputPort< RTT::error_msg > outport_error_msg;
};

#endif //OROCOS_SURGERY_PLAN_COMPONENT_HPP
