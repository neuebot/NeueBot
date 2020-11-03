/*****************************************************************************
  File: procedure_coordinator-component.hpp

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

#ifndef OROCOS_PROCEDURE_COORDINATOR_COMPONENT_HPP
#define OROCOS_PROCEDURE_COORDINATOR_COMPONENT_HPP

#include "procedure_coordinator_types.hpp"
#include "procedure_coordinator-errorcode.hpp"

#include <kdl/frames.hpp>
#include <rtt/RTT.hpp>

#include <vector>

enum LOG_LEVEL{INFO=0, WARNING, ERROR, FATAL};

struct SingleProcedure {
    SingleProcedure(std::vector<double> &target, std::vector<double> &entry) :
        target_(target),
        entry_(entry)
    {}

    std::vector<double> target_;
    std::vector<double> entry_;
    KDL::Frame fr_;
};

class ProcedureCoordinator : public RTT::TaskContext {

public:
    ProcedureCoordinator(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    bool m_moving;
    KDL::Frame m_pose;
    int m_conf;
    double m_psi;

    int m_trajectory_selected; //!< Stores the ID of locked trajectory or -1 if none selected.
    std::vector<double> m_entry;
    std::vector<double> m_target;
    KDL::Frame m_current_trajectory;
    std::shared_ptr<SingleProcedure> m_sp;

protected: //Attributes
    double m_approach_distance;
    double m_drill_distance;
    double m_retract_distance;
    double m_probe_distance;
    double m_safe_distance;
    double m_trajectory_step;
    double m_nullspace_step;

    KDL::Frame approach_pose;
    double approach_psi;

    unsigned int m_report_level;
    PCErrorCode m_ec;

    RTT::Attribute<bool> a_registration_successful;
    RTT::Attribute<bool> a_surgery_plan_setup;

private:
    /*!
     * \brief LockSurgicalTrajectory Calculates the transformation for the surgery trajectory (Entry and Target points)
     * Besides, this function is responsible to trigger the trajectory analysis, i.e. check if the best configuration and nullspace
     * to move along this trajectory without reaching joint limits or singularities.
     * When the trajectory is locked and the analysis performed, the user is notified whether the current robot configuration allows
     * to perform the trajectory. In the same communication, it is passed the alternatives configurations and arm angles.
     * At this moment, the choice of an alternative configuration and arm angle is directly asked from the user.
     *
     * The scores (possible configurations and arm angle) are kept, and checked before Skull drill, retract, insert probe...
     * \param id Key identifier of selected trajectory.
     * \return
     */
    RTT::approach_scores LockSurgicalTrajectory(int id);

    /*!
     * \brief ApproachSurgicalTrajectory Sets the frame, conf and psi (pose) of the selected approach.
     * The Controller will attempt two-step motion to reach the approach frame. First the robot will move the
     * elbow to align with the chosen psi / GC values. Then it will move PTP to the final approach frame.
     *
     * \param frame Approach selected frame.
     * \param conf Approach configuration.
     * \param psi Approach elbow angle.
     * \param rel_vel Relative velocity
     * \param red_scrs Redundancy scores
     * \return Returned codes:
     * Code  | Meaning
       ----- | -------------
       0     | Approach trajectory is feasible.
       1     | Approach requires user input.
       2     | Approach not feasible, see error message.
     */
    int ApproachSurgicalTrajectory(const KDL::Frame &frame, const int conf, const double psi, const double rel_vel,
                                   RTT::redundancy_scores &red_scrs);
    /*!
     * \brief ApproachSurgicalTrajectory Sets the frame, conf and psi (pose) of the selected approach.
     * The Controller will analyse the arc trajectory to reach the target end-effector frame from the current pose.
     * The method will generate different outputs depending on the nullspace solutions from the analyse method.
     * If the arc trajectory is feasible under the current configuration and arm angle, it returns true and executes the
     * arc motion. If the motion is feasible under the current configuration but not for the current arm angle, the
     * function will return false, and the user will be prompt with the feasible nullspace interval within the current configuration.
     * Ultimately, if the motion is not possible for the current configuration, the robot will execute a PTP motion instead of a
     * task space arc.
     *
     * \param frame Approach selected frame.
     * \param rel_vel Relative velocity
     * \param red_scrs Redundancy scores
     * \return Returned codes:
     * Code  | Meaning
       ----- | -------------
       0     | Approach trajectory is feasible.
       1     | Approach requires user input.
       2     | Approach not feasible, see error message.
     */
    int ArcApproachSurgicalTrajectory(const KDL::Frame &frame, const double rel_vel, RTT::redundancy_scores &red_scrs);

    /*!
     * \brief SkullDrillStep Moves the arm in a linear path from the position it stands, to the specified 'Drill distance'.
     * \param rel_vel Relative velocity of approach.
     */
    void SkullDrillStep(const double rel_vel);

    /*!
     * \brief RetractStep Retracts the arm from where it stands to a safe position. This step is used to exchange tools during the procedure,
     * at a safe distance from the patient.
     * \param rel_vel Relative velocity of approach.
     */
    void RetractStep(const double rel_vel);

    /*!
     * \brief InsertProbeStep Moves the arm in a linear path from the position it stands, to the specified 'Insert probe distance'.
     * \param rel_vel Relative velocity of approach.
     */
    void InsertProbeStep(const double rel_vel);

    /*!
     * \brief ReturnBasePose Executes a two step motion that takes the robot to its base starting position.
     * The robot starts from the base pose to approach any trajectory.
     * \param rel_vel
     */
    void ReturnBasePose(const double rel_vel);

    /*!
     * \brief IsRegistrationSuccessful Checks whether the robot registration process was concluded.
     * \return True if registration was completed successfully.
     */
    bool IsRegistrationComplete();

    /*!
     * \brief IsSurgeryPlanSetup Checks whether the surgery plan was setup.
     * \return True if surgery plan was setup.
     */
    bool IsSurgeryPlanSetup();

    /*!
     * \brief HandleErrorMsg Handles messages from non-server components.
     * It converts the error codes to a LOG type (Info, Warning, Error or Fatal) and
     * forwards the message to the user.
     * \param e Received error message.
     */
    void HandleErrorMsg(RTT::error_msg e);

private: //Interfaces SurgeryPlan
    bool ConfirmSurgeryPlan(const RTT::vector_int &id, const RTT::vector_vector_double &target,
                            const RTT::vector_vector_double &entry);
    bool RetrieveSurgeryPlan(RTT::vector_int &id, RTT::vector_vector_double &target,
                             RTT::vector_vector_double &entry);
    void RemoveAllSurgeryTrajectories();
    bool ShowSurgeryTrajectory(const int id);
    void ShowAllSurgeryTrajectories();

    bool LoadSurgeryPlanFile(const std::string &filename, RTT::vector_int &id, RTT::vector_vector_double &target, RTT::vector_vector_double &entry);
    bool SaveSurgeryPlanFile(const std::string &filename);
    void SelectSurgicalTrajectory(const int id);

private: //Interfaces Registration
    void LoadWorkPiecePoints(const RTT::vector_vector_double &wpoints);
    void LoadRegistrationPoints(const RTT::vector_vector_double &rpoints);
    bool ComputeRegistrationMatrix(KDL::Frame &transformation, double &cum_err);
    bool SaveInterlockRegistrationMatrix(const KDL::Frame &base_tool, KDL::Frame &transformation);
    bool LoadCalibrationFile(const std::string &filename, KDL::Frame &transformation, double &cum_err);
    bool SaveCalibrationFile(const std::string &filename);
    
protected: //Operation Callers
    //Test Supervisor
    RTT::OperationCaller<void()> CallStopRobot;
    RTT::OperationCaller<void()> CallShutdownRobot;
    RTT::OperationCaller<void()> CallResumeRobot;
    RTT::OperationCaller<void(const double)> CallSetRelativeVelocity;
    RTT::OperationCaller<void(const int)> CallSetToolType;
    RTT::OperationCaller<int(void)> CallGetToolType;

    RTT::OperationCaller<void(std::vector<double>)> CallJointMotion;
    RTT::OperationCaller<void(KDL::Frame, const int, const int)> CallCoordinateJointMotion;
    RTT::OperationCaller<void(KDL::Frame, const int, const int)> CallAdjustedCoordinateJointMotion;
    RTT::OperationCaller<void(KDL::Frame, const int, const int)> CallLinearMotion;
    RTT::OperationCaller<void(KDL::Vector, KDL::Frame, const int, const int)> CallArcMotionFrame;
    RTT::OperationCaller<void(RTT::vector_frame, const int, const int)> CallComposedMotion;
    RTT::OperationCaller<bool(KDL::Frame, const int, const double, const int, const int)> CallApproachMotion;
    RTT::OperationCaller<void(KDL::Frame, const int, const int, KDL::Frame, const double)> CallReturnMotion;
    RTT::OperationCaller<void(KDL::Frame, const int, const int)> CallIncrementMotion;
    RTT::OperationCaller<bool(KDL::Frame, const int, const int, RTT::redundancy_scores&)> CallAnalyseLinearMotion;
    RTT::OperationCaller<bool(KDL::Vector, KDL::Frame, const int, const int,
                              RTT::redundancy_scores&)> CallAnalyseArcMotionFrame;

    //VREP FRI
    RTT::OperationCaller<KDL::Frame()> CallGetSurgeryReference;
    RTT::OperationCaller<bool(const double)> CallMoveEndEffectorTool;

    //Surgery Plan
    RTT::OperationCaller<bool(const RTT::vector_int &, const RTT::vector_vector_double &,
                              const RTT::vector_vector_double &)> CallConfirmSurgeryPlan;
    RTT::OperationCaller<bool(RTT::vector_int &, RTT::vector_vector_double &,
                              RTT::vector_vector_double &)> CallRetrieveSurgeryPlan;
    RTT::OperationCaller<void()> CallRemoveAllTrajectories;
    RTT::OperationCaller<bool(const int)> CallShowSurgeryTrajectory;
    RTT::OperationCaller<void()> CallShowAllTrajectories;
    RTT::OperationCaller<bool(const std::string&, RTT::vector_int &, RTT::vector_vector_double &,
                              RTT::vector_vector_double &)> CallLoadSurgeryPlanFile;
    RTT::OperationCaller<bool(const std::string&)> CallSaveSurgeryPlanFile;
    RTT::OperationCaller<bool(const int)> CallSelectSurgeryTrajectory;
    RTT::OperationCaller<bool(const int, RTT::vector_frame &, RTT::vector_int &, std::vector<double> &,
                              RTT::vector_vector_double &)> CallVerifyTrajectory;
    RTT::OperationCaller<bool(const int, std::vector<double>&, std::vector<double>&)> CallGetSurgeryTrajectoryParameters;

    //Registration
    RTT::OperationCaller<void(const RTT::vector_vector_double &)> CallLoadWorkPiecePoints;
    RTT::OperationCaller<void(const RTT::vector_vector_double &)> CallLoadRegistrationPoints;
    RTT::OperationCaller<bool(KDL::Frame &, double &)> CallComputeRegistrationMatrix;
    RTT::OperationCaller<bool(const KDL::Frame &, KDL::Frame &)> CallSaveInterlockRegistrationMatrix;
    RTT::OperationCaller<bool(const std::string &, KDL::Frame &, double &)> CallLoadCalibrationFile;
    RTT::OperationCaller<bool(const std::string &)> CallSaveCalibrationFile;
    RTT::OperationCaller<bool(KDL::Frame&)> CallGetRegistrationMatrix;

protected:
    RTT::SendHandle<bool(int)> sh_select_surgery_trajectory;

protected: //Ports
    RTT::InputPort < bool > inport_robot_moving;
    RTT::InputPort < RTT::posture > inport_robot_posture;

    RTT::InputPort < RTT::error_msg > inport_error_msg;
    RTT::OutputPort < RTT::error_msg > outport_error_msg;
};

#endif //OROCOS_PROCEDURE_COORDINATOR_COMPONENT_HPP
