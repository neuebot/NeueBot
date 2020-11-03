/*****************************************************************************
  File: test_supervisor-component.hpp

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

#ifndef OROCOS_TASK_SUPERVISOR_COMPONENT_HPP
#define OROCOS_TASK_SUPERVISOR_COMPONENT_HPP

#include "test_supervisor-errors.hpp"
#include "test_supervisor_types.hpp"

#include <kdl/frames.hpp>
#include <rtt/SendHandle.hpp>
#include <rtt/os/Semaphore.hpp>
#include <rtt/RTT.hpp>

#include <atomic>
#include <queue>
#include <vector>

/*!
 * \brief The STATE enum enumerates the states that the supervisor can be operating.
 * Independent of the state the component still functions and takes continuous updates from each system
 */
enum STATE {
    MON = 1000,
    NORM = 1100,
    CAR = 1101,
    KIN = 1102,
    JTC = 1103,
    OTGP = 1200,
    OTGF = 1300
};

/*!
 * \brief The MOTION enum enumerates types of Cartesian motion.
 */
enum MOTION {
    OTH = 2000,
    LIN = 2001,
    ARC = 2002,
    CMP = 2003,
    NSP = 2004,
};

/*!
 * \brief The REFERENCE_FRAME enum defines the identifiers for the transformation of the relative motion.
 *
 * \note The End-effector transformation (frame) is the reference frame of the tool base respective
 * to the robot base frame. If an end-effector tool can be moved (e.g. trepan or probe), this displacement
 * is not accounted.
 *
 * TODO: GUARANTEE IT MATCHES TRANSFORMATIONS
 */
enum REFERENCE_FRAME {
    BASE=0,
    FLANGE,
    END_EFFECTOR,
    WORK,
};

/*!
 * \brief The END_EFFECTORS enum defines the identifiers for the end-effectors attached to the robot.
 *
 * TODO: GUARANTEE IT MATCHES TRANSFORMATIONS
 */
enum END_EFFECTORS {
    NONE        = 0,
    EMPTY       = 1,
    CAMERA      = 2,
    PROBE       = 3,
    TREPAN      = 4,
    EEBASE      = 5,
    BASEBUTTONS = 6,
    BASEPROBE   = 7,
    BASETREPAN  = 8,
    POINTER     = 9
};

struct PTPMotion {
    enum Type {PointToPoint, PointToPointHollow/*, MotionThroughPoints, MotionThroughPointsHollow, NullspaceMotion*/};

    Type type_;
    std::vector<double> joints_;
    double rel_vel_;

    PTPMotion(const std::vector<double> &joints, const bool hollow, const double rel_vel = 1.0) {
        joints_ = joints;
        rel_vel_ = rel_vel;
        if(!hollow)
            type_ = Type::PointToPoint;
        else
            type_ = Type::PointToPointHollow;
    }
};

class TestSupervisor : public RTT::TaskContext {

public:
    TestSupervisor(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    unsigned int m_nj;
    std::vector<double> m_lim;
    double m_lsample_step;

private:
    unsigned int m_report_level;
    ErrorCodes m_ec;
    std::queue<PTPMotion> m_queue;
    std::vector<double> m_current_jpos;
    std::vector<double> m_current_jvel;
    std::vector<double> m_ext_torque;
    std::vector<double> m_current_force;
    KDL::Frame m_current_frame;
    double m_current_psi;
    unsigned int m_current_gc;
    bool m_control_hollow;
    double m_relative_velocity;
    double m_tolerance;

    unsigned int m_transition_counter; //!< Counts the number of component cycles to wait between motion transitions

    //! Reference collectible from sent operation needs to be a common reference
    //!     accross the update cycle and the function from which the operation was sent
    //! Same goes for variables used accross the component
    std::vector<std::vector<double> > m_calculated_joints;
    std::vector<KDL::Frame> m_target_frames;
    std::vector<double> m_target_times;
    std::vector<double> m_target_psis;

    //! Current end-effector - robot base frame stored when the gripper is pressed
    KDL::Frame m_start_frame;
    bool m_transmit;

    //! Number of received target frames without interruption
    int m_count;

private: ///Approach trajectory data
    std::vector<int> m_approach_GC;
    std::vector<double> m_approach_psi;
    std::vector<KDL::Frame> m_approach_frame;
    std::vector<std::vector<double> > m_approach_scores;

protected:
    RTT::Attribute<unsigned int> a_nj;
    RTT::Attribute<std::vector<double> > a_xmin;
    RTT::Attribute<std::vector<double> > a_xmax;

private:
    bool m_simulation;
    bool m_robot_moving; //!< Flag that holds true if the robot is currently moving
    bool m_waiting; //!< Flag that holds true if the is waiting for the next motion
    STATE m_state; //!< State variable holds the current supervisor state
    MOTION m_motion; //!< State variable holds the current type of supervisor motion

private:
    //! Basic Robot Control Calls
    /*!
     * \brief StopRobot Calls for an imediate stop of any robot action. While stoped the
     * robot cannot perform anymore actions. The robot can recover from the current state
     * however.
     */
    void StopRobot();
    /*!
     * \brief ResumeRobot Recovers the robot to a functioning state after being stoped.
     */
    void ResumeRobot();
    /*!
     * \brief ShutdownRobot Likewise to the StopRobot() call, this function also stops
     * any robot motion. Moreover, a call to this function makes the robot completely
     * shutdown, beyond recovery. The system needs to be rebooted to resume operation.
     */
    void ShutdownRobot();

    //! Direct Joint Controller Call
    /*!
     * \brief JointMotion generates a Point to Point motion in joint space to the specified values.
     * \param jpos vector of target joint positions.
     */
    void JointMotion(std::vector<double> jpos);

    //! Single Kinematic Module Call
    /*!
     * \brief CoordinateJointMotion generates a Point to Point motion in joint space from the current position to
     * a new position specified by a Cartesian target. It keeps the same arm angle (psi) and the same global configuration
     * parameter (GC).
     * \param target frame that specifies the target frame which is translated to joint space.
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     */
    void CoordinateJointMotion(KDL::Frame target, const int ref_frame_id, const int tool_id);
    /*!
     * \brief AdjustedCoordinateJointMotion generates a Point to Point motion in joint space from the current position to
     * a new position specified by a Cartesian target.
     * The difference relative to the \ref CoordinateJointMotion() is that this function does not keep the same redundancy parameters.
     * Instead it tries to find the best fitting redundancy parameters at all times. The arm angle is adjusted to move away from singularities
     * and joint limits.
     * \param target frame that specifies the target frame which is translated to joint space.
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     */
    void AdjustedCoordinateJointMotion(KDL::Frame target, int ref_frame_id, int tool_id);

    //! Cartesian Space Motions
    /*!
     * \brief IncrementMotion generates a linear trajectory along a specified increment (rotation and position).
     * The motion unfolds according to the specified coordinate frame, at a maximum relative_velocity [0, 1].
     * \param increment frame that specifies the rotation and position increments.
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     */
    void IncrementMotion(KDL::Frame increment, const int ref_frame_id, const int tool_id);
    /*!
     * \brief LinearMotion generates a linear trajectory from the current pose to the target transformation. The motion is executed according to the
     * specified reference frame and with respect to the selected tool.
     * \param target Target frame with respect to the frame of reference.
     * \param ref_frame_id Frame of reference. See transformations component.
     * \param tool_id Tool id, sets the tool transformation. See transformations component.
     */
    void LinearMotion(KDL::Frame target, const int ref_frame_id, const int tool_id);
    /*!
     * \brief ArcMotion generates an Arc Motion trajectory around a center point, in the 'tangent' direction and executing a relative rotation.
     * As for the other motions, also this one is relative to a reference frame and executed at a relative specified velocity.
     * \note For some wicked reason, operations are limited to 5 parameters. Thus, we chose to compact tangent and end-rotation in the same KDL::Frame.
     * There is no relation with the position and rotation.
     * \param center_point Position of the center of the arc relative to the same reference as the start_frame.
     * \param final_frame Final frame to reach.
     * \param arc_angle Angle of the arc to be described in the trajectory.
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     */
    void ArcMotionFree(KDL::Vector center_point, KDL::Frame final_frame, const double arc_angle, const int ref_frame_id, const int tool_id);
    /*!
     * \brief ArcMotion generates an Arc Motion trajectory the vicinity of a center point, to move from the start/current frame towards the specified
     * final frame.
     * As for the other motions, also this one is relative to a reference frame and executed at a relative specified velocity.
     * \note This ArcMotion function guarantees that the trajectory path leaves the current frame and arrives at the specified final frame using the
     * smallest possible arc. This is assured by calculating the bisection line between the segment that connects the initial and final positions.
     * The new motion center is found at an equal radius from the initial and final positions, inscribed in the planed formed by the initial, final and
     * center_point. Moreover, it is guaranteed that the arc is performed always from the initial to the final frame, and that the arc direction is
     * concave to the specified center_point.
     * \param center_point Position that forms the arc plane together with the current and final frame positions. The arc form is always concave to the specified center_point.
     * \param final_frame Final frame to reach.
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     */
    void ArcMotionFrame(KDL::Vector center_point, KDL::Frame final_frame, const int ref_frame_id, const int tool_id);
    /*!
     * \brief ComposedMotion generates a spline motion that passes through the specified frames.
     * \param frames Vector of N*12 values, which contains N Frames. Should be demarshalled as:
     * Frame[i].p = {vector[i*12], vector[i*12+1], vector[i*12+2]}
     * Frame[i].M = {vector[i*12+3], vector[i*12+4], ... , vector[i*12+11]}
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     */
    void ComposedMotion(RTT::vector_frame frames, const int ref_frame_id, const int tool_id);
    /*!
     * \brief AdjustElbow Adjusts the robot posture inside the nullspace (without changing pose).
     *  Three scenarios can happen here:
        1) We don't change configuration, and from current psi to target psi we can freely move without crossing avoid regions.
        2) We don't change configuration, but from current psi to target psi, we pass an avoid region
        3) We change configuration
        For scenario 1), the robot will adjust its posture without moving the end-effector.
        For scenarios 2) and 3) the robot will make large displacements. Thus, the routine is to go to the zero mechanical
        position and return to the target position. Since its a PTP, it is not affected by joint limits or singularities.
        The later case implies a sequential motion of the hinge, pivot and then again hinge joints.
     * \param psi Target psi value
     * \param gc Target configuration
     */
    bool AdjustElbow(const double psi, const int gc);

    /*!
     * \brief ApproachMotion Executes a nullspace motion to the desired conf and psi. Then it moves to the target
     * frame in the specified frame of reference.
     * TODO: The supervisor should support multiple motions, which is not simple using the current scheme of sent operations.
     * Either we move back to a sequential call to the respective components or we figure out a way to have this queue of motions
     * that involve multiple components using the send / callback mode.
     * \param target Approach final frame.
     * \param conf Approach configuration.
     * \param psi Approach elbow angle.
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     * \return
     */
    bool ApproachMotion(KDL::Frame target, const int conf, const double psi, const int ref_frame_id, const int tool_id);
    /*!
     * \brief ReturnMotion Executes a linear motion to reach target frame and then a joint motion to reach the joints coordinates.
     * Not the prettiest solution, but the feasible in the available time.
     * TODO: The supervisor should support multiple motions, which is not simple using the current scheme of sent operations.
     * Either we move back to a sequential call to the respective components or we figure out a way to have this queue of motions
     * that involve multiple components using the send / callback mode.
     * \param target Target frame for the linear movement
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     * \param approach_pose Home pose.
     * \param approach_psi Elbow angle selected for this trajectory.
     * \return
     */
    bool ReturnMotion(KDL::Frame target, const int ref_frame_id, const int tool_id, KDL::Frame approach_pose,
                      const double approach_psi);

    //! Cartesian Space Motion Handling
    /*!
     * \brief ProcessCartesianTrajectory handler function called upon conclusion of Cartesian Controller
     * trajectory generation. The trajectory coordinates are retrieved, and the information parsed to the
     * Kinematics Module.
     */
    bool ProcessCartesianTrajectory();
    /*!
     * \brief ProcessJointTrajectory handler function called upon conclusion of Kinematics Module trajectory
     * calculation. Kinematics Module is responsible for transforming a Cartesian Trajectory into a Joint
     * Trajectory. Once the Trajectory is calculated, the joint coordinates are sent to the Joint Controller.
     */
    bool ProcessJointTrajectory();

    //! PTP Motion Queue execution
    /*!
     * \brief ExecuteNextMotion Executes the next joint action on queue.
     */
    void ExecuteNextJointMotion();

    //! Shadow Robot Instant Motions - Preview Robot
    /*!
     * \brief InstantJointMotion instantly positions the hollow robot in the target joints positions.
     * The hollow robot has passive joints and can instantly move to the desired position.
     * \param target_joints target joint positions.
     */
    void InstantJointMotion(const std::vector<double> target_joints);
    /*!
     * \brief InstantJointMotion instantly positions the hollow robot in the target joints positions.
     * The hollow robot has passive joints and can instantly move to the desired position.
     * \param target Target joint positions.
     * \param ref_frame_id Frame of reference. See transformations component.
     * \param tool_id Tool id, sets the tool transformation. See transformations component.
     */
    void InstantCoordinateJointMotion(const KDL::Frame target, const int ref_frame_id, const int tool_id);
    /*!
     * \brief InstantNullspaceCoordinateJointMotion
     * \param target Target robot pose relative to the reference frame.
     * \param psi Target arm angle.
     * \param conf Target global configuration.
     * \param ref_frame_id Frame of reference. See transformations component.
     * \param tool_id Tool id, sets the tool transformation. See transformations component.
     */
    void InstantNullspaceCoordinateJointMotion(const KDL::Frame target, const int conf, const double psi,
                                               const int ref_frame_id, const int tool_id);
    /*!
     * \brief InstantAdjustElbow instantly adjusts the hollow robot arm to match the passed psi and gc.
     * The hollow robot has passive joints and can instantly move to the desired position.
     * \param psi target nullspace parameter.
     * \param gc target robot configuration.
     */
    void InstantAdjustElbow(const double psi, const int gc);
    /*!
     * \brief InstantShadowReal instantly overlaps hollow robot over the real robot position.
     * This is called previous to preview operations, to guarantee the hollow robot starts from the
     * same position as the real robot.
     */
    void InstantShadowReal();

    //! Trajectory Evaluation Calls
    /*!
    * \brief CalculateBestApproach In this function we compute the best approach for the given trajectory from the robot
    * start point. For a given trajectory (set of possible approach frames), we generate up to 8 solutions, one for each
    * configuration. This solution includes the ordered [conf, psi and frame] that returns the highest augmented manipulability
    * factorized score.
    *
    * Due to the heavy computation process, this function is partitioned into a processing and collecting function (\ref CollectApproachResults())
    *
    * \param target Trajectory target position.
    * \param entry Trajectory entry position.
    * \param approach_step Sampling step of the orientation around the linear trajectory.
    * \param distance Distance from the entry point along the z-axis.
    */
    bool CalculateBestTrajectoryApproach(const std::vector<double> &target, const std::vector<double> &entry, double approach_step, double distance);

    void CollectApproachResults(RTT::vector_int &conf_max, std::vector<double> &psi_max, RTT::vector_frame &frames_max,
                                RTT::vector_vector_double &configuration_scores);

    void GenerateApproachFrames(const std::vector<double> &target, const std::vector<double> &entry, double approach_step, double distance,
                                const KDL::Frame &reference, const KDL::Frame &tool, std::vector<KDL::Frame> &frames);
    /*!
     * \brief AnalyseTrajectory For a specific set of frames calculates the possible robot configurations as well as the possible
     * arm angle intervals to avoid reaching the joint limits or singularities.
     * \param target_frames Trajectory sequence of frames.
     * \param possible Possible list of robot configurations.
     * \param initial_interval Initial arm angle (nullspace) intervals to perform the trajectory.
     * \return Returns true if the trajectory can be performed from at least one configuration.
     */
    bool AnalyseTrajectory(RTT::vector_frame &target_frames, RTT::vector_int &possible, RTT::vector_vector_double &initial_interval);

    /*!
     * \brief AnalyseLinearMotion Conducts the AnalyseTrajectory() for a required linear motion in task space.
     * Uses the same parameters as the LinearMotion(), but returns the set of nullspace possible solutions by referenbce.
     * \param target Target frame with respect to the frame of reference.
     * \param ref_frame_id Frame of reference. See transformations component.
     * \param tool_id Tool id, sets the tool transformation. See transformations component.
     * \param solution RTT::redundancy_scores object containint possible configurations and arm angles.
     */
    bool AnalyseLinearMotion(KDL::Frame target, const int ref_frame_id, const int tool_id, RTT::redundancy_scores &solution);

    /*!
     * \brief AnalyseArcMotionFrame Conducts the AnalyseTrajectory() for a required arc motion in task space.
     * Uses the same paremeters as the ArcMotionFrame(), but returns the set of nullspace possible solutions by referenbce.
     * \param center_point Position that forms the arc plane together with the current and final frame positions. The arc form is always concave to the specified center_point.
     * \param final_frame Final frame to reach.
     * \param ref_frame_id specified frame of reference. See transformations component.
     * \param tool_id specified tool id, sets the tool transformation. See transformations component.
     * \param solution RTT::redundancy_scores object containint possible configurations and arm angles.
     */
    bool AnalyseArcMotionFrame(KDL::Vector center_point, KDL::Frame final_frame, const int ref_frame_id, const int tool_id, RTT::redundancy_scores &solution);

    /*!
     * \brief ScoresTrajectory For a given trajectory, given the list of possible arm angle invervals for the possible robot configurations
     * it calculates the best order of configurations (closer to current) and scores the arm angle comparing to the current position.
     * \param possible List of possible robot configurations for the given trajectory.
     * \param initial_interval Intervals of possible arm angles for the given trajectory, for each configuration.
     * \param gc_alt List of possible robot configurations ordered, from closer to farther configuration (in terms of robot displacement).
     * \param scores Returns the list of scores for the arm angle for the list of possible robot configurations.
     * \return Returned codes:
     * Code  | Meaning
       ----- | -------------
       0     | Current psi is fitting to perform the trajectory.
       1     | Change psi to perform the trajectory.
       2     | Change gc to perform the trajectory.
       3     | No psi in any gc allows to perform trajectory.
     */
    int ScoresTrajectory(RTT::vector_int &possible, RTT::vector_vector_double &initial_interval, RTT::vector_int &gc_alt,
                         RTT::vector_vector_double &scores);
    /*!
     * \brief CheckCurrentNSScores Calculates the nullspace scores for the current arm pose.
     * \return nsscores Nullspace scores for the current arm configuration and position.
     */
    std::vector<double> ScoresCurrentPoseCurrentGC();
    /*!
     * \brief ScoresCurrentPoseAlternativeGC Calculates the nullspace scores for the current pose
     * and alternative configuration.
     * \param gc Alternative configuration
     * \return nsscores Nullspace scores for the current arm configuration and position.
     */
    std::vector<double> ScoresCurrentPoseAlternativeGC(const int gc);

    //! Set Get Basic Properties / Tools
    /*!
     * \brief SetRelativeVelocity
     * \param relative_velocity
     */
    void SetRelativeVelocity(double relative_velocity);
    /*!
     * \brief SetToolType sets the tool attached to the robot end-effector, \e Transformations component.
     * \param type Type of tool
     */
    void SetToolType(const int type);
    /*!
     * \brief SetWorkFrame sets the new work frame relative to the robot base frame.
     * This transformation is relative to the robot base frame.
     * \param base_work KDL::Frame of Work frame relative to the Base frame
     */
    void SetWorkFrame(const KDL::Frame base_work);
    /*!
     * \brief GetToolType Returns the current attached tool type, \e Transformations component.
     * \return Type of tool enum.
     */
    int GetToolType();
    /*!
     * \brief GetToolFrame returns the transformation matrix from the flange to the
     * currently attached tool by reference.
     * \param tool KDL::Frame of flange-tool transformation matrix.
     */
    void GetToolFrame(KDL::Frame &tool);
    /*!
     * \brief GetWorkFrame returns the transformation matrix from the base to the
     * work frame by reference.
     * \param ref KDL::Frame of base-work transformation matrix.
     */
    void GetWorkFrame(KDL::Frame &ref);

    //! Set Alternative Supervisor Control Modes
    /*!
     * \brief SetHollowControl sets which robot is the target of the robot motion commands.
     * This function should be called whenever a new target for robot motion is set.
     * \param hollow_control boolean value that defines whether to control the hollow or real robot.
     */
    void SetHollowControl(const bool hollow_control);
    /*!
     * \brief SetOTGPosition Sets the online trajectory generation mode. The component receives new 'coarse' positions to reach
     * and generates a new motion profile interpolating the motion with a finer control.
     * \param otg Sets the online trajectory generation mode if true.
     * \return Success.
     */
    bool SetOTGPosition(bool otg);
    /*!
     * \brief SetOTGPosture
     * \param otg
     */
    bool SetOTGPosture(bool otg);

    //! Error Handling
    /*!
     * \brief HandleErrorMsg Receives error messages from components, delivers information to user.
     * The content of the message is delivered or not depending on the report level (INFO, WARNING, ERROR...)
     * \param e error message containing: error level and message.
     */
    void HandleErrorMsg(RTT::error_msg e);

protected:
    //VRepFRI
    RTT::OperationCaller<void(const bool)> SetHollowVisibility;

    //Joint Trajectory Controller
    RTT::OperationCaller<void(void)> CallerStopMotion;
    RTT::OperationCaller<bool(const std::vector<double>&,const double)> CallerSetPointToPoint;
    RTT::OperationCaller<bool(const std::vector<std::vector<double> >&,const std::vector<double>&)> CallerSetMotionThroughPoints;
    RTT::OperationCaller<bool(bool)> CallerSetOTGPosition;
    RTT::OperationCaller<bool(bool)> CallerSetOTGVelocity;
    RTT::OperationCaller<void(void)> CallerRecoverFromError;

    //Joint Trajectory Controller Hollow
    RTT::OperationCaller<bool(const std::vector<double>&)> CallerInstantMotion;
    RTT::OperationCaller<bool(const std::vector<double>&,const double)> CallerSetPointToPointHollow;
    RTT::OperationCaller<bool(const std::vector<std::vector<double> >&,const std::vector<double>&)> CallerSetMotionThroughPointsHollow;

    //Kinematics Module
    RTT::OperationCaller<bool(const std::vector<KDL::Frame>&, std::vector<std::vector<double> >&)> CallerCalculateJointTrajectory;
    RTT::OperationCaller<bool(KDL::Frame, std::vector<double>&, std::vector<std::vector<double> >&)> CallerCalculateNullspaceJointTrajectory;
    RTT::OperationCaller<bool(const RTT::vector_vector_double &, const RTT::vector_frame &, RTT::vector_int &,
                              std::vector<double> &, RTT::vector_frame &)> CallerCalculateTrajectoryApproach;
    RTT::OperationCaller<bool(std::vector<KDL::Frame>&, std::vector<int>&, std::vector<std::vector<double> >&)> CallerAnalyseTrajectory;
    RTT::OperationCaller<bool(const RTT::vector_frame &, const int, const double, std::vector<double> &)> CallerAnalyseTrajectoryApproach;
    RTT::OperationCaller<int(std::vector<int>&, std::vector<std::vector<double> >&, std::vector<int>&,
                             std::vector<std::vector<double> >&)> CallerScoresTrajectory;
    RTT::OperationCaller<void(std::vector<double>&, KDL::Frame &, unsigned int &, double &)>CallerForwardKinematics;
    RTT::OperationCaller<bool(KDL::Frame,unsigned int,double,std::vector<double>&)> CallerInverseKinematics;
    RTT::OperationCaller<void(const std::vector<double>&, std::vector<double>&)> CallerCalculateExternalForces;
    RTT::OperationCaller<bool(KDL::Frame,std::vector<double>&)> CallerAdjustedInverseKinematics;
    RTT::OperationCaller<bool(std::vector<double>&)> CallerScoresCurrentPoseCurrentGC;
    RTT::OperationCaller<bool(const int, std::vector<double>&)> CallerScoresCurrentPoseAlternativeGC;
    RTT::OperationCaller<void(bool)> CallerSetOnlineTrajectoryGeneration;

    //Cartesian Trajectory Controller
    RTT::OperationCaller<void(const KDL::Frame,const KDL::Frame,const double)> CallerGenerateLinearTrajectory;
    RTT::OperationCaller<void(const KDL::Frame,const KDL::Vector,const KDL::Frame,const double,const double)> CallerGenerateArcTrajectory;
    RTT::OperationCaller<void(const std::vector<KDL::Frame>,const double,const double)> CallerGenerateComposedTrajectory;
    RTT::OperationCaller<bool(const double, const double, std::vector<double>&, std::vector<double>&)> CallerGenerateNullspaceTrajectory;
    RTT::OperationCaller<bool(std::vector<KDL::Frame>&,std::vector<double>&)> CallerGetTrajectory;
    RTT::OperationCaller<void(KDL::Frame, KDL::Frame, const double, std::vector<KDL::Frame>&)> CallerSampleLinearTrajectory;

    //Transformations
    RTT::OperationCaller<void(const KDL::Frame)> CallerSetWorkFrame;
    RTT::OperationCaller<bool(const int)> CallerSetToolType;
    RTT::OperationCaller<int()> CallerGetToolType;
    RTT::OperationCaller<void(const int, KDL::Frame&)> CallerGetToolTypeFrame;
    RTT::OperationCaller<void(KDL::Frame&)> CallerGetToolFrame;
    RTT::OperationCaller<bool(const int,KDL::Frame&)> CallerGetReferenceFrame;

    //KUKA LBR FRI
    RTT::OperationCaller<void()> CallerRecoverFRI;
    RTT::OperationCaller<void()> CallerStopRobotFRI;
    RTT::OperationCaller<void()> CallerResumeRobotFRI;

protected:
    //! SendHandle for Cartesian component
    RTT::SendHandle<void(KDL::Frame,KDL::Frame,double)> sh_lin_motion;
    RTT::SendHandle<void(KDL::Frame,KDL::Vector,KDL::Frame,double,double)> sh_arc_motion;
    RTT::SendHandle<void(std::vector<KDL::Frame>,double,double)> sh_cmp_motion;
    RTT::SendHandle<bool(const double, const double, std::vector<double>&, std::vector<double>&)> sh_nsp_motion;
    RTT::SendHandle<void(KDL::Frame, KDL::Frame, const double, std::vector<KDL::Frame>&)> sh_samp_lin_path;
    //! SendHandle Kinematics component
    RTT::SendHandle<bool(const std::vector<KDL::Frame>&, std::vector<std::vector<double> >&)> sh_calc_joint_traj;
    RTT::SendHandle<bool(KDL::Frame, std::vector<double>&, std::vector<std::vector<double> >&)> sh_calc_ns_traj;
    //! SendHandle JTC component
    RTT::SendHandle<bool(const std::vector<double>&,const double)> sh_ptp_motion;
    RTT::SendHandle<bool(const std::vector<std::vector<double> >&,const std::vector<double>&)> sh_mtp_motion;

protected:
    //Joint Input Ports
    RTT::InputPort < std::vector<double> > inport_target_online_joint_positions;
    RTT::InputPort < std::vector<double> > inport_target_online_joint_velocities;
    RTT::InputPort < RTT::posture > inport_target_online_robot_posture;

    //Robot State Ports
    RTT::InputPort < std::vector<double> > inport_current_joint_positions;
    RTT::InputPort < std::vector<double> > inport_external_joint_torques;
    RTT::InputPort < RTT::posture > inport_current_robot_posture;
    RTT::InputPort < double > inport_current_endeffector_position;
    RTT::InputPort < bool > inport_robot_moving;
    RTT::InputPort < RTT::error_msg > inport_error_msg;

    //Force Dimension Haptic Ports
    RTT::InputPort< KDL::Frame > inport_target_relative_frame;
    RTT::OutputPort< std::vector<double> > outport_joint_torque_force;

    //KUKA Ports
    RTT::InputPort < int > inport_robot_session_state;
    RTT::InputPort < double > inport_tracking_performance;
    RTT::OutputPort < int > outport_robot_session_state;
    RTT::OutputPort < double > outport_tracking_performance;

    RTT::OutputPort < std::vector<double> > outport_online_joint_positions;
    RTT::OutputPort < std::vector<double> > outport_online_joint_velocities;

    RTT::OutputPort < RTT::error_msg > outport_error_msg;
    RTT::OutputPort < KDL::Frame > outport_online_frame;

private:
    void print(std::string name, KDL::Frame frame);
};

#endif //OROCOS_TASK_SUPERVISOR_COMPONENT_HPP
