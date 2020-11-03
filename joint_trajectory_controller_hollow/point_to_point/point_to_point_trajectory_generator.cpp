#include "point_to_point_trajectory_generator.hpp"

#include <cmath>

PointToPointTrajectoryGenerator::PointToPointTrajectoryGenerator(const unsigned int n_joints, const double cycle_time,
                                                                 const double max_vel, const double max_acc, const double max_jerk) :
    m_nj(n_joints),
    m_dt(cycle_time),
    m_v_max(max_vel),
    m_a_max(max_acc),
    m_j_max(max_jerk)
{
    //Initialize RML API, Input and Output parameters for JOINT POSITION CONTROL
    RML = std::make_shared<ReflexxesAPI>(m_nj,m_dt);
    IPpos = std::make_shared<RMLPositionInputParameters>(m_nj);
    OPpos = std::make_shared<RMLPositionOutputParameters>(m_nj);
    IPvel = std::make_shared<RMLVelocityInputParameters>(m_nj);
    OPvel = std::make_shared<RMLVelocityOutputParameters>(m_nj);
    //RML_FINAL_STATE_REACHED, to start component with a new cycle
    ResultValue = ReflexxesAPI::RML_FINAL_STATE_REACHED;

    for(unsigned int i = 0; i<m_nj; i++) {
        //                      ---  POSITION  ---
        //When launching the component the robot should have null vel and acc
        IPpos->CurrentVelocityVector->VecData[i] = 0.0;
        IPpos->CurrentAccelerationVector->VecData[i] = 0.0;
        //Defining max velocity, acceleration and jerk
        IPpos->MaxVelocityVector->VecData[i] = m_v_max;
        IPpos->MaxAccelerationVector->VecData[i] = m_a_max;
        IPpos->MaxJerkVector->VecData[i] = m_j_max;
        //Actuate on all joints
        IPpos->SelectionVector->VecData[i] = true;

        //                      ---  VELOCITY  ---
        //When launching the component the robot should have null vel and acc
        IPvel->CurrentVelocityVector->VecData[i] = 0.0;
        IPvel->CurrentAccelerationVector->VecData[i] = 0.0;
        //Defining max acceleration and jerk
        IPvel->MaxAccelerationVector->VecData[i] = m_a_max;
        IPvel->MaxJerkVector->VecData[i] = m_j_max;
        //Actuate on all joints
        IPvel->SelectionVector->VecData[i] = true;
    }
    //Define synchronization behavior
    Flagspos.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    Flagspos.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::KEEP_TARGET_VELOCITY;
    Flagspos.EnableTheCalculationOfTheExtremumMotionStates = true; //5% computation effort

    Flagsvel.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    Flagsvel.EnableTheCalculationOfTheExtremumMotionStates = true;
}

PointToPointTrajectoryGenerator::~PointToPointTrajectoryGenerator() {
}

int PointToPointTrajectoryGenerator::get_ResultValue()
{
    return ResultValue;
}

bool PointToPointTrajectoryGenerator::InitiateNewPositionMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                                                        const vector<double> &target_joint_positions, const double max_rel_vel)
{
    bool success = false;
    if(ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
        success = true;
        //RML only starts a new trajectory when fresh current joint pos and vel variables are read
        for(unsigned int j = 0; j<m_nj; ++j) {
            //Defining velocity limit
            IPpos->MaxVelocityVector->VecData[j] = m_v_max * max_rel_vel;
            //Final state
            IPpos->TargetPositionVector->VecData[j] = target_joint_positions[j];
            IPpos->TargetVelocityVector->VecData[j] = 0.0;
            //TEST
            IPpos->CurrentPositionVector->VecData[j] = current_joint_positions[j];
            IPpos->CurrentVelocityVector->VecData[j] = current_joint_velocities[j];
        }
    }
    return success;
}

bool PointToPointTrajectoryGenerator::InitiateNewVelocityMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                                                                const vector<double> &target_joint_velocities, const double max_rel_vel)
{
    bool success = false;
    if(ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
        success = true;
        //RML only starts a new trajectory when fresh current joint pos and vel variables are read
        for(unsigned int j = 0; j<m_nj; ++j) {
            //Final state
            IPvel->TargetVelocityVector->VecData[j] = target_joint_velocities[j];
        }
        for(unsigned int i = 0; i<m_nj; ++i) {
            IPvel->CurrentPositionVector->VecData[i] = current_joint_positions[i];
            IPvel->CurrentVelocityVector->VecData[i] = current_joint_velocities[i];
        }
    }
    return success;
}

int PointToPointTrajectoryGenerator::GenerateNextPositionMotionState(vector<double> &next_joint_positions, const vector<double> &current_joint_positions) {
    //Where magic happens
    ResultValue = RML->RMLPosition( *IPpos.get(), OPpos.get(), Flagspos );
    if(ResultValue >= 0) {
        next_joint_positions.resize(m_nj);
        for(unsigned int j = 0; j<m_nj; j++) {
            next_joint_positions[j] = OPpos->NewPositionVector->VecData[j];
        }
        //Update RML Input parameters
        if(IPpos->CheckForValidity()) {
            for(unsigned int i = 0; i<m_nj; ++i) {
                IPpos->CurrentPositionVector->VecData[i] = OPpos->NewPositionVector->VecData[i];
                IPpos->CurrentVelocityVector->VecData[i] = OPpos->NewVelocityVector->VecData[i];
                IPpos->CurrentAccelerationVector->VecData[i] = OPpos->NewAccelerationVector->VecData[i];
            }
        }
    }
    return ResultValue;
}

int PointToPointTrajectoryGenerator::GenerateNextVelocityMotionState(vector<double> &next_joint_positions, vector<double> &next_joint_velocities,
                                                                     const vector<double> &current_joint_positions)
{
    //Where magic happens
    ResultValue = RML->RMLVelocity( *IPvel.get(), OPvel.get(), Flagsvel );
    if(ResultValue >= 0) {
        next_joint_positions.resize(m_nj);
        for(unsigned int j = 0; j<m_nj; j++) {
            next_joint_positions[j] = OPvel->NewPositionVector->VecData[j];
            next_joint_velocities[j] = OPvel->NewVelocityVector->VecData[j];
        }
        //Update RML Input parameters
        if(IPvel->CheckForValidity()) {
            for(unsigned int i = 0; i<m_nj; ++i) {
                IPvel->CurrentPositionVector->VecData[i] = OPvel->NewPositionVector->VecData[i];
                IPvel->CurrentVelocityVector->VecData[i] = OPvel->NewVelocityVector->VecData[i];
                IPvel->CurrentAccelerationVector->VecData[i] = OPvel->NewAccelerationVector->VecData[i];
            }
        }
    }
    return ResultValue;
}

void PointToPointTrajectoryGenerator::AdaptPositionMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                                                 const vector<double> &target_joint_positions, const double max_rel_vel)
{
    for(unsigned int j = 0; j<m_nj; ++j) {
        //Defining velocity limit
        IPpos->MaxVelocityVector->VecData[j] = m_v_max * max_rel_vel;
        //Current state
        IPpos->CurrentPositionVector->VecData[j] = current_joint_positions[j];
        IPpos->CurrentVelocityVector->VecData[j] = current_joint_velocities[j];
        //Final state
        IPpos->TargetPositionVector->VecData[j] = target_joint_positions[j];
        IPpos->TargetVelocityVector->VecData[j] = 0.0;
    }
}

void PointToPointTrajectoryGenerator::AdaptVelocityMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                                                          const vector<double> &target_joint_velocities)
{
    for(unsigned int j = 0; j<m_nj; ++j) {
        //Defining velocity limit
        //Current state
        IPvel->CurrentPositionVector->VecData[j] = current_joint_positions[j];
        IPvel->CurrentVelocityVector->VecData[j] = current_joint_velocities[j];
        //Final state
        IPvel->TargetVelocityVector->VecData[j] = target_joint_velocities[j];
    }
}

void PointToPointTrajectoryGenerator::RecoverFromErrorState() {
    ResultValue = ReflexxesAPI::RML_FINAL_STATE_REACHED;
}


