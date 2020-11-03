#include "surgery_trajectories_component.h"
#include "../v_repExtSurgRobotControl.h"

#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Component.hpp>

#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace RTT;
using namespace Eigen;
using namespace std;

SurgeryTrajectoriesComponent::SurgeryTrajectoriesComponent(const string &name) : TaskContext(name) {
    this->addOperation("GetSurgeryReference", &SurgeryTrajectoriesComponent::GetSurgeryReference, this, ClientThread);
    this->addOperation("AddTrajectory",&SurgeryTrajectoriesComponent::AddTrajectory, this, OwnThread);
    this->addOperation("RemoveTrajectories",&SurgeryTrajectoriesComponent::RemoveTrajectories, this, OwnThread);
    this->addOperation("ShowTrajectory",&SurgeryTrajectoriesComponent::ShowTrajectory, this, OwnThread);
    this->addOperation("ShowTrajectories",&SurgeryTrajectoriesComponent::ShowTrajectories, this, OwnThread);
}

bool SurgeryTrajectoriesComponent::configureHook() {
    //Set component activity
    this->setActivity( new Activity(os::LowestPriority, 0.0) );

    return true;
}

bool SurgeryTrajectoriesComponent::startHook() {
    return true;
}

void SurgeryTrajectoriesComponent::updateHook() {
}

void SurgeryTrajectoriesComponent::stopHook() {
}

void SurgeryTrajectoriesComponent::cleanupHook() {
}

void SurgeryTrajectoriesComponent::GetSurgeryReference(std::vector<double> &surg_ref) {
    std::vector<double> matrix;
    v_repExtPQGetSurgeryReference(matrix);
    surg_ref = matrix;
}

void SurgeryTrajectoriesComponent::AddTrajectory(const int id, const std::vector<double> &target, const std::vector<double> &entry)
{
    // Here we receive the target and entry points, convert to a position (vector) and orientation (quaternion)
    // Which will be passed to the v_rep function as a set of 2 variables:
    // 1. The trajectory ID: int
    // 2. A trajectory vector composed of: [px, py, pz, qx, qy, qz, qw]

    //Z-axis is the dir vector.
    //Y-axis is the cross product of Z-axis and the front-vector [1 0 0]
    //X-axis is the complementar vector given by the cross product of Y-axis and Z-axis
    Vector3d front(1,0,0);
    Vector3d zaxis(target[0]-entry[0], target[1]-entry[1], target[2]-entry[2]);
    zaxis.normalize();
    //If z-axis is collinear with front vector, the dot product is 1 and cross product is the zero vector
    Vector3d yaxis(0,1,0);
    if(std::abs(zaxis.dot(front)) != 1)
    {
        yaxis = zaxis.cross(front);
        yaxis.normalize();
    }
    Vector3d xaxis = yaxis.cross(zaxis);
    xaxis.normalize();
    Matrix3d rot(3,3);
    rot << xaxis, yaxis, zaxis;
    Quaterniond q(rot);
    q.normalize();

    std::vector<double> trajectory_vector(7);
    trajectory_vector[0] = target[0];
    trajectory_vector[1] = target[1];
    trajectory_vector[2] = target[2];
    trajectory_vector[3] = q.x();
    trajectory_vector[4] = q.y();
    trajectory_vector[5] = q.z();
    trajectory_vector[6] = q.w();

    v_repExtPQAddTrajectory(id, trajectory_vector);
}

void SurgeryTrajectoriesComponent::RemoveTrajectories() {
    v_repExtPQRemoveAll();
}

void SurgeryTrajectoriesComponent::ShowTrajectory(const int id) {
    v_repExtPQShowTrajectory(id);
}

void SurgeryTrajectoriesComponent::ShowTrajectories() {
    v_repExtPQShowAll();
}
