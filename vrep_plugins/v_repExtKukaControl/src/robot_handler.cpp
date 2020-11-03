#include "robot_handler.h"

#include "../v_repExtKukaControl.h"

#include <algorithm>
#include <iostream>

using namespace std;

RobotHandler::RobotHandler()
{
    kuka.name = "LBR_iiwa_7_R800";
    kuka.dofs = 7;

    kuka_hlw.name = "LBR_iiwa_7_R800_hlw";
    kuka_hlw.dofs = 7;
}

void RobotHandler::InitRobots()
{
    //Get Robot Handles
    v_repExtGetRobotHandles(kuka);
    v_repExtGetRobotHandles(kuka_hlw);
    //Get Joint Limits - only get limits for 1 robot since they are identical (real and hollow)
    v_repExtGetJointLimits(kuka);
    //Copy limits to hollow robot
    kuka_hlw.jplim = kuka.jplim;
    kuka_hlw.jvlim = kuka.jvlim;
}

void RobotHandler::GetJointPositions(std::vector<double> &pos)
{
    v_repExtGetJointPositions(kuka.jhandles, pos);
}

void RobotHandler::GetJointVelocities(std::vector<double> &vel)
{
    v_repExtGetJointVelocities(kuka.jhandles, vel);
}

void RobotHandler::GetJointPositionsHollow(std::vector<double> &hpos)
{
    v_repExtGetJointPositions(kuka_hlw.jhandles, hpos);
}

void RobotHandler::GetJointVelocitiesHollow(std::vector<double> &hvel)
{
    v_repExtGetJointVelocities(kuka_hlw.jhandles, hvel);
}

void RobotHandler::SetJointPositions(const std::vector<double> &dpos)
{
    v_repExtSetJointPositions(kuka.jhandles, dpos, false);
}

void RobotHandler::SetJointPositionsHollow(const std::vector<double> &dhpos)
{
    v_repExtSetJointPositions(kuka_hlw.jhandles, dhpos, true);
}

void RobotHandler::SetHollowVisibility(const bool visible)
{
    v_repExtSetHollowVisibility(kuka_hlw.handle, visible);
}

bool RobotHandler::GetSurgeryReference(std::vector<double> &matrix)
{
    return v_repExtGetSurgeryReference(kuka.handle, matrix);
}


