// Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.3.2 on August 29th 2016

#pragma once

#include "robot_handler.h"
#include "endeffector_handler.h"
#include "surgery_trajectories_handler.h"
#include "ik_handler.h"

#include <string>

#ifdef _WIN32
    #define VREP_DLLEXPORT extern "C" __declspec(dllexport)
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #define VREP_DLLEXPORT extern "C"
#endif /* __linux || __APPLE__ */


// The 3 required entry points of the V-REP plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);

/*!
 * Guaranteeing the VRep standard API functions are always called from the VRep main thread.
 *
 * Instead of directly calling this functions from communication threads, the information is
 * passed to queues that are push/poped from the communication or VRep thread.
 *
 * To clear the plugin files, it is created objects that encapsulate information relative to
 * robots, end-effectors or surgical trajectories.
 */

//Robot Communication Functions
void v_repExtPQGetJointPositions(std::vector<double> &pos);
void v_repExtPQGetJointVelocities(std::vector<double> &vel);
void v_repExtPQGetJointPositionsHollow(std::vector<double> &hpos);
void v_repExtPQGetJointVelocitiesHollow(std::vector<double> &hvel);
void v_repExtPQSetJointPositions(const std::vector<double> &dpos);
void v_repExtPQSetJointPositionsHollow(const std::vector<double> &dhpos);
void v_repExtPQActivateBrakes(const bool lock); //only real
void v_repExtPQSetHollowVisibility(const bool visible); //only hollow

//End Effector Communication Functions
int v_repExtPQGetTool();
void v_repExtPQSetTool(const int type);
void v_repExtPQMoveTool(const double dist);
double v_repExtPQGetToolPosition();

//Trajectories Communication Functions
void v_repExtPQGetSurgeryReference(std::vector<double> &matrix); //set on initialization
void v_repExtPQAddTrajectory(const int id, const std::vector<double> &traj_vec);
void v_repExtPQRemoveTrajectory(const int id);
void v_repExtPQRemoveAll();
void v_repExtPQShowTrajectory(const int show_handle);
void v_repExtPQShowAll();

namespace _internal_ {
    //Robot Control
    void v_repExtGetRobotHandles(RobotHandler *robot);
    void v_repExtGetIKHandles(RobotHandler *robot);
    bool v_repExtGetJointPositions(const std::vector<int> &jhandles, std::vector<double> &pos);
    bool v_repExtGetJointVelocities(const std::vector<int> &jhandles, std::vector<double> &vel);
    bool v_repExtSetJointPositions(const std::vector<int> &jhandles, const std::vector<double> &dpos, const bool passive);
    bool v_repExtActivateBrakes(const std::vector<int> &jhandles, const bool lock);
    bool v_repExtSetHollowVisibility(const int robot_handle, const bool visible);

    //End Effector Control
    bool v_repExtInitializeEndEffectors(EndEffectorHandler *endeffector);
    bool v_repExtLoadTool(Tool &t);
    bool v_repExtSetTool(const int tool_handle, const int conn_handle);
    bool v_repExtRemoveTool(const Tool &current);
    bool v_repExtMoveTool(const Tool &current, const double dist);
    void v_repExtGetToolPosition(const int joint_handle, double &cur_pos);

    //Surgery Trajectories Control
    void v_repExtGetSurgeryReference(const int robot_handle);
    bool v_repExtAddTrajectory(Trajectory *t);
    bool v_repExtRemoveTrajectories(const std::vector<int> &handle);
    bool v_repExtShowTrajectories(const std::vector<int> &show_handles, const std::vector<int> &hide_handles);

    //General
    void v_repExtErrorDialog(const std::string &message);
}



