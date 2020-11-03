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

bool v_repExtGetRobotHandles(Robot &robot);
bool v_repExtGetJointLimits(Robot &robot);

bool v_repExtGetJointPositions(const std::vector<int> &jhandles, std::vector<double> &pos);
bool v_repExtGetJointVelocities(const std::vector<int> &jhandles, std::vector<double> &vel);
bool v_repExtSetJointPositions(const std::vector<int> &jhandles, const std::vector<double> &dpos, const bool passive);
bool v_repExtSetHollowVisibility(const int robot_handle, const bool visible);
bool v_repExtGetSurgeryReference(const int robot_handle, std::vector<double> &matrix);

void v_repExtErrorDialog(const std::string &message);

