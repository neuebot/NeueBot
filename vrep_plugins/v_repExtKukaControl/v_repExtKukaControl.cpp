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
#include "v_repExtKukaControl.h"
#include "plugin_deployer.h"

#include "vrep/include/scriptFunctionData.h"
#include "vrep/include/v_repLib.h"
#include "vrep/include/v_repConst.h"
#include "vrep/include/v_repTypes.h"

#include <cstring>
#include <memory>
#include <iostream>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#endif /* __linux || __APPLE__ */

#ifdef __APPLE__
#define _stricmp strcmp
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_NAME "KukaControl"
#define PLUGIN_VERSION 1 //

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

using namespace std;

//GLOBAL variables
//SETUP plugin deployment
std::string comp_name = "kukacontrol_interface";
double period = 0.05;

PluginDeployer pd(comp_name, period);

bool v_repExtGetRobotHandles(Robot &robot)
{
    //Get robot handles
    int rhandle = simGetObjectHandle(robot.name.c_str());
    if(rhandle != -1)
    {
        robot.handle = rhandle;
        //Get robot joint handles
        int bit_options, num_objects;
        bit_options = 1; //bit0 set (1): exclude the tree base from the returned array
        int *ret = simGetObjectsInTree(rhandle,sim_object_joint_type,bit_options,&num_objects);
        if(ret != nullptr)
        {
            if(num_objects == robot.dofs)
            {
                robot.jhandles.assign(ret, ret+num_objects);
            }
            else
            {
                v_repExtErrorDialog("Number of joint objects in the robot does not correspond to the model.");
                return false;
            }
        }
        else {
            v_repExtErrorDialog("Could not retrieve the robot joint handles.");
            return false;
        }
    }
    else {
        v_repExtErrorDialog("Could not retrieve the robot handle.");
        return false;
    }
    return true;
}

bool v_repExtGetJointLimits(Robot &robot)
{
    simBool cyclic;
    float limits[2]; //[0] minimum, [1] range
    robot.jplim.resize(robot.dofs);
    robot.jvlim.resize(robot.dofs);
    for(int i=0; i<robot.dofs; ++i)
    {
        //Get position limits
        if(simGetJointInterval(robot.jhandles[i],&cyclic,limits) != -1)
        {
            robot.jplim[i] = std::pair<double, double>(limits[0], limits[0] + limits[1]);
        }
        else
        {
            v_repExtErrorDialog("Could not retrieve robot's joints position limits.");
            return false;
        }
        //Get velocity limits
        float parameter;
        if(simGetObjectFloatParameter(robot.jhandles[i],sim_jointfloatparam_upper_limit,&parameter) == 1)
        {
            robot.jvlim[i] = parameter;
        }
        else
        {
            v_repExtErrorDialog("Could not retrieve robot's joints velocity limit.");
            return false;
        }
    }
    return true;
}

void v_repExtErrorDialog(const string &message)
{
    simDisplayDialog("Kuka Control Plugin Error",
                     message.c_str(),
                     sim_dlgstyle_ok,
                     NULL, //no input
                     NULL, //title default colors
                     NULL, //title default colors
                     NULL); //custom uiHandle
}

bool v_repExtGetJointPositions(const std::vector<int> &jhandles, std::vector<double> &pos)
{
    pos.resize(jhandles.size());
    for(size_t i=0; i<jhandles.size(); ++i)
    {
        simFloat jpos;
        if(simGetJointPosition(jhandles[i], &jpos) != -1)
        {
            pos[i] = static_cast<double>(jpos);
        }
        else
        {
            v_repExtErrorDialog("Could not retrieve current joint positions.");
            return false;
        }
    }
    return true;
}

bool v_repExtGetJointVelocities(const std::vector<int> &jhandles, std::vector<double> &vel)
{
    vel.resize(jhandles.size());
    for(size_t i=0; i<jhandles.size(); ++i)
    {
        simFloat jvel;
        if(simGetObjectFloatParameter(jhandles[i],sim_jointfloatparam_velocity,&jvel) != -1)
        {
            vel[i] = static_cast<double>(jvel);
        }
        else
        {
            v_repExtErrorDialog("Could not retrieve current joint velocities.");
            return false;
        }
    }
    return true;
}

bool v_repExtSetJointPositions(const std::vector<int> &jhandles, const std::vector<double> &dpos, const bool passive)
{
    if(jhandles.size() != dpos.size())
    {
        v_repExtErrorDialog("Joint handles specified do not match target joint positions.");
        return false;
    }

    if(passive)
    {
        for(size_t i=0; i<jhandles.size(); ++i)
        {
            if(simSetJointPosition(jhandles[i], dpos[i]) == -1)
            {
                v_repExtErrorDialog("Could not set desired joint position for a passive joint.");
                return false;
            }
        }
    }
    else
    {
        for(size_t i=0; i<jhandles.size(); ++i)
        {
            if(simSetJointTargetPosition(jhandles[i], dpos[i]) == -1)
            {
                v_repExtErrorDialog("Could not set desired joint position for a dynamic joint.");
                return false;
            }
        }
    }
    return true;
}

bool v_repExtSetHollowVisibility(const int robot_handle, const bool visible)
{
    if(visible) {
        int prop = simGetModelProperty(robot_handle);
        int not_prop = prop & ~sim_modelproperty_not_visible;
        if(simSetModelProperty(robot_handle,not_prop) == -1)
        {
            v_repExtErrorDialog("Could not set Hollow Robot to visible");
            return false;
        }
    }
    else {
        int prop = simGetModelProperty(robot_handle);
        int not_prop = prop | sim_modelproperty_not_visible;
        if(simSetModelProperty(robot_handle,not_prop) == -1)
        {
            v_repExtErrorDialog("Could not set Hollow Robot to invisible");
            return false;
        }
    }
    return true;
}

bool v_repExtGetSurgeryReference(const int robot_handle, std::vector<double> &matrix)
{
    matrix.resize(12);
    int surg_ref_handle = simGetObjectHandle("SurgeryReference");
    if(surg_ref_handle != -1)
    {
        simFloat mat_arr[12];
        if(simGetObjectMatrix(surg_ref_handle, robot_handle, mat_arr) != -1)
        {
            matrix.assign(mat_arr, mat_arr+12);
        }
        else
        {
            v_repExtErrorDialog("Could not retrieve surgery reference matrix.");
            return false;
        }
    }
    else
    {
        v_repExtErrorDialog("Could not retrieve surgery reference handle.");
        return false;
    }
    return true;
}

// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
    // Dynamically load and bind V-REP functions:
    // ******************************************
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the V-REP library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\v_rep.dll";
#elif defined (__linux)
    temp+="/libv_rep.so";
#elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the V-REP library:
    vrepLib=loadVrepLibrary(temp.c_str());
    if (vrepLib==NULL)
    {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start '" << PLUGIN_NAME << "'.\n";
        return(0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start '" << PLUGIN_NAME << "'.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    // Check the version of V-REP:
    // ******************************************
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
    if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start '" << PLUGIN_NAME << "'.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    std::vector<int> inArgs;

    // Register the new Lua command "simExtEndEffector_functions":

    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    void* retVal=nullptr;

    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

    // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the V-REP user manual.

    if (message==sim_message_eventcallback_refreshdialogs)
    {
        refreshDlgFlag=true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too
    }

    if (message==sim_message_eventcallback_menuitemselected)
    { // A custom menu bar entry was selected..
      // here you could make a plugin's main dialog visible/invisible
    }

    if (message==sim_message_eventcallback_instancepass)
    {   // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:
        int flags=auxiliaryData[0];
        bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
        bool instanceSwitched=((flags&64)!=0);

        if (instanceSwitched)
        { // React to an instance switch here!!
        }

        if (sceneContentChanged)
        { // we actualize plugin objects for changes in the scene
            refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message==sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))
    }

    if (message==sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start
        simAddStatusbarMessage("Launched ORB. End Effector plugin communication channels initialized.");
        pd.LaunchComm();
    }

    if (message==sim_message_eventcallback_simulationended)
    { // Simulation just ended
        simAddStatusbarMessage("Closed ORB. End Effector plugin communication channels stopped.");
        pd.StopComm();
    }

    if (message==sim_message_eventcallback_moduleopen)
    { // A script called simOpenModule (by default the main script). Is only called during simulation.
    }

    if (message==sim_message_eventcallback_modulehandle)
    { // A script called simHandleModule (by default the main script). Is only called during simulation.
    }

    if (message==sim_message_eventcallback_moduleclose)
    { // A script called simCloseModule (by default the main script). Is only called during simulation.
    }

    if (message==sim_message_eventcallback_instanceswitch)
    { // We switched to a different scene. Such a switch can only happen while simulation is not running
    }

    if (message==sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)
    }

    if (message==sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)
    }

    // You can add many more messages to handle here

    if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag=false;
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}
