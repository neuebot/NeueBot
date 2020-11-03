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
#include "v_repExtEndEffector.h"
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

#define PLUGIN_NAME "EndEffector"
#define PLUGIN_VERSION 1 //

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

using namespace std;

//GLOBAL variables
//SETUP plugin deployment
std::string comp_name = "endeffector_interface";
double period = 0.05;

PluginDeployer pd(comp_name, period);

bool v_repExtGetRobotConnectionInfo(int &conn_handle)
{
    int handle = simGetObjectHandle("LBR_iiwa_7_R800_connection");
    if(handle != -1)
    {
        conn_handle = handle;
        return true;
    }
    v_repExtErrorDialog("Could not retrieve robot connection handle.");
    return false;
}

bool v_repExtLoadTool(Tool &t)
{
    int handle = simGetObjectHandle(t.name.c_str());
    simAddStatusbarMessage(t.name.c_str());
    if(handle!=-1)
    {
        t.base_handle = handle;
        float pos[3], ori[3];
        if(simGetObjectPosition(handle, -1, pos) != -1 &&
                simGetObjectOrientation(handle, -1, ori) != -1)
        {
            t.origin_pos.insert(t.origin_pos.begin(), pos, pos+3);
            t.origin_ori.insert(t.origin_ori.begin(), ori, ori+3);
        }
        else
        {
            v_repExtErrorDialog("Could not retrieve original tool position and orientation.");
            return false;
        }
        if(t.has_joint)
        {
            string joint_name = t.name + "_jnt";
            int jhandle = simGetObjectHandle(joint_name.c_str());
            if(jhandle != -1)
            {
                t.joint_handle = jhandle;
                float limits[2]; //[0] minimum, [1] range
                simBool cyclic; //no
                if(simGetJointInterval(t.joint_handle, &cyclic, limits) != -1)
                {
                    t.limits = std::pair<double,double>(limits[0], limits[0] + limits[1]);
                }
                else
                {
                    v_repExtErrorDialog("Could not retrieve tool's joint limits.");
                    return false;
                }
            }
            else
            {
                v_repExtErrorDialog("Could not retrieve tool's joint handle.");
                return false;
            }
        }
    }
    else
    {
        v_repExtErrorDialog("Could not retrieve tool's base handle.");
        return false;
    }

    return true;
}

bool v_repExtSetTool(const int tool_handle, const int conn_handle)
{
    //Flag +sim_handleflag_assembly allows to move the end-effector to desired position
    if(simSetObjectParent(tool_handle+sim_handleflag_assembly,conn_handle,false) == -1)
    {
        v_repExtErrorDialog("Could not attach new tool.");
        return false;
    }
    return true;
}

bool v_repExtRemoveTool(const Tool &current)
{
    //Remove current end-effector and place it in pos and ori specified.
    //Make child parentless (do not keep in place)
    if(simSetObjectParent(current.base_handle,-1,false) != -1)
    {
        simFloat pos[3], ori[3];
        std::copy(current.origin_pos.begin(), current.origin_pos.end(), pos);
        std::copy(current.origin_ori.begin(), current.origin_ori.end(), ori);

        if(simSetObjectPosition(current.base_handle,-1,pos) != -1 &&
            simSetObjectOrientation(current.base_handle,-1,ori) != -1)
        {
            if(current.has_joint)
            {
                if(simSetJointPosition(current.joint_handle, 0.0) == -1)
                {
                    v_repExtErrorDialog("Could not move tool joint to its original position.");
                    return false;
                }
            }
            return true;
        }
        else
        {
            v_repExtErrorDialog("Could not place the tool on the table.");
            return false;
        }
    }
    else
    {
        v_repExtErrorDialog("Could not detach tool.");
        return false;
    }
}

bool v_repExtMoveTool(const Tool &current, const double dist)
{
    float cpos;
    if(simGetJointPosition(current.joint_handle, &cpos) != -1)
    {
        //Truncate distance to not go beyond limits
        float dpos = cpos + dist;
        dpos = std::max(dpos, current.limits.first);
        dpos = std::min(dpos, current.limits.second);
        string signal = "moveDistance" + std::to_string(current.joint_handle);
        //Motion controller set in Lua simRMLMoveToJointPosition
        if(simSetIntegerSignal((simChar*)"inMotion", 1) != -1 &&
                simSetFloatSignal(signal.c_str(),dpos) != -1)
        {
            return true;
        }
        else
        {
            v_repExtErrorDialog("Could not execute tool joint motion.");
            return false;
        }
    }
    else
    {
        v_repExtErrorDialog("Could not retrieve current tool joint position.");
        return false;
    }
}

void v_repExtGetToolPos(const int joint_handle, double &cur_pos)
{
    float cpos;
    if(simGetJointPosition(joint_handle, &cpos) != -1)
    {
        cur_pos = static_cast<double>(cpos);
    }
}

void v_repExtErrorDialog(const string &message)
{
    simDisplayDialog("End Effector Plugin Error",
                     message.c_str(),
                     sim_dlgstyle_ok,
                     NULL, //no input
                     NULL, //title default colors
                     NULL, //title default colors
                     NULL); //custom uiHandle
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
