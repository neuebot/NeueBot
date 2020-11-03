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
#include "v_repExtSurgRobotControl.h"
#include "plugin_deployer.h"
#include "string_concurrent_queue.h"

#include "vrep/include/scriptFunctionData.h"
#include "vrep/include/v_repLib.h"
#include "vrep/include/v_repConst.h"
#include "vrep/include/v_repTypes.h"

#include <cmath>
#include <algorithm>
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

#define PLUGIN_NAME "SurgRobotControl"
#define PLUGIN_VERSION 1 //

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

using namespace std;

//Component Initialization Variables
bool plugin_loaded = false;
std::string robot_comp = "robot_interface";
std::string endeffector_comp = "endeffector_interface";
std::string traj_comp = "trajectories_interface";
PluginDeployer pd(robot_comp, endeffector_comp, traj_comp);

//Object Handlers
std::shared_ptr<RobotHandler> ptr_robot;
std::shared_ptr<RobotHandler> ptr_robot_hlw;
std::shared_ptr<EndEffectorHandler> ptr_end_effector;
std::shared_ptr<SurgeryTrajectoriesHandler> ptr_surg_traj;
std::shared_ptr<IKHandler> ptr_ik;

std::string rname;
StringConcurrentQueue error_scq;
StringConcurrentQueue error_mcq;

/// LOAD PLUGIN FUNCTION

// --------------------------------------------------------------------------------------
// simExtSurgRobotControl_init: Launch this plugin
// --------------------------------------------------------------------------------------
#define LUA_INIT_SURGROBOTCONTROL_COMMAND "simExtSurgRobotControl_init" // the name of the new Lua command

const int inArgs_INIT_SURGROBOTCONTROL[] = {
    1,
    sim_script_arg_string,0,
};

void LUA_INIT_SURGROBOTCONTROL_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_INIT_SURGROBOTCONTROL,inArgs_INIT_SURGROBOTCONTROL[0],LUA_INIT_SURGROBOTCONTROL_COMMAND))
    {
        pd.Init();
        plugin_loaded = true;

        std::vector<CScriptFunctionDataItem>* inData = D.getInDataPtr();

        rname = inData->at(0).stringData[0];

        ptr_robot = std::make_shared<RobotHandler>(rname);
        ptr_robot_hlw = std::make_shared<RobotHandler>(rname + "_hlw");
        ptr_end_effector = std::make_shared<EndEffectorHandler>();
        ptr_surg_traj = std::make_shared<SurgeryTrajectoriesHandler>();
        ptr_ik = std::make_shared<IKHandler>(rname + "_hlw");

        if(plugin_loaded)
        {
            _internal_::v_repExtGetRobotHandles(ptr_robot.get());
            _internal_::v_repExtGetRobotHandles(ptr_robot_hlw.get());
            _internal_::v_repExtInitializeEndEffectors(ptr_end_effector.get());
            _internal_::v_repExtGetSurgeryReference(ptr_robot->handle);
            _internal_::v_repExtGetIKHandles(ptr_robot_hlw.get());

            //Get Surgery Reference
            _internal_::v_repExtGetSurgeryReference(ptr_robot->handle);
        }
    }
    D.writeDataToStack(p->stackID);
}

// --------------------------------------------------------------------------------------
// simExtSurgRobotControl_cleanup: Cleanup this plugin
// --------------------------------------------------------------------------------------
#define LUA_CLEANUP_SURGROBOTCONTROL_COMMAND "simExtSurgRobotControl_cleanup" // the name of the new Lua command

void LUA_CLEANUP_SURGROBOTCONTROL_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,nullptr,0,LUA_CLEANUP_SURGROBOTCONTROL_COMMAND))
    {
        ptr_ik.reset();
        ptr_surg_traj.reset();
        ptr_end_effector.reset();
        ptr_robot_hlw.reset();
        ptr_robot.reset();

        pd.Cleanup();
        plugin_loaded = false;
    }
    D.writeDataToStack(p->stackID);
}

////////////////////////////////////////////////////////////////////////////////////////
/// ROBOT PLUGIN INTERFACE

void v_repExtPQGetJointPositions(std::vector<double> &pos)
{
    if(ptr_robot->ready)
    {
        pos = ptr_robot->get_cur_pos();
    }
}

void v_repExtPQGetJointVelocities(std::vector<double> &vel)
{
    if(ptr_robot->ready)
    {
        vel = ptr_robot->get_cur_vel();
    }
}

void v_repExtPQGetJointPositionsHollow(std::vector<double> &hpos)
{
    if(ptr_robot_hlw->ready)
    {
        hpos = ptr_robot_hlw->get_cur_pos();
    }
}

void v_repExtPQGetJointVelocitiesHollow(std::vector<double> &hvel)
{
    if(ptr_robot_hlw->ready)
    {
        hvel = ptr_robot_hlw->get_cur_vel();
    }
}

void v_repExtPQSetJointPositions(const std::vector<double> &dpos)
{
    if(ptr_robot->ready)
    {
        ptr_robot->tar_jpos.push_front(dpos);
    }
}

void v_repExtPQSetJointPositionsHollow(const std::vector<double> &dhpos)
{
    if(ptr_robot_hlw->ready)
    {
        ptr_robot_hlw->tar_jpos.push_front(dhpos);
    }
}

void v_repExtPQActivateBrakes(const bool lock)
{
    if(ptr_robot->ready)
    {
        ptr_robot->act_brakes.push_back(lock);
    }
}

void v_repExtPQSetHollowVisibility(const bool visible)
{
    if(ptr_robot_hlw->ready)
    {
        ptr_robot_hlw->set_vis.push_back(visible);
    }
}


////////////////////////////////////////////////////////////////////////////////////////
/// END EFFECTOR PLUGIN INTERFACE

int v_repExtPQGetTool()
{
    int type = -1;
    if(ptr_end_effector->ready)
    {
        type = ptr_end_effector->current_tool;
    }
    return type;
}

void v_repExtPQSetTool(const int type)
{
    if(ptr_end_effector->ready)
    {
        ptr_end_effector->set_tool.push_back(type);
    }
}

void v_repExtPQMoveTool(const double dist)
{
    if(ptr_end_effector->ready)
    {
        if(ptr_end_effector->tools_vector[ptr_end_effector->current_tool].has_joint)
        {
            ptr_end_effector->move_tool.push_front(dist);
        }
    }
}

double v_repExtPQGetToolPosition()
{
    double tool_position = 0.0;
    if(ptr_end_effector->ready)
    {
        tool_position = ptr_end_effector->tool_pos;
    }
    return tool_position;
}

////////////////////////////////////////////////////////////////////////////////////////
/// TRAJECTORIES PLUGIN INTERFACE
void v_repExtPQGetSurgeryReference(std::vector<double> &matrix)
{
    matrix = ptr_surg_traj->GetSurgReferenceFrame();
}

void v_repExtPQAddTrajectory(const int id, const std::vector<double> &traj_vec)
{
    if(!ptr_surg_traj->ToAddTrajectory(id, traj_vec))
    {
        error_scq.push_back("Trajectory ID added already exists.");
    }
}

void v_repExtPQRemoveTrajectory(const int id)
{
    if(!ptr_surg_traj->ToRemoveTrajectory(id))
    {
        error_scq.push_back("Trajectory ID to remove does not exist.");
    }
}

void v_repExtPQRemoveAll()
{
    ptr_surg_traj->ToRemoveAll();
}

void v_repExtPQShowTrajectory(const int id)
{
    if(!ptr_surg_traj->ToShowTrajectory(id))
    {
        error_scq.push_back("Trajectory ID to show does not exist.");
    }
}

void v_repExtPQShowAll()
{
    ptr_surg_traj->ToShowAll();
}

namespace _internal_ {

//////////////////////////////////// ROBOT INTERNAL ////////////////////////////////////

void v_repExtGetRobotHandles(RobotHandler *robot)
{
    //Get robot handles
    int rhandle = simGetObjectHandle(robot->name.c_str());
    if(rhandle != -1)
    {
        robot->handle = rhandle;
        //Get robot joint handles
        int bit_options, num_objects;
        bit_options = 1; //bit0 set (1): exclude the tree base from the returned array
        int *ret = simGetObjectsInTree(rhandle,sim_object_joint_type,bit_options,&num_objects);
        if(ret != nullptr)
        {
            if(num_objects == robot->dofs)
            {
                robot->jhandles.assign(ret, ret+num_objects);
                robot->ready = true;
            }
            else
            {
                error_scq.push_back("Number of joint objects in the robot does not correspond to the model.");
                return;
            }
        }
        else {
             error_scq.push_back("Could not retrieve the robot joint handles.");
             return;
        }
        string tipname = robot->name + "_tip";
        int tiphandle = simGetObjectHandle(tipname.c_str());
        if(tiphandle != -1) {
            robot->tip_handle = tiphandle;
        }
        else {
            error_scq.push_back("Could not retrieve the robot's tip object handle.");
        }
        string connname = robot->name + "_connection";
        int connhandle = simGetObjectHandle(connname.c_str());
        if(connhandle != -1) {
            robot->conn_handle = connhandle;
        }
        else {
            error_scq.push_back("Could not retrieve the robot's connection object handle.");
        }
    }
    else {
        error_scq.push_back("Could not retrieve the robot handle.");
        return;
    }

    //Get joint limits
    simBool cyclic;
    float limits[2]; //[0] minimum, [1] range
    robot->jplim.resize(robot->dofs);
    robot->jvlim.resize(robot->dofs);
    for(int i=0; i<robot->dofs; ++i)
    {
        //Get position limits
        if(simGetJointInterval(robot->jhandles[i],&cyclic,limits) != -1)
        {
            robot->jplim[i] = std::pair<double, double>(limits[0], limits[0] + limits[1]);
        }
        else
        {
            error_scq.push_back("Could not retrieve robot's joints position limits.");
            return;
        }
        //Get velocity limits
        float parameter;
        if(simGetObjectFloatParameter(robot->jhandles[i],sim_jointfloatparam_upper_limit,&parameter) == 1)
        {
            robot->jvlim[i] = parameter;
        }
        else
        {
            error_scq.push_back("Could not retrieve robot's joints velocity limit.");
            return;
        }
    }
}

void v_repExtGetIKHandles(RobotHandler *robot)
{
    //Check if IK controlled robot is initialized and ready
    if(robot->ready) {
        //Get IK handles
        int ikhandle = simGetIkGroupHandle(ptr_ik->ik_group_handle_str.c_str());
        if(ikhandle != -1)
        {
            int maniphandle = simGetObjectHandle(ptr_ik->manipSphere_str.c_str());
            if(maniphandle != -1) {
                ptr_ik->ik_group_handle = ikhandle;
                ptr_ik->manipSphere_handle = maniphandle;

                ptr_ik->ready = true;
            }
            else {
                //Just an information to the user
                error_scq.push_back("Could not retrieve manipulation sphere handle.");
            }
        }
        else {
            //Just an information to the user
            error_scq.push_back("Could not retrieve IK group handle.");
        }
    }
    else {
        //Just an information to the user
        error_scq.push_back("The robot handle specified does not have an assigned ik group.");
        return;
    }
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
            error_scq.push_back("Could not retrieve current joint positions.");
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
            error_scq.push_back("Could not retrieve current joint velocities.");
            return false;
        }
    }
    return true;
}

bool v_repExtSetJointPositions(const std::vector<int> &jhandles, const std::vector<double> &dpos, const bool passive)
{
    if(jhandles.size() != dpos.size())
    {
        error_scq.push_back("Number of joint handles specified do not match target joint positions.");
        return false;
    }

    if(passive)
    {
        for(size_t i=0; i<jhandles.size(); ++i)
        {
            if(simSetJointPosition(jhandles[i], dpos[i]) == -1)
            {
                error_scq.push_back("Could not set desired joint position for a passive joint.");
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
                error_scq.push_back("Could not set desired joint position for a dynamic joint.");
                return false;
            }
        }
    }
    return true;
}

bool v_repExtActivateBrakes(const std::vector<int> &jhandles, const bool lock) {
    if(lock)
    {
        for(size_t i=0; i<jhandles.size(); ++i)
        {
            if(simSetObjectInt32Parameter(jhandles[i], sim_jointintparam_ctrl_enabled, 0) != 1) {
                error_scq.push_back("Could not lock robot joints.");
                return false;
            }
        }        
    }
    else
    {
        for(size_t i=0; i<jhandles.size(); ++i)
        {
            if(simSetObjectInt32Parameter(jhandles[i], sim_jointintparam_ctrl_enabled, 1) != 1) {
                error_scq.push_back("Could not unlock robot joints.");
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
            error_scq.push_back("Could not set Hollow Robot to visible");
            return false;
        }
    }
    else {
        int prop = simGetModelProperty(robot_handle);
        int not_prop = prop | sim_modelproperty_not_visible;
        if(simSetModelProperty(robot_handle,not_prop) == -1)
        {
            error_scq.push_back("Could not set Hollow Robot to invisible");
            return false;
        }
    }
    return true;
}

//////////////////////////////// END-EFFECTOR INTERNAL /////////////////////////////////

bool v_repExtInitializeEndEffectors(EndEffectorHandler *endeffector)
{
    //Load Tools
    for(auto it = endeffector->tools_vector.begin()+1; it != endeffector->tools_vector.end(); it++)
    {
        if(!_internal_::v_repExtLoadTool(*it))
        {
            //return false;
        }
    }

    endeffector->ready = true;
    return true;
}

bool v_repExtLoadTool(Tool &t)
{
    int handle = simGetObjectHandle(t.name.c_str());
    if(handle!=-1)
    {
        t.base_handle = handle;
        float pos[3], ori[3];
        if(simGetObjectPosition(handle, -1, pos) != -1 &&
                simGetObjectOrientation(handle, -1, ori) != -1)
        {
            t.origin_pos.assign(std::begin(pos), std::end(pos));
            t.origin_ori.assign(std::begin(ori), std::end(ori));
        }
        else
        {
            error_scq.push_back("Could not retrieve original tool position and orientation.");
            return false;
        }
        std::string tipname = t.name + "_tip";
        int tiphandle = simGetObjectHandle(tipname.c_str());
        if(tiphandle != -1)
        {
            t.tip_handle = tiphandle;
        }
        else {
            std::string msg = "Could not retrieve " + t.name + " tip handle.";
            error_scq.push_back(msg);
            return false;
        }

        if(t.has_joint)
        {
            std::string joint_name = t.name + "_jnt";
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
                    error_scq.push_back("Could not retrieve tool's joint limits.");
                    return false;
                }
            }
            else
            {
                error_scq.push_back("Could not retrieve tool's joint handle.");
                return false;
            }
        }
    }
    else
    {
        std::string msg = "Could not retrieve " + t.name + " base handle.";
        // DISABLED TO NOT SHOW MESSAGE FROM PREVIOUS EE
//        error_scq.push_back(msg);
        return false;
    }

    return true;
}

bool v_repExtSetTool(const int tool_handle, const int conn_handle)
{
    //Flag +sim_handleflag_assembly allows to move the end-effector to desired position
    if(simSetObjectParent(tool_handle+sim_handleflag_assembly,conn_handle,false) == -1)
    {
        error_scq.push_back("Could not attach new tool.");
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
                    error_scq.push_back("Could not move tool joint to its original position.");
                    return false;
                }
            }
        }
        else
        {
            error_scq.push_back("Could not place the tool on the table.");
            return false;
        }
    }
    else
    {
        error_scq.push_back("Could not detach tool.");
        return false;
    }
    return true;
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
            error_scq.push_back("Could not execute tool joint motion.");
            return false;
        }
    }
    else
    {
        error_scq.push_back("Could not retrieve current tool joint position.");
        return false;
    }
}

void v_repExtGetToolPosition(const int joint_handle, double &cur_pos)
{
    float cpos;
    if(simGetJointPosition(joint_handle, &cpos) != -1)
    {
        cur_pos = static_cast<double>(cpos);
    }
}

//////////////////////////////// TRAJECTORIES INTERNAL /////////////////////////////////

void v_repExtGetSurgeryReference(const int robot_handle)
{
    std::vector<double> matrix(12);
    int surg_ref_handle = simGetObjectHandle("SurgeryReference");
    if(robot_handle != -1 && surg_ref_handle != -1)
    {
        simFloat mat_arr[12];
        if(simGetObjectMatrix(surg_ref_handle, robot_handle, mat_arr) != -1)
        {
            matrix.assign(mat_arr, mat_arr+12);
            ptr_surg_traj->SetSurgReferenceFrame(matrix);
        }
        else
        {
            //Called every cycle
            error_mcq.push_back("Could not retrieve surgery reference matrix.");
        }
    }
    else
    {
        //Called every cycle
        error_mcq.push_back("Could not retrieve surgery or the robot base reference handle.");
    }
}

bool v_repExtAddTrajectory(Trajectory *t) {
    int surg_ref = simGetObjectHandle("SurgeryReference");
    if(surg_ref == -1)
    {
        error_scq.push_back("Could not retrieve handle of surgery reference frame.");
        return false;
    }
    int handle = simLoadModel("models/surgery/Trajectories/surgery_trajectory_30.ttm");
    if(handle == -1)
    {
        error_scq.push_back("Could not load surgery trajectory model.");
        return false;
    }
    t->handle = handle;

    //Cast to float
    std::vector<float> p(3), o(4);
    std::copy(t->traj_vec.begin(), t->traj_vec.begin()+3, p.begin());
    std::copy(t->traj_vec.begin()+3, t->traj_vec.end(), o.begin());
    if(simSetObjectPosition(handle, surg_ref, &p[0]) != -1 &&
            simSetObjectQuaternion(handle, surg_ref, &o[0]) == -1)
    {
        error_scq.push_back("Could not set surgery trajectory position and orientation.");
        return false;
    }
    return true;
}

bool v_repExtRemoveTrajectories(const std::vector<int> &traj_handles) {
    for(int traj_handle : traj_handles)
    {
        int sph_h = simGetObjectChild(traj_handle, 0);
        int cyl_h = simGetObjectChild(traj_handle, 1);
        if(sph_h != -1 && cyl_h != -1)
        {
            //Remove child objects (cylinder and sphere)
            if(simRemoveObject(sph_h) == -1 || simRemoveObject(cyl_h) == -1)
            {
                error_scq.push_back("Could not remove the specified surgery trajectory shapes.");
                return false;
            }
            //Remove parent object
            if(simRemoveObject(traj_handle) == -1)
            {
                error_scq.push_back("Could not remove the specified surgery trajectory base object.");
                return false;
            }
        }
        else
        {
            error_scq.push_back("Could not access the specified surgery trajectory shapes to remove.");
            return false;
        }
    }
    return true;
}

bool v_repExtShowTrajectories(const std::vector<int> &show_handles, const std::vector<int> &hide_handles) {
    //deselect all objects
    simRemoveObjectFromSelection(sim_handle_all, 0); //second parameter ignored if first is sim_handle_all

    //hide all trajectories
    for(int hide : hide_handles)
    {
        int prop = simGetModelProperty(hide);
        int not_prop = prop | sim_modelproperty_not_visible;
        if(simSetModelProperty(hide,not_prop) == -1)
        {
            error_scq.push_back("Could not make surgery trajectories invisible");
            return false;
        }
    }

    for(int show : show_handles)
    {
        //show selected trajectory
        int prop = simGetModelProperty(show);
        int not_prop = prop & ~sim_modelproperty_not_visible;
        if(simSetModelProperty(show,not_prop) == -1)
        {
            error_scq.push_back("Could not make surgery trajectories visible");
            return false;
        }
    }

    return true;
}

////////////////////////////////// GENERAL INTERNAL ///////////////////////////////////

void v_repExtErrorDialog(const string &message)
{
    simDisplayDialog("Surgical Robot Control Plugin Error",
                     message.c_str(),
                     sim_dlgstyle_ok,
                     NULL, //no input
                     NULL, //title default colors
                     NULL, //title default colors
                     NULL); //custom uiHandle
}

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

    // Register the new Lua command "simExtSurgRobotControl_functions":
    simRegisterScriptCallbackFunction(strConCat(LUA_INIT_SURGROBOTCONTROL_COMMAND,"@",PLUGIN_NAME),strConCat("void",LUA_INIT_SURGROBOTCONTROL_COMMAND,"(string robotName)"),LUA_INIT_SURGROBOTCONTROL_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_CLEANUP_SURGROBOTCONTROL_COMMAND,"@",PLUGIN_NAME),strConCat("void",LUA_CLEANUP_SURGROBOTCONTROL_COMMAND,"()"),LUA_CLEANUP_SURGROBOTCONTROL_CALLBACK);

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
        bool sceneLoaded=((flags&8)!=0);

        if (sceneLoaded)
        { // React to a scene load here!!
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
    {
    }

    if (message==sim_message_eventcallback_simulationended)
    { // Simulation just ended
    }

    if (message==sim_message_eventcallback_moduleopen)
    {
        if ( (customData==NULL)||(std::string("SurgRobotControl").compare((char*)customData)==0) ) // is the command also meant for this plugin?
        {
            if(plugin_loaded)
            {
                simAddStatusbarMessage("Launched ORB. End Effector plugin communication channels initialized.");
                pd.InitComponents();
                pd.StartComm();
            }
        }
    }

    if (message==sim_message_eventcallback_modulehandle)
    {
        if ( (customData==NULL)||(std::string("SurgRobotControl").compare((char*)customData)==0) ) // is the command also meant for this plugin?
        {
            if(plugin_loaded)
            {
                //Robot Real
                if(ptr_robot->ready)
                {
                    //Read Joints
                    std::vector<double> jpos, jvel;
                    if(_internal_::v_repExtGetJointPositions(ptr_robot->jhandles, jpos))
                        ptr_robot->set_cur_pos(jpos);
                    if(_internal_::v_repExtGetJointVelocities(ptr_robot->jhandles, jvel))
                        ptr_robot->set_cur_vel(jvel);

                    //Set Joints
                    if(!ptr_robot->tar_jpos.empty())
                    {
                        if(_internal_::v_repExtSetJointPositions(ptr_robot->jhandles, ptr_robot->tar_jpos.back(), false))
                            ptr_robot->tar_jpos.pop_back();
                        else
                            ptr_robot->tar_jpos.clear();
                    }

                    //Activate brakes
                    if(!ptr_robot->act_brakes.empty())
                    {
                        _internal_::v_repExtActivateBrakes(ptr_robot->jhandles, ptr_robot->act_brakes.only_back());
                    }
                }
                //Robot Hollow
                if(ptr_robot_hlw->ready)
                {
                    //Read Joints
                    std::vector<double> jhpos, jhvel;
                    if(_internal_::v_repExtGetJointPositions(ptr_robot_hlw->jhandles, jhpos))
                        ptr_robot_hlw->set_cur_pos(jhpos);
                    if(_internal_::v_repExtGetJointVelocities(ptr_robot_hlw->jhandles, jhvel))
                        ptr_robot_hlw->set_cur_vel(jhvel);

                    //Set Joints
                    if(!ptr_robot_hlw->tar_jpos.empty())
                    {
                        if(_internal_::v_repExtSetJointPositions(ptr_robot_hlw->jhandles, ptr_robot_hlw->tar_jpos.back(), true))
                        {
                            ptr_robot_hlw->tar_jpos.pop_back();
                        }
                        else
                        {
                            ptr_robot_hlw->tar_jpos.clear();
                        }
                        //Does not handle IK group and moves manipSphere to the position of the robot's tip
                        if(ptr_end_effector->current_tool != NONE) {
                            simFloat matrix[12];
                            simGetObjectMatrix(ptr_end_effector->tools_vector.at(ptr_end_effector->current_tool).tip_handle, ptr_robot->conn_handle, matrix);
                            simSetObjectMatrix(ptr_ik->manipSphere_handle, ptr_robot_hlw->conn_handle, matrix);

                            simSetObjectMatrix(ptr_robot_hlw->tip_handle, ptr_robot_hlw->conn_handle, matrix);
                        }
                        else {
                            simFloat matrix[12];
                            simGetObjectMatrix(ptr_robot_hlw->conn_handle, -1, matrix);
                            simSetObjectMatrix(ptr_ik->manipSphere_handle, -1, matrix);

                            simSetObjectMatrix(ptr_robot_hlw->tip_handle, -1, matrix);
                        }
                    }
                    else
                    {
                        //Explicit handling of IK group enabled
                        if(ptr_ik->ready)
                        {
                            simHandleIkGroup(ptr_ik->ik_group_handle);
                        }
                    }

                    //Set visibility
                    if(!ptr_robot_hlw->set_vis.empty())
                    {
                        _internal_::v_repExtSetHollowVisibility(ptr_robot_hlw->handle, ptr_robot_hlw->set_vis.only_back());
                    }
                }

                //EndEffectors
                if(ptr_end_effector->ready)
                {
                    Tool &t = ptr_end_effector->tools_vector.at(ptr_end_effector->current_tool);
                    //Read Tool Position
                    if(t.has_joint)
                    {
                        double tpos;
                        _internal_::v_repExtGetToolPosition(t.joint_handle, tpos);
                        ptr_end_effector->tool_pos = tpos;
                    }

                    //Move Tool
                    if(!ptr_end_effector->move_tool.empty() && t.has_joint)
                    {
                        if(_internal_::v_repExtMoveTool(t, ptr_end_effector->move_tool.back()))
                            ptr_end_effector->move_tool.pop_back();
                        else
                            ptr_end_effector->move_tool.clear();
                    }

                    //Set Tool
                    if(!ptr_end_effector->set_tool.empty())
                    {
                        if(ptr_end_effector->current_tool != NONE)
                        {
                            if(_internal_::v_repExtRemoveTool(t))
                            {
                                ptr_end_effector->current_tool = NONE;
                                //Move the manipSphere back to robot's tip
                                simFloat matrix[12];
                                simGetObjectMatrix(ptr_robot_hlw->conn_handle, -1, matrix);
                                simSetObjectMatrix(ptr_ik->manipSphere_handle, -1, matrix);

                                simSetObjectMatrix(ptr_robot_hlw->tip_handle, -1, matrix);
                            }
                        }
                        int tool = ptr_end_effector->set_tool.back();
                        if(ptr_end_effector->current_tool == NONE && tool != NONE)
                        {
                            if(_internal_::v_repExtSetTool(ptr_end_effector->tools_vector.at(tool).base_handle, ptr_robot->conn_handle))
                            {
                                ptr_end_effector->current_tool = tool;
                                //Move the manipSphere to tool's tip
                                simFloat matrix[12];
                                simGetObjectMatrix(ptr_end_effector->tools_vector.at(tool).tip_handle, ptr_robot->conn_handle, matrix);
                                simSetObjectMatrix(ptr_ik->manipSphere_handle, ptr_robot_hlw->conn_handle, matrix);

                                simSetObjectMatrix(ptr_robot_hlw->tip_handle, ptr_robot_hlw->conn_handle, matrix);
                            }
                        }
                        ptr_end_effector->set_tool.clear();
                    }
                }

                //Trajectories
                {
                    //Get Surgery Reference - REFERENCE TRANSFORMATION CALCULATED EVERY CYCLE
//                    _internal_::v_repExtGetSurgeryReference(ptr_robot->handle);

                    //Show trajectory
                    if(!ptr_surg_traj->show_traj.empty())
                    {
                        std::vector<int> show_handles, hide_handles;
                        if(ptr_surg_traj->show_traj.size() != ptr_surg_traj->surgery_plan.size()) //show last
                        {
                            for(Trajectory &t : ptr_surg_traj->surgery_plan)
                            {
                                if(t.id != ptr_surg_traj->show_traj.back() && t.visible)
                                {
                                    hide_handles.push_back(t.handle);
                                    t.visible = false;
                                }
                                if(t.id == ptr_surg_traj->show_traj.back() && !t.visible)
                                {
                                    show_handles.push_back(t.handle);
                                    t.visible = true;
                                }
                            }
                            _internal_::v_repExtShowTrajectories(show_handles, hide_handles);

                        }
                        ptr_surg_traj->show_traj.clear();
                    }
                    //Show all
                    if(!ptr_surg_traj->show_all_traj.empty())
                    {
                        std::vector<int> show_handles, hide_handles;
                        for(size_t i=0; i<ptr_surg_traj->surgery_plan.size(); ++i) {
                            if(!ptr_surg_traj->surgery_plan[i].visible)
                            {
                                show_handles.push_back(ptr_surg_traj->surgery_plan[i].handle);
                                ptr_surg_traj->surgery_plan[i].visible = true;
                            }
                        }
                        _internal_::v_repExtShowTrajectories(show_handles, hide_handles);
                        ptr_surg_traj->show_all_traj.clear();
                    }

                    //Add new trajectories
                    if(ptr_surg_traj->PlanSize() > 0)
                    {
                        Trajectory &t = ptr_surg_traj->PlanBack();
                        if(_internal_::v_repExtAddTrajectory(&t))
                        {
                            ptr_surg_traj->surgery_plan.push_back(t);
                        }
                        ptr_surg_traj->PlanClear(); //WHY
                    }

                    //Remove trajectory
                    if(!ptr_surg_traj->rm_traj.empty())
                    {
                        //check if id exists
                        auto it = std::find_if(ptr_surg_traj->surgery_plan.begin(), ptr_surg_traj->surgery_plan.end(), [&](const Trajectory& t){
                            return (t.id == ptr_surg_traj->rm_traj.back());
                        });
                        if(it!=ptr_surg_traj->surgery_plan.end())
                        {
                            std::vector<int> traj_handle(1, it->handle);
                            if(_internal_::v_repExtRemoveTrajectories(traj_handle))
                            {
                                ptr_surg_traj->surgery_plan.erase(it);
                            }
                        }
                        ptr_surg_traj->rm_traj.clear();
                    }
                    //Remove all
                    if(!ptr_surg_traj->rm_all_traj.empty())
                    {
                        std::vector<int> remove_handles;
                        for(size_t i=0; i<ptr_surg_traj->surgery_plan.size(); ++i)
                        {
                            remove_handles.push_back(ptr_surg_traj->surgery_plan[i].handle);
                        }
                        if(_internal_::v_repExtRemoveTrajectories(remove_handles))
                        {
                            ptr_surg_traj->surgery_plan.clear();
                        }

                        ptr_surg_traj->rm_all_traj.clear();
                    }
                }

                //Display Errors
                if(error_scq.size() > 0)
                {
                    _internal_::v_repExtErrorDialog(error_scq.back());
                    error_scq.pop_back();
                }

                //Display Errors
                if(error_mcq.size() > 0)
                {
                    simAddStatusbarMessage(error_mcq.back().c_str());
                    error_mcq.pop_back();
                }
            }
        }
    }

    if (message==sim_message_eventcallback_moduleclose)
    {
        if ( (customData==NULL)||(std::string("SurgRobotControl").compare((char*)customData)==0) ) // is the command also meant for this plugin?
        {
            //Initialization
            if(plugin_loaded)
            {
                if(ptr_robot->ready) {
                    ptr_robot->tar_jpos.clear();

                    ptr_robot->act_brakes.clear();
                }

                if(ptr_robot_hlw->ready)
                {
                    ptr_robot_hlw->tar_jpos.clear();

                    ptr_robot_hlw->act_brakes.clear();
                    ptr_robot_hlw->set_vis.clear();
                }

                if(ptr_end_effector->ready) {
                    //Remove attached tools
                    Tool &t = ptr_end_effector->tools_vector.at(ptr_end_effector->current_tool);

                    if(ptr_end_effector->current_tool != NONE)
                    {
                        if(_internal_::v_repExtRemoveTool(t))
                        {
                            ptr_end_effector->current_tool = NONE;
                            //Move the manipSphere back to robot's tip
                            simFloat matrix[12];
                            simGetObjectMatrix(ptr_robot_hlw->conn_handle, -1, matrix);
                            simSetObjectMatrix(ptr_ik->manipSphere_handle, -1, matrix);

                            simSetObjectMatrix(ptr_robot_hlw->tip_handle, -1, matrix);
                        }
                    }

                    ptr_end_effector->set_tool.clear(); //tool types to set
                    ptr_end_effector->remove_tool.clear();
                    ptr_end_effector->move_tool.clear();
                }

                //Clear the surgery plan and all trajectories
                std::vector<int> remove_handles;
                //Get trajectory handles in simulation
                for(size_t i=0; i<ptr_surg_traj->surgery_plan.size(); ++i)
                {
                    remove_handles.push_back(ptr_surg_traj->surgery_plan[i].handle);
                }
                //Remove objects associated with handles
                if(_internal_::v_repExtRemoveTrajectories(remove_handles))
                {
                    //Clears all queues and the surgery plan
                    ptr_surg_traj->Cleanup();
                }

                simAddStatusbarMessage("Closed ORB. End Effector plugin communication channels stopped.");
                pd.StopComm();
                pd.CleanupComponents();
            }
        }

        //Clear string queues
        error_mcq.clear();
        error_scq.clear();
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
