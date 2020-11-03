#include "tool_handler.h"

#include "../v_repExtEndEffector.h"

#include <algorithm>
#include <iostream>

using namespace std;

ToolHandler::ToolHandler()
{
    //Set initial tool
    m_current_tool = 0;

    //Populate tools
    for(auto it = tools_map.begin(); it != tools_map.end(); it++) {
        Tool t;
        t.type = it->first;
        t.name = it->second;
        if(t.type == PROBE || t.type == TREPAN)
        {
            t.has_joint = true;
        }
        else {
            t.has_joint = false;
        }
        m_tools.push_back(t);
    }
}

bool ToolHandler::InitTools()
{
    //Get Robot Connection handle
    v_repExtGetRobotConnectionInfo(m_conn_handle);
    //Load each tool from the vector (NONE not included)
    for(auto it = m_tools.begin()+1; it != m_tools.end(); it++) {
        if(!v_repExtLoadTool(*it))
        {
            cerr << "Could not load one of the end-effectors check what is going on" << endl;
            return false;
        }
    }
    return true;
}

void ToolHandler::SetTool(const TOOL_TYPES tt)
{
    //For performance increase consider m_tools[m_current_tool]
    if(m_current_tool != NONE)
    {
        if(v_repExtRemoveTool(m_tools[m_current_tool]))
        {
            m_current_tool = NONE;
        }
    }
    if(m_current_tool == NONE)
    {
        auto it = std::find_if(m_tools.begin(), m_tools.end(), [tt](const Tool &t){return t.type == tt;});
        if(it != m_tools.end())
        {
            if(v_repExtSetTool(it->base_handle, m_conn_handle))
            {
                m_current_tool = it->type;
            }
        }
    }
}

void ToolHandler::MoveTool(const double dist)
{
    //Check if current tool has joint
    if(m_tools[m_current_tool].has_joint)
    {
        v_repExtMoveTool(m_tools[m_current_tool], dist);
    }
}

void ToolHandler::GetToolPos(double &cur_pos)
{
    double pos = 0.0;
    if(m_tools[m_current_tool].has_joint)
    {
        v_repExtGetToolPos(m_tools[m_current_tool].joint_handle, pos);
        cur_pos = pos;
    }
}


