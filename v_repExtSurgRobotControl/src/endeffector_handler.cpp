#include "endeffector_handler.h"

#include <algorithm>

using namespace std;

EndEffectorHandler::EndEffectorHandler() :
    ready(false),
    current_tool(NONE)
{
    //Populate tools
    for(auto it = tools_map.begin(); it != tools_map.end(); it++) {
        Tool t;
        t.type = it->first;
        t.name = it->second;
        if(t.type == PROBE || t.type == TREPAN ||
                t.type == BASEPROBE || t.type == BASETREPAN)
        {
            t.has_joint = true;
        }
        else {
            t.has_joint = false;
        }
        tools_vector.push_back(t);
    }
}

