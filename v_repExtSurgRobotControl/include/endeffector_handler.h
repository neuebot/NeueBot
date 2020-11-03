/*****************************************************************************
  File: endeffector_handler.h

  Version: 1.0
  Author: Carlos Faria <carlosfaria89@gmail.com>
  Maintainer: Carlos Faria <carlosfaria89@gmail.com>

  Copyright (C) 2017 Carlos André de Oliveira Faria. All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef V_REPEXT_SURGROBOTCONTROL_ENDEFFECTOR_HANDLER
#define V_REPEXT_SURGROBOTCONTROL_ENDEFFECTOR_HANDLER

#include "basic_concurrent_queue.h"

#include <atomic>
#include <map>
#include <string>
#include <vector>

enum TOOL_TYPES {
    NONE,
    EMPTY,
    CAMERA,
    PROBE,
    TREPAN,
    EEBASE,
    BASEBUTTONS,
    BASEPROBE,
    BASETREPAN,
    POINTER
};

const std::map<TOOL_TYPES, std::string> tools_map {
    {NONE,   ""         },
    {EMPTY,  "eebuttons"  },
    {CAMERA, "eecamera" },
    {PROBE,  "eebuttonsprobe"  },
    {TREPAN, "eebuttonstrepan" },
    {EEBASE, "eebase" },
    {BASEBUTTONS, "eebasebuttons" },
    {BASEPROBE, "eebaseprobe" },
    {BASETREPAN, "eebasetrepan" },
    {POINTER, "eepointer"}
};

//Base Tool struct
struct Tool {
    int base_handle;
    std::string name;
    TOOL_TYPES type;
    bool has_joint;
    std::vector<float> origin_pos;
    std::vector<float> origin_ori;
    int joint_handle;
    int tip_handle;
    std::pair<float, float> limits;
};

class EndEffectorHandler {
public:
    //Populates Tool vector with types and required name
    EndEffectorHandler();

public:
    bool ready;
    std::vector<Tool> tools_vector;

public:
    BasicConcurrentQueue<int> set_tool; //tool types to set
    BasicConcurrentQueue<int> remove_tool;
    BasicConcurrentQueue<double> move_tool;
    std::atomic<double> tool_pos;
    std::atomic<int> current_tool;

};

#endif //V_REPEXT_SURGROBOTCONTROL_ENDEFFECTOR_HANDLER

