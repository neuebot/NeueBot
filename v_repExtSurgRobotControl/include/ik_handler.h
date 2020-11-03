/*****************************************************************************
  File: ik_handler.h

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

#ifndef V_REPEXT_SURGROBOTCONTROL_IK_HANDLER
#define V_REPEXT_SURGROBOTCONTROL_IK_HANDLER

#include <string>
#include <vector>

class IKHandler {
public:
    IKHandler(const std::string& robot_name);

public:
    std::string ik_group_handle_str;
    std::string manipSphere_str;

    int ik_group_handle;
    int manipSphere_handle;

    bool ready;
};

#endif //V_REPEXT_SURGROBOTCONTROL_ROBOT_HANDLER

