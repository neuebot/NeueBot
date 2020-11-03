/*****************************************************************************
  File: robot_handler.h

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

#ifndef V_REPEXT_SURGROBOTCONTROL_ROBOT_HANDLER
#define V_REPEXT_SURGROBOTCONTROL_ROBOT_HANDLER

#include "basic_concurrent_queue.h"
#include "vector_concurrent_queue.h"

#include <map>
#include <mutex>
#include <string>
#include <vector>

class RobotHandler {
public:
    RobotHandler(const std::string &rname);

public:
    int handle;
    std::string name;
    int dofs;
    std::vector<int> jhandles;
    int tip_handle;
    int conn_handle;
    std::vector< std::pair<double, double> > jplim;
    std::vector<double> jvlim;
    bool ready;

public:
    VectorConcurrentQueue<double> tar_jpos;

    BasicConcurrentQueue<bool> act_brakes;
    BasicConcurrentQueue<bool> set_vis;

public:
    std::vector<double> get_cur_pos();
    void set_cur_pos(const std::vector<double> &pos);
    std::vector<double> get_cur_vel();
    void set_cur_vel(const std::vector<double> &vel);

private:
    std::vector<double> cur_pos;
    std::vector<double> cur_vel;
    std::mutex mutex_cpos;
    std::mutex mutex_cvel;

};

#endif //V_REPEXT_SURGROBOTCONTROL_ROBOT_HANDLER

