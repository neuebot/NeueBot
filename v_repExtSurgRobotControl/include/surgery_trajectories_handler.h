/*****************************************************************************
  File: surgery_trajectories_handler.h

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

#ifndef V_REPEXT_SURGTRAJECTORIES_HANDLER
#define V_REPEXT_SURGTRAJECTORIES_HANDLER

#include "basic_concurrent_queue.h"

#include <vector>

struct Trajectory {
    int id;
    int handle;
    std::vector<double> traj_vec;
    bool visible;

    Trajectory(const int id_, const std::vector<double> &traj_vec_) :
        id(id_),
        handle(-1),
        traj_vec(traj_vec_),
        visible(true)
    {}
};

class SurgeryTrajectoriesHandler {
public:
    SurgeryTrajectoriesHandler();

    bool ToAddTrajectory(const int id, const std::vector<double> &traj_vec);
    bool ToRemoveTrajectory(const int id);
    void ToRemoveAll();
    bool ToShowTrajectory(const int id);
    void ToShowAll();

    //Surgery Frame Interface
    std::vector<double> GetSurgReferenceFrame();
    void SetSurgReferenceFrame(const std::vector<double> &srf);

    //Trajectories to add Inteface
    int PlanSize();
    Trajectory& PlanBack();
    void PlanPopBack();
    void PlanClear();

    void Cleanup();

    int IndexOfID(const int id);

public:
    std::vector<Trajectory> surgery_plan;

public:
    BasicConcurrentQueue<int> add_traj;
    BasicConcurrentQueue<int> rm_traj;
    BasicConcurrentQueue<bool> rm_all_traj;
    BasicConcurrentQueue<int> show_traj;
    BasicConcurrentQueue<bool> show_all_traj;

private:
    std::vector<double> m_frame_matrix;
    std::mutex m_frame_mutex;
    std::deque<Trajectory> m_add_to_plan;
    std::mutex m_plan_mutex;
};

#endif // V_REPEXT_SURGTRAJECTORIES_HANDLER
