/*****************************************************************************
  File: plugin_deployer.h

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

#ifndef V_REPEXT_SURGROBOTCONTROL_PLUGIN_DEPLOYER
#define V_REPEXT_SURGROBOTCONTROL_PLUGIN_DEPLOYER

#include "robot_component.h"
#include "endeffector_component.h"
#include "surgery_trajectories_component.h"

#include <future>
#include <vector>
#include <string>

class PluginDeployer {
public:
    PluginDeployer(const std::string &robot_comp, const std::string &endeffector_comp,
                   const std::string &trajectories_comp);
    ~PluginDeployer();

    //Runs on VREP start
    void Init();
    //Runs on scene open
    void InitComponents();

    void StartComm();
    void StopComm();

    //Runs on scene close
    void CleanupComponents();
    //Runs on VREP shutdown
    void Cleanup();

private:
    std::string m_robot_comp;
    std::string m_endeffector_comp;
    std::string m_trajectories_comp;
    std::shared_ptr<RobotComponent> m_rc;
    std::shared_ptr<EndEffectorComponent> m_eec;
    std::shared_ptr<SurgeryTrajectoriesComponent> m_stc;

    std::future<void> m_thr;
};

#endif // V_REPEXT_SURGROBOTCONTROL_PLUGIN_DEPLOYER
