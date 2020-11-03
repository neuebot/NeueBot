/*****************************************************************************
  File: surgery_trajectories_component.h

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

#ifndef V_REPEXT_SURGTRAJECTORIES_COMPONENT
#define V_REPEXT_SURGTRAJECTORIES_COMPONENT

#include <kdl/frames.hpp>
#include <rtt/RTT.hpp>

class SurgeryTrajectoriesComponent : public RTT::TaskContext {
public:
    SurgeryTrajectoriesComponent(const std::string &name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    double m_period;
    std::string m_name;
    unsigned int m_dofs;

protected: //Operations
    void GetSurgeryReference(std::vector<double> &surg_ref);

    void AddTrajectory(const int id, const std::vector<double> &target, const std::vector<double> &entry);
    void RemoveTrajectories();

    void ShowTrajectory(const int id);
    void ShowTrajectories();

};

#endif // V_REPEXT_SURGTRAJECTORIES_COMPONENT
