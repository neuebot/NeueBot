/*****************************************************************************
  File: endeffector_component.h

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

#ifndef V_REPEXT_SURGROBOTCONTROL_ENDEFFECTOR_COMPONENT
#define V_REPEXT_SURGROBOTCONTROL_ENDEFFECTOR_COMPONENT

#include <rtt/RTT.hpp>

class EndEffectorComponent : public RTT::TaskContext {
public:
    EndEffectorComponent(const std::string &name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    double m_period;

protected: //Ports
    RTT::OutputPort<double> outport_endeffector_position;

protected: //Operations
    int GetCurrentEndEffector();
    void AttachEndEffector(const int ee);
    void MoveEndEffector(const double dist);
};

#endif // V_REPEXT_SURGROBOTCONTROL_ENDEFFECTOR_COMPONENT
