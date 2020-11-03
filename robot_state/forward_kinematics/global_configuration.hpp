/*****************************************************************************
  File: global_configuration.hpp

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

#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

class GlobalConfiguration {
public:
    GlobalConfiguration(unsigned int rconf) {
        m_arm = ((rconf & 1) == 0 ? 1 : -1);
        m_elbow = ((rconf & 2) == 0 ? 1 : -1);
        m_wrist = ((rconf & 4) == 0 ? 1 : -1);
    }

    int arm() {
        return m_arm;
    }

    int elbow() {
        return m_elbow;
    }

    int wrist() {
        return m_wrist;
    }


private:
    int m_arm;
    int m_elbow;
    int m_wrist;
};

#endif //GLOBAL_CONFIGURATION_H
