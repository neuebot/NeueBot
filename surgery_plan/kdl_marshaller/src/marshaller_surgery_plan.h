
/*****************************************************************************
  File: marshaller_surgery_plan.h

  Version: 1.0
  Author: Carlos Faria <carlosfaria89@gmail.com>
  Maintainer: Carlos Faria <carlosfaria89@gmail.com>

  Copyright (C) 2018 Carlos André de Oliveira Faria. All rights reserved.

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

#ifndef MARSHALLER_SURGERY_PLAN_H
#define MARSHALLER_SURGERY_PLAN_H

#include "kdl_marshaller.h"

namespace KDL {

/*!
 * \brief The MarshallerSurgeryPlan class handles surgery plan files marshalling process.
 * The file is setup to store the trajectories ID, entry and target points.
 *
 * The class offers methods to store and retrieve this values.
 *
 * TODO: Use a struct/class to define trajectories, and is known to the component.
 * This should include the id, entry and target field.
 *
 * Check input for correct size
 *
 */
class MarshallerSurgeryPlan : public Marshaller {

public:
    MarshallerSurgeryPlan();

     void LoadSurgeryPlanFile(const std::string &filename);
     void SaveSurgeryPlanFile(const std::string &filename);

     void SetTrajectories(const std::vector<int> &unique_id, const std::vector<std::vector<double> > &target_points,
                          const std::vector<std::vector<double> > &entry_points);
     void GetTrajectories(std::vector<int> &unique_id, std::vector<std::vector<double> > &target_points,
                          std::vector<std::vector<double> > &entry_points);

private:
    std::vector<int> m_id;
    std::vector<std::vector<double> > m_target;
    std::vector<std::vector<double> > m_entry;



};

}

#endif // MARSHALLER_SURGERY_PLAN_H
