
/*****************************************************************************
  File: marshaller_calibration.h

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

#ifndef MARSHALLER_CALIBRATION_H
#define MARSHALLER_CALIBRATION_H

#include "kdl_marshaller.h"

namespace KDL {

/*!
 * \brief The MarshallerCalibration class handles calibration files marshalling process.
 * The file is setup to store the workpiece and registered points, along with the calculated
 * transformation.
 *
 * The class offers methods to store and retrieve this values.
 *
 */
class MarshallerCalibration : public Marshaller {

public:
    MarshallerCalibration();

    void LoadCalibrationFile(const std::string &filename);
    void SaveCalibrationFile(const std::string &filename);

    void SetCalibrationParameters(const std::vector<std::vector<double> > &wpoints, const std::vector<std::vector<double> > &rpoints,
                                  const KDL::Frame &transformation, double error);
    void GetCalibrationParameters(std::vector<std::vector<double> > &wpoints, std::vector<std::vector<double> > &rpoints,
                                  KDL::Frame &transformation, double &error);
private:
    std::vector<std::vector<double> > m_workpiece_points;
    std::vector<std::vector<double> > m_registration_points;

    KDL::Frame m_transformation;
    double m_error;

};

}

#endif // MARSHALLER_CALIBRATION_H
