
/*****************************************************************************
  File: kdl-marshaller.h

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

#ifndef KDL_MARSHALLER_H
#define KDL_MARSHALLER_H

#include <kdl/frames.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace KDL {

/*!
 * \brief The Marshaller class handles KDL type variables and parses them
 * to and from JSON format.
 *
 */
class Marshaller {

public:
    Marshaller();

    void LoadFile(const std::string &file_name);
    void SaveFile(const std::string &file_name);

protected:
    std::vector<double> get_std_vector(boost::property_tree::ptree &std_vector_json);
    KDL::Vector get_kdl_vector(boost::property_tree::ptree &kdl_vector_json);
    KDL::Rotation get_kdl_rotation(boost::property_tree::ptree &kdl_rotation_json);
    KDL::Frame get_kdl_frame(boost::property_tree::ptree &kdl_frame_json);

protected:
    boost::property_tree::ptree set_std_vector(std::vector<double> &vector);
    boost::property_tree::ptree set_kdl_vector(KDL::Vector &vector);
    boost::property_tree::ptree set_kdl_rotation(KDL::Rotation &rotation);
    boost::property_tree::ptree set_kdl_frame(KDL::Frame &frame);

protected:
    boost::property_tree::ptree m_root;
};

}

#endif // KDL_MARSHALLER_H
