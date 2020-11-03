/*****************************************************************************
  File: kuka_lbr_fri-errorcode.hpp

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

#ifndef OROCOS_KUKA_LBR_FRI_ERRORCODE_HPP
#define OROCOS_KUKA_LBR_FRI_ERRORCODE_HPP

#include <utility>
#include <map>
#include <string>

// [x000 - x199] Info
// [x200 - x399] Warning
// [x300 - x599] Error
// [x600 - x999] Critical
enum ERRORCODE {
//    NOTREADEE = 1201,
//    NOTATTACH = 1202,
//    NOTREADREF = 1203,
//    NOTMOVEEE = 1204
};

class KLFErrorCode {

public:
    std::map <ERRORCODE, std::string > dict = {
//        {NOTREADEE, "[VF] Could not read the current end-effector type."},
//        {NOTATTACH, "[VF] Could not attach the end-effector."},
//        {NOTREADREF, "[VF] Could read the surgery reference frame."},
//        {NOTMOVEEE, "[VF] Could move the tool in the end-effector."}
    };
};

#endif //OROCOS_KUKA_LBR_FRI_ERRORCODE_HPP
