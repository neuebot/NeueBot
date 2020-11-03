/*****************************************************************************
  File: transformations-component.hpp

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

#ifndef OROCOS_TRANSFORMATIONS_COMPONENT_HPP
#define OROCOS_TRANSFORMATIONS_COMPONENT_HPP

#include "transformations-errorcode.hpp"
#include "transformations_types.hpp"

#include <kdl/frames.hpp>

#include <rtt/RTT.hpp>

/*!
 * \brief The Transformations class handles and stores the transformations between robot base and work frame, and robot flange and tool frame.
 *  The class stores 5 pre-defined end-effectors, with associated transformations (\ref END_EFFECTORS).
 *
 * \note At the moment, the base frame of the robot is assumed to be the Identity Matrix. In a future release the user may adjust this value.
 */
class Transformations : public RTT::TaskContext {

public:
    /*!
     * \brief The REFERENCE_FRAME enum defines the identifiers for the transformation of the relative motion.
     *
     * \note The End-effector transformation (frame) is the reference frame of the tool base respective
     * to the robot base frame. If an end-effector tool can be moved (e.g. trepan or probe), this displacement
     * is not accounted.
     */
    enum REFERENCE_FRAME {
        BASE=0,
        FLANGE,
        END_EFFECTOR,
        WORK,
    };

    /*!
     * \brief The END_EFFECTORS enum defines the identifiers for the end-effectors attached to the robot.
     */
    enum END_EFFECTORS {
        NONE        = 0,
        EMPTY       = 1,
        CAMERA      = 2,
        PROBE       = 3,
        TREPAN      = 4,
        EEBASE      = 5,
        BASEBUTTONS = 6,
        BASEPROBE   = 7,
        BASETREPAN  = 8,
        POINTER     = 9
    };

public:
    Transformations(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void errorHook();

private:
    /*!
     * \brief SetWorkFrame sets the new work frame relative to the robot base frame.
     * This transformation is relative to the robot base frame.
     * \param base_work KDL::Frame of Work frame relative to the Base frame
     */
    void SetWorkFrame(const KDL::Frame base_work);

    /*!
     * \brief SetToolType sets the Tool transformation matrix from the flange to the tool base frame.
     * \param type identifier of the type of end-effector. Each tool transformation is specified in the 'KUKA_TOOLS.xml' Properties file.
     * \return success bool, true if the end-effector id exists.
     */
    bool SetToolType(const int type);
    /*!
     * \brief GetToolType returns the identifier to the attached end-effector.
     * \return type identifier of the attached end-effector, refer to \ref END_EFFECTORS.
     */
    int GetToolType();
    /*!
     * \brief GetToolFrame returns the current frame from flange to tool.
     * \param flange_tool transformation of tool in the flange reference frame
     */
    void GetToolFrame(KDL::Frame &flange_tool);
    /*!
     * \brief GetToolTypeFrame returns the frame from flange to tool of a specified type of end-effector.
     * \param type identifier of the attached end-effector, refer to \ref END_EFFECTORS.
     * \param flange_tool transformation of tool in the flange reference frame
     */
    void GetToolTypeFrame(const int type, KDL::Frame &flange_tool);
    /*!
     * \brief GetReferenceFrame gets the transformation from the robot base frame specified by the identifier.
     * \param reference_id int identifier of the frame to be retrieved, refer to \ref REFERENCE_FRAME.
     * \param frame KDL::Frame reference to the requested frame.
     * \return success, true if the reference frame id exists.
     */
    bool GetReferenceFrame(const int reference_id, KDL::Frame &frame);

private:
    KDL::Frame m_ee_none; //!< KDL::Frame with the Identity Matrix
    KDL::Frame m_ee_empty; //!< KDL::Frame with the transformation matrix to the empty end-effector reference
    KDL::Frame m_ee_probe; //!< KDL::Frame with the transformation matrix to the probe end-effector reference
    KDL::Frame m_ee_trepan; //!< KDL::Frame with the transformation matrix to the trepan end-effector reference
    KDL::Frame m_ee_base; //!< KDL::Frame with the transformation matrix to the interlock base end-effector reference
    KDL::Frame m_ee_basebuttons; //!< KDL::Frame with the transformation matrix to the interlock base buttons end-effector reference
    KDL::Frame m_ee_baseprobe; //!< KDL::Frame with the transformation matrix to the interlock base probe end-effector reference
    KDL::Frame m_ee_basetrepan; //!< KDL::Frame with the transformation matrix to the interlock base trepan end-effector reference
    KDL::Frame m_ee_pointer; //!< KDL::Frame with the transformation matrix to the pointer end-effector reference

    END_EFFECTORS m_end_effector_id; //!< This stores the id of the current attached end-effector (useful for other components)
    TErrorCode m_ec;

private:
    KDL::Frame m_work_transf; //!< Work reference frame relative to the robot base referential.
    KDL::Frame m_flange_transf; //!< Flange frame relative to the Base reference frame, as of last update.
    KDL::Frame m_tool_transf; //!< Tool reference frame relative to the robot flange referential.

    RTT::Attribute<bool> a_simulation;

protected: //Operation Callers
    RTT::OperationCaller<void(KDL::Frame const)> CallSetFlangeToolFrame;
    RTT::OperationCaller<void(KDL::Frame const)> CallSetWorkBaseFrame;

    RTT::OperationCaller<void(const int)> CallAttachEndEffector;
    RTT::OperationCaller<int()> CallGetCurrentEndEffector;

protected: //Properties
    std::vector<double> m_ee_none_v;
    std::vector<double> m_ee_empty_v;
    std::vector<double> m_ee_probe_v;
    std::vector<double> m_ee_trepan_v;
    std::vector<double> m_ee_pointer_v;
    std::vector<double> m_ee_base_v;
    std::vector<double> m_ee_basebuttons_v;
    std::vector<double> m_ee_baseprobe_v;
    std::vector<double> m_ee_basetrepan_v;

    RTT::InputPort< RTT::posture > inport_robot_posture;
    RTT::OutputPort< RTT::error_msg > outport_error_msg;
};

#endif //OROCOS_TRANSFORMATIONS_COMPONENT_HPP
