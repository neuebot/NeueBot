/*****************************************************************************
  File: registration-component.hpp

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

#ifndef OROCOS_REGISTRATION_COMPONENT_HPP
#define OROCOS_REGISTRATION_COMPONENT_HPP

#include "registration-errorcode.hpp"
#include "registration_types.hpp"

#include "marshaller_calibration.h"

#include <rtt/RTT.hpp>

class Registration : public RTT::TaskContext {
public:
    Registration(const std::string &component_name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    KDL::MarshallerCalibration m_marshaller;

    std::vector<std::vector<double> > m_work_points; //!< Registration points as specified in the work piece frame
    std::vector<std::vector<double> > m_regs_points; //!< Registration points as read by the registration aparatus
    KDL::Frame m_registration_matrix;
    bool m_registration_successful;

    double m_error;
    REGErrorCode m_ec; //!< Registration error code object
    double m_tolerance;

private:
    KDL::Frame m_testbox_transf; //!< KDL::Frame with the transformation from the Interlocks to the Surg Ref Frame

private:
    /*!
     *  \brief rigid_lms_registration Computes the registration matrix between the two paramter sets of points.
     *  Caller guarantees that the passed point sets are of equal size and of ordered content.
     *  \param Pa Set of points in the work reference frame.
     *  \param Pb Set of points in the registration apparatus frame.
     *  \param res Resulting KDL::Frame with the registration transformation matrix.
     *  \return double value with the error of the cumulative process. This error is only returned if a redudant
     *  number of points is passed to register (>3).
     */
    double rigid_lms_registration(std::vector<std::vector<double> > &Pa, std::vector<std::vector<double> > &Pb, KDL::Frame &res);

protected: //Operations
    /*!
     *  \brief LoadWorkPiecePoints Loads work piece points in the work piece reference frame.
     *  \param wpoints Work piece points ordered.
     */
    void LoadWorkPiecePoints(const RTT::vector_vector_double &wpoints);

    /*!
     *  \brief LoadRegistrationPoints Loads work piece points registered to the registration reference frame.
     *  \param rpoints Work piece registered points.
     */
    void LoadRegistrationPoints(const RTT::vector_vector_double &rpoints);

    /*!
     *  \brief ComputeRegistrationMatrix Computes the best fitting transformation matrix that expresses the translation
     *  and rotation of the point sets from the work piece poins to the analogous and ordered points in the registration
     *  reference frame.
     *
     *  The component performs a set of checks to guarantee that the point sets are loaded and prepared for registration.
     *  \return Cumulative error of registration.
     */
    bool ComputeRegistrationMatrix(KDL::Frame &transformation, double &error);

    /*!
     * \brief SaveInterlocksRegistrationMatrix Saves the registration matrix from the Interlocks touch method.
     * The method receives the base tool transformation, which is then multiplied by the transformation from the
     * interlocks to the surgery reference (box transformation).
     * The box transformation is loaded from the parameters at the moment.
     * For the interlock registration method, the EEBase tool should be attached.
     * \param base_tool Transformation from the base of the robot reference frame to the tool reference.
     * \param transformation Output transformation from the base of the robot to the surgery reference frame.
     * \return true if successful
     */
    bool SaveInterlockRegistrationMatrix(const KDL::Frame &base_tool, KDL::Frame &transformation);

    /*!
     * \brief LoadCalibrationFile
     * \param filename
     * \param base_work
     * \param error
     * \return
     */
    bool LoadCalibrationFile(const std::string &filename, KDL::Frame &base_work, double &error);

    /*!
     * \brief SaveCalibrationFile
     * \param filename
     */
    bool SaveCalibrationFile(const std::string &filename);

    /*!
     * \brief GetRegistrationMatrix Returns the current registration matrix after sucessful registration.
     * \param reg Registration matrix returned by parameter.
     * \return True if registration successful
     */
    bool GetRegistrationMatrix(KDL::Frame &reg);

protected:
    std::vector<double> m_testbox_transf_v;

protected: //Operation Callers
    RTT::OperationCaller<void(KDL::Frame &)> CallSetWorkFrame;

protected: //Ports
    RTT::OutputPort< RTT::error_msg > outport_error_msg;

};

#endif //OROCOS_REGISTRATION_COMPONENT_HPP
