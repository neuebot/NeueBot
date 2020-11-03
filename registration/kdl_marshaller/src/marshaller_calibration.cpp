#include "marshaller_calibration.h"

using namespace boost::property_tree;
using namespace std;

KDL::MarshallerCalibration::MarshallerCalibration() :
    Marshaller ()
{

}

void KDL::MarshallerCalibration::SetCalibrationParameters(const std::vector<std::vector<double> > &wpoints, const std::vector<std::vector<double> > &rpoints,
                                                          const KDL::Frame &transformation, double error)
{
    m_workpiece_points = wpoints;
    m_registration_points = rpoints;
    m_transformation = transformation;
    m_error = error;
}

void KDL::MarshallerCalibration::GetCalibrationParameters(std::vector<std::vector<double> > &wpoints, std::vector<std::vector<double> > &rpoints,
                                                          KDL::Frame &transformation, double &error)
{
    wpoints = m_workpiece_points;
    rpoints = m_registration_points;
    transformation = m_transformation;
    error = m_error;
}

void KDL::MarshallerCalibration::LoadCalibrationFile(const string &filename)
{
    m_workpiece_points.clear();
    m_registration_points.clear();

    //Load file properties to base class m_root variable
    LoadFile(filename);

    //Get Workpiece points
    for(ptree::value_type &pt : m_root.get_child("wpoints"))
    {
        m_workpiece_points.push_back(get_std_vector(pt.second));
    }

    //Get Workpiece points
    for(ptree::value_type &pt : m_root.get_child("rpoints"))
    {
        m_registration_points.push_back(get_std_vector(pt.second));
    }

    //Get Transformation
    m_transformation = get_kdl_frame(m_root.get_child("transformation"));

    //Get Transformation error
    m_error = m_root.get<double>("error");
}

void KDL::MarshallerCalibration::SaveCalibrationFile(const std::string &filename)
{
    //Clear m_root
    m_root.clear();

    //Set Workpiece points
    ptree wpoints;
    for(std::vector<double> &vec : m_workpiece_points)
    {
        wpoints.push_back(std::make_pair("",set_std_vector(vec)));
    }
    m_root.add_child("wpoints", wpoints);

    //Set Workpiece points
    ptree rpoints;
    for(std::vector<double> &vec : m_registration_points)
    {
        rpoints.push_back(std::make_pair("",set_std_vector(vec)));
    }
    m_root.add_child("rpoints", rpoints);

    //Set Transformation
    m_root.add_child("transformation", set_kdl_frame(m_transformation));

    //Set Transformation error
    m_root.put("error", m_error);

    //Save file with properties of base class m_root
    SaveFile(filename);
}


