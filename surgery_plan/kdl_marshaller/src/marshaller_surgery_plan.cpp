#include "marshaller_surgery_plan.h"

using namespace boost::property_tree;
using namespace std;

KDL::MarshallerSurgeryPlan::MarshallerSurgeryPlan() :
    Marshaller ()
{

}

void KDL::MarshallerSurgeryPlan::LoadSurgeryPlanFile(const string &filename)
{
    m_id.clear();
    m_target.clear();
    m_entry.clear();

    //Load file properties to base class m_root variable
    LoadFile(filename);

    //Get Workpiece points
    for(ptree::value_type &traj : m_root)
    {
        //Get ID
        m_id.push_back(traj.second.get<int>("id",0));

        //Get Target positions
        m_target.push_back(get_std_vector(traj.second.get_child("target")));

        //Get Entry positions
        m_entry.push_back(get_std_vector(traj.second.get_child("entry")));
    }
}

void KDL::MarshallerSurgeryPlan::SaveSurgeryPlanFile(const string &filename)
{
    //Clear m_root
    m_root.clear();

    //Make sure
    for(size_t traj = 0; traj < m_id.size(); ++traj) {
        //Create trajectory ptree
        ptree trajectory;

        //Store ID
        trajectory.put("id", m_id[traj]);

        //Store Target
        trajectory.add_child("target", set_std_vector(m_target[traj]));

        //Store Entry
        trajectory.add_child("entry", set_std_vector(m_entry[traj]));

        m_root.add_child("trajectory", trajectory);
    }

    //Save file with properties of base class m_root
    SaveFile(filename);
}

void KDL::MarshallerSurgeryPlan::SetTrajectories(const std::vector<int> &unique_id, const std::vector<std::vector<double> > &target_points,
                                                 const std::vector<std::vector<double> > &entry_points)
{
    m_id = unique_id;

    m_target = target_points;
    m_entry = entry_points;
}

void KDL::MarshallerSurgeryPlan::GetTrajectories(std::vector<int> &unique_id, std::vector<std::vector<double> > &target_points,
                                                 std::vector<std::vector<double> > &entry_points)
{
    unique_id = m_id;

    target_points = m_target;
    entry_points = m_entry;
}
