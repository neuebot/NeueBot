#include "surgery_trajectories_handler.h"

#include <algorithm>

using namespace std;

SurgeryTrajectoriesHandler::SurgeryTrajectoriesHandler() {

}

bool SurgeryTrajectoriesHandler::ToAddTrajectory(const int id, const std::vector<double> &traj_vec)
{
    //check if id exists
    auto it = std::find_if(surgery_plan.begin(), surgery_plan.end(), [&](const Trajectory& t){
        return (t.id == id);
    });

    if(it == surgery_plan.end()) {
        Trajectory t(id, traj_vec);
        std::lock_guard<std::mutex> lk(m_plan_mutex);
        m_add_to_plan.push_front(t);
        return true;
    }
    return false; //ID already exists
}

bool SurgeryTrajectoriesHandler::ToRemoveTrajectory(const int id)
{
    //check if id exists
    auto it = std::find_if(surgery_plan.begin(), surgery_plan.end(), [&](const Trajectory& t){
        return (t.id == id);
    });

    if(it != surgery_plan.end()) {
        rm_traj.push_front(it->id);
        return true;
    }
    return false; //ID does not exist
}

bool SurgeryTrajectoriesHandler::ToShowTrajectory(const int id)
{
    //check if id exists
    auto it = std::find_if(surgery_plan.begin(), surgery_plan.end(), [&](const Trajectory& t){
        return (t.id == id);
    });

    if(it != surgery_plan.end()) {
        show_traj.push_front(it->id);
        return true;
    }
    return false; //ID does not exist
}

void SurgeryTrajectoriesHandler::ToRemoveAll()
{
    rm_all_traj.push_front(true);
}

void SurgeryTrajectoriesHandler::ToShowAll()
{
    show_all_traj.push_front(true);
}

std::vector<double> SurgeryTrajectoriesHandler::GetSurgReferenceFrame()
{
    std::lock_guard<std::mutex> lk(m_frame_mutex);
    return m_frame_matrix;
}

void SurgeryTrajectoriesHandler::SetSurgReferenceFrame(const std::vector<double> &srf)
{
    std::lock_guard<std::mutex> lk(m_frame_mutex);
    m_frame_matrix = srf;
}

int SurgeryTrajectoriesHandler::PlanSize() {
    std::lock_guard<std::mutex> lk(m_plan_mutex);
    return m_add_to_plan.size();
}

Trajectory& SurgeryTrajectoriesHandler::PlanBack()
{
    std::lock_guard<std::mutex> lk(m_plan_mutex);
    return m_add_to_plan.back();
}

void SurgeryTrajectoriesHandler::PlanPopBack()
{
    std::lock_guard<std::mutex> lk(m_plan_mutex);
    m_add_to_plan.pop_back();
}

void SurgeryTrajectoriesHandler::PlanClear() {
    std::lock_guard<std::mutex> lk(m_plan_mutex);
    m_add_to_plan.clear();
}

void SurgeryTrajectoriesHandler::Cleanup() {
    std::lock_guard<std::mutex> lk(m_plan_mutex);

    m_add_to_plan.clear();
    add_traj.clear();
    rm_traj.clear();
    rm_all_traj.clear();
    show_traj.clear();
    show_all_traj.clear();

    surgery_plan.clear();
}
