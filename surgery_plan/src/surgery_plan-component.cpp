#include "surgery_plan-component.hpp"
#include "kdl/frames.hpp"
#include <rtt/Component.hpp>
#include <rtt/marsh/Marshalling.hpp>

#include <Eigen/Dense>

using namespace KDL;
using namespace RTT;
using namespace Eigen;
using namespace std;

SurgeryPlan::SurgeryPlan(const string &component_name) :
    TaskContext(component_name, PreOperational),
    //VREP_FRI
    AddTrajectories("AddTrajectories"),
    RemoveTrajectories("RemoveTrajectories"),
    ShowTrajectory("ShowTrajectory"),
    ShowTrajectories("ShowTrajectories"),
    //TASK_SUPERVISOR
    CallCalculateBestTrajectoryApproach("CalculateBestTrajectoryApproach"),
    CallCollectApproachResults("CollectApproachResults")
{
    //PROPERTIES AND ATTRIBUTES
    this->addProperty("Tolerance", m_tolerance);
    this->addProperty("SafeDistance", m_safe_distance);
    this->addProperty("TrajectoryApproachStep", m_trajectory_approach_step);
    this->addProperty("NullspaceStep", m_nullspace_step);
    this->addAttribute("surgery_plan_setup", m_surgery_plan_setup);

    //PORTS
    this->ports()->addPort("outport_error_msg", outport_error_msg);

    //OPERATIONS
    this->addOperation("ConfirmSurgeryPlan", &SurgeryPlan::ConfirmSurgeryPlan , this, OwnThread);
    this->addOperation("RetrieveSurgeryPlan", &SurgeryPlan::RetrieveSurgeryPlan , this, OwnThread);
    this->addOperation("RemoveAllTrajectories", &SurgeryPlan::RemoveAllTrajectories, this, OwnThread);
    this->addOperation("ShowSurgeryTrajectory", &SurgeryPlan::ShowSurgeryTrajectory, this, OwnThread);
    this->addOperation("ShowAllTrajectories", &SurgeryPlan::ShowAllTrajectories, this, OwnThread);
    this->addOperation("LoadSurgeryPlanFile", &SurgeryPlan::LoadSurgeryPlanFile, this, OwnThread);
    this->addOperation("SaveSurgeryPlanFile", &SurgeryPlan::SaveSurgeryPlanFile, this, OwnThread);
    this->addOperation("SelectSurgeryTrajectory", &SurgeryPlan::SelectSurgeryTrajectory, this, OwnThread);
    this->addOperation("VerifyTrajectory", &SurgeryPlan::VerifyTrajectory, this, OwnThread);
    this->addOperation("GetSurgeryTrajectoryParameters", &SurgeryPlan::GetSurgeryTrajectoryParameters, this, OwnThread);

    //OPERATIONCALLERS
    this->requires("VF_SP")->addOperationCaller(AddTrajectories);
    this->requires("VF_SP")->addOperationCaller(RemoveTrajectories);
    this->requires("VF_SP")->addOperationCaller(ShowTrajectory);
    this->requires("VF_SP")->addOperationCaller(ShowTrajectories);

    this->requires("TS_SP")->addOperationCaller(CallCalculateBestTrajectoryApproach);
    this->requires("TS_SP")->addOperationCaller(CallCollectApproachResults);
}

bool SurgeryPlan::configureHook() {
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *PC = new char[128];
        strcpy(PC, WS);
        strcat(PC, "/Properties/PROCEDURE_CONSTANTS.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(PC)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] PC;
    }

    if(hasPeer("procedure_coordinator")) {
        TaskContext *pc_ptr = getPeer("procedure_coordinator");
        a_current_conf = pc_ptr->getAttribute("CurrentConfiguration");

    }
    else {
        log(Error) << "Could not connect surgery_plane to procedure_coordinator component." << endlog();
        return false;
    }

    if(hasPeer("test_vrep_fri")) {
        TaskContext *tvf_ptr = getPeer("test_vrep_fri");
        this->requires("VF_SP")->connectTo( tvf_ptr->provides("VF_SP") );
        if(!requires("VF_SP")->ready()) {
            log(Error) << "Failed to require vrep_fri sub-service 'VF_SP'." << endlog();
            return false;
        }
    }

    if(hasPeer("test_supervisor")) {
        TaskContext *ts_ptr = getPeer("test_supervisor");
        this->requires("TS_SP")->connectTo( ts_ptr->provides("TS_SP") );
        if(!requires("TS_SP")->ready()) {
            log(Error) << "Failed to require task_supervisor sub-service 'TS_SP'." << endlog();
            return false;
        }
    }

    //Set Data Sample
    error_msg e;
    e.level = 9999;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);

    //Find if surgery plan was setup
    m_surgery_plan_setup = false;

    //Marshaller
    m_marshaller = KDL::MarshallerSurgeryPlan();

    return true;
}

bool SurgeryPlan::startHook(){
    return true;
}

void SurgeryPlan::updateHook(){
}

void SurgeryPlan::stopHook() {
}

void SurgeryPlan::cleanupHook() {
}

bool SurgeryPlan::ConfirmSurgeryPlan(const vector_int &id, const vector_vector_double &target,
                                     const vector_vector_double &entry) {
    //Clear Trajectory Plan
    m_trajectories.clear();

    error_msg emsg;
    if(id.data.empty() || target.data.empty() || entry.data.empty()) {
        emsg.level = EMPTYPLN;
        emsg.msg = m_ec.dict[EMPTYPLN];
        outport_error_msg.write(emsg);
        return false;
    }

    //Check if number of points and ids match
    if(id.data.size() != target.data.size())
    {
        emsg.level = MISMTRID;
        emsg.msg = m_ec.dict[MISMTRID];
        outport_error_msg.write(emsg);
        return false;
    }

    //Check if number of entry points match number of target points
    if(entry.data.size() != target.data.size())
    {
        emsg.level = MISMTREN;
        emsg.msg = m_ec.dict[MISMTREN];
        outport_error_msg.write(emsg);
        return false;
    }

    //Check if repeated ids
    std::vector<int> copy_id = id.data;
    std::sort(copy_id.begin(), copy_id.end());
    if(std::adjacent_find(copy_id.begin(), copy_id.end()) != copy_id.end())
    {
        emsg.level = REPEATID;
        emsg.msg = m_ec.dict[REPEATID];
        outport_error_msg.write(emsg);
        return false;
    }

    //Send Trajectories to VREP
    AddTrajectories.call(id, target, entry);

    //Add to component member
    for(size_t t = 0; t < id.data.size(); t++)
    {
        m_trajectories.emplace_back(id.data[t], target.data[t], entry.data[t]);
    }

    //Surgery plan setup
    m_surgery_plan_setup = true;

    emsg.level = SURGPUPL;
    emsg.msg = m_ec.dict[SURGPUPL];
    outport_error_msg.write(emsg);

    return true;
}

bool SurgeryPlan::RetrieveSurgeryPlan(vector_int &id, vector_vector_double &target, vector_vector_double &entry)
{
    if(m_surgery_plan_setup) {
        std::vector<int> t_ids;
        std::vector<std::vector<double> > t_tar, t_ent;

        for(Trajectory &t : m_trajectories) {
            t_ids.push_back(t.id_);
            t_tar.push_back(t.target_);
            t_ent.push_back(t.entry_);
        }

        id.data = t_ids;
        target.data = t_tar;
        entry.data = t_ent;

        return true;
    }
    return false;
}

void SurgeryPlan::RemoveAllTrajectories() {
    SendHandle<void()> handle = RemoveTrajectories.send();

    m_trajectories.clear();
}

bool SurgeryPlan::ShowSurgeryTrajectory(const int id) {
    //check if trajectory already exists
    auto it = std::find_if(m_trajectories.begin(), m_trajectories.end(), [&](const Trajectory& t){
        return (t.id_ == id);
    });
    if(it != m_trajectories.end()) {
        SendHandle<void(const int)> handle = ShowTrajectory.send(id);

        error_msg e;
        e.level = LOG_LEVEL::INFO;
        e.msg = string("[SP] Selected trajectory with ID " + to_string(id) + ".");
        outport_error_msg.write(e);
        return true;
    }
    else {
        error_msg e;
        e.level = LOG_LEVEL::WARNING;
        e.msg = "[SP] Trajectory ID to select does not exist.";
        outport_error_msg.write(e);
    }
    return false;
}

void SurgeryPlan::ShowAllTrajectories() {
    SendHandle<void()> handle = ShowTrajectories.send();
    for(Trajectory &t : m_trajectories) {
        t.visible_ = true;
    }
}

bool SurgeryPlan::LoadSurgeryPlanFile(const string &filename, vector_int &id, vector_vector_double &target, vector_vector_double &entry)
{
    error_msg emsg;
    std::vector<int> tmp_id;
    std::vector<std::vector<double> > tmp_entry, tmp_target;

    //Clear current trajectory list
    RemoveAllTrajectories();

    //Get values from file
    m_marshaller.LoadSurgeryPlanFile(filename);
    m_marshaller.GetTrajectories(tmp_id, tmp_target, tmp_entry);

    if(tmp_id.empty() || tmp_target.empty() || tmp_entry.empty()) {
        emsg.level = EMPTYPLN;
        emsg.msg = m_ec.dict[EMPTYPLN];
        outport_error_msg.write(emsg);
        return false;
    }

    //Check if number of points and ids match
    if(tmp_id.size() != tmp_target.size())
    {
        emsg.level = MISMTRID;
        emsg.msg = m_ec.dict[MISMTRID];
        outport_error_msg.write(emsg);
        return false;
    }

    //Check if number of entry points match number of target points
    if(tmp_entry.size() != tmp_target.size())
    {
        emsg.level = MISMTREN;
        emsg.msg = m_ec.dict[MISMTREN];
        outport_error_msg.write(emsg);
        return false;
    }

    //Check if repeated ids
    std::vector<int> copy_id = tmp_id;
    std::sort(copy_id.begin(), copy_id.end());
    if(std::adjacent_find(copy_id.begin(), copy_id.end()) != copy_id.end())
    {
        emsg.level = REPEATID;
        emsg.msg = m_ec.dict[REPEATID];
        outport_error_msg.write(emsg);
        return false;
    }

    id.data = tmp_id;
    target.data = tmp_target;
    entry.data = tmp_entry;

    return true;
}

bool SurgeryPlan::SaveSurgeryPlanFile(const string &filename)
{
    std::vector<int> id;
    std::vector<std::vector<double> > target, entry;

    for(Trajectory &t : m_trajectories) {
        id.push_back(t.id_);

        target.push_back(t.target_);
        entry.push_back(t.entry_);
    }

    m_marshaller.SetTrajectories(id, target, entry);
    m_marshaller.SaveSurgeryPlanFile(filename);

    return true;
}

bool SurgeryPlan::SelectSurgeryTrajectory(const int id)
{
    //deselect any trajectory
    for(Trajectory &traj : m_trajectories) {
        traj.selected_ = false;
    }

    //check if trajectory already exists
    auto it = std::find_if(m_trajectories.begin(), m_trajectories.end(), [&](const Trajectory& t){
        return (t.id_ == id);
    });
    //if id does not exist in trajectories list
    if(it != m_trajectories.end()) {

        //Show only selected trajectory
        ShowSurgeryTrajectory(id);

        if(!it->setup_) {
            std::vector<double>
                    target = it->target_,
                    entry = it->entry_;

            //Call TestSupervisor to handle
            bool res = CallCalculateBestTrajectoryApproach.call(target, entry, m_trajectory_approach_step, m_safe_distance);

            return res;
        }

        return true;
    }
    else {
        error_msg emsg;
        emsg.level = NOTFOUND;
        emsg.msg = m_ec.dict[NOTFOUND];
        outport_error_msg.write(emsg);
    }
    return false;
}

bool SurgeryPlan::VerifyTrajectory(const int id, RTT::vector_frame &frames, RTT::vector_int& conf, std::vector<double> &psi,
                                   RTT::vector_vector_double &scores)
{
    //check if trajectory already exists
    auto it = std::find_if(m_trajectories.begin(), m_trajectories.end(), [&](const Trajectory& t){
        return (t.id_ == id);
    });
    //if id does not exist in trajectories list
    if(it != m_trajectories.end()) {

        if(!it->setup_) {
            std::vector<double>
                    target = it->target_,
                    entry = it->entry_;

            //Collect data from task supervisor
            CallCollectApproachResults.call(conf, psi, frames, scores);

            if(!conf.data.empty()) {
                it->SetApproaches(frames.data, conf.data, psi, scores.data);
            }
        }

        // Reorder approaches based on the current gc
        it->OrderApproaches(a_current_conf.get());
        // Get ordered approaches
        it->GetApproaches(frames.data, conf.data, psi, scores.data);

        if(!it->approaches_.empty()) {
            //select current traj
            it->selected_ = true;

            return true;
        }
        else {
            error_msg emsg;
            emsg.level = NOTFEASB;
            emsg.msg = m_ec.dict[NOTFEASB];
            outport_error_msg.write(emsg);
        }
    }
    else {
        error_msg emsg;
        emsg.level = NOTFOUND;
        emsg.msg = m_ec.dict[NOTFOUND];
        outport_error_msg.write(emsg);
    }
    return false;
}

bool SurgeryPlan::GetSurgeryTrajectoryParameters(const int id, std::vector<double> &target, std::vector<double> &entry)
{
    //check if trajectory already exists
    auto it = std::find_if(m_trajectories.begin(), m_trajectories.end(), [&](const Trajectory& t){
        return (t.id_ == id);
    });
    //if id does not exist in trajectories list
    if(it != m_trajectories.end()) {
        target = it->target_;
        entry = it->entry_;
    }
    else {
        error_msg emsg;
        emsg.level = NOTFOUND;
        emsg.msg = m_ec.dict[NOTFOUND];
        outport_error_msg.write(emsg);
    }
    return false;
}


ORO_CREATE_COMPONENT(SurgeryPlan)
