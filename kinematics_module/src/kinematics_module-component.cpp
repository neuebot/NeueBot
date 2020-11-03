#include "kinematics_module-component.hpp"

#include <eigen3/Eigen/Dense>

#include <rtt/Property.hpp>
#include <rtt/Component.hpp>
#include <rtt/marsh/Marshalling.hpp>

#include <cstring>
#include <cstdlib>
#include <string>
#include <cmath>
#include <algorithm>
#include <utility>
#include <fstream>

#define _USE_MATH_DEFINES

using namespace KDL;
using namespace Eigen;
using namespace RTT;
using namespace boost::icl;
using namespace std;

KinematicsModule::KinematicsModule(const string &component_name) :
    TaskContext(component_name, PreOperational)
{
    //PROPERTIES
    this->addProperty("NJoints", m_nj).doc("Number of joints.");
    this->addProperty("Tolerance", m_tol).doc("Calculations tolerance.");
    this->addProperty("LinkLength", m_lnk).doc("Robot Link Length.");
    this->addProperty("JointLimits", m_lim).doc("Maximum allowed joint position.");
    this->addProperty("KappaNS", m_K).doc("Psi value calculation variable K.");
    this->addProperty("AlphaNS", m_alpha).doc("Psi value calculation variable alpha.");
    this->addProperty("MaxChangeNS", m_max_change_ns).doc("Maximum allowed psi variation between sparse trajectory points.");
    this->addProperty("SingInterval", m_sing_interval).doc("Avoidance interval margin in nullspace around a singularity.");
    this->addProperty("DetectProxim", m_detect_proxim).doc("Maximum allowed nullspace acceleration.");

    this->addProperty("Denavit-Hartenberg-A", m_dh_a).doc("Displacement from one Frame to the next along x-axis.");
    this->addProperty("Denavit-Hartenberg-Alpha", m_dh_alpha).doc("Rotation from one Frame to the next in turn of the previous x-axis.");
    this->addProperty("Denavit-Hartenberg-D", m_dh_d).doc("Displacement from one Frame to the next along z-axis.");
    this->addProperty("Denavit-Hartenberg-Theta", m_dh_theta).doc("Rotation from one Frame to the next in turn of the previous z-axis.");

    //PORTS
    this->ports()->addPort("inport_current_robot_posture", inport_current_robot_posture).doc("Input the current robot parameters.");
    this->ports()->addPort("inport_current_joint_positions", inport_current_joint_positions).doc("Input the current joint positions.");
    this->ports()->addPort("inport_online_frame", inport_online_frame).doc("Input next target online frame.");
    this->ports()->addPort("outport_online_joint_positions", outport_online_joint_positions).doc("Output of the online joint positions");
    this->ports()->addPort("outport_error_msg", outport_error_msg).doc("Output the result value and error codes of the component to task supervisor.");

    this->ports()->addPort("inport_external_joint_torques", inport_external_joint_torques).doc("Input port the current joint external torques.");
    this->ports()->addPort("outport_current_force", outport_current_force).doc("Output port the current external force.");

    //OPERATIONS
    //Calculate Kinematics for a Trajectory
    this->addOperation("CalculateJointTrajectory", &KinematicsModule::CalculateJointTrajectory, this, OwnThread);
    this->addOperation("CalculateTrajectoryApproach", &KinematicsModule::CalculateTrajectoryApproach, this, OwnThread);
    this->addOperation("AnalyseTrajectory", &KinematicsModule::AnalyseTrajectory, this, OwnThread);
    this->addOperation("AnalyseTrajectoryApproach", &KinematicsModule::AnalyseTrajectoryApproach, this, OwnThread);
    this->addOperation("ScoresTrajectory", &KinematicsModule::ScoresTrajectory, this, OwnThread);
    this->addOperation("CalculateNullspaceJointTrajectory", &KinematicsModule::CalculateNullspaceJointTrajectory, this, OwnThread);
    //Calculate Kinematics for a single pose.
    this->addOperation("ForwardKinematics", &KinematicsModule::ForwardKinematics, this, OwnThread);
    this->addOperation("InverseKinematics", &KinematicsModule::InverseKinematics, this, OwnThread);
    this->addOperation("CalculateExternalForces", &KinematicsModule::CalculateExternalForces, this, OwnThread);
    this->addOperation("AdjustedInverseKinematics", &KinematicsModule::AdjustedInverseKinematics, this, OwnThread);
    this->addOperation("ScoresCurrentPoseCurrentGC", &KinematicsModule::ScoresCurrentPoseCurrentGC, this, OwnThread);
    this->addOperation("ScoresCurrentPoseAlternativeGC", &KinematicsModule::ScoresCurrentPoseAlternativeGC, this, OwnThread);
    this->addOperation("SetOnlineTrajectoryGeneration", &KinematicsModule::SetOnlineTrajectoryGeneration, this, ClientThread);
}

bool KinematicsModule::configureHook(){
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *RSP = new char[128];
        char *NP = new char[128];
        strcpy(RSP, WS);
        strcat(RSP, "/Properties/ROBOT_STRUCTURE_PROPERTIES.xml");
        strcpy(NP, WS);
        strcat(NP, "/Properties/NULLSPACE_PROPERTIES.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(RSP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(NP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] NP;
        delete[] RSP;
    }
    else
    {
        log(Error) << "Could not find HAPTIC_WORKSPACE environment variable." << endlog();
        return false;
    }

    m_dh.resize(m_nj, vector<double>(4));
    for(size_t j=0; j<m_nj; ++j) {
        m_dh[j][0] = m_dh_a[j];
        m_dh[j][1] = m_dh_alpha[j];
        m_dh[j][2] = m_dh_d[j];
        m_dh[j][3] = m_dh_theta[j];
    }

    m_kinematic_solver = std::make_shared<KinematicSolver>(m_nj, m_tol, m_lnk, m_lim, m_dh, m_K, m_alpha,
                                                           m_max_change_ns, m_sing_interval, m_detect_proxim);

    //Setting
    m_mode = KIN_MODE::TFC;

    //Setup outport data sample
    outport_online_joint_positions.setDataSample(std::vector<double>(m_nj));
    outport_current_force.setDataSample(std::vector<double>(3));

    //Setup error message data sample
    error_msg e;
    e.level = 9999;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);

    bool connections = true;
    connections &= inport_online_frame.connected();
    connections &= inport_current_robot_posture.connected();
    connections &= inport_current_joint_positions.connected();
    return connections;
}

bool KinematicsModule::startHook() {
    return true;
}

void KinematicsModule::updateHook() {
    //Read current joint positions
    posture current_robot_param;
    std::vector<double> current_joint_positions;
    std::vector<double> current_joint_torques;
    KDL::Frame online_frame;
    bool new_online_frame;

    //! Run every cycle
    if(inport_current_robot_posture.read(current_robot_param) == NewData) {
        m_current_frame = current_robot_param.frame;
        m_current_psi = current_robot_param.psi;
        m_current_gc = current_robot_param.GC;
    }
    if(inport_current_joint_positions.read(current_joint_positions) == NewData) {
        m_current_joint_positions = current_joint_positions;
    }
    //! To check later
    new_online_frame = (inport_online_frame.read(online_frame) == NewData);

    //Operating in Online Trajectory Generation Mode
    if(m_mode == KIN_MODE::OTG) {
        if(new_online_frame) {
            std::vector<double> online_joint_positions;
            double new_psi;
            error_msg err;
            int res = m_kinematic_solver->IterationInverseKinematics(online_frame, m_current_gc, m_current_psi, online_joint_positions, new_psi);
            if(res!=0) {
                log(Warning) << "Cannot follow target trajectory!" << endlog();
            }
            switch (res) {
            case 0:
                outport_online_joint_positions.write(online_joint_positions);
                break;
            case 1:
                err.level = IMPPSI;
                err.msg = m_ec.dict[IMPPSI];
                outport_error_msg.write(err);
                break;
            case 2:
                err.level = OUTWS;
                err.msg = m_ec.dict[OUTWS];
                outport_error_msg.write(err);
                break;
            }
        }
    }

    if(inport_external_joint_torques.read(current_joint_torques) == NewData) {
        std::vector<double> current_force(3);
        CalculateExternalForces(current_joint_torques, current_force);

        outport_current_force.write(current_force);
    }
}

void KinematicsModule::stopHook() {
}

void KinematicsModule::cleanupHook() {
}

void KinematicsModule::SetOnlineTrajectoryGeneration(bool otg)
{
    if(otg) {
        m_mode = KIN_MODE::OTG;
    }
    else {
        m_mode = KIN_MODE::TFC;
    }
}

bool KinematicsModule::CalculateJointTrajectory(const std::vector<Frame> &target_frames, std::vector<std::vector<double> > &output_joints)
{
    error_msg emsg;

    //Check if frames are possible
    if(CheckWaypointsInWorkspace(target_frames)) {

//        int res = m_kinematic_solver->BatchInverseKinematics(target_frames, m_current_gc, m_current_psi, output_joints);
        int res = m_kinematic_solver->BatchInverseKinematicsIntervalAvoidance(target_frames, m_current_gc, m_current_psi, output_joints);

        switch(res) {
        case 0:
            return true;
        case 1:
            // TODO: Prompt new initial posture
            emsg.level = IMPPSI;
            emsg.msg = m_ec.dict[IMPPSI];
            outport_error_msg.write(emsg);
            return false;
        case 2:
            // TODO: Differentiate joint limits from leaving workspace
            emsg.level = EXCDJLIM;
            emsg.msg = m_ec.dict[EXCDJLIM];
            outport_error_msg.write(emsg);
            return false;
        }
    }
    else {
        emsg.level = OUTWS;
        emsg.msg = m_ec.dict[OUTWS];
        outport_error_msg.write(emsg);
        return false;
    }
    return true;
}

bool KinematicsModule::CalculateNullspaceJointTrajectory(Frame current_frame, std::vector<double> &psi_vec, std::vector<std::vector<double> > &output_joints)
{
    error_msg emsg;

    if(m_kinematic_solver->BatchNullspaceInverseKinematics(current_frame, m_current_gc, psi_vec, output_joints)) {
        emsg.level = EXISTTRAJ;
        emsg.msg = m_ec.dict[EXISTTRAJ];
        outport_error_msg.write(emsg);
        return true;
    }
    else {
        emsg.level = OUTREACH;
        emsg.msg = m_ec.dict[OUTREACH];
        outport_error_msg.write(emsg);
        return false;
    }
}

bool KinematicsModule::CalculateTrajectoryApproach(const RTT::vector_vector_double &conf_desired, const RTT::vector_frame &frames, RTT::vector_int &conf_max,
                                                   std::vector<double> &psi_max, RTT::vector_frame &frames_max)
{
    conf_max.data.assign(8, -1);
    psi_max.assign(8,0.0);
    frames_max.data.assign(8, KDL::Frame::Identity());

    std::vector<double> manip_max(8, 0.0);
    bool solution = false;

    //Iterate through all configurations
    for(size_t conf=0; conf<8; conf++) {
        interval_set<double> allow, desired;
        desired = m_kinematic_solver->vector2iclinterval(conf_desired.data[conf]);
        //Iterate through all poses
        for(size_t i=0; i<frames.data.size(); ++i) {
            //Compute allowed intervals for the given pose and conf
            m_kinematic_solver->SinglePoseIntervals(frames.data[i], conf, allow);

            interval_set<double> combined;
            combined += (allow & desired);

            if(!combined.empty()) {
                double best_psi;
                double best_manip;
                bool exists = ScoreApproachFrames(frames.data[i], conf, combined, best_psi, best_manip);
                // If there is at least one possible solution / psi, we store the maximum obtained manipulability
                // and nullspace configuration
                if(exists) {
                    if(best_manip > manip_max[conf]) {
                        conf_max.data[conf] = conf;
                        psi_max[conf] = best_psi;
                        frames_max.data[conf] = frames.data[i];

                        manip_max[conf] = best_manip;
                        solution = true;
                    }
                }
            }
        }
    }

    SortBestApproach(conf_max.data, psi_max, frames_max.data, manip_max);

    return solution;
}

bool KinematicsModule::ScoresCurrentPoseCurrentGC(std::vector<double> &ns_scores) {
    error_msg err;
    if(!m_kinematic_solver->SinglePoseScores(m_current_frame, m_current_gc, m_current_psi, ns_scores)) {
        err.level = OUTWS;
        err.msg = m_ec.dict[OUTWS];
        outport_error_msg.write(err);
        return false;
    }
    return true;
}

bool KinematicsModule::ScoresCurrentPoseAlternativeGC(const int gc, std::vector<double> &ns_scores) {
    error_msg err;
    if(!m_kinematic_solver->SinglePoseScores(m_current_frame, gc, m_current_psi, ns_scores)) {
        err.level = OUTWS;
        err.msg = m_ec.dict[OUTWS];
        outport_error_msg.write(err);
        return false;
    }
    return true;
}

bool KinematicsModule::AnalyseTrajectory(vector<KDL::Frame> &target_frames,
                                        vector<int> &possible, vector<vector<double> > &initial_interval) {
    vector<unsigned int> upossible;
    initial_interval.clear();

    bool res = m_kinematic_solver->CheckNullspace(target_frames, upossible, initial_interval);

    possible.assign(upossible.begin(), upossible.end());
    return res;
}

bool KinematicsModule::AnalyseTrajectoryApproach(const vector_frame &frames, const int gc, const double psi, std::vector<double> &scores)
{
    std::vector<double> initial_interval;
    bool res = m_kinematic_solver->CheckConfigurationNullspace(frames.data, gc, initial_interval);

    m_kinematic_solver->ArmAngleScore(m_kinematic_solver->vector2iclinterval(initial_interval), psi, scores);

    return res;
}

int KinematicsModule::ScoresTrajectory(vector<int> &possible, vector<vector<double> > &initial_interval, vector<int> &gc_alt,
                                       vector<vector<double> > &scores)
{
    //Clear input
    error_msg err;
    int res = 0;
    gc_alt.clear();
    scores.clear();

    //Check if there is any possible configuration
    if(possible.empty()) {
        err.level = NOPOSSGC;
        err.msg = m_ec.dict[NOPOSSGC];
        res = 3;
    }
    else {
        //Getting order of better suited robot configurations
        vector<int> gc_index;
        m_kinematic_solver->GlobConfOrder(m_current_gc, possible, gc_index, gc_alt);

        //Interval at current configuration
        boost::icl::interval_set<double> current_interval;

        //Combining the trajectory interval with the approach interval
        //Obtaining the score for the joined intervals
        for(size_t i=0; i<gc_alt.size(); ++i) {
            vector<double> score;
            boost::icl::interval_set<double> pose_interval, trajectory_interval, joined_interval;

            //Operations to combine both trajectory and approach intervals
            vector<double> this_interval = initial_interval.at(gc_index[i]);
            m_kinematic_solver->SinglePoseIntervals(m_current_frame, gc_alt[i], pose_interval);
            for(size_t j=0; j<this_interval.size(); j=j+2) {
                trajectory_interval += boost::icl::continuous_interval<double>::closed(this_interval[j*2], this_interval[j*2+1]);
            }
            //Intersection
            joined_interval = (pose_interval & trajectory_interval);

            //If the better fitted configuration is the current one, store the joined interval
            if(gc_alt[i] == m_current_gc) {
                current_interval = joined_interval;
            }

            m_kinematic_solver->ArmAngleScore(joined_interval, m_current_psi, score);
            scores.push_back(score);
        }
        //Check if the current configuration is possible
        if(m_current_gc != gc_alt.front()) {
            err.level = IMPPSI;
            err.msg = m_ec.dict[IMPPSI];
            res = 2;
        }
        else {
            //Use the stored current_interval to calculate if the current psi allows for the trajectory to be performed
            bool psi_sol = false;
            vector<double> current_interval_vector = m_kinematic_solver->iclinterval2vector(current_interval);
            for(size_t i=0; i<current_interval_vector.size()/2; ++i) {
                if(m_current_psi > current_interval_vector[i*2] && m_current_psi < current_interval_vector[i*2+1]) {
                    psi_sol = true;
                }
            }
            if(!psi_sol) {
                err.level = CHANGEPSI;
                err.msg = m_ec.dict[CHANGEPSI];
                res = 1;
            }
            else {
                err.level = EXISTTRAJ;
                err.msg = m_ec.dict[EXISTTRAJ];
                res = 0;
            }
        }
    }
    outport_error_msg.write(err);
    return res;
}

void KinematicsModule::ForwardKinematics(std::vector<double> &joints, Frame &frame, unsigned int &gc, double &psi) {
    vector<double> jrad = joints;
    /// CHECK!!!
    //Convert to rad 
    for_each(jrad.begin(), jrad.end(), [](double &el){el *= (M_PI/180);});
    m_kinematic_solver->ForwardKinematics(jrad, gc, psi, frame);
}

bool KinematicsModule::InverseKinematics(const Frame target, const unsigned int gc, const double psi, std::vector<double> &joints) {
    error_msg err;
    if (m_kinematic_solver->SingleInverseKinematics(target,gc,psi,joints)) {
        err.level = EXISTIKSOL;
        err.msg = m_ec.dict[EXISTIKSOL];
        outport_error_msg.write(err);
        return true;
    }
    else {
        err.level = EXCDJLIM;
        err.msg = m_ec.dict[EXCDJLIM];
        outport_error_msg.write(err);
        return false;
    }
}

bool KinematicsModule::AdjustedInverseKinematics(const Frame target, std::vector<double> &joints)
{
    //Check solutions for current configuration
    std::vector<double> conf_scores;
    if(m_kinematic_solver->SinglePoseScores(target, m_current_gc, m_current_psi, conf_scores))
    {
        //Minimum positive score
        auto min_score = std::min_element(std::begin(conf_scores), std::end(conf_scores),
            [](const double &t1, const double &t2) {return t1 > 0 && (t2 <= 0 || t1 < t2);}
        );

        //Index 0 starts at 180
        auto value = (std::distance(std::begin(conf_scores), min_score) - 180) * (M_PI / 180);

        return InverseKinematics(target, m_current_gc, (double)value, joints);
    }
    //Else check for next configurations - ordered
    else {
        int possible_array[8] = {0, 1, 2, 3, 4, 5, 6, 7};
        std::vector<int> possible(possible_array, possible_array + 8), order, index_order;
        //Calculate order of best configurations
        m_kinematic_solver->GlobConfOrder(m_current_gc, possible, index_order, order);

        //Current configuration was already tested
        for(size_t i=1; i<order.size(); ++i) {
            if(m_kinematic_solver->SinglePoseScores(target, m_current_psi, i, conf_scores))
            {
                //Minimum positive score
                auto min_score = std::min_element(std::begin(conf_scores), std::end(conf_scores),
                    [](const double &t1, const double &t2) {return t1 > 0 && (t2 <= 0 || t1 < t2);}
                );

                //Index 0 starts at 180
                auto value = std::distance(std::begin(conf_scores), min_score) - 180 * (M_PI / 180);

                return InverseKinematics(target, m_current_gc, (double)value, joints);
            }
        }
    }
}

void KinematicsModule::SortBestApproach(std::vector<int> &conf_max, std::vector<double> &psi_max, std::vector<Frame> &frames_max, std::vector<double> &manip_max)
{
    std::vector<Approach> vapproach;
    for(size_t i=0; i<conf_max.size(); ++i) {
        vapproach.emplace_back(conf_max[i], psi_max[i], frames_max[i], manip_max[i]);
    }

    //remove all elements whose conf is -1 (not possible)
    vapproach.erase(std::remove_if(vapproach.begin(), vapproach.end(), [](Approach &p){
                        return (p._conf == -1);
                    }));

    //Order by manip value
    std::stable_sort(vapproach.begin(), vapproach.end(), [](const Approach &a, const Approach &b)
    { return a._manip > b._manip; });

    //! THE CURRENT CONFIGURATION IS ONLY DECIDED IN THE LOCK STEP (PROCEDURE COORDINATOR)
//    //if the current configuration has solutions we want it to be the first
//    auto it = std::find_if(vapproach.begin(), vapproach.end(), [&](Approach& a){
//        return (a._conf == m_current_gc);
//    });
//    if(it != vapproach.end()) {
//        //Rotates the iterator that matches the current global configuration to the start of the order vector
//        std::rotate(vapproach.begin(), it, it+1);
//    }

    //Resize
    conf_max.resize(vapproach.size());
    psi_max.resize(vapproach.size());
    frames_max.resize(vapproach.size());
    manip_max.resize(vapproach.size());

    for(size_t i=0; i<vapproach.size(); ++i)
    {
        conf_max[i] = vapproach[i]._conf;
        psi_max[i] = vapproach[i]._psi;
        frames_max[i] = vapproach[i]._frame;
        manip_max[i] = vapproach[i]._manip;
    }
}

bool KinematicsModule::ScoreApproachFrames(const Frame &frame, int conf, boost::icl::interval_set<double> &interval, double &best_psi, double &best_manip)
{
    std::vector<pair<double,double> > idx;
    std::vector<double> vmanip;
    double psi = 0;

    //TODO: Create a Property for spacing
    double spacing = m_max_change_ns * 6;
    double alpha = m_alpha;

    std::vector<double> vintrv = m_kinematic_solver->iclinterval2vector(interval);
    for(size_t i=1; i<vintrv.size(); i=i+2) {
        //First possible psi is the lower limit plus the spacing
        psi = vintrv[i-1] + spacing;
        //Then the new psi samples are the previous plus the spacing until reaching the upper limit
        while(psi < vintrv[i]) {
            idx.push_back(std::make_pair(psi, i));
            psi += spacing;
        }
    }

    //If the interval(s) are so narrow that not only a single psi is possible we skip it
    if(idx.empty()) {
        return false;
    }

    //Else we look through the possible samples to find the corresponding manipulability index
    for(size_t p=0; p<idx.size(); ++p) {
        std::vector<double> q;
        double x = idx[p].first;
        m_kinematic_solver->SingleInverseKinematics(frame, conf, x, q);
        // The score is influenced by the distance to the psi interval limits
        double low_lim = vintrv[idx[p].second-1];
        double up_lim = vintrv[idx[p].second];
        double dist_factor = 1 + (-exp(-alpha*(x-low_lim)) - exp(-alpha*(up_lim-x)));

        double manipulability = m_kinematic_solver->Manipulability(q, KinematicSolver::MANIP::AugmentedTranslational) * dist_factor;
        vmanip.push_back(manipulability);
    }

    //Determine the maximum manipulability and respective index (psi)
    auto it_max = std::max_element(vmanip.begin(), vmanip.end());
    best_manip = *it_max;
    size_t index = std::distance(vmanip.begin(), it_max);
    best_psi = idx[index].first;
    return true;
}

void KinematicsModule::CalculateExternalForces(const std::vector<double> &ext_torques, std::vector<double> &ext_forces)
{
    Eigen::VectorXd forces(6);
    Eigen::VectorXd torques = Eigen::VectorXd::Map(ext_torques.data(), ext_torques.size());
    Eigen::Matrix<double, 6, Dynamic, RowMajor, 6, 7> jacobian;

    if(!m_current_joint_positions.empty())
    {
        m_kinematic_solver->GeometricJacobian(m_current_joint_positions, jacobian);

        forces = jacobian * torques;
        ext_forces.assign(forces.data(), forces.data() + 6);
    }
}

bool KinematicsModule::CheckWaypointsInWorkspace(const std::vector<Frame> &target_frames) {
    Vector xbs(0.0, 0.0, m_lnk[0]);
    Vector xwf(0.0, 0.0, m_lnk[3]);
    for(KDL::Frame f : target_frames) {
        Vector xsee = f.p - xbs;
        Vector xsw = xsee - f.M * xwf;

        if(xsw.Norm() > m_lnk[1] + m_lnk[2] || xsw.Norm() < m_lnk[1] - m_lnk[2]) {
            return false;
        }
    }
    return true;
}

void KinematicsModule::PrintToFile(const std::vector<std::vector<double> > &joints, const std::vector<double> &psis) {
    ofstream intfile, pintfile;
    string jpath = "/home/carlos/MATLAB/Trajectory/Kinematics/LastJoint.txt";
    string ppath = "/home/carlos/MATLAB/Trajectory/Kinematics/LastPSI.txt";
    intfile.open(jpath);
    pintfile.open(ppath);
    for(size_t i=0; i<joints.size(); ++i) {
        if(!joints[i].empty()) {
            for(size_t j=0; j<joints.front().size(); ++j) {
                intfile << joints[i][j] << ',';
            }
            intfile << m_current_psi << ',' << m_current_gc << endl; // psi , GC

            pintfile << psis[i] << endl;
        }
        else {
            break;
        }
    }
    intfile.close();
    pintfile.close();
}

void KinematicsModule::PrintTESTS(const std::vector<double> &j1t, const std::vector<double> &j3t, const std::vector<double> &j5t, const std::vector<double> &j7t) {
    ofstream tfile;
    string tpath = "/home/carlos/MATLAB/Trajectory/Kinematics/TESTS.txt";
    tfile.open(tpath);
    for(size_t i=0; i<j1t.size(); ++i) {
        tfile << j1t[i] << ','  << j3t[i] << ','  << j5t[i] << ','  << j7t[i] << ',' << endl;
    }
    tfile.close();
}

ORO_CREATE_COMPONENT(KinematicsModule)
