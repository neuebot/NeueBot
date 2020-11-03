#include "motion_through_points_trajectory_generator.hpp"
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

MotionThroughPointsTrajectoryGenerator::MotionThroughPointsTrajectoryGenerator(const unsigned int &n_joints, const double &cycle_time, const double &max_vel, const double &max_acc) :
    m_nj(n_joints),
    m_sr(cycle_time),
    m_v_max(max_vel),
    m_a_max(max_acc)
{
    x_min.assign(m_nj, 0.0);
    x_max.assign(m_nj, 0.0);

    //Assigning Joint Limits: Position Velocity and Acceleration (this will be changed)
    x_min[0] = -M_PI*170/180;    x_max[0] = M_PI*170/180;
    x_min[1] = -M_PI*120/180;    x_max[1] = M_PI*120/180;
    x_min[2] = -M_PI*170/180;    x_max[2] = M_PI*170/180;
    x_min[3] = -M_PI*120/180;    x_max[3] = M_PI*120/180;
    x_min[4] = -M_PI*170/180;    x_max[4] = M_PI*170/180;
    x_min[5] = -M_PI*120/180;    x_max[5] = M_PI*120/180;
    x_min[6] = -M_PI*175/180;    x_max[6] = M_PI*175/180;

    v_max.assign(m_nj, m_v_max);
    a_max.assign(m_nj, m_a_max);
}

bool MotionThroughPointsTrajectoryGenerator::InitiateNewMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                                                               const vector<vector<double> > &via_points, const vector<double> &time_via_points,
                                                               const vector<double> &limit_acc, const vector<double> &inner_acc)
{
    bool success = true;
    //Check if size of via_points and time at each via_point are consistent
    if(via_points.size() != time_via_points.size()) {
        throw invalid_argument("Number of received via-points does not match the time-parameterization.");
    }

    //Check if parameters fit the joint limits inposed
    for(auto &vp : via_points) {
        for(unsigned int i=0; i<m_nj; i++) {
            if(vp[i] < x_min[i] || vp[i] > x_max[i]) {
                throw invalid_argument("Via-point exceeds the joint limits");
            }
        }
    }

    for(unsigned int i=0; i<m_nj; i++) {
        if(std::abs(limit_acc[i]) > m_a_max || std::abs(inner_acc[i]) > m_a_max) {
            throw invalid_argument("Acceleration parameters exceed the maximum value.");
        }
    }

    vector<vector<double> > qv;   //vector of N vectors of 7 (set of joint positions)
    vector<vector<double> > q;    //vector of 7 vectors of N (via points for each joint)
    vector<double> t;     //time to pass by via points, same for all joints
    vector<double> lim_a(limit_acc);    //initial acceleration
    vector<double> inn_a(inner_acc);    //final acceleration

    qv.reserve(via_points.size() + 1);
    t.reserve(time_via_points.size() + 1);

    //Adding current joint position as first element of qv, and 0.0 as first element of t
    qv.push_back(current_joint_positions);
    qv.insert(qv.end(),via_points.begin(),via_points.end());
    t.push_back(0.0);
    t.insert(t.end(),time_via_points.begin(), time_via_points.end());

    //Number of via points
    unsigned int N = qv.size();

    //Transpose qv to q
    q = vector<vector <double> > (m_nj, vector<double>(N, 0.0));
    for(unsigned int i=0; i<N; i++) {
        for(unsigned int j=0; j<m_nj; j++) {
            q[j][i] = qv[i][j];
        }
    }

    std::ofstream ctcfile;
    ctcfile.open("/home/carlos/MATLAB/Debug/01_ctc.csv");
    for(unsigned int i=0; i<qv.size(); ++i) {
        for(unsigned int j=0; j<m_nj; ++j) {
            ctcfile << qv[i][j] << ",";
        }
        ctcfile << t[i] << endl;
    }
    ctcfile.close();

    std::shared_ptr<MotionProfile> mp[m_nj];
    vector<vector<double> > segments_t(m_nj,vector<double>(N*2, 0.0)); //vector of 7 vectors of N*2 segments
    vector<vector<double> > segments_q(m_nj,vector<double>(N*2, 0.0));
    vector<vector<double> > segments_v(m_nj,vector<double>(N*2, 0.0));
    vector<vector<double> > segments_a(m_nj,vector<double>(N*2, 0.0));

    for(size_t i=0; i<m_nj; ++i) {
        mp[i] = std::make_shared<MotionProfile>(q[i],t,lim_a[i],inn_a[i]);
        mp[i]->CalculateSegmentTimes();
        mp[i]->GetSegments(segments_t[i],segments_q[i],segments_v[i],segments_a[i]);
    }

    //Interpolate Path
    m_num_it = static_cast<unsigned int>(ceil(t.back() / m_sr)); //floor ceil
    vector<double> tmp(m_num_it, 0.0);
    m_q_it.assign(m_nj, tmp);
    m_v_it.assign(m_nj, tmp);
    m_a_it.assign(m_nj, tmp);
    m_t_it.assign(m_num_it, 0.0);

    unsigned int idx = 1; //interval index

    //Setting first values of m_t_it and m_q_it
    m_t_it[0] = t[0];
    for(size_t i=0; i<m_nj; ++i) {
        m_q_it[i][0] = q[i][0];
    }

    for(size_t i=1; i<m_num_it; ++i) {
        m_t_it[i] = i * m_sr;
        for(size_t j=0; j<m_nj; ++j) {
            double inter_t = m_t_it[i] - segments_t[j][idx-1];
            //Checking if current interval is a blend (0)
            if(idx % 2 == 1) {
                m_q_it[j][i] = segments_q[j][idx-1] + (segments_v[j][idx] * inter_t) + (0.5 * segments_a[j][idx] * pow(inter_t,2));
            }
            else {
                m_q_it[j][i] = segments_q[j][idx-1] + (segments_v[j][idx] * inter_t);
            }
            m_v_it[j][i] = (m_q_it[j][i] - m_q_it[j][i-1]) / m_sr;
            m_a_it[j][i] = (m_v_it[j][i] - m_v_it[j][i-1]) / m_sr;
            //Checking if sample_t belongs to another interval
            if((m_t_it[i] + m_sr) > segments_t[j][idx] && idx <= N*2) {
                idx++;
            }
        }
    }

    //Set index of current sample to 0
    m_index_it = 0;

    std::ofstream jtcfile;
    std::ofstream jtcfile2;
    jtcfile.open("/home/carlos/MATLAB/Debug/01_jtc.csv");
    jtcfile2.open("/home/carlos/MATLAB/Debug/02_jtc.csv");
    //TODO: Check if velocities or accelerations go over the limit
    for(unsigned int i=0; i<m_num_it; ++i) {
        for(unsigned int j=0; j<m_nj; ++j) {
            //store
            jtcfile << m_q_it[j][i] << ",";
            jtcfile2 << m_v_it[j][i] << ",";

            if(m_v_it[j][i] > m_v_max || m_a_it[j][i] > m_a_max)
                std::cout << "Velocity or Acceleration exceeds limts." <<
                             "\nVel: " << m_v_it[j][i] << "; Max Vel: " << m_v_max <<
                             "\nAcc: " << m_a_it[j][i] << ", Max Acc: " << m_a_max <<
                             "\n [CHECK motion_through_points...cpp]" << std::endl;
                success = true; //TODO!!!
                //break;
        }
        jtcfile << m_t_it[i] << endl;
        jtcfile2 << m_t_it[i] << endl;
    }
    jtcfile.close();
    jtcfile2.close();

    return success;
}

int MotionThroughPointsTrajectoryGenerator::GenerateNextMotionState(vector<double> &next_joint_positions) {
    //Runs through all samples and consecutively sends the next joint position (m_index_it)
    //ResultValue: 0 = running, 1 = finished
    int ResultValue = 0;
    next_joint_positions = vector<double> (m_nj);
    for(unsigned int j=0; j<m_nj; j++) {
        next_joint_positions[j] = m_q_it[j][m_index_it];
    }
    m_index_it++;
    if(m_index_it == m_num_it) {
        ResultValue = 1;
    }
    return ResultValue;
}

