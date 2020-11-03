#include "motion_through_points_trajectory_generator.hpp"

#include <memory>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

MotionThroughPointsTrajectoryGenerator::MotionThroughPointsTrajectoryGenerator(const unsigned int &n_joints, const double &cycle_time, const double &max_vel,
                                                                               const double &max_acc, const std::vector<double> &limits) :
    m_dt(cycle_time),
    m_v_max(max_vel),
    m_a_max(max_acc),
    m_nj(n_joints)
{
    v_max.assign(m_nj, m_v_max);
    a_max.assign(m_nj, m_a_max);
    m_lim = limits;
}

int MotionThroughPointsTrajectoryGenerator::InitiateNewPositionMotion(const vector<double> &current_joint_positions, const vector<double> &current_joint_velocities,
                                                                    const vector<vector<double> > &via_points, const vector<double> &time_via_points,
                                                                    const vector<double> &init_vel, const vector<double> &end_vel, const vector<double> &init_acc, const vector<double> &end_acc)
{
    //Check if size of via_points and time at each via_point are consistent
    if(via_points.size() != time_via_points.size()) {
        return 1; //Number of received via-points does not match the time-parameterization.
    }

    //Check if parameters fit the joint limits imposed
    for(auto &vp : via_points) {
        for(unsigned int i=0; i<m_nj; i++) {
            if(vp[i] < -m_lim[i] || vp[i] > m_lim[i]) {
                return 2; //Via-point exceeds the joint limits
            }
        }
    }

    for(unsigned int i=0; i<m_nj; i++) {
        if(std::abs(init_vel[i]) > m_v_max || std::abs(end_vel[i]) > m_v_max) {
            return 3; //Velocity parameters exceed the maximum value
        }
        if(std::abs(init_acc[i]) > m_a_max || std::abs(end_acc[i]) > m_a_max) {
            return 4; //Acceleration parameters exceed the maximum value
        }
    }

    vector<vector<double> > qv;   //vector of N vectors of 7 (set of joint positions)
    vector<vector<double> > q;    //vector of 7 vectors of N (via points for each joint)
    vector<double> t;     //time to pass by via points, same for all joints
    vector<double> vi(init_vel);    //initial velocity
    vector<double> vf(end_vel);    //final velocity
    vector<double> ai(init_acc);    //initial acceleration
    vector<double> af(end_acc);    //final acceleration

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

    //For each joint there is a set of paths, and each path has 4 polynomials
    //poly[joint][polynomial][coefficient] is a vector of 7 joint vectors, containing N-1 polynomials with 4 coeficients
    vector<vector<vector<double> > > poly(m_nj, vector<vector<double> > (N-1, vector<double> (4,0.0)));
    for(unsigned int k=0; k<m_nj; k++) {
        if(!ComputePolynomials(q[k],t,vi[k],vf[k],ai[k],af[k],poly[k])) {
            return 5; //Not enough via-points.
        }
    }

    //Interpolate Path
    m_num_it = static_cast<unsigned int>(floor(t.back() / m_dt));
    m_q_it.clear();
    vector<double> tmp(m_num_it, 0.0);
    m_q_it = vector<vector<double> > (m_nj, tmp);
    m_v_it = vector<vector<double> > (m_nj, tmp);
    m_a_it = vector<vector<double> > (m_nj, tmp);
    m_t_it = vector<double>(m_num_it, 0.0);
    double t_it = 0;
    bool path_segment_found = false;
    unsigned int k = 0;
    double tau = 0.0;
    for(unsigned int i=0; i<m_num_it; i++) {
        //find out to which interval it belongs
        while(!path_segment_found && k!=N) {
            if(t_it>=t[k] && t_it<=t[k+1])
               path_segment_found = true;
            else
               k++;
        }
        path_segment_found = false;
        tau = t_it-t[k];

        for(unsigned int j=0; j<m_nj; j++) {
            m_q_it[j][i] = poly[j][k][0] + poly[j][k][1]*tau + poly[j][k][2]*pow(tau,2) + poly[j][k][3]*pow(tau,3);
            m_v_it[j][i] = poly[j][k][1] + 2*poly[j][k][2]*tau + 3*poly[j][k][3]*pow(tau,2);
            m_a_it[j][i] = 2*poly[j][k][2] + 6*poly[j][k][3]*tau;
        }
        m_t_it[i] = t_it;
        t_it += m_dt;
    }
    //Set index of current sample to 0
    m_index_it = 0;

    PrintToFile(current_joint_positions, via_points, time_via_points);

    return 0; //OK
}

int MotionThroughPointsTrajectoryGenerator::GenerateNextPositionMotionState(vector<double> &next_joint_positions) {
    //Runs through all samples and consecutively sends the next joint position (m_index_it)
    //ResultValue: 0 = running, 1 = finished, 2 = error (no state)
    int ResultValue = 0;
    if(m_q_it[0].empty()) {
        ResultValue = 2;
    }
    else {
        next_joint_positions = vector<double> (m_nj);
        for(unsigned int j=0; j<m_nj; j++) {
            next_joint_positions[j] = m_q_it[j][m_index_it];
        }
        m_index_it++;
        if(m_index_it == m_num_it) {
            ResultValue = 1;
        }
    }
    return ResultValue;
}

bool MotionThroughPointsTrajectoryGenerator::ComputePolynomials(const std::vector<double> &via_points,
                                                                       const std::vector<double> &time_via_points,
                                                                       const double &initial_velocity, const double &end_velocity,
                                                                       const double &initial_acceleration, const double &end_acceleration,
                                                                       std::vector<std::vector<double> > &poly)
{
    unsigned int N = via_points.size();
    if(N <= 2) {
        return false; //Not enough via points
    }
    std::vector<double> v(N); //via points velocity of a joint
    std::vector<double> dt(N-1); //time interval between each time sample
    // polynomial theta_k = ak0 + ak1*t + ak2*t² + ak3*t³
    // polynomials = [ak0, ak1, ak2, ak3] for each k = 1,...N-1
    std::vector<std::vector<double> > tmp_poly(N-1, vector<double>(4, 0.0));

    //Compute Intermediate Velocities
    MatrixXd A1(N-2, N-2);
    A1 << MatrixXd::Zero(N-2, N-2);
    VectorXd x1(N-2);
    x1 << VectorXd::Zero(N-2);
    VectorXd b1(N-2);
    b1 << VectorXd(N-2);

    for(unsigned int i=0; i<N-1; i++) {
        dt[i] = time_via_points[i+1] - time_via_points[i];
    }

    //Generating A matrix
    //Check if we have more than the strict minimum number of via points (3)
    if(N>3) {
        for(unsigned int k=0; k<N-2; k++) {
            A1(k,k) = 2*(dt[k]+dt[k+1]);
            b1(k) = (3/(dt[k]*dt[k+1])) * (pow(dt[k],2) * (via_points[k+2]-via_points[k+1]) + (pow(dt[k+1],2) * (via_points[k+1]-via_points[k])));
            if(k==0) {
                A1(k,k+1) = dt[k];
                b1(k) = b1(k)-(dt[k+1]*initial_velocity);
            }
            else if(k==N-3) {
                A1(k,k-1) = dt[k+1];
                b1(k) = b1(k)-(dt[k]*end_velocity);
            }
            else {
                A1(k,k+1) = dt[k];
                A1(k,k-1) = dt[k+1];
            }
        }
    }
    //If we only have 3 via points, the A and b matrices only have 1 element each
    else {
        A1(0,0) = 2*(dt[0]+dt[1]);
        b1(0) = (3/(dt[0]*dt[1])) * (pow(dt[0],2) * (via_points[2]-via_points[1]) + (pow(dt[1],2) * (via_points[1]-via_points[0])));
    }

    //Velocities at intermediate via points
    x1 = A1.fullPivHouseholderQr().solve(b1);
    v[0] = initial_velocity;
    v[N-1] = end_velocity;
    for(unsigned int i=1; i<N-1; i++) {
        v[i] = x1(i-1);
    }

    //Compute Polynomials
    MatrixXd A2 = MatrixXd::Zero(2,2);
    VectorXd b2 = VectorXd::Zero(2);
    VectorXd x2 = VectorXd::Zero(2);
    //polynomial_k = [ak0 ak1 ak2 ak3]
    for(unsigned int j=0; j<N-1; j++) {
        //position (ak0) and velocity (ak1) coeficientsvector<vector<double> > &poly
        tmp_poly[j][0] = via_points[j];
        tmp_poly[j][1] = v[j];
        //System of Linear Equationsvector<vector<double> > &poly
        A2(0,0) = pow(dt[j],2);
        A2(0,1) = pow(dt[j],3);
        A2(1,0) = 2*dt[j];
        A2(1,1) = 3*pow(dt[j],2);
        b2(0) = via_points[j+1]-via_points[j]-v[j]*dt[j];
        b2(1) = v[j+1]-v[j];
        //x2 = A2.ldlt().solve(b2);
        x2 = A2.colPivHouseholderQr().solve(b2);
        tmp_poly[j][2] = x2(0);
        tmp_poly[j][3] = x2(1);
    }
    poly = tmp_poly;

    //Possible to compute the polynomials for these path
    return true;
}

void MotionThroughPointsTrajectoryGenerator::PrintToFile(const vector<double> &qs, const vector<vector<double> > &cq, const vector<double> &t)
{
    std::ofstream ctcfileq;
    ctcfileq.open("/home/carlos/MATLAB/Trajectory/Joint/LastJointCart.txt");
    //! Print starting position
    for(unsigned int k=0; k<m_nj; ++k) {
        ctcfileq << qs[k] << ",";
    }
    ctcfileq << "0.0" << endl;
    //! Print each position after
    for(unsigned int i=0; i<cq.size(); ++i) {
        for(unsigned int j=0; j<m_nj; ++j) {
            ctcfileq << cq[i][j] << ",";
        }
        ctcfileq << t[i] << endl;
    }
    ctcfileq.close();

    std::ofstream jtcfile;
    std::ofstream jtcfile2;
    jtcfile.open("/home/carlos/MATLAB/Trajectory/Joint/LastJoint1.txt");
    jtcfile2.open("/home/carlos/MATLAB/Trajectory/Joint/LastJoint2.txt");
    //TODO: Check if velocities or accelerations go over the limit
    for(unsigned int i=0; i<m_num_it; ++i) {
        for(unsigned int j=0; j<m_nj; ++j) {
            //store
            jtcfile << m_q_it[j][i] << ",";
            jtcfile2 << m_v_it[j][i] << ",";
        }
        jtcfile << m_t_it[i] << endl;
        jtcfile2 << m_t_it[i] << endl;
    }
    jtcfile.close();
    jtcfile2.close();
}

