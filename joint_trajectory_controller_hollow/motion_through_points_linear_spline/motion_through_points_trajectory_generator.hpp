#include "motion_profile.hpp"
#include <memory>

class MotionThroughPointsTrajectoryGenerator {

public:
    MotionThroughPointsTrajectoryGenerator(const unsigned int &n_joints, const double &cycle_time, const double &max_vel, const double &max_acc);

//Trajectory Parameters
private:
    const double m_sr;    //sample rate
    const double m_v_max; //maximum allowed joint velocity
    const double m_a_max; //maximum allowed joint acceleration
    const unsigned int m_nj; //number of joints
    std::vector<double> x_min, x_max;
    std::vector<double> v_max;
    std::vector<double> a_max;

//Outputs
private:
    unsigned int m_num_it;    //index of each sample
    std::vector<std::vector<double> > m_q_it; //sample positions
    std::vector<std::vector<double> > m_v_it; //sample velocities
    std::vector<std::vector<double> > m_a_it; //sample accelerations
    std::vector<double> m_t_it;  //sample times
    unsigned int m_index_it;   //path segment

public:
    bool InitiateNewMotion(const std::vector<double> &current_joint_positions, const std::vector<double> &current_joint_velocities,
                           const std::vector<std::vector<double> > &via_points, const std::vector<double> &time_via_points,
                           const std::vector<double> &limit_acc, const std::vector<double> &inner_acc);
    int GenerateNextMotionState(std::vector<double> &next_joint_positions);
};
