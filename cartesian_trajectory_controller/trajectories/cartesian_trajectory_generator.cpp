#include "cartesian_trajectory_generator.hpp"
#include <kdl/rotational_interpolation_sa.hpp>
#include <cmath>

#include <kdl/utilities/error.h>
#include <iostream>

using namespace KDL;
using namespace std;

CartesianTrajectoryGenerator::CartesianTrajectoryGenerator(const double cycle_time, const double max_cart_vel, const double max_cart_acc,
                                                           const double eq_radius, const double max_ns_vel, const double max_ns_acc) :
    m_max_cart_vel(max_cart_vel),
    m_max_cart_acc(max_cart_acc),
    m_eq_radius(eq_radius),
    m_max_ns_vel(max_ns_vel),
    m_max_ns_acc(max_ns_acc),
    m_dt(cycle_time)
{}

CartesianTrajectoryGenerator::~CartesianTrajectoryGenerator()
{}

void CartesianTrajectoryGenerator::GenerateLinearMotion(const Frame &start_frame, const Frame &end_frame, const VELOCITY_PROFILE vel_prof) {
    Path_Line *path_line = new Path_Line(start_frame, end_frame, new RotationalInterpolation_SingleAxis(), m_eq_radius);
    VelocityProfile *vel_prof_obj = SelectVelocityProfile(vel_prof);
    vel_prof_obj->SetProfile(0, path_line->PathLength());
    m_segments.push_back(new Trajectory_Segment(path_line, vel_prof_obj));
}

void CartesianTrajectoryGenerator::GenerateLinearMotion(const Frame &start_frame, const Frame &end_frame, const double max_vel,
                                                        const double max_acc, const VELOCITY_PROFILE vel_prof) {
    Path_Line *path_line = new Path_Line(start_frame, end_frame, new RotationalInterpolation_SingleAxis(), m_eq_radius);
    VelocityProfile *vel_prof_obj = SelectVelocityProfile(vel_prof, max_vel, max_acc);
    vel_prof_obj->SetProfile(0, path_line->PathLength());
    m_segments.push_back(new Trajectory_Segment(path_line, vel_prof_obj));
}

void CartesianTrajectoryGenerator::GenerateArcMotion(const Frame &start_frame, const Vector &arc_center, const Vector &arc_dir, const Rotation &end_rot,
                                                     const double angle, const VELOCITY_PROFILE vel_prof) {
    Path_Circle *path_circle = new Path_Circle(start_frame, arc_center, arc_dir, end_rot, angle, new RotationalInterpolation_SingleAxis(), m_eq_radius);
    VelocityProfile *vel_prof_obj = SelectVelocityProfile(vel_prof);
    vel_prof_obj->SetProfile(0, path_circle->PathLength());
    m_segments.push_back(new Trajectory_Segment(path_circle, vel_prof_obj));
}


void CartesianTrajectoryGenerator::GenerateArcMotion(const Frame &start_frame, const Vector &arc_center, const Vector &arc_dir, const Rotation &end_rot,
                                                     const double angle, const double max_vel, const double max_acc, const VELOCITY_PROFILE vel_prof) {
     Path_Circle *path_circle = new Path_Circle(start_frame, arc_center, arc_dir, end_rot, angle, new RotationalInterpolation_SingleAxis(), m_eq_radius);
    VelocityProfile *vel_prof_obj = SelectVelocityProfile(vel_prof, max_vel, max_acc);
    vel_prof_obj->SetProfile(0, path_circle->PathLength());
    m_segments.push_back(new Trajectory_Segment(path_circle, vel_prof_obj));
}

void CartesianTrajectoryGenerator::GenerateComposedMotion(const vector<Frame> &path_frames, const double radius, const VELOCITY_PROFILE vp) {
    Path_RoundedComposite* path = new Path_RoundedComposite(radius,m_eq_radius,new RotationalInterpolation_SingleAxis());
    for(Frame frame : path_frames) {
        path->Add(frame);
    }
    // always call Finish() at the end, otherwise the last segment will not be added.
    path->Finish();
    // Trajectory defines a motion of the robot along a path.
    VelocityProfile *vel_prof_obj = SelectVelocityProfile(vp);
    vel_prof_obj->SetProfile(0, path->PathLength());
    m_segments.push_back(new Trajectory_Segment(path, vel_prof_obj));
}

void CartesianTrajectoryGenerator::GenerateComposedMotion(const std::vector<Frame> &path_frames, const double radius,
                                                          const double max_vel, const double max_acc, const VELOCITY_PROFILE vp) {
    Path_RoundedComposite* path = new Path_RoundedComposite(radius,m_eq_radius,new RotationalInterpolation_SingleAxis());
    for(Frame frame : path_frames) {
        path->Add(frame);
    }
    // always call Finish() at the end, otherwise the last segment will not be added.
    path->Finish();
    // Trajectory defines a motion of the robot along a path.
    VelocityProfile *vel_prof_obj = SelectVelocityProfile(vp, max_vel, max_acc);
    vel_prof_obj->SetProfile(0, path->PathLength());
    m_segments.push_back(new Trajectory_Segment(path, vel_prof_obj));
}

void CartesianTrajectoryGenerator::GenerateNullspaceMotion(const double cur_psi, const double ref_psi, std::vector<double> &psi_vec, std::vector<double> &time_vec) {
    VelocityProfile *ptr = new VelocityProfile_Trap(m_max_ns_vel, m_max_ns_acc);
    ptr->SetProfile(cur_psi, ref_psi);
    //Number of samples
    size_t ns = ceil(ptr->Duration() / m_dt);
    psi_vec.resize(ns), time_vec.resize(ns);
    for(size_t t=0; t<ns-1; ++t) {
        time_vec[t] = (t+1)*m_dt;
        psi_vec[t] = ptr->Pos(time_vec[t]);
    }
    time_vec[ns-1] = ptr->Duration();
    psi_vec[ns-1] = ref_psi;
}

void CartesianTrajectoryGenerator::AddPauseSegment(const double pause_time, const KDL::Frame &current_frame) {
    //If you want to add a pause mid segments!
    m_segments.push_back(new Trajectory_Stationary(pause_time, current_frame));
}

bool CartesianTrajectoryGenerator::GetTargetTrajectory(vector<Frame> &target_poses, vector<double> &target_poses_times, const bool destroy) {
    bool success = false;
    if(m_segments.size() > 0) {
        //Create a KDL::Trajectory_Composite from each segment
        m_trajectory = std::make_shared<Trajectory_Composite>(); //destroys segments on destructor
        for(KDL::Trajectory *segment : m_segments) {
            m_trajectory->Add(segment); //acquire ownership of segment pointers
        }
        //Generating Target Poses, according to sampling rate m_dt
        m_num_it = ceil(m_trajectory->Duration()/m_dt);
        //Initializing m_target_frames, Vector of N vectors, containing the position and orientation (quaternion) of each sample pose
        m_target_frames.resize(m_num_it);
        //Initializing m_target_times, Vector of N doubles containing the time at each sample pose.
        m_target_times.resize(m_num_it);
        for(unsigned int i=0; i<m_num_it; i++) {
            m_target_frames[i] = m_trajectory->Pos(i*m_dt + m_dt);
            m_target_times[i] = (i*m_dt + m_dt);
        }
        //Store current trajectory in member variables
        target_poses = m_target_frames;
        target_poses_times = m_target_times;
        //Deletes trajectory segments
        if(destroy)
            m_segments.clear();

        success = true;
    }
    return success;
}

void CartesianTrajectoryGenerator::SampleLinearTrajectory(const Frame &start_frame, const Frame &end_frame, const double step, std::vector<Frame> &frames) {
    //Normalize input Rotation matrices from start_frame and end_frame or Path_Line breaks....
    KDL::Vector
            sxx(start_frame.M.UnitX()),
            syy(start_frame.M.UnitY()),
            szz(start_frame.M.UnitZ()),

            exx(end_frame.M.UnitX()),
            eyy(end_frame.M.UnitY()),
            ezz(end_frame.M.UnitZ());

    sxx.Normalize();
    syy.Normalize();
    szz.Normalize();

    exx.Normalize();
    eyy.Normalize();
    ezz.Normalize();

    KDL::Frame norm_start(KDL::Rotation(sxx, syy, szz), start_frame.p);
    KDL::Frame norm_end(KDL::Rotation(exx, eyy, ezz), end_frame.p);

    Path_Line *path_line = new Path_Line(norm_start, norm_end, new RotationalInterpolation_SingleAxis(), m_eq_radius);
    int num_samples = ceil(path_line->PathLength() / step);
    frames.assign(num_samples, Frame::Identity());
    for(size_t i=0; i<num_samples; ++i) {
        frames[i] = path_line->Pos(i*step);
    }

    //ANALYSE FRAMES
//    PrintToFile(frames, "/home/carlos/MATLAB/Trajectory/Cartesian/Sample.txt");
}

VelocityProfile* CartesianTrajectoryGenerator::SelectVelocityProfile(const VELOCITY_PROFILE vel_prof, const bool starting) {
    VelocityProfile *ptr;
    switch(vel_prof) {
    case(VELOCITY_PROFILE::TRAP):
        ptr = new VelocityProfile_Trap(m_max_cart_vel, m_max_cart_acc);
        break;
    case(VELOCITY_PROFILE::TRAP_HALF):
        ptr = new VelocityProfile_TrapHalf(m_max_cart_vel, m_max_cart_acc, starting);
        break;
    case(VELOCITY_PROFILE::RECT):
        ptr = new VelocityProfile_Rectangular(m_max_cart_vel);
        break;
    default:
        throw runtime_error("Selected a non-existing velocity profile.");
    }
    return ptr;
}

VelocityProfile* CartesianTrajectoryGenerator::SelectVelocityProfile(const VELOCITY_PROFILE vel_prof, const double max_vel, const double max_acc,
                                                         const bool starting) {
    VelocityProfile *ptr;
    switch(vel_prof) {
    case(VELOCITY_PROFILE::TRAP):
        ptr = new VelocityProfile_Trap(max_vel, max_acc);
        break;
    case(VELOCITY_PROFILE::TRAP_HALF):
        ptr = new VelocityProfile_TrapHalf(max_vel, max_acc, starting);
        break;
    case(VELOCITY_PROFILE::RECT):
        ptr = new VelocityProfile_Rectangular(max_vel);
        break;
    default:
        throw runtime_error("Selected a non-existing velocity profile.");
    }
    return ptr;
}

void CartesianTrajectoryGenerator::PrintToFile(const vector<Frame> &target_frames, const string &path) {
    ofstream myfile;
    myfile.open(path);
    for(unsigned int i=0; i<target_frames.size(); i++) {
        for(unsigned int k=0; k<4; k++) {
            for(unsigned int l=0; l<4; l++) {
                myfile << target_frames[i](k,l) << "\t";
            }
        }
        myfile << "\n";
    }
    myfile.close();
}
