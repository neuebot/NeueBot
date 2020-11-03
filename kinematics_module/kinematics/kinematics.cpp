#include "kinematics.hpp"
#include "global_configuration.hpp"

//TO REMOVE
#include <fstream>

using namespace KDL;
using namespace Eigen;
using namespace boost::icl;
using namespace std;

KinematicSolver::KinematicSolver(const unsigned int nj, const double tol, const std::vector<double> &links,
                                 const std::vector<double> &limits, const std::vector<std::vector<double> > &dh,
                                 const double Kappa, const double alpha, const double max_change,
                                 const double sing_interval, const double detect_proxim) :
    m_nj(nj),
    m_tol(tol),
    m_K(Kappa),
    m_alpha(alpha),
    m_max_change_ns(max_change),
    m_sing_interval(sing_interval),
    m_detect_proxim(detect_proxim),
    m_lnk(links),
    m_lim(limits),
    m_dh(dh)
{
}

KinematicSolver::~KinematicSolver() {}

//////////////////////////////////////////////  BASIC KINEMATICS  //////////////////////////////////////////////
void KinematicSolver::ForwardKinematics(const vector<double> &joints, unsigned int &gc, double &psi, Frame &pose) {
    gc = ((int)(joints[1] < 0)) + (2 * ((int)(joints[3] < 0))) + (4 * ((int)(joints[5] < 0)));
    GlobalConfiguration GC(gc);

    // Calculate FK with joint values
    vector<Matrix4d> tr(m_nj, Matrix4d::Identity());
    for (size_t i = 0; i < m_nj; ++i) {
        if(i==0) {
            tr[0] = denavit_hartenberg(m_dh[0][0],m_dh[0][1],m_dh[0][2],m_dh[0][3] + joints[0]);
        }
        else {
            tr[i] = tr[i-1] * denavit_hartenberg(m_dh[i][0],m_dh[i][1],m_dh[i][2],m_dh[i][3] + joints[i]);
        }
    }
    // Reference to KDL Frame
    pose = eigen2kdlframe(tr.back());

    Vector3d xs = tr[0].block<3,1>(0,3);
    Vector3d xe = tr[3].block<3,1>(0,3);
    Vector3d xw = tr[5].block<3,1>(0,3);

    Vector3d xsw = xw - xs;
    Vector3d xse = xe - xs;
    xsw.normalize();
    xse.normalize();

    // Calculate the psi - Arm Angle
    // The arm plane is defined by the plane formed by the xs, xe and xw points
    // The reference plane is defined by the xs and xw and a xe0 (explained below)
    // . A virtual robotic manipulator is created from the KUKA LBR iiwa manipulator
    //   structure. Equal in everything least the 3rd joint, which is fixed as 0.
    // . Now we compute the Inverse Kinematics to place the virtual robot in the
    //   same pose as the real robot.
    // . The elbow position of this virtual robot is xe0. Thus, with xs, xw and
    //   xe0 we form the reference plane
    Vector3d ref_plane;
    Matrix3d rot_elbow;
    ReferencePlane(tr.back(), GC.elbow(), ref_plane, rot_elbow);

    // ref_plane is the vector normal to the reference plane: xs-xe0-xw
    // cur_plane is the vector normal to the current plane:   xs-xe-xw
    Vector3d cur_plane = xse.cross(xsw);

    ref_plane.normalize();
    cur_plane.normalize();

    double cos_ns = ref_plane.dot(cur_plane);
    if(abs(cos_ns)>1) {
        cos_ns = sgn(cos_ns);
    }
    // this vector will give the sign of the psi
    Vector3d auxv = ref_plane.cross(cur_plane);
    if(auxv.norm() > m_tol) {
        psi = sgn(auxv.dot(xsw)) * acos(cos_ns);
    }
    else {
        if((ref_plane - cur_plane).norm() < m_tol) {
            psi = 0;
        }
        else {
            psi = M_PI;
        }
    }
}

bool KinematicSolver::SingleInverseKinematics(const Frame &pose, const unsigned int gc, const double psi, std::vector<double> &joints) {
    return SingleInverseKinematics(kdlframe2eigen(const_cast<Frame&>(pose)), gc, psi, joints);
}

bool KinematicSolver::SingleInverseKinematics(const Matrix4d &pose, const unsigned int gc, const double psi, vector<double> &joints) {
    //Initialize joints
    joints.assign(7,0.0);

    // Global configuration parameter
    GlobalConfiguration GC(gc);

    Vector3d xend = pose.block<3,1>(0,3);
    Vector3d xs0(0.0, 0.0, m_lnk[0]);
    Vector3d xwt(0.0, 0.0, m_lnk[3]);
    Vector3d xw0 = xend - pose.block<3,3>(0,0) * xwt;
    Vector3d xsw = xw0 - xs0;
    double dsw = xsw.norm();
    double dse = m_lnk[1];
    double dew = m_lnk[2];

    // Check if position is reachable, within workspace.
    if(dse + dew - xsw.norm() < m_tol || abs(dse - dew) - xsw.norm() > m_tol) { // "Pose outside workspace"
        return false;
    }
    // Check if position is reachable, within workspace.
    //assert(abs((pow(dsw,2) - pow(dse,2) - pow(dew,2)) - (2*dse*dew)) > m_tol && "Elbow singularity. Tip at reach limit.");

    // -- Joint 4 --
    // Elbow joint can be directly calculated since it does only depend on the
    // robot configuration and the xsw vector
    // Calculated with the Law of Cosines
    joints[3] = GC.elbow() * acos((pow(dsw,2) - pow(dse,2) - pow(dew,2))/(2*dse*dew));
    //Check if value is within joint limits
    if(joints[3] < -m_lim[3] || joints[3] > m_lim[3]) {
        return false;
    }

    Vector3d ref_plane;
    Matrix3d rot_elbow;
    ReferencePlane(pose, GC.elbow(), ref_plane, rot_elbow);

    Matrix4d T34 = denavit_hartenberg(m_dh[3][0],m_dh[3][1],m_dh[3][2],m_dh[3][3] + joints[3]);

    // Skew symmetric matrix of normalized xsw
    xsw.normalize();
    Matrix3d skew_xsw = skew_symmetric(xsw);

    // Auxiliary matrices
    Matrix3d As, Bs, Cs;
    As = skew_xsw * rot_elbow;
    Bs = -(skew_xsw * skew_xsw) * rot_elbow;
    Cs = (xsw * xsw.transpose()) * rot_elbow;

    Matrix3d R03 = As*sin(psi) + Bs*cos(psi) + Cs;

    // T03 transformation matrix (DH parameters)
    //[ cos(j1)*cos(j2)*cos(j3) - sin(j1)*sin(j3), cos(j1)*sin(j2), cos(j3)*sin(j1) + cos(j1)*cos(j2)*sin(j3), 0.4*cos(j1)*sin(j2)]
    //[ cos(j1)*sin(j3) + cos(j2)*cos(j3)*sin(j1), sin(j1)*sin(j2), cos(j2)*sin(j1)*sin(j3) - cos(j1)*cos(j3), 0.4*sin(j1)*sin(j2)]
    //[                          -cos(j3)*sin(j2),         cos(j2),                          -sin(j2)*sin(j3),  0.4*cos(j2) + 0.34]
    //[                                         0,               0,                                         0,                   1]
    joints[0] = atan2(GC.arm() * R03(1,1), GC.arm() * R03(0,1));
    joints[1] = GC.arm() * acos(R03(2,1));
    joints[2] = atan2(GC.arm() * -R03(2,2), GC.arm() * -R03(2,0));
    //Check if values are within joint limits
    for(size_t i=0; i<3; ++i) {
        if(joints[i] < -m_lim[i] || joints[i] > m_lim[i]) {
            return false;
        }
    }

    Matrix3d R34 = T34.block<3,3>(0,0);
    Matrix3d Aw, Bw, Cw;
    Aw = R34.transpose() * As.transpose() * pose.block<3,3>(0,0);
    Bw = R34.transpose() * Bs.transpose() * pose.block<3,3>(0,0);
    Cw = R34.transpose() * Cs.transpose() * pose.block<3,3>(0,0);

    Matrix3d R47 = Aw*sin(psi) + Bw*cos(psi) + Cw;

    // T47 transformation matrix (DH parameters)
    //[ cos(j5)*cos(j6)*cos(j7) - sin(j5)*sin(j7), - cos(j7)*sin(j5) - cos(j5)*cos(j6)*sin(j7), cos(j5)*sin(j6), (63*cos(j5)*sin(j6))/500]
    //[ cos(j5)*sin(j7) + cos(j6)*cos(j7)*sin(j5),   cos(j5)*cos(j7) - cos(j6)*sin(j5)*sin(j7), sin(j5)*sin(j6), (63*sin(j5)*sin(j6))/500]
    //[                          -cos(j7)*sin(j6),                             sin(j6)*sin(j7),         cos(j6),   (63*cos(j6))/500 + 2/5]
    //[                                         0,                                           0,               0,                        1]
    joints[4] = atan2(GC.wrist() * R47(1,2), GC.wrist() * R47(0,2));
    joints[5] = GC.wrist() * acos(R47(2,2));
    joints[6] = atan2(GC.wrist() * R47(2,1), GC.wrist() * -R47(2,0));
    //Check if values are within joint limits
    for(size_t i=4; i<7; ++i) {
        if(joints[i] < -m_lim[i] || joints[i] > m_lim[i]) {
            return false;
        }
    }

    return true;
}

int KinematicSolver::IterationInverseKinematics(const Frame &pose, const unsigned int gc, const double st_psi,
                                                 std::vector<double> &joints, double &new_psi) {
    return IterationInverseKinematics(kdlframe2eigen(const_cast<Frame&>(pose)), gc, st_psi, joints, new_psi);
}

int KinematicSolver::IterationInverseKinematics(const Matrix4d &pose, const unsigned int gc, const double st_psi,
                                            vector<double> &joints, double &new_psi) {
    double j4;
    vector<vector<double> > coeffs;
    interval_set<double> intrvls;

    if(CoeffCalculation(pose,gc,coeffs,j4)) {
        PsiIntervals(coeffs,gc,intrvls);
        if(IterationNewPsi(intrvls, st_psi, new_psi)) {
            InverseKinematics(coeffs, j4, gc, new_psi, joints);
        }
        else {
            return 1; //Current PSI not in a feasible interval.
        }
    }
    else {
        return 2; //Trajectory goes beyond manipulator reach.
    }
    return 0; // success
}

int KinematicSolver::BatchInverseKinematics(const std::vector<Frame> &poses, const unsigned int gc, const double st_psi, std::vector<std::vector<double> > &joints) {
    std::vector<Matrix4d> mat_poses(poses.size());
    for(size_t i=0; i<poses.size(); ++i) {
        mat_poses[i] = kdlframe2eigen(const_cast<Frame&>(poses[i]));
    }

    return BatchInverseKinematics(mat_poses, gc, st_psi, joints);
}

int KinematicSolver::BatchInverseKinematicsIntervalAvoidance(const std::vector<Frame> &poses, const unsigned int gc, const double st_psi, std::vector<std::vector<double> > &joints) {
    vector<interval_set<double> > intrvls, shadow;
    if(BatchPsiIntervals(poses, gc, intrvls)) {
        if(!ShadowIntervals(intrvls, m_max_change_ns, shadow)) {
            return 3;
        }
    }
    else {
        // Pose outside workspace
        return 3;
    }

    std::vector<Matrix4d> mat_poses(poses.size());
    for(size_t i=0; i<poses.size(); ++i) {
        mat_poses[i] = kdlframe2eigen(const_cast<Frame&>(poses[i]));
    }

    return BatchInverseKinematicsIntervalAvoidance(mat_poses, gc, st_psi, shadow, joints);
}

int KinematicSolver::BatchInverseKinematics(const std::vector<Matrix4d> &poses, const unsigned int gc, const double st_psi, std::vector<std::vector<double> > &joints) {
    double j4, next_psi;
    vector<vector<double> > coeffs;
    interval_set<double> intrvls;
    joints.resize(poses.size());

    double cur_psi = st_psi;
    for(size_t i=0; i<poses.size(); ++i) {
        if(CoeffCalculation(poses[i],gc,coeffs,j4)) {
            PsiIntervals(coeffs,gc,intrvls);
            if(IterationNewPsi(intrvls, cur_psi, next_psi)) {
                InverseKinematics(coeffs, j4, gc, next_psi, joints[i]);
                cur_psi = next_psi;
            }
            else {
                return 1; //Current PSI not in a feasible interval.
            }
        }
        else {
            return 2; //Trajectory goes beyond manipulator reach
        }
    }
    return 0; // success
}

int KinematicSolver::BatchInverseKinematicsIntervalAvoidance(const std::vector<Matrix4d> &poses, const unsigned int gc, const double st_psi,
                                                             const std::vector<boost::icl::interval_set<double> > &shadow, std::vector<std::vector<double> > &joints)
{
    double j4, next_psi;
    vector<vector<double> > coeffs;
    interval_set<double> intrvls;
    joints.resize(poses.size());

    double cur_psi = st_psi;
    for(size_t i=0; i<poses.size(); ++i) {
        if(CoeffCalculation(poses[i],gc,coeffs,j4)) {
            //reverse order
            intrvls = shadow.at(i);
            if(IterationNewPsi(intrvls, cur_psi, next_psi)) {
                InverseKinematics(coeffs, j4, gc, next_psi, joints[i]);
                cur_psi = next_psi;
            }
            else {
                return 1; //Current PSI not in a feasible interval.
            }
        }
        else {
            return 2; //Trajectory goes beyond manipulator reach
        }
    }
    return 0; // success
}

bool KinematicSolver::SinglePoseIntervals(const Frame &pose, const int gc, boost::icl::interval_set<double> &interval) {
    double elbow_jnt;
    vector<vector<double> > coeffs;

    // Calculate intervals
    if(CoeffCalculation(kdlframe2eigen(const_cast<Frame&>(pose)), gc, coeffs, elbow_jnt)) {
        PsiIntervals(coeffs,gc,interval);

        return true;
    }
    return false;
}

bool KinematicSolver::SinglePoseScores(const KDL::Frame &pose, const unsigned int gc, const double psi, std::vector<double> &scores) {
    double elbow_jnt;
    vector<vector<double> > coeffs;
    interval_set<double> intrvls;
    scores.assign(360, -1.0);

    // Calculate intervals
    if(CoeffCalculation(kdlframe2eigen(const_cast<Frame&>(pose)), gc, coeffs, elbow_jnt)) {
        PsiIntervals(coeffs,gc,intrvls);
        ArmAngleScore(intrvls, psi, scores);

        return true;
    }
    return false;
}

bool KinematicSolver::BatchNullspaceInverseKinematics(const Frame &pose, const unsigned int gc, const std::vector<double> &psi_vec, std::vector<std::vector<double> > &joints)
{
    Matrix4d mat_pose = kdlframe2eigen(const_cast<Frame&>(pose));

    return BatchNullspaceInverseKinematics(mat_pose, gc, psi_vec, joints);
}

void KinematicSolver::GeometricJacobian(const std::vector<double> &joints, Eigen::Matrix<double, 6, Dynamic, RowMajor, 6, 7> &jacobian)
{
    jacobian = Eigen::Matrix<double, 6, Dynamic, RowMajor, 6, 7>(6,7);

    unsigned int gc;
    double psi;
    Frame pose;
    ForwardKinematics(joints, gc, psi, pose);


    // Calculate FK with joint values
    Vector3d pe;
    pe << pose.p.x(), pose.p.y(), pose.p.z();
    vector<Matrix4d> tr(m_nj+1, Matrix4d::Identity());
    for (size_t i = 0; i < m_nj; ++i) {
        tr[i+1] = tr[i] * denavit_hartenberg(m_dh[i][0],m_dh[i][1],m_dh[i][2],m_dh[i][3] + joints[i]);

        //Jacobian Position
        Vector3d jacpos = tr[i].block<3,1>(0,2).cross(pe - tr[i].block<3,1>(0,3));
        Vector3d jacori = tr[i].block<3,1>(0,2);
        jacobian.col(i) << jacpos, jacori;
    }
}

double KinematicSolver::Manipulability(const std::vector<double> &joints, KinematicSolver::MANIP opt)
{
    double manip = 0;

    //Get Jacobian
    Matrix<double, 6, Dynamic, RowMajor, 6, 7> J;
    GeometricJacobian(joints, J);

    MatrixXd Id(7, 7);
    MatrixXd SubM1(3,7);
    MatrixXd SubM2(3,7);
    MatrixXd pSubM1, pSubM2;

    Id << MatrixXd::Identity(7,7);
    SubM1 << J.block<3,7>(0,0);
    SubM2 << J.block<3,7>(3,0);
    pSubM1 = pseudoInverse(SubM1);
    pSubM2 = pseudoInverse(SubM2);

    switch(opt) {
    case MANIP::OriginalTranslational:
        //Original 1985 Yoshikawa
        manip = sqrt( ( SubM1 * SubM1.transpose() ).determinant() );
        break;
    case MANIP::AugmentedTranslational:
        //Augmented 1990 Yoshikawa
        manip = sqrt( ( SubM1 * ( Id - pSubM2*SubM2 ) * SubM1.transpose() ).determinant() ); //translational strong sense
        break;
    case MANIP::OriginalRotational:
        //Original 1985 Yoshikawa
        manip = sqrt( ( SubM2 * SubM2.transpose() ).determinant() );
        break;
    case MANIP::AugmentedRotational:
        //Augmented 1990 Yoshikawa
        manip = sqrt( ( SubM2 * ( Id - pSubM1*SubM1 ) * SubM2.transpose() ).determinant() ); //translational strong sense
        break;
    default:
        throw "invalid manipulability option";
    }

    return manip;
}

bool KinematicSolver::BatchNullspaceInverseKinematics(const Matrix4d &pose, unsigned int gc, const std::vector<double> &psi_vec, std::vector<std::vector<double> > &joints)
{
    double j4;
    vector<vector<double> > coeffs;
    size_t size = psi_vec.size();
    joints.resize(size);

    for(size_t i=0; i<size; ++i) {
        if(CoeffCalculation(pose,gc,coeffs,j4)) {
            InverseKinematics(coeffs, j4, gc, psi_vec[i], joints[i]);
        }
        else {
            return false; //Trajectory goes beyond manipulator reach
        }
    }
    return true; // success
}

bool KinematicSolver::BatchPsiIntervals(const std::vector<Frame> &poses, const unsigned int gc, vector<interval_set<double> > &intrvls) {
    double elbow_jnt;
    vector<vector<double> > coeffs;

    intrvls.resize(poses.size());
    interval_set<double> intrvls_all;

    for(size_t i=0; i<poses.size(); ++i) {
        // Calculate intervals
        if(CoeffCalculation(kdlframe2eigen(poses[i]), gc, coeffs, elbow_jnt)) {
            PsiIntervals(coeffs,gc,intrvls[i]);
        }
        else {
            return false;
        }
    }
    return true;
}

// INTERNAL
bool KinematicSolver::ReferencePlane(const Matrix4d &pose, const int elbow, Vector3d &ref_plane, Matrix3d &rot_elbow) {
    vector<double> joints(7, 0.0);
    Vector3d xend = pose.block<3,1>(0,3);
    Vector3d xs0(0.0, 0.0, m_lnk[0]);
    Vector3d xwt(0.0, 0.0, m_lnk[3]);
    Vector3d xw0 = xend - pose.block<3,3>(0,0) * xwt;
    Vector3d xsw = xw0 - xs0;
    double dsw = xsw.norm();
    double dse = m_lnk[1];
    double dew = m_lnk[2];

    // Check if position is reachable, within workspace.
    if(abs((pow(dsw,2) - pow(dse,2) - pow(dew,2)) - (2*dse*dew)) < m_tol) { // "Pose outside workspace!"
        return false;
    }
    // -- Joint 4 --
    // Elbow joint can be directly calculated since it does only depend on the
    // robot configuration and the xsw vector
    // Calculated with the Law of Cosines
    joints[3] = elbow * acos((pow(dsw,2) - pow(dse,2) - pow(dew,2))/(2*dse*dew));

    // -- Joint 1 --
    // Since joint3 is locked as 0, the only joint to define the orientation of
    // the xsw vector in the xy-plane is joint 1. Therefore and since we are
    // only interested in the transformation T03 (disregarding joint limits), we
    // chose to simply set joint 1 as the atan of xsw y and x coordinates
    // (even if if goes beyond the joint limit).

    // If x and y are 0, joint1 it is not defined.
    // Joint1 = 0; but should study possibility to being another value.
    Vector3d zz(0,0,1);
    if((xsw.cross(zz)).norm() > m_tol) {
        joints[0] = atan2(xsw(1),xsw(0));
    }
    else {
        joints[0] = 0;
    }

    // -- Joint 2 --
    // Can be found through geometric relations
    // Let phi be the angle E-S-W, and theta2 the angle (z-axis)-S-E.
    // Then, theta2 = atan2(r,xsw(3)) -/+ phi.
    // phi can be calculated, as a function of theta3:
    //   atan2(dew*sin(theta4),dse+dew*cos(theta4))
    // z-axis
    //   ^
    //   |  E O------------O W
    //   |   /        .
    //   |  /      .
    //   | /    .    xsw
    //   |/  .
    // S O___________________ r-axis
    //
    double r = sqrt(pow(xsw(0),2) + pow(xsw(1),2));
    double phi = acos((pow(dse,2)+pow(dsw,2)-pow(dew,2))/(2*dse*dsw));

    joints[1] = atan2(r, xsw(2)) + elbow * phi;

    // -- Joint 3 --
    // Is 0 by definition (ref plane)
    joints[2] = 0.0;

    vector<Matrix4d> Tvir(3, Matrix4d::Identity());
    for(size_t i=0; i<3; ++i) {
        if(i==0) {
            Tvir[0] = denavit_hartenberg(m_dh[0][0],m_dh[0][1],m_dh[0][2],m_dh[0][3] + joints[0]);
        }
        else {
            Tvir[i] = Tvir[i-1] * denavit_hartenberg(m_dh[i][0],m_dh[i][1],m_dh[i][2], m_dh[i][3] + joints[i]);
        }
    }

    // With T03 we can calculate the reference elbow position and with it the
    // vector normal to the reference plane.
    Vector3d v1 = (Tvir.back().block<3,1>(0,3) - xs0); // unit vector from shoulder to elbow
    Vector3d v2 = xw0 - xs0;
    v1.normalize(); // unit vector from shoulder to elbow
    v2.normalize(); // unit vector from shoulder to wrist

    ref_plane = v1.cross(v2);
    rot_elbow = Tvir[2].block<3,3>(0,0);
    return true;
}

bool KinematicSolver::CoeffCalculation(const Matrix4d &pose, const unsigned int gc, vector<vector<double> > &coeffs, double &j4) {
    // Global configuration parameter
    GlobalConfiguration GC(gc);

    Vector3d xend = pose.block<3,1>(0,3);
    Vector3d xs0(0.0, 0.0, m_lnk[0]);
    Vector3d xwt(0.0, 0.0, m_lnk[3]);
    Vector3d xw0 = xend - pose.block<3,3>(0,0) * xwt;
    Vector3d xsw = xw0 - xs0;
    double dsw = xsw.norm();
    double dse = m_lnk[1];
    double dew = m_lnk[2];

    // Check if position is reachable, within workspace.
    if(dse + dew - dsw < m_tol || abs(dse - dew) - dsw > m_tol) { // "Pose outside workspace"
        return false;
    }

    // -- Joint 4 --
    // Calculated with the Law of Cosines
    j4 = GC.elbow() * acos((pow(dsw,2) - pow(dse,2) - pow(dew,2))/(2*dse*dew));

    if(abs(j4) > m_lim[3]) {
        return false;
    }


    Vector3d ref_plane;
    Matrix3d rot_elbow;
    ReferencePlane(pose, GC.elbow(), ref_plane, rot_elbow);

    Matrix4d T34 = denavit_hartenberg(m_dh[3][0],m_dh[3][1],m_dh[3][2], m_dh[3][3] + j4);

    // Skew symmetric matrix of normalized xsw
    xsw.normalize();
    Matrix3d skew_xsw = skew_symmetric(xsw);

    // Auxiliary matrices
    Matrix3d As, Bs, Cs;
    As = skew_xsw * rot_elbow;
    Bs = -(skew_xsw * skew_xsw) * rot_elbow;
    Cs = (xsw * xsw.transpose()) * rot_elbow;

    Matrix3d R34 = T34.block<3,3>(0,0);
    Matrix3d Aw, Bw, Cw;
    Aw = R34.transpose() * As.transpose() * pose.block<3,3>(0,0);
    Bw = R34.transpose() * Bs.transpose() * pose.block<3,3>(0,0);
    Cw = R34.transpose() * Cs.transpose() * pose.block<3,3>(0,0);

    // Coefficients for this arm
    coeffs.clear();
    // theta1 coeff: as22, as12, bs22, bs12, cs22, cs12
    // theta2 coeff: as32, bs32, cs32
    // theta3 coeff: -as33, -as31, -bs33, -bs31, -cs33, -cs31
    // theta5 coeff: aw23, aw13, bw23, bw13, cw23, cw13
    // theta6 coeff: aw33, bw33, cw33
    // theta7 coeff: aw32, -aw31, bw32, -bw31, cw32, -cw31
    coeffs.push_back({ GC.arm()* As(1,1), GC.arm()* As(0,1), GC.arm()* Bs(1,1), GC.arm()* Bs(0,1), GC.arm()* Cs(1,1), GC.arm()* Cs(0,1)});
    coeffs.push_back({ As(2,1),  Bs(2,1),  Cs(2,1)});
    coeffs.push_back({ GC.arm()*-As(2,2), GC.arm()*-As(2,0), GC.arm()*-Bs(2,2), GC.arm()*-Bs(2,0), GC.arm()*-Cs(2,2), GC.arm()*-Cs(2,0)});
    coeffs.push_back({ GC.wrist()*Aw(1,2), GC.wrist()* Aw(0,2), GC.wrist()*Bw(1,2), GC.wrist()* Bw(0,2), GC.wrist()*Cw(1,2), GC.wrist()* Cw(0,2)});
    coeffs.push_back({ Aw(2,2),  Bw(2,2),  Cw(2,2)});
    coeffs.push_back({ GC.wrist()*Aw(2,1), GC.wrist()*-Aw(2,0), GC.wrist()*Bw(2,1), GC.wrist()*-Bw(2,0), GC.wrist()*Cw(2,1), GC.wrist()*-Cw(2,0)});

    return true;
}

void KinematicSolver::InverseKinematics(const vector<vector<double> > &coeffs, const double j4, const unsigned int gc, const double psi, vector<double> &joints) {
    //Initialize joints
    joints.assign(7,0.0);
    joints[3] = j4;

    // Global configuration parameter
    GlobalConfiguration GC(gc);

    joints[0] = atan2(coeffs[0][0]*sin(psi) + coeffs[0][2]*cos(psi) + coeffs[0][4],
                      coeffs[0][1]*sin(psi) + coeffs[0][3]*cos(psi) + coeffs[0][5]);
    joints[1] = GC.arm() * acos((coeffs[1][0]*sin(psi) + coeffs[1][1]*cos(psi) + coeffs[1][2]));
    joints[2] = atan2(coeffs[2][0]*sin(psi) + coeffs[2][2]*cos(psi) + coeffs[2][4],
                      coeffs[2][1]*sin(psi) + coeffs[2][3]*cos(psi) + coeffs[2][5]);

    joints[4] = atan2(coeffs[3][0]*sin(psi) + coeffs[3][2]*cos(psi) + coeffs[3][4],
                      coeffs[3][1]*sin(psi) + coeffs[3][3]*cos(psi) + coeffs[3][5]);
    joints[5] = GC.wrist() * acos((coeffs[4][0]*sin(psi) + coeffs[4][1]*cos(psi) + coeffs[4][2]));
    joints[6] = atan2(coeffs[5][0]*sin(psi) + coeffs[5][2]*cos(psi) + coeffs[5][4],
            coeffs[5][1]*sin(psi) + coeffs[5][3]*cos(psi) + coeffs[5][5]);
}

//////////////////////////////////////////////  NULLSPACE CALCULATION  //////////////////////////////////////////////
void KinematicSolver::PsiFunction(const vector<double> &coeff, const double theta, vector<double> &psi) {
    // Parameters
    double ac, bc, cc;
    // Check if function is solved for pivot or hinge joint by the size of the coeff vector.
    if(coeff.size() == 6) { // pivot joints
        double tan_t = tan(theta);
        ac = tan_t * (coeff[5]-coeff[3]) + (coeff[2]-coeff[4]); // ac = tan_t*(c(2)-b(2)) + (b(1)-c(1));
        bc = tan_t * (2*coeff[1]) - (2*coeff[0]);               // bc = tan_t*(2*a(2)) - (2*a(1));
        cc = tan_t * (coeff[3]+coeff[5]) - (coeff[2]+coeff[4]); // cc = tan_t*(b(2)+c(2)) - (b(1)+c(1));
    }
    else { // hinge joints, coeff.size() == 3
        double cos_t = cos(theta);
        ac = cos_t + coeff[1] - coeff[2]; // ac = cos_t + b - c;
        bc = -2 * coeff[0];               // bc = -2*a;
        cc = cos_t - coeff[1] - coeff[2]; // cc = cos_t - b - c;
    }

    //Verification
    if(pow(bc,2) - 4*ac*cc >= 0) {
        double sol1 = 2*atan(-(bc - (sqrt(pow(bc,2) - 4*ac*cc)))/(2*ac));
        double sol2 = 2*atan(-(bc + (sqrt(pow(bc,2) - 4*ac*cc)))/(2*ac));

        if(CheckSolution(coeff, theta, sol1)) {
            psi.push_back(sol1);
        }
        if(CheckSolution(coeff, theta, sol2)) {
            psi.push_back(sol2);
        }
    }
}

void KinematicSolver::PivotLimits(const vector<double> &coeff, const double limit, vector<double> &intervals) {
    vector<double> sing_intervals;

    double at = coeff[3] * coeff[4] - coeff[2] * coeff[5]; // at = bd*cn - bn*cd;
    double bt = coeff[0] * coeff[5] - coeff[1] * coeff[4]; // bt = an*cd - ad*cn;
    double ct = coeff[0] * coeff[3] - coeff[1] * coeff[2]; // ct = an*bd - ad*bn;

    // Check singular points
    double proximity = abs(pow(at,2) + pow(bt,2) - pow(ct,2));
    if(proximity < m_detect_proxim) {
        double t = std::abs(proximity - m_detect_proxim) / m_detect_proxim;
        // Courtesy of Alec Jacobson - http://www.alecjacobson.com/weblog/?p=2293
        double branch = (10*pow(t,3) - 15*pow(t,4) + 6*pow(t,5)) * m_sing_interval;

        double psi_sing = 2*atan(at/(bt - ct));
        if( (psi_sing + branch) > M_PI) {
            sing_intervals.push_back(-M_PI);
            sing_intervals.push_back(psi_sing - branch);
        } else if( (psi_sing - branch) < -M_PI) {
            sing_intervals.push_back(psi_sing + branch);
            sing_intervals.push_back(M_PI);
        } else {
            sing_intervals.push_back(-M_PI);
            sing_intervals.push_back(psi_sing - branch);
            sing_intervals.push_back(psi_sing + branch);
            sing_intervals.push_back(M_PI);
        }
    }

    // Stationary points not always map the theta max or min. Specially when we
    // have configurations with negative shoulder, elbow or wrist values, we
    // tend to have discontinuities in the function theta(psi) eq.23.
    // These discontinuities are not contemplated in Shimizu paper, due to the
    // values reported and the joint limits of his setup.
    // These discontinuities are not singularities, but are related to the
    // domain of tan. Thus, when a monotonic function reaches -pi or pi a
    // discontinuity happens, and the theta value shifts 2*pi or -2*pi.
    // Since it crosses the joint limit values, they create a new interval where
    // psi values lead to joint limit violation.
    vector<double> psi_lim;
    PsiFunction(coeff, -limit, psi_lim); // push_back to psi_lim
    PsiFunction(coeff, limit, psi_lim);  // push_back to psi_lim

    // Check if interval is empty
    if(!psi_lim.empty()) {
        // Standard sort algorithm (<)
        sort(psi_lim.begin(), psi_lim.end());
        // Should always have a pair size
        assert(psi_lim.size()%2==0 && "Odd number of interval limits.");
        // Classify if limit points are entering avoid (1) or feasible zone (0)
        vector<bool> lim_class;
        for(double lim : psi_lim) {
            // theta value either -jl or jl
            double tlim = atan2((coeff[0]*sin(lim) + coeff[2]*cos(lim) + coeff[4]),
                                (coeff[1]*sin(lim) + coeff[3]*cos(lim) + coeff[5]));
            // derivative value at either -jl or jl
            double dlim = at*sin(lim) + bt*cos(lim) + ct;
            lim_class.push_back(sgn(tlim)==sgn(dlim));
        }
        // lim_class is either [0 1 0 1 ...] or [1 0 1 0 ...]
        // If it starts in an enter avoid, means that it starts in an allowed
        // interval, so we concatenate [-pi ptlim pi] so it always starts with
        // an enter allowed
        if(lim_class.front() == true) {
            psi_lim.insert(psi_lim.begin(), -M_PI);
            psi_lim.push_back(M_PI);
        }
        intervals = psi_lim;
    }
    else {
        // If no psi values match the joint limits border: either no solutions
        // are allowed or all solutions are allowed.
        // So we just need to check the value of any point contained in the
        // function to know if the codomain is within or outside joint limits
        double psi = 0.5;
        // theta value either -jl or jl
        double tlim = atan2((coeff[0]*sin(psi) + coeff[2]*cos(psi) + coeff[4]),
                            (coeff[1]*sin(psi) + coeff[3]*cos(psi) + coeff[5]));
        if(tlim > -limit && tlim < limit) {
            // All interval is possible
            intervals.resize(2);
            intervals[0] = -M_PI;
            intervals[1] = M_PI;
        }
        else {
            // No interval possible
            // intervals -> empty
        }
    }
    // Add singularity intervals
    if(!sing_intervals.empty()) {
        vector<vector<double> > intervals_set;
        interval_set<double> icl_interval;
        intervals_set.push_back(intervals);
        intervals_set.push_back(sing_intervals);
        IntersectIntervals(intervals_set, icl_interval);
        intervals = iclinterval2vector(icl_interval);
    }
}

void KinematicSolver::HingeLimits(const vector<double> &coeff, const int conf, const double limit, vector<double> &intervals) {
    vector<double> psi_lim;
    PsiFunction(coeff, -limit, psi_lim); // push_back to psi_lim
    PsiFunction(coeff, limit, psi_lim);  // push_back to psi_lim

    // Check if interval is empty
    if(!psi_lim.empty()) {
        // Standard sort algorithm (<)
        sort(psi_lim.begin(), psi_lim.end());
        // Should always have a pair size
        assert(psi_lim.size()%2==0 && "Odd number of interval limits.");
        // Classify if limit points are entering avoid (1) or feasible zone (0)
        vector<bool> lim_class;
        for(double lim : psi_lim) {
            // theta value either -jl or jl
            double tlim = conf * acos(coeff[0]*sin(lim) + coeff[1]*cos(lim) + coeff[2]);
            //derivative of tlim is
            // acos'(f(x)) = (conf) * [ - f'(x) / sqrt(1-x²) ]
            // derivative value at either -jl or jl
            double x = coeff[0]*sin(lim) + coeff[1]*cos(lim) + coeff[2];
            double dx = coeff[0]*cos(lim)-coeff[1]*sin(lim);
            double dlim = conf * -dx / sqrt(1 - pow(x,2));
            lim_class.push_back(sgn(tlim)==sgn(dlim));
        }
        // lim_class is either [0 1 0 1 ...] or [1 0 1 0 ...]
        // If it starts in an enter avoid, means that it starts in an allowed
        // interval, so we concatenate [-pi ptlim pi] so it always starts with
        // an enter allowed
        if(lim_class.front() == true) {
            psi_lim.insert(psi_lim.begin(), -M_PI);
            psi_lim.push_back(M_PI);
        }
        intervals = psi_lim;
    }
    else {
        // If no psi values match the joint limits border: either no solutions
        // are allowed or all solutions are allowed.
        // So we just need to check the value of any point contained in the
        // function to know if the codomain is within or outside joint limits
        double psi = 0.5;
        // theta value either -jl or jl
        double tlim = conf * acos(coeff[0]*sin(psi) + coeff[1]*cos(psi) + coeff[2]);
        if(tlim > -limit && tlim < limit) {
            // All interval is possible
            intervals.resize(2);
            intervals[0] = -M_PI;
            intervals[1] = M_PI;
        }
        else {
            // No interval possible
            // intervals -> empty
        }
    }
}

void KinematicSolver::PsiIntervals(const std::vector<std::vector<double> > &coeffs, const unsigned int gc,
                                   interval_set<double> &feasible) {
    // Configuration
    GlobalConfiguration GC(gc);

    vector<vector< double> > intervals(6);

    PivotLimits(coeffs[0], m_lim[0], intervals[0]);             // Joint 1
    HingeLimits(coeffs[1], GC.arm(), m_lim[1], intervals[1]);   // Joint 2
    PivotLimits(coeffs[2], m_lim[2], intervals[2]);             // Joint 3

    PivotLimits(coeffs[3], m_lim[4], intervals[3]);             // Joint 5
    HingeLimits(coeffs[4], GC.wrist(), m_lim[5], intervals[4]); // Joint 6
    PivotLimits(coeffs[5], m_lim[6], intervals[5]);             // Joint 7

    IntersectIntervals(intervals, feasible);
}

bool KinematicSolver::CheckSolution(const std::vector<double> &coeff, const double jl, const double psi) {
    bool verification = false;
    //Check solutions
    if(coeff.size() == 6) { // pivot joints
        double theta = atan2((coeff[0]*sin(psi) + coeff[2]*cos(psi) + coeff[4]),(coeff[1]*sin(psi) + coeff[3]*cos(psi) + coeff[5]));
        verification = ( abs(jl - theta) < m_tol );
    }
    else if(coeff.size() == 3) { //hinge joints
        double theta = acos(coeff[0]*sin(psi) + coeff[1]*cos(psi) + coeff[2]);
        verification = ( abs(jl - theta) < m_tol );
    }
    return verification;
}

/////////////////////////////////////////////////  SOLVE NULLSPACE  /////////////////////////////////////////////////
bool KinematicSolver::CheckNullspace(const std::vector<Frame> &target_frames, std::vector<unsigned int> &gc_order,
                                         std::vector<std::vector<double> > &initial_psi)
{
    for(size_t rc=0; rc<8; ++rc) {
        vector<interval_set<double> > intrvls, shadow;
        if(BatchPsiIntervals(target_frames, rc, intrvls)) {
            if(!ShadowIntervals(intrvls, m_max_change_ns, shadow)) {
                continue;
            }
            else {
                gc_order.push_back(rc);
                initial_psi.push_back(iclinterval2vector(shadow.front()));
            }
        }
        else {
            // Pose outside workspace
            continue;
        }
    }
    return (!gc_order.empty()); //returns false if there is no solution for any GC
}

bool KinematicSolver::CheckConfigurationNullspace(const std::vector<Frame> &target_frames, const int conf, std::vector<double> &initial_psi)
{
    vector<interval_set<double> > intrvls, shadow;
    if(BatchPsiIntervals(target_frames, conf, intrvls)) {
        if(ShadowIntervals(intrvls, m_max_change_ns, shadow)) {
            initial_psi = iclinterval2vector(shadow.front());
            return true;
        }
    }
    //TODO: Explain why
    return false;
}

void KinematicSolver::ArmAngleScore(const interval_set<double> &interval, const double psi, std::vector<double> &ns_scores)
{
    // Calculate scores of intervals
    ns_scores.assign(360, -1.0);
    vector<discrete_interval<int> > dis_int;
    double psi_deg = psi*180/M_PI;
    double sing_interval_deg = m_sing_interval*180/M_PI;
    // Convert to discrete deg intervals
    for(auto &it : interval) {
        double up_deg = it.upper()*180/M_PI;
        double lw_deg = it.lower()*180/M_PI;
        for(int val = ceil(lw_deg); val < floor(up_deg); ++val) {
            /// Part of the score that relates to the distance to the current psi
            ns_scores[val+180] = abs(val - round(psi_deg));
            /// Part of the score that relates to the distance to limits
            double spc = 180;//up_deg - lw_deg;
            double term1 = m_K*(spc/2);
            double term2 = exp(-m_alpha*((val - lw_deg)/spc)) + exp(-m_alpha*((up_deg - val)/spc));
            ns_scores[val+180] += abs(term1*term2);
        }
    }
}

bool KinematicSolver::IterationNewPsi(const interval_set<double> &feasible, const double cur_psi, double &new_psi) {
    if(contains(feasible, cur_psi)) { //cur_psi contained in feasible intervals
        //find interval containing cur_psi
        auto it = feasible.find(cur_psi);
        double K = m_K; // control strenght of repulsion [0,1]
        double alpha = m_alpha; // constant controls distance to limits of starting the repulsion

        double spc = it->upper() - it->lower();
        double term1 = K*(spc/2);
        double term2 = exp(-alpha*((cur_psi - it->lower())/spc)) - exp(-alpha*((it->upper() - cur_psi)/spc));

        /// Happens that when the current psi value is too close to the limit, the new value overshoots.
        /// To control this, the new value is truncated to a maximum specified value.
        int sign = sgn(term1*term2); //sign of psi variation
        new_psi = cur_psi + sign * std::min(abs(term1*term2), m_max_change_ns);
    }
    else {
        bool possible = false;
        double ups = cur_psi + m_max_change_ns;
        double lws = cur_psi - m_max_change_ns;
        double altup, altlw;
        double distup = 2*M_PI;
        double distlw = 2*M_PI;
        if(contains(feasible, ups)) {
            possible = true;

            auto itu = feasible.find(ups);
            // Alternative up is equal to the mid-point of the interval containing
            // the lower interval bound of the next interval to the current psi
            // and the max upper psi.
            altup = ((ups - itu->lower()) / 2) + itu->lower();
            distup = altup - cur_psi;
        }
        if(contains(feasible, lws)) {
            possible = true;

            auto itl = feasible.find(lws);
            // Alternative low is equal to the mid-point of the interval containing
            // the upper interval bound of the previous interval to the current psi
            // and the min lower psi.
            altlw = ((itl->upper() - lws) / 2) + lws;
            distlw = cur_psi - altlw;
        }
        if(possible) {
            new_psi = (distlw < distup ? altlw : altup);
        }
        else {
            //Current PSI not in a feasible interval.
            return false;
        }
    }
    return true;
}

void KinematicSolver::GlobConfOrder(const unsigned int gc, std::vector<unsigned int> &conf) {
    conf.resize(8);
    conf[0] = gc;
    //Check wrist, elbow and shoulder position

    bool w0 = gc & 4;
    bool e0 = gc & 2;
    bool s0 = gc & 1;

    auto new_gc = [](bool w, bool e, bool s) {
        return (w*4 + e*2 + s*1);};

    //Follows the logic of trying neighbour configurations that imply least movements in the kinematic chain
    //gc_order[0] = new_gc( w0, e0, s0);
    conf[1] = new_gc(!w0, e0, s0);
    conf[2] = new_gc( w0,!e0, s0);
    conf[3] = new_gc(!w0,!e0, s0);
    conf[4] = new_gc( w0, e0,!s0);
    conf[5] = new_gc(!w0, e0,!s0);
    conf[6] = new_gc( w0,!e0,!s0);
    conf[7] = new_gc(!w0,!e0,!s0);
}

void KinematicSolver::GlobConfOrder(const int gc, std::vector<int> &possible, std::vector<int> &gc_index, std::vector<int> &conf) {
    bool w0 = gc & 4;
    bool e0 = gc & 2;
    bool s0 = gc & 1;

    auto new_gc = [](bool w, bool e, bool s) {
        return (w*4 + e*2 + s*1);};

    vector<int> all_gc(8);
    all_gc[0] = new_gc( w0, e0, s0);
    all_gc[1] = new_gc(!w0, e0, s0);
    all_gc[2] = new_gc( w0,!e0, s0);
    all_gc[3] = new_gc(!w0,!e0, s0);
    all_gc[4] = new_gc( w0, e0,!s0);
    all_gc[5] = new_gc(!w0, e0,!s0);
    all_gc[6] = new_gc( w0,!e0,!s0);
    all_gc[7] = new_gc(!w0,!e0,!s0);

    //Follows the logic of trying neighbour configurations that imply least movements in the kinematic chain
    for(size_t gc=0; gc<8; ++gc) {
        if(std::find(possible.begin(), possible.end(), all_gc[gc]) != possible.end()) {
            conf.push_back(all_gc[gc]);
        }
    }

    // Initializes vector with incremental values.
    gc_index.resize(conf.size());

    // Goes through all conf elements
    // Finds the element conf [i] in the vector possible and return index.
    for(size_t i=0; i<conf.size(); ++i) {
        auto it = std::find_if(possible.begin(), possible.end(), [&](const int val){
            return (val == conf[i]);
        });

        gc_index[i] = std::distance(possible.begin(), it);
    }
}

bool KinematicSolver::ShadowIntervals(const std::vector<boost::icl::interval_set<double> > &intervals, const double max_nschange,
                                     std::vector<boost::icl::interval_set<double> > &shadow) {
    if(intervals.size() != 0) {
        shadow.resize(intervals.size());
        shadow.back() = intervals.back();
        for(size_t i=intervals.size()-1; i>0; --i) {
            interval_set<double>
                    shadow_interval, //combination of previous iteration shadow intervals
                    combined_intervals; //combination of current iteration intervals
            for(auto sit : shadow[i]) {
                double lo = sit.lower();
                double up = sit.upper();
                //If interval j first value is not -PI reduce by max_nschange  -->|##
                if(std::abs(lo - (-M_PI)) > m_tol) {
                    lo = std::max(lo - max_nschange, -M_PI); //saturate at -pi
                    lo = std::min(lo, up); //avoid invertions
                }
                //If interval j second value is not PI reduce by max_nschange  ##|<--
                if(std::abs(up - M_PI) > m_tol) {
                    up = std::min(up + max_nschange, M_PI); //saturate at pi
                    up = std::max(up, lo); //avoid invertions
                }
                shadow_interval += continuous_interval<double>::closed(lo,up);
            }
            for(auto iit : intervals[i-1]) {
                combined_intervals += continuous_interval<double>::closed(iit.lower(),iit.upper());
            }
            //New shadow is the intersection of past shadow and current intervals
            shadow[i-1] = (shadow_interval & combined_intervals);
            //Check if there is any possible interval
            if(shadow[i-1].empty()) {
                return false;
            }
        }
    }
    return true;
}

/////////////////////////////////////////////  PSI INTERVAL MANIPULATION  /////////////////////////////////////////////
void KinematicSolver::IntersectIntervals(const std::vector<std::vector<double> > &intervals, interval_set<double> &interval_out) {
    typedef interval_set<double> TIntervalSet;
    vector<TIntervalSet> intSet(intervals.size());
    vector<TIntervalSet> intSetMem(intervals.size()+1);
    intSetMem[0] += continuous_interval<double>::open(-M_PI, M_PI);

    // Through all intervals
    for(size_t i=0; i<intervals.size(); ++i) {
        // Convert intervals to interval_sets
        for(size_t j=0; j<intervals[i].size(); j=j+2) {
            intSet[i] += continuous_interval<double>::open(intervals[i][j], intervals[i][j+1]);
        }
        intSetMem[i+1] += (intSetMem[i] & intSet[i]);
    }

    interval_out = intSetMem.back();
}

void KinematicSolver::IntersectIntervals(const std::vector<interval_set<double> > &intervals, interval_set<double> &interval_out) {
    typedef interval_set<double> TIntervalSet;
    vector<TIntervalSet> intSetMem(intervals.size()+1);
    intSetMem[0] += continuous_interval<double>::open(-M_PI, M_PI);

    // Through all intervals
    for(size_t i=0; i<intervals.size(); ++i) {
        intSetMem[i+1] += (intSetMem[i] & intervals[i]);
    }

    interval_out = intSetMem.back();
}

/////////////////////////////////////////////  AUXILIARY AND CONVERTIONS  /////////////////////////////////////////////
Matrix3d KinematicSolver::skew_symmetric(const Vector3d &vec) {
    Matrix3d rot = Matrix3d::Zero();
    rot(0,1) = -vec(2);
    rot(0,2) =  vec(1);
    rot(1,0) =  vec(2);
    rot(1,2) = -vec(0);
    rot(2,0) = -vec(1);
    rot(2,1) =  vec(0);

    return rot;
}

Matrix4d KinematicSolver::denavit_hartenberg(const double a, const double alpha, const double d, const double theta) {
    Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a*cos(theta),
         sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a*sin(theta),
                0.0,               sin(alpha),               cos(alpha),            d,
                0.0,                      0.0,                      0.0,          1.0;

    return T;
}

KDL::Frame KinematicSolver::eigen2kdlframe(const Matrix4d &mat) {
    KDL::Frame T;
    Matrix3d mat_rot = mat.block<3,3>(0,0).transpose();
    std::copy(mat_rot.data(), mat_rot.data()+9, T.M.data);
    T.p = KDL::Vector(mat(0,3), mat(1,3), mat(2,3));
    return T;
}

Matrix4d KinematicSolver::kdlframe2eigen(const Frame &T) {
    Matrix4d mat;
    double rot_data[9];
    std::copy(T.M.data, T.M.data+9, rot_data);
    mat = Matrix4d::Identity();
    mat.block<3,3>(0,0) = Eigen::Map<Matrix3d>(rot_data,3,3).transpose();
    mat.col(3) << T.p.x(), T.p.y(), T.p.z(), 1;
    return mat;
}

std::vector<double> KinematicSolver::iclinterval2vector(const interval_set<double> &interval) {
    std::vector<double> interval_out;
    for(auto &it : interval) {
        interval_out.push_back(it.lower());
        interval_out.push_back(it.upper());
    }
    return interval_out;
}

boost::icl::interval_set<double> KinematicSolver::vector2iclinterval(const std::vector<double> &interval)
{
    boost::icl::interval_set<double> interval_out;
    for(size_t i=1; i<interval.size(); i=i+2) {
        //Avoid closed intervals at -M_PI and M_PI
        if( i==1 && (interval.front() + M_PI < m_tol) && (interval.back() - M_PI < m_tol)) {
            interval_out += continuous_interval<double>::left_open(interval[i-1], interval[i]);
        }
        else {
            interval_out += continuous_interval<double>::closed(interval[i-1], interval[i]);
        }
    }
    return interval_out;
}

void KinematicSolver::printtofile(const char* file, std::vector<double> &vec)
{
    std::ofstream tfile;
    string tpath = file;
    tfile.open(tpath, std::ios::app);
    for(size_t i=0; i<vec.size(); ++i) {
        tfile << vec[i] << ", ";
    }
    tfile << endl;
    tfile.close();
}

