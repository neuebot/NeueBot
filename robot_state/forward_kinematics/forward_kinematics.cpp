#include "forward_kinematics.hpp"
#include "global_configuration.hpp"

using namespace KDL;
using namespace Eigen;
using namespace std;

ForwardKinematicSolver::ForwardKinematicSolver(const unsigned int nj, const double tol, const std::vector<double> &links,
                                 const std::vector<double> &limits, const std::vector<std::vector<double> > &dh) :
    m_nj(nj),
    m_tol(tol),
    m_lnk(links),
    m_lim(limits),
    m_dh(dh)
{
}

ForwardKinematicSolver::~ForwardKinematicSolver() {}

void ForwardKinematicSolver::ForwardKinematics(const vector<double> &joints, unsigned int &rconf, double &psi, Frame &pose) {
    rconf = ((int)(joints[1] < 0)) + (2 * ((int)(joints[3] < 0))) + (4 * ((int)(joints[5] < 0)));
    GlobalConfiguration GC(rconf);

    // Calculate FK with joint values
    vector<Matrix4d> tr(m_nj, Matrix4d::Identity());
    for (size_t i = 0; i < m_nj; ++i) {
        if(i==0) {
            tr[0] = denavit_hartenberg(m_dh[0][0],m_dh[0][1],m_dh[0][2],m_dh[0][3]+joints[0]);
        }
        else {
            tr[i] = tr[i-1] * denavit_hartenberg(m_dh[i][0],m_dh[i][1],m_dh[i][2],m_dh[i][3]+joints[i]);
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

    // Calculate the nsparam - Arm Angle
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
    // this vector will give the sign of the nsparam
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

bool ForwardKinematicSolver::ReferencePlane(const Matrix4d &pose, const int elbow, Vector3d &ref_plane, Matrix3d &rot_elbow) {
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
            Tvir[i] = Tvir[i-1] * denavit_hartenberg(m_dh[i][0],m_dh[i][1],m_dh[i][2],m_dh[i][3] + joints[i]);
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

Matrix4d ForwardKinematicSolver::denavit_hartenberg(const double a, const double alpha, const double d, const double theta) {
    Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a*cos(theta),
         sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a*sin(theta),
                0.0,               sin(alpha),               cos(alpha),            d,
                0.0,                      0.0,                      0.0,          1.0;

    return T;
}

KDL::Frame ForwardKinematicSolver::eigen2kdlframe(const Matrix4d &mat) {
    KDL::Frame T;
    Matrix3d mat_rot = mat.block<3,3>(0,0).transpose();
    std::copy(mat_rot.data(), mat_rot.data()+9, T.M.data);
    T.p = KDL::Vector(mat(0,3), mat(1,3), mat(2,3));
    return T;
}
