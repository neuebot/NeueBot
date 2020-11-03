#include "registration-component.hpp"

#include "kdl/frames.hpp"
#include <rtt/Component.hpp>
#include <rtt/marsh/Marshalling.hpp>

#include <Eigen/Dense>

using namespace KDL;
using namespace RTT;
using namespace Eigen;
using namespace std;

Registration::Registration(const std::string &component_name) :
    TaskContext(component_name, PreOperational)
{
    //PROPERTIES AND ATTRIBUTES
    this->addProperty("Tolerance", m_tolerance);
    this->addProperty("TestBoxTransf", m_testbox_transf_v);

    this->addAttribute("registration_successful", m_registration_successful);

    //OPERATIONS
    this->addOperation("LoadWorkPiecePoints", &Registration::LoadWorkPiecePoints, this, ClientThread);
    this->addOperation("LoadRegistrationPoints", &Registration::LoadRegistrationPoints, this, ClientThread);
    this->addOperation("ComputeRegistrationMatrix", &Registration::ComputeRegistrationMatrix, this, OwnThread);
    this->addOperation("SaveInterlockRegistrationMatrix", &Registration::SaveInterlockRegistrationMatrix, this, OwnThread);

    this->addOperation("LoadCalibrationFile", &Registration::LoadCalibrationFile, this, OwnThread);
    this->addOperation("SaveCalibrationFile", &Registration::SaveCalibrationFile, this, OwnThread);
    this->addOperation("GetRegistrationMatrix", &Registration::GetRegistrationMatrix, this, OwnThread);

    //PORTS
    this->ports()->addPort("outport_error_msg", outport_error_msg).doc("Output the result value and error codes of the component to the procedure coordinator.");
}

bool Registration::configureHook() {
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *PC = new char[128];
        strcpy(PC, WS);
        strcat(PC, "/Properties/REGISTRATION_PROPERTIES.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(PC)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] PC;
    }
    else {
        log(Error) << "Could not find WORKSPACE environment variable." << endlog();
        return false;
    }

    if(hasPeer("test_supervisor")) {
        TaskContext *ts_ptr = getPeer("test_supervisor");
        CallSetWorkFrame = ts_ptr->getOperation("SetWorkFrame");
        CallSetWorkFrame.setCaller(ts_ptr->engine());
    }

    m_marshaller = KDL::MarshallerCalibration();
    m_registration_matrix = Frame::Identity();

    m_registration_successful = false;

    //Defining end-effector matrices
    m_testbox_transf = Frame(Rotation::Quaternion(m_testbox_transf_v[3], m_testbox_transf_v[4], m_testbox_transf_v[5], m_testbox_transf_v[6]),
            Vector(m_testbox_transf_v[0], m_testbox_transf_v[1], m_testbox_transf_v[2]));

    return true;
}

bool Registration::startHook() {

  return true;
}

void Registration::updateHook() {
}

void Registration::stopHook() {
}

void Registration::cleanupHook() {

}

double Registration::rigid_lms_registration(std::vector<std::vector<double> > &Pa, std::vector<std::vector<double> > &Pb, KDL::Frame &res)
{
    size_t npts = Pa.size(); // same as Pb.size()
    // create Matrix3Xd to store points
    Matrix3Xd
            PaM = Matrix3Xd::Zero(3, npts),
            PbM = Matrix3Xd::Zero(3, npts),
            cross = Matrix3d::Zero(3, 3);
    // fill values
    for(size_t i=0; i<npts; ++i) {
        PaM.col(i) << Pa[i][0], Pa[i][1], Pa[i][2];
        PbM.col(i) << Pb[i][0], Pb[i][1], Pb[i][2];
    }
    // center of mass of point sets
    Vector3d Ma(3), Mb(3);
    for(size_t c=0; c<3; ++c) {
        Ma[c] = PaM.row(c).mean();
        Mb[c] = PbM.row(c).mean();
    }
    // cross correlation function
    for(size_t i=0; i<npts; ++i) {
        cross = cross + ((PaM.col(i) - Ma) * (PbM.col(i) - Mb).transpose());
    }
    cross = 1 / static_cast<double>(npts) * cross;

    // calculate the parameters for the quaternion extraction
    Matrix3d A = cross - cross.transpose();
    Vector3d D(A(1,2), A(2,0), A(0,1));
    Matrix4d Q;
    Q.row(0) << cross.trace(), D.transpose();
    Q.block<3,1>(1,0) = D;
    Q.block<3,3>(1,1) = (cross + cross.transpose() - cross.trace() * Matrix3d::Identity());

    // eigen solver for symmetric matrices
    // https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
    SelfAdjointEigenSolver<Matrix4d> es;
    es.compute(Q,ComputeEigenvectors);
    Matrix4d evec = es.eigenvectors();
    Vector4d eval = es.eigenvalues();

    // Find the maximum eigenval and its position
    // [max_v max_p] = max(abs(diag(E_val)));
    double max = eval(0);
    size_t max_idx = 0;
    for(size_t i=1; i<4; ++i) {
        if(eval(i) > max) {
            max = eval(i);
            max_idx = i;
        }
    }

    // Calculates the quaternion of the maximum eigen value vector
    Vector4d maxEigVec = evec.col(max_idx).normalized();
    Quaterniond quat = Quaterniond(maxEigVec(0), maxEigVec(1), maxEigVec(2), maxEigVec(3));

    Matrix3d rot = quat.toRotationMatrix();
    Matrix3d rot_t = rot.transpose();
    Vector3d pos = (Mb - (rot)*Ma).transpose();

    //Calculate the registration error
    Matrix4d fr = Matrix4d::Identity();
    fr.block<3,3>(0,0) = rot;
    fr.block<3,1>(0,3) = pos;

    double sum_err = 0.0;
    for(size_t i=0; i<npts; ++i) {
        Vector4d p1, p2;
        p1 << PaM(0,i), PaM(1,i), PaM(2,i), 1;
        p2 << PbM(0,i), PbM(1,i), PbM(2,i), 1;
        Vector4d np1 = fr * p1;

        // Sum of Euclidean distances between each point
        sum_err += sqrt( pow(p2(0) - np1(0),2) + pow(p2(1) - np1(1),2) + pow(p2(2) - np1(2),2) );
    }

    //convert to KDL::Frame
    std::copy(rot_t.data(), rot_t.data()+9, res.M.data);
    res.p = KDL::Vector(pos(0), pos(1), pos(2));

    return sum_err;
}

void Registration::LoadWorkPiecePoints(const vector_vector_double &wpoints)
{
    //Load points to internal data
    m_work_points = wpoints.data;

    error_msg emsg;
    emsg.level = LOADWPTS;
    emsg.msg = m_ec.dict[LOADWPTS];
    outport_error_msg.write(emsg);
}

void Registration::LoadRegistrationPoints(const vector_vector_double &rpoints)
{
    //Load points to internal data
    m_regs_points = rpoints.data;

    error_msg emsg;
    emsg.level = LOADRPTS;
    emsg.msg = m_ec.dict[LOADRPTS];
    outport_error_msg.write(emsg);
}

bool Registration::ComputeRegistrationMatrix(Frame &transformation, double &error)
{
    error_msg emsg;
    Frame res;

    //Check there are work piece points available
    if(m_work_points.empty()) {
        emsg.level = NOWRKPTS;
        emsg.msg = m_ec.dict[LOADRPTS];
        outport_error_msg.write(emsg);
        return false;
    }

    //Check there are work piece registered points available
    if(m_regs_points.empty()) {
        emsg.level = NOREGPTS;
        emsg.msg = m_ec.dict[NOREGPTS];
        outport_error_msg.write(emsg);
        return false;
    }

    //Check if both set of points match in size
    if(m_work_points.size() != m_regs_points.size()) {
        emsg.level = MISMATCH;
        emsg.msg = m_ec.dict[MISMATCH];
        outport_error_msg.write(emsg);
        return false;
    }

    //Compute the registration matrix
    error = rigid_lms_registration(m_work_points, m_regs_points, res);
    transformation = res;

    //Checks if resulting registration error is below 3mm
    if(error > 3e-3) {
        emsg.level = REGERROR;
        emsg.msg = m_ec.dict[REGERROR];
        outport_error_msg.write(emsg);
        return false;
    }
    else {
        m_registration_matrix = res;

        emsg.level = COMPLETE;
        emsg.msg = m_ec.dict[COMPLETE];
        outport_error_msg.write(emsg);

        m_registration_successful = true;
    }

    CallSetWorkFrame.call(res);

    return true;
}

bool Registration::SaveInterlockRegistrationMatrix(const Frame &base_tool, Frame &transformation)
{
    // Reset Work and Reg points
    m_work_points.clear();
    m_regs_points.clear();

    // Base Surg Transformation
    transformation = base_tool * m_testbox_transf;
    m_registration_matrix = transformation;

    CallSetWorkFrame.call(transformation);

    m_registration_successful = true;

    return true;
}

bool Registration::LoadCalibrationFile(const string &filename, Frame &base_work, double &error)
{
    //Get values from file
    m_marshaller.LoadCalibrationFile(filename);
    m_marshaller.GetCalibrationParameters(m_work_points, m_regs_points, m_registration_matrix, m_error);

    /// TEST

//    KDL::Vector p2(-0.000044109, -0.000475098, -0.001551884);
//    KDL::Rotation r2(0.999980,0.003304,0.005433,
//                     -0.003317,0.999992,0.002265,
//                     -0.005426,-0.002283,0.999983);

//    KDL::Frame icp_error_matrix(r2, p2);

//    KDL::Rotation r3 = KDL::Rotation::RotY(-atan2(1.5,160));
//    KDL::Frame adjust_matrix(r3, Vector(0.0, 0.0, 0.0));

    KDL::Rotation ry = KDL::Rotation::RotY(-atan2(1.25,160));
    KDL::Rotation rx = KDL::Rotation::RotX(-atan2(0.35,100));
    KDL::Frame adjust_matrix(ry*rx, Vector(0.0, 0.0, -0.00075));

    KDL::Frame transformation = m_registration_matrix * adjust_matrix;

    /// END TEST

    error_msg emsg;
    Frame res;
    //Check there are work piece points available
    if(m_work_points.empty()) {
        emsg.level = NOWRKPTS;
        emsg.msg = m_ec.dict[LOADRPTS];
        outport_error_msg.write(emsg);
    }

    //Check there are work piece registered points available
    if(m_regs_points.empty()) {
        emsg.level = NOREGPTS;
        emsg.msg = m_ec.dict[NOREGPTS];
        outport_error_msg.write(emsg);
    }

    //Check if both set of points match in size
    if(m_work_points.size() != m_regs_points.size()) {
        emsg.level = MISMATCH;
        emsg.msg = m_ec.dict[MISMATCH];
        outport_error_msg.write(emsg);
        return false;
    }

    //Checks if resulting registration error is below 3mm
    if(m_error > 3e-3) {
        emsg.level = REGERROR;
        emsg.msg = m_ec.dict[REGERROR];
        outport_error_msg.write(emsg);
        return false;
    }
    else {
        emsg.level = LOADREGS;
        emsg.msg = m_ec.dict[LOADREGS];
        outport_error_msg.write(emsg);

        m_registration_successful = true;
    }

    CallSetWorkFrame.call(transformation);

    emsg.level = COMPLETE;
    emsg.msg = m_ec.dict[COMPLETE];
    outport_error_msg.write(emsg);

    base_work = transformation;
    error = m_error;

    return true;
}

bool Registration::SaveCalibrationFile(const string &filename)
{
    error_msg emsg;
    Frame res;
    //Check if registration was successful
    if(m_registration_successful) {
        emsg.level = SAVEREGS;
        emsg.msg = m_ec.dict[SAVEREGS];
        outport_error_msg.write(emsg);
    }
    else {
        emsg.level = UNSUCREG;
        emsg.msg = m_ec.dict[UNSUCREG];
        outport_error_msg.write(emsg);
        return false;
    }

    m_marshaller.SetCalibrationParameters(m_work_points, m_regs_points, m_registration_matrix, m_error);
    m_marshaller.SaveCalibrationFile(filename);

    return true;
}

bool Registration::GetRegistrationMatrix(Frame &reg)
{
    bool success = false;
    if(m_registration_successful) {
        reg = m_registration_matrix;
        success = true;
    }
    return success;
}



ORO_CREATE_COMPONENT(Registration)
