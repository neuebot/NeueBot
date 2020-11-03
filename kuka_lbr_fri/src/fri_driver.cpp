#include "fri_driver.hpp"

using namespace KUKA::FRI;
using namespace std;

FriDriver::FriDriver() {
    //Hard-coded properties
    conn_prop.robot_type = "KUKA_LBR_IIWA_7_R800";
    conn_prop.local_address = "192.170.10.100";
    conn_prop.local_port = 30200;
    conn_prop.remote_address = "192.170.10.2";
    conn_prop.remote_port = 30200;

    //Connection to FRI
    fri_connection = std::make_shared<UdpConnection>();
    fri_connected = false;

    //Client to FRI
    fri_client = std::make_shared<FRIJointOverlayClient>();

    //Application that puts together the connection and client
    fri_client_application = std::make_shared<ClientApplication>(*fri_connection.get(), *fri_client.get());
}

bool FriDriver::FriConnect() {
    fri_connected = fri_client_application->connect(conn_prop.remote_port, conn_prop.remote_address.c_str());
    return fri_connected;
}

void FriDriver::FriDisconnect() {
    fri_client_application->disconnect();
    fri_connected = false;
}

// Don't have concurrency, the fri component Execution engine
// makes the call to the step which synchronizes the data with the component
bool FriDriver::FriStep() {
    //Execute a step and exchange messages
    if(fri_client_application->step()) {
        ///READ JOINT POSITIONS
        const LBRState& last_state = fri_client->robotState();
        // JOINT POSITIONS
        // Measured
        std::vector<double> mjq(last_state.getMeasuredJointPosition(), last_state.getMeasuredJointPosition()+NJ);
        pvq_mjpos.produce(mjq);
        // Commanded
        std::vector<double> tjq(last_state.getCommandedJointPosition(), last_state.getCommandedJointPosition()+NJ);
        pvq_cjpos.produce(tjq);

        // JOINT TORQUES
        // Measured
        std::vector<double> mjt(last_state.getMeasuredTorque(), last_state.getMeasuredTorque()+NJ);
        pvq_mjtor.produce(mjt);
        // External
        std::vector<double> ejt(last_state.getExternalTorque(), last_state.getExternalTorque()+NJ);
        pvq_ejtor.produce(ejt);

        // Copy enums to internal managed kuka_state variable
        CopyState(last_state);
        //Times
        time_stamp_sec = last_state.getTimestampSec();
        time_stamp_nano = last_state.getTimestampNanoSec();
        sample_time = last_state.getSampleTime();
        //Perfomance measure
        performance = last_state.getTrackingPerformance();
    }
}

bool FriDriver::isFriConnected() {
    return fri_connected;
}

void FriDriver::SetTargetJointPositions(const std::vector<double> &tar) {
    //Vector queue object stored in client
    fri_client->SetTargetJointPositions(tar);
}

bool FriDriver::GetCurrentJointPositions(std::vector<double> &cur) {
    if(!pvq_mjpos.empty()) {
        cur = pvq_mjpos.consume();
        return true;
    }
    return false;
}

bool FriDriver::GetInterpolatedJointPositions(std::vector<double> &ipo) {
    if(!pvq_ijpos.empty()) {
        ipo = pvq_ijpos.consume();
        return true;
    }
    return false;
}

bool FriDriver::GetCommandedJointPositions(std::vector<double> &com) {
    if(!pvq_cjpos.empty()) {
        com = pvq_cjpos.consume();
        return true;
    }
    return false;
}

bool FriDriver::GetCurrentJointTorques(std::vector<double> &curt) {
    if(!pvq_mjtor.empty()) {
        curt = pvq_mjtor.consume();
        return true;
    }
    return false;
}

bool FriDriver::GetExternalJointTorques(std::vector<double> &extt) {
    if(!pvq_ejtor.empty()) {
        extt = pvq_ejtor.consume();
        return true;
    }
    return false;
}

int FriDriver::getConnectionQuality()
{
    return static_cast<int>(kuka_state.connection_quality);
}

int FriDriver::getSessionState() {
    return static_cast<int>(kuka_state.session_state);
}

double FriDriver::getTrackingPerformance()
{
    return performance;
}

void FriDriver::CopyState(const LBRState &state) {
   kuka_state.client_mode = state.getClientCommandMode();
   kuka_state.connection_quality = state.getConnectionQuality();
   kuka_state.control_mode = state.getControlMode();
   kuka_state.drive_state = state.getDriveState();
   kuka_state.operation_mode = state.getOperationMode();
   kuka_state.overlay_type = state.getOverlayType();
   kuka_state.safety_state = state.getSafetyState();
   kuka_state.session_state = state.getSessionState();
}
