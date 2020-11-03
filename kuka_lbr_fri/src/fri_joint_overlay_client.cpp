#include "fri_joint_overlay_client.hpp"

#include <cmath>
#include <iostream>

using namespace KUKA::FRI;
using namespace std;

static double MAX_DQ = 0.01; // 0.01 / 0.005 = 2 rad/s

FRIJointOverlayClient::FRIJointOverlayClient() : LBRClient(),
    safe(true)
{
}

FRIJointOverlayClient::~FRIJointOverlayClient()
{
}

void FRIJointOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
    //STATE TRANSITION HANDLING
    if( oldState == MONITORING_READY && newState == COMMANDING_WAIT ) {
        // Transitioning from other modes
        std::vector<double> cjp(robotState().getMeasuredJointPosition(), robotState().getMeasuredJointPosition()+7);
        // Update position to command as the current position to avoid large jumps
        pvq_tjpos.refresh(cjp);
    }
}

//******************************************************************************
void FRIJointOverlayClient::monitor()
{
   robotCommand().setJointPosition(robotState().getCommandedJointPosition());
}

//******************************************************************************
void FRIJointOverlayClient::waitForCommand()
{
   robotCommand().setJointPosition(robotState().getCommandedJointPosition());
}

//******************************************************************************
void FRIJointOverlayClient::command()
{
    //After sending the first sample, we take control over the sent positions
    if(!pvq_tjpos.empty()) {
        // Consume latest sample (for oldest use consume_one instead)
        std::vector<double> tjp = pvq_tjpos.consume();

        //Check position to send against current position
        bool safe = true;
        std::vector<double> cjp(robotState().getMeasuredJointPosition(), robotState().getMeasuredJointPosition()+7);
        for(size_t i=0; i<7; ++i) {
            double dq = std::abs(tjp[i] - cjp[i]);
            if( dq > MAX_DQ ) {
                cout << "Prevented Motion!!!" << endl;
                cout << "Joint " << i << " moving " << dq << endl;
                cout << "Maximum Allowed " << MAX_DQ << endl;
                cout << "Relaunch this component." << endl;

                safe = false;
            }
        }
        // Convert std::vector<double> to double*
        if(safe) {
            robotCommand().setJointPosition(&tjp[0]);
        }
        else {
            // Default behavior - maintain the last commanded position
            robotCommand().setJointPosition(robotState().getCommandedJointPosition());
        }
    }
    else {
        // Default behavior - maintain the last commanded position
        robotCommand().setJointPosition(robotState().getCommandedJointPosition());
    }
}

void FRIJointOverlayClient::SetTargetJointPositions(const std::vector<double> &tjp)// const
{
    pvq_tjpos.produce(tjp);
}
