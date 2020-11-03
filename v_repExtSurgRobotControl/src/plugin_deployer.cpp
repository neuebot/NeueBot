#include "plugin_deployer.h"

#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/os/startstop.h>

using namespace RTT::types;
using namespace RTT::corba;
using namespace RTT;
using namespace std;

PluginDeployer::PluginDeployer(const std::string &robot_comp, const std::string &endeffector_comp,
                               const std::string &trajectories_comp) :
    m_robot_comp(robot_comp), //robot component name in network
    m_endeffector_comp(endeffector_comp), //end effector component name in network
    m_trajectories_comp(trajectories_comp) //trajectories component name in network
{

}

PluginDeployer::~PluginDeployer()
{

}

void PluginDeployer::Init()
{
    //Emulate main arguments
    char arg0[] = "";
    char* argv[] = {&arg0[0], nullptr};
    int   argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;
    //Initialize Deployer
    __os_init(argc, argv);
}

void PluginDeployer::InitComponents()
{
    //Emulate main arguments
    char arg0[] = "";
    char* argv[] = {&arg0[0], nullptr};
    int   argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;

    //Create instance of component
    m_rc = std::make_shared<RobotComponent>(m_robot_comp);
    m_eec = std::make_shared<EndEffectorComponent>(m_endeffector_comp);
    m_stc = std::make_shared<SurgeryTrajectoriesComponent>(m_trajectories_comp);

    //Setup CORBA
    RTT::corba::TaskContextServer::InitOrb(argc, argv);
    //Create servers
    RTT::corba::TaskContextServer::Create(m_rc.get());
    RTT::corba::TaskContextServer::Create(m_eec.get());
    RTT::corba::TaskContextServer::Create(m_stc.get());

    m_thr = std::async(std::launch::async, RTT::corba::TaskContextServer::RunOrb);

    //Start Components
    m_rc->configure();
    m_eec->configure();
    m_stc->configure();
}

void PluginDeployer::StartComm()
{
    m_rc->start();
    m_eec->start();
    m_stc->start();
}

void PluginDeployer::StopComm()
{
    //Stop Component
    m_rc->stop();
    m_eec->stop();
    m_stc->stop();
}

void PluginDeployer::CleanupComponents()
{
    m_rc->cleanup();
    m_eec->cleanup();
    m_stc->cleanup();

    //    RTT::corba::TaskContextServer::CleanupServers();
    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();

    //Destroy objects
    m_rc.reset();
    m_eec.reset();
    m_stc.reset();
}

void PluginDeployer::Cleanup()
{
    //Close Deployer - Crashes simulator on re-opening scene
    //__os_exit();
}

