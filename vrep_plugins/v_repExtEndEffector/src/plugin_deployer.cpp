#include "plugin_deployer.h"
#include "corba_interface.h"

#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/os/startstop.h>

using namespace RTT::types;
using namespace RTT::corba;
using namespace RTT;
using namespace std;

PluginDeployer::PluginDeployer(const std::string &comp_name, const double period) :
    m_name(comp_name), //"endeffector_interface"
    m_period(period) //0.1
{
    //Emulate main arguments
    char arg0[] = "";
    char* argv[] = {&arg0[0], nullptr};
    int   argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;
    //Initialize Deployer
    __os_init(argc, argv);
}

PluginDeployer::~PluginDeployer()
{
    //Close Deployer
    __os_exit();
}

void PluginDeployer::LaunchComm()
{
    //Emulate main arguments
    char arg0[] = "";
    char* argv[] = {&arg0[0], nullptr};
    int   argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;

    //Create instance of component and set activity
    m_ci = std::make_shared<CorbaInterface>(m_name);
    m_ci->setActivity( new Activity(os::HighestPriority, m_period) );

    //Setup CORBA
    RTT::corba::TaskContextServer::InitOrb(argc, argv);
    RTT::corba::TaskContextServer::Create(m_ci.get());

    //Start Component
    m_ci->configure();
    m_ci->start();

    m_thr = std::async(std::launch::async, RTT::corba::TaskContextServer::RunOrb);
    //RTT::corba::TaskContextServer::ThreadOrb();
}

void PluginDeployer::StopComm()
{
    //Stop Component
    m_ci->stop();
    m_ci->cleanup();

    try {
        RTT::corba::TaskContextServer::ShutdownOrb();
        RTT::corba::TaskContextServer::DestroyOrb();
    } catch (...) {
        cerr << "Unknown exception caught!" << endl;
    }

    //Destroy object
//    m_ci.reset();
}

