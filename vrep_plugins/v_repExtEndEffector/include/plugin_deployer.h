#ifndef V_REPEXT_ENDEFFECTOR_PLUGIN_DEPLOYER
#define V_REPEXT_ENDEFFECTOR_PLUGIN_DEPLOYER

#include "corba_interface.h"

#include <future>
#include <string>

class PluginDeployer {
public:
    PluginDeployer(const std::string &comp_name, const double period);
    ~PluginDeployer();

    void LaunchComm();
    void StopComm();

private:
    std::future<void> m_thr;
    std::string m_name;
    double m_period;
    std::shared_ptr<CorbaInterface> m_ci;

};

#endif // V_REPEXT_ENDEFFECTOR_PLUGIN_DEPLOYER
