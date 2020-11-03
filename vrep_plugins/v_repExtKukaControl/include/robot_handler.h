#ifndef V_REPEXT_KUKACONTROL_ROBOT_HANDLER
#define V_REPEXT_KUKACONTROL_ROBOT_HANDLER

#include <map>
#include <string>
#include <vector>

//Robot
struct Robot {
    int handle;
    std::string name;
    int dofs;
    //bool hollow;
    std::vector<int> jhandles;
    std::vector< std::pair<double, double> > jplim;
    std::vector<double> jvlim;
};

class RobotHandler {
public:
    RobotHandler();

    void InitRobots();

    void GetJointPositions(std::vector<double> &pos);
    void GetJointVelocities(std::vector<double> &vel);
    void GetJointPositionsHollow(std::vector<double> &hpos);
    void GetJointVelocitiesHollow(std::vector<double> &hvel);

    void SetJointPositions(const std::vector<double> &dpos);
    void SetJointPositionsHollow(const std::vector<double> &dhpos);

    void SetHollowVisibility(const bool visible);
    bool GetSurgeryReference(std::vector<double> &matrix);

public:
    Robot kuka;
    Robot kuka_hlw;
};

#endif //V_REPEXT_KUKACONTROL_TOOL_HANDLER

