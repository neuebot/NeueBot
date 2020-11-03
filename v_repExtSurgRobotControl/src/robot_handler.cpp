#include "robot_handler.h"

using namespace std;

RobotHandler::RobotHandler(const std::string &rname) :
    name(rname),
    dofs(7),
    ready(false)
{
}

std::vector<double> RobotHandler::get_cur_pos()
{
    std::lock_guard<std::mutex> lk(mutex_cpos);
    return cur_pos;
}

void RobotHandler::set_cur_pos(const std::vector<double> &pos)
{
    std::lock_guard<std::mutex> lk(mutex_cpos);
    cur_pos = pos;
}

std::vector<double> RobotHandler::get_cur_vel()
{
    std::lock_guard<std::mutex> lk(mutex_cvel);
    return cur_vel;
}

void RobotHandler::set_cur_vel(const std::vector<double> &vel)
{
    std::lock_guard<std::mutex> lk(mutex_cvel);
    cur_vel = vel;
}

