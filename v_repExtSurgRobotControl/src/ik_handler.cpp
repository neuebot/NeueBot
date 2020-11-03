#include "ik_handler.h"

using namespace std;

IKHandler::IKHandler(const string &robot_name) :
    ready(false)
{
    ik_group_handle_str = robot_name + "_IK_group";
    manipSphere_str = robot_name + "_manipSphere";
}


