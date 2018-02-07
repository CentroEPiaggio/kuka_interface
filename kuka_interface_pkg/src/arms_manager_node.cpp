#include "kuka_interface_pkg/arms_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arms_manager_node");

    ArmsManager AM;
    AM.run();

    return 0;
}