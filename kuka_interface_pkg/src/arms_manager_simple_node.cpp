#include "kuka_interface_pkg/arms_manager_simple.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arms_manager_node");

    ArmsManagerSimple AM;
    AM.init();
    AM.run();

    return 0;
}