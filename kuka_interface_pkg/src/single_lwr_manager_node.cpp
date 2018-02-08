#include "kuka_interface_pkg/single_lwr_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_lwr_manager_node");

    SingleLWRManager manager;
    manager.run();

    return 0;
}