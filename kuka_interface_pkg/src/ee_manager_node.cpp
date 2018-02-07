#include "kuka_interface_pkg/ee_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ee_manager_node");

    device_list dl;
    
    device_params soft_hand_params;
    soft_hand_params.ID = 5;
    soft_hand_params.min = 0;
    soft_hand_params.max = 18000;
    soft_hand_params.topic = "/hand_cmd";
    
    dl["soft_hand"] = soft_hand_params;
    
    device_params velvet_params;
    velvet_params.ID = 30;
    velvet_params.min = -32000;
    velvet_params.max = 32000;
    velvet_params.topic = "/velvet_cmd";
    
    dl["velvet"] = velvet_params;

    EndEffectorsManager EEM(dl,"/dev/ttyUSB0");
    EEM.run();

    return 0;
}