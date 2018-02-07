#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <string>
#include "kuka_interface_pkg/qbmove_communications.h"

#define ID_HAND 5
#define ID_VELVET 30

#define MIN_HAND 0
#define MAX_HAND 18000

#define MIN_VELVET -32000
#define MAX_VELVET 32000

typedef std::string device_name;

struct device_params
{
    int ID;
    int min;
    int max;
    std::string topic;
};

typedef std::map<device_name,device_params> device_list;

class EndEffectorsManager
{
public:
    EndEffectorsManager(device_list devices_, std::string port);
    ~EndEffectorsManager();
    void run();

private:
    ros::NodeHandle nh;

    double cmd_hand = 0.0;
    double cmd_velvet = 0.0;

    void activate(int ID);
    void deactivate(int ID);
    void move_device(int ID, short int pos);
    
    comm_settings cs;

    std::string name_1, name_2;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    void callback_1(const std_msgs::Float64& msg);
    void callback_2(const std_msgs::Float64& msg);

    device_list devices;
};