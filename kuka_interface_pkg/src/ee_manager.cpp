#include "kuka_interface_pkg/ee_manager.h"

EndEffectorsManager::EndEffectorsManager(device_list devices_, std::string port)
{
    openRS485(&cs,port.c_str());
    usleep(10000);

    int i = 0;
    for(auto device:devices_)
    {
        ROS_INFO_STREAM("creating device "<<device.first<<" with ID: "<<device.second.ID<<" listening to "<<device.second.topic);
        devices[device.first] = device.second;
	if(i==0)
	{
	    name_1 = device.first;
	    sub_1 = nh.subscribe(device.second.topic.c_str(),1,&EndEffectorsManager::callback_1,this);
	}
	if(i==1)
	{
	    name_2 = device.first;
	    sub_2 = nh.subscribe(device.second.topic.c_str(),1,&EndEffectorsManager::callback_2,this);
	}  
	i++;

	activate(device.second.ID);
    usleep(100000);
    }
}

void EndEffectorsManager::activate(int ID)
{
    commActivate(&cs,ID,1);
    usleep(1000);
}

void EndEffectorsManager::deactivate(int ID)
{
    commActivate(&cs,ID,0);
    usleep(1000);
}

void EndEffectorsManager::move_device(int ID, short int pos)
{
    short int inputs[2];
    inputs[0] = pos;
    inputs[1] = 0;

    commSetInputs(&cs,ID,inputs);
    usleep(1000);
}
void EndEffectorsManager::callback_1(const std_msgs::Float64& msg)
{
    cmd_hand = (1-msg.data)*devices.at(name_1).min + msg.data*devices.at(name_1).max;

    ROS_INFO_STREAM("moving "<<name_1<<" to "<<cmd_hand);

    move_device(devices.at(name_1).ID,(short int)cmd_hand);
}

void EndEffectorsManager::callback_2(const std_msgs::Float64& msg)
{
    cmd_velvet = (1-msg.data)*devices.at(name_2).min + msg.data*devices.at(name_2).max;

     ROS_INFO_STREAM("moving "<<name_2<<" to "<<cmd_velvet);

    move_device(devices.at(name_2).ID,(short int)cmd_velvet);
}

void EndEffectorsManager::run()
{
    ros::Rate r(100.0);

    while(ros::ok())
    {
    	ros::spinOnce();
        
        //r.sleep();
        usleep(10000);
    }
}

EndEffectorsManager::~EndEffectorsManager()
{
    for(auto device:devices)
    {
	deactivate(device.second.ID);
    }

    closeRS485(&cs);
    usleep(10000);
}