#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

class SingleLWRManager
{
public:
    SingleLWRManager();
    ~SingleLWRManager();
    void run();

private:
    void callback_command(const geometry_msgs::Pose& msg);

    ros::NodeHandle nh;
    trajectory_msgs::JointTrajectory trajectory;
    ros::Publisher command_publisher;
    ros::Subscriber command_subscriber;
};