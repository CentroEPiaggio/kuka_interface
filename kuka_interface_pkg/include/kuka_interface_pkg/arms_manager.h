#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

class ArmsManager
{
public:
    ArmsManager();
    ~ArmsManager();
    void run();

private:
    void callback_left(const geometry_msgs::Pose& msg);
    void callback_right(const geometry_msgs::Pose& msg);

    void store_reference(const geometry_msgs::Pose& in, trajectory_msgs::JointTrajectory& out);

    ros::NodeHandle nh;
    trajectory_msgs::JointTrajectory traj_left, traj_right;
    ros::Publisher pub_command_left, pub_command_right;
    ros::Subscriber sub_left, sub_right;
};