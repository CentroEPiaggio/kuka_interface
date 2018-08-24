#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <dynamic_reconfigure/server.h>
#include <kuka_interface_pkg/config_toolConfig.h>
#include "message_filters/subscriber.h"
#include <shared_msgs/CommandTrajectory.h>
#include <shared_msgs/FeedbackTrajectory.h>
#include <mutex>
#include <atomic>

class ArmsManager
{
public:
    ArmsManager();
    ~ArmsManager();
    void init();
    void run();

private:
    void callback_left(const geometry_msgs::Pose& msg);
    void callback_right(const geometry_msgs::Pose& msg);
    void callback_left_aux(const trajectory_msgs::JointTrajectory& msg);
    void callback_right_aux(const trajectory_msgs::JointTrajectory& msg); 
    void callback_command_force_left_aux(const shared_msgs::CommandTrajectory& msg);
    void callback_command_force_right_aux(const shared_msgs::CommandTrajectory& msg);
    void state_callback_left(const sensor_msgs::JointState& msg);
    void state_callback_right(const sensor_msgs::JointState& msg);
    void emergency_callback(const std_msgs::Bool& msg);
    void FTsensor_callback_right(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void FTsensor_callback_left(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void sensor_thread_callback();


    dynamic_reconfigure::Server<kuka_interface_pkg::config_toolConfig> server;
    dynamic_reconfigure::Server<kuka_interface_pkg::config_toolConfig>::CallbackType configFun;
    void config_callback(kuka_interface_pkg::config_toolConfig &config, uint32_t level);

    bool active_left = false;
    bool active_right = false;
    bool use_left_arm_ = false;
    bool use_right_arm_ = false;
    bool use_force_sensor_right_ = false;
    bool use_force_sensor_left_ = false;
    std::string world_frame;

    tf::Vector3 bias_force_right_;
    tf::Vector3 bias_force_left_;
    int calibration_number = 100;
    int calibration_counter_right_ = 0;
    int calibration_counter_left_ = 0;
    double ee_mass_right, ee_mass_left;

    std::atomic<int> force_flag_right;
    std::atomic<int> force_flag_left;
    void store_reference(const geometry_msgs::Pose& in, trajectory_msgs::JointTrajectory& out);
    bool gravityCompensation(float mass, geometry_msgs::WrenchStamped msg, tf::Vector3& f);

    ros::NodeHandle nh, private_nh_;
    std::mutex traj_left_mutex, traj_right_mutex;
    trajectory_msgs::JointTrajectory traj_left, traj_right;
    ros::Publisher pub_command_left, pub_command_right;
    ros::Publisher pub_flag_force_right, pub_flag_force_left;
    ros::Subscriber sub_left, sub_right;
    ros::Subscriber sub_left_aux, sub_right_aux;
    ros::Subscriber sub_command_force_left_aux, sub_command_force_right_aux;
    ros::Subscriber sub_left_state;
    ros::Subscriber sub_right_state;
    ros::Subscriber sub_emergency_right;
    ros::Subscriber sub_emergency_left;

    message_filters::Subscriber<geometry_msgs::WrenchStamped>* right_force_sub_;
    tf::MessageFilter<geometry_msgs::WrenchStamped>* right_force_tf_filter_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped>* left_force_sub_;
    tf::MessageFilter<geometry_msgs::WrenchStamped>* left_force_tf_filter_;
    tf::TransformListener tf_listener_;

    //sensor thread
    boost::thread* sensor_thread;
    std::atomic_bool sensor_thread_stopped;

    std::atomic<double> force_ths[6];		//can be not atomic
    std::atomic<int> sensitivity_axis_right[6]; //can be not atomic
    std::atomic<int> sensitivity_axis_left[6];	//can be not atomic
    std::atomic_bool right_arm_stopped;
    std::atomic_bool left_arm_stopped;

};
