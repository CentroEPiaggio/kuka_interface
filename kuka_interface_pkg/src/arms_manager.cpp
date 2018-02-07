#include "kuka_interface_pkg/arms_manager.h"

ArmsManager::ArmsManager()
{
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.time_from_start.sec=0;
    point.time_from_start.nsec=100000000;

    traj_left.joint_names.push_back("left_arm_a1_joint");
    traj_left.joint_names.push_back("left_arm_a2_joint");
    traj_left.joint_names.push_back("left_arm_e1_joint");
    traj_left.joint_names.push_back("left_arm_a3_joint");
    traj_left.joint_names.push_back("left_arm_a4_joint");
    traj_left.joint_names.push_back("left_arm_a5_joint");
    traj_left.joint_names.push_back("left_arm_a6_joint");

    traj_left.points.push_back(point);

    traj_right.joint_names.push_back("right_arm_a1_joint");
    traj_right.joint_names.push_back("right_arm_a2_joint");
    traj_right.joint_names.push_back("right_arm_e1_joint");
    traj_right.joint_names.push_back("right_arm_a3_joint");
    traj_right.joint_names.push_back("right_arm_a4_joint");
    traj_right.joint_names.push_back("right_arm_a5_joint");
    traj_right.joint_names.push_back("right_arm_a6_joint");

    traj_right.points.push_back(point);

    sub_right = nh.subscribe("/kuka_command_right",1,&ArmsManager::callback_right,this);
    sub_left = nh.subscribe("/kuka_command_left",1,&ArmsManager::callback_left,this);
    pub_command_right = nh.advertise<trajectory_msgs::JointTrajectory>("/right_arm/joint_trajectory_controller/command", 10);
    pub_command_left = nh.advertise<trajectory_msgs::JointTrajectory>("/left_arm/joint_trajectory_controller/command", 10);
}

void ArmsManager::callback_left(const geometry_msgs::Pose& msg)
{
    store_reference(msg,traj_left);
    pub_command_left.publish(traj_left);
}

void ArmsManager::callback_right(const geometry_msgs::Pose& msg)
{
    store_reference(msg,traj_right);
    pub_command_right.publish(traj_right);
}

void ArmsManager::store_reference(const geometry_msgs::Pose& in, trajectory_msgs::JointTrajectory& out)
{
    out.points.at(0).positions.at(0) = in.position.x;
    out.points.at(0).positions.at(1) = in.position.y;
    out.points.at(0).positions.at(2) = in.position.z;
    out.points.at(0).positions.at(3) = in.orientation.x;
    out.points.at(0).positions.at(4) = in.orientation.y;
    out.points.at(0).positions.at(5) = in.orientation.z;
    out.points.at(0).positions.at(6) = in.orientation.w;    
}

void ArmsManager::run()
{
    ros::Rate r(100);

    while(ros::ok())
    {
	    r.sleep();
	    ros::spinOnce();
    }

}

ArmsManager::~ArmsManager()
{

}