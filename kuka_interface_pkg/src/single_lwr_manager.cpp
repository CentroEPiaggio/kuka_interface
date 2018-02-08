#include "kuka_interface_pkg/single_lwr_manager.h"

SingleLWRManager::SingleLWRManager()
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

    trajectory.joint_names.push_back("lwr_a1_joint");
    trajectory.joint_names.push_back("lwr_a2_joint");
    trajectory.joint_names.push_back("lwr_e1_joint");
    trajectory.joint_names.push_back("lwr_a3_joint");
    trajectory.joint_names.push_back("lwr_a4_joint");
    trajectory.joint_names.push_back("lwr_a5_joint");
    trajectory.joint_names.push_back("lwr_a6_joint");

    trajectory.points.push_back(point);

    command_subscriber = nh.subscribe("/kuka_command",1,&SingleLWRManager::callback_command,this);
    command_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/lwr/joint_trajectory_controller/command", 10);
}

void SingleLWRManager::callback_command(const geometry_msgs::Pose& msg)
{
    trajectory.points.at(0).positions.at(0) = msg.position.x;
    trajectory.points.at(0).positions.at(1) = msg.position.y;
    trajectory.points.at(0).positions.at(2) = msg.position.z;
    trajectory.points.at(0).positions.at(3) = msg.orientation.x;
    trajectory.points.at(0).positions.at(4) = msg.orientation.y;
    trajectory.points.at(0).positions.at(5) = msg.orientation.z;
    trajectory.points.at(0).positions.at(6) = msg.orientation.w;  
    
    command_publisher.publish(trajectory);
}

void SingleLWRManager::run()
{
    ros::Rate r(100);

    while(ros::ok())
    {
	    r.sleep();
	    ros::spinOnce();
    }

}

SingleLWRManager::~SingleLWRManager()
{

}