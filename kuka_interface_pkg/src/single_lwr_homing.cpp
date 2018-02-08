#include <ros/ros.h>
#include <ros/rate.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

bool received_state = false;
std::vector<double> q0;

void state_callback(const sensor_msgs::JointState& msg)
{
	if(received_state) return;

	q0.push_back(msg.position.at(0));
	q0.push_back(msg.position.at(1));
	q0.push_back(msg.position.at(6)); //NOTE: this is because e1 is at the end of the state vector
	q0.push_back(msg.position.at(2));
	q0.push_back(msg.position.at(3));
	q0.push_back(msg.position.at(4));
	q0.push_back(msg.position.at(5));

	received_state=true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "single_lwr_homing");
	ros::NodeHandle n_;
	double rateHZ = 100;

	std::vector<double> homing, homing_right;

	homing.push_back(1.0); //a1	
	homing.push_back(1.0);  //a2
	homing.push_back(1.0);   //e1
	homing.push_back(1.0); //a3
	homing.push_back(1.0); //a4
	homing.push_back(1.0);  //a5
	homing.push_back(1.0); //a6

	geometry_msgs::Pose command_msg;

	ros::Subscriber sub = n_.subscribe("/joint_states",1,&state_callback);
	ros::Publisher pub_command_l = n_.advertise<geometry_msgs::Pose>("/kuka_command", 10);

	ros::Rate r(rateHZ);

	bool active=false;

	while(!received_state && ros::ok())
	{
		r.sleep();
		ros::spinOnce();
	}

	double alpha=0.0;

	while(ros::ok())
	{
		command_msg.position.x = alpha*homing.at(0) + (1-alpha)*q0.at(0);
		command_msg.position.y = alpha*homing.at(1) + (1-alpha)*q0.at(1);
		command_msg.position.z = alpha*homing.at(2) + (1-alpha)*q0.at(2);
		command_msg.orientation.x = alpha*homing.at(3) + (1-alpha)*q0.at(3);
		command_msg.orientation.y = alpha*homing.at(4) + (1-alpha)*q0.at(4);
		command_msg.orientation.z = alpha*homing.at(5) + (1-alpha)*q0.at(5);
		command_msg.orientation.w = alpha*homing.at(6) + (1-alpha)*q0.at(6);

		pub_command_l.publish(command_msg);

		r.sleep();
		ros::spinOnce();

		alpha+=0.0025;
		if(alpha>1) break;
	}

	std::cout<<"Homing: DONE"<<std::endl;

	return 0;
}