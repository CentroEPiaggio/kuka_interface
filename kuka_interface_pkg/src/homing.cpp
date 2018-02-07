#include <ros/ros.h>
#include <ros/rate.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

bool active_left = false;
bool active_right = false;

std::vector<double> q0_l, q0_r;
ros::Publisher pub_h, pub_v;

void reset_ee()
{
	std_msgs::Float64 msg;
	msg.data = 0.0;

	pub_h.publish(msg);
	pub_v.publish(msg);
}

void state_callback_left(const sensor_msgs::JointState& msg)
{
	if(active_left) return;

	q0_l.push_back(msg.position.at(0));
	q0_l.push_back(msg.position.at(1));
	q0_l.push_back(msg.position.at(6));
	q0_l.push_back(msg.position.at(2));
	q0_l.push_back(msg.position.at(3));
	q0_l.push_back(msg.position.at(4));
	q0_l.push_back(msg.position.at(5));

	active_left=true;
}

void state_callback_right(const sensor_msgs::JointState& msg)
{
	if(active_right) return;

	q0_r.push_back(msg.position.at(0));
	q0_r.push_back(msg.position.at(1));
	q0_r.push_back(msg.position.at(6));
	q0_r.push_back(msg.position.at(2));
	q0_r.push_back(msg.position.at(3));
	q0_r.push_back(msg.position.at(4));
	q0_r.push_back(msg.position.at(5));

	active_right=true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "homing");
	ros::NodeHandle n_;
	double rateHZ = 100;

	pub_h = n_.advertise<std_msgs::Float64>("/hand_cmd", 10);
	pub_v = n_.advertise<std_msgs::Float64>("/velvet_cmd", 10);

	std::cout<<"Select arm:"<<std::endl;
	std::cout<<"0: left"<<std::endl;
	std::cout<<"1: right"<<std::endl;
	std::cout<<"2: both"<<std::endl;
	std::cout<<"3: ee reset"<<std::endl;
	std::cout<<"4: kuka dance"<<std::endl;

	int arm=-1;
	std::cin>>arm;

	std::vector<double> homing_left, homing_right;

	homing_left.push_back(-0.1); //a1	
	homing_left.push_back(-1.3);  //a2
	homing_left.push_back(0.1);   //e1
	homing_left.push_back(-2.05); //a3
    homing_left.push_back(-0.05); //a4
	homing_left.push_back(0.25);  //a5
	homing_left.push_back(-0.35); //a6

	homing_right.push_back(0.1); //a1
	homing_right.push_back(-1.47);  //a2
	homing_right.push_back(-0.1);   //e1
	homing_right.push_back(-0.785); //a3
    homing_right.push_back(0.1); //a4
	homing_right.push_back(0.785);  //a5
	homing_right.push_back(-0.1); //a6

	bool using_left = false;
	bool using_right = false;

	if(arm==0) //left
	{
		std::cout<<"Homing: left"<<std::endl;

		using_left=true;
	}
	else if(arm==1) //right
	{
		std::cout<<"Homing: right"<<std::endl;

		using_right=true;
	}
	else if(arm==2)
	{
	 	std::cout<<"Homing: both"<<std::endl;

		using_left = true;
		using_right = true;
	}
	else if(arm==3)
	{
	 	std::cout<<"Homing: ee reset"<<std::endl;

	 	reset_ee();

	 	return 0;
	}
	else if(arm==4)
	{
	 	std::cout<<"Homing: dance"<<std::endl;

		using_left = true;
		using_right = true;
	}
	else	
	{
		std::cout<<"WRONG!"<<std::endl;
		return -1;
	}

	geometry_msgs::Pose msg_left, msg_right;

	ros::Subscriber sub_l = n_.subscribe("/left_arm/joint_states",1,&state_callback_left);
	ros::Publisher pub_command_l = n_.advertise<geometry_msgs::Pose>("/kuka_command_left", 10);
	ros::Subscriber sub_r = n_.subscribe("/right_arm/joint_states",1,&state_callback_right);
	ros::Publisher pub_command_r = n_.advertise<geometry_msgs::Pose>("/kuka_command_right", 10);

	ros::Rate r(rateHZ);

	bool active=false;

	while(!active && ros::ok())
	{
		if(using_left)
			if(using_right) active = active_left && active_right;
			else active = active_left;
		else
			if(using_right) active = active_right;
			else active=false;

		r.sleep();
		ros::spinOnce();
	}

	double alpha=0.0;

	while(ros::ok())
	{
		if(using_left)
		{
			msg_left.position.x = alpha*homing_left.at(0) + (1-alpha)*q0_l.at(0);
			msg_left.position.y = alpha*homing_left.at(1) + (1-alpha)*q0_l.at(1);
			msg_left.position.z = alpha*homing_left.at(2) + (1-alpha)*q0_l.at(2);
			msg_left.orientation.x = alpha*homing_left.at(3) + (1-alpha)*q0_l.at(3);
			msg_left.orientation.y = alpha*homing_left.at(4) + (1-alpha)*q0_l.at(4);
			msg_left.orientation.z = alpha*homing_left.at(5) + (1-alpha)*q0_l.at(5);
			msg_left.orientation.w = alpha*homing_left.at(6) + (1-alpha)*q0_l.at(6);

			pub_command_l.publish(msg_left);
		}
		if(using_right)
		{
			msg_right.position.x = alpha*homing_right.at(0) + (1-alpha)*q0_r.at(0);
			msg_right.position.y = alpha*homing_right.at(1) + (1-alpha)*q0_r.at(1);
			msg_right.position.z = alpha*homing_right.at(2) + (1-alpha)*q0_r.at(2);
			msg_right.orientation.x = alpha*homing_right.at(3) + (1-alpha)*q0_r.at(3);
			msg_right.orientation.y = alpha*homing_right.at(4) + (1-alpha)*q0_r.at(4);
			msg_right.orientation.z = alpha*homing_right.at(5) + (1-alpha)*q0_r.at(5);
			msg_right.orientation.w = alpha*homing_right.at(6) + (1-alpha)*q0_r.at(6);

			pub_command_r.publish(msg_right);
		}

		r.sleep();
		ros::spinOnce();

		alpha+=0.0025;
		if(alpha>1) break;
	}

	alpha = 1.0;
	bool increasing = false;

	if(arm==4)
	{
		while(ros::ok())
		{
			if(using_left)
			{
				msg_left.position.x = alpha*homing_left.at(0) + (1-alpha)*(homing_left.at(0)+0.1);
				msg_left.position.y = alpha*homing_left.at(1) + (1-alpha)*(homing_left.at(1)+0.1);
				msg_left.position.z = alpha*homing_left.at(2) + (1-alpha)*(homing_left.at(2)+0.1);
				msg_left.orientation.x = alpha*homing_left.at(3) + (1-alpha)*(homing_left.at(3)+0.1);
				msg_left.orientation.y = alpha*homing_left.at(4) + (1-alpha)*(homing_left.at(4)+0.1);
				msg_left.orientation.z = alpha*homing_left.at(5) + (1-alpha)*(homing_left.at(5)+0.1);
				msg_left.orientation.w = alpha*homing_left.at(6) + (1-alpha)*(homing_left.at(6)+0.1);
			
				pub_command_l.publish(msg_left);
			}
			if(using_right)
			{
				msg_right.position.x = alpha*homing_right.at(0) + (1-alpha)*(homing_right.at(0)+0.1);
				msg_right.position.y = alpha*homing_right.at(1) + (1-alpha)*(homing_right.at(1)+0.1);
				msg_right.position.z = alpha*homing_right.at(2) + (1-alpha)*(homing_right.at(2)+0.1);
				msg_right.orientation.x = alpha*homing_right.at(3) + (1-alpha)*(homing_right.at(3)+0.1);
				msg_right.orientation.y = alpha*homing_right.at(4) + (1-alpha)*(homing_right.at(4)+0.1);
				msg_right.orientation.z = alpha*homing_right.at(5) + (1-alpha)*(homing_right.at(5)+0.1);
				msg_right.orientation.w = alpha*homing_right.at(6) + (1-alpha)*(homing_right.at(6)+0.1);

				pub_command_r.publish(msg_right);
			}

			if(increasing)
			{
				alpha+=0.02;
				if(alpha>1)
				{
					alpha=1;
					increasing=false;
				}
			}
			else
			{
				alpha-=0.02;
				if(alpha<0)
				{
					alpha=0;
					increasing=true;
				}
			}

			r.sleep();
			ros::spinOnce();
		}
	}

	std::cout<<"Homing: DONE"<<std::endl;

	return 0;
}