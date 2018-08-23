#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/WrenchStamped.h>

geometry_msgs::WrenchStamped msg_left, msg_right;
bool received_left, received_right;

void handle_msg_left(const geometry_msgs::WrenchStamped& msg)
{
  msg_left = msg;
  received_left = true;
}

void handle_msg_right(const geometry_msgs::WrenchStamped& msg)
{
  msg_right = msg;
  received_right = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_force_sensor_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher pub_right = nh.advertise<geometry_msgs::WrenchStamped>("/my_sensor_right/ft_sensor_hw/my_sensor_right", 10);
    ros::Publisher pub_left = nh.advertise<geometry_msgs::WrenchStamped>("/my_sensor_left/ft_sensor_hw/my_sensor_left", 10);
    ros::Subscriber sub_left = nh.subscribe("/desired_force_left", 1, handle_msg_left);
    ros::Subscriber sub_right = nh.subscribe("/desired_force_right", 1, handle_msg_right);

    ros::Rate f(10);
    
    received_left = received_right = false;
    
    while(ros::ok())
    { 
      if (received_right)
      {
	msg_right.header.stamp = ros::Time::now();
	pub_right.publish(msg_right);
      }
      
      if (received_left)
      {
	pub_left.publish(msg_left);
	msg_left.header.stamp = ros::Time::now();
      }
      
      ros::spinOnce();
      f.sleep();      
    }

    return 0;
}