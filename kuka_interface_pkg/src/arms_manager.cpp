#include "kuka_interface_pkg/arms_manager.h"

ArmsManager::ArmsManager(): private_nh_("~")
{
    private_nh_.param("use_right_arm", use_right_arm_, true);
    private_nh_.param("use_left_arm", use_left_arm_, true);
    private_nh_.param("ee_mass_right", ee_mass_right, 0.0); //FIXME to be estimated
    private_nh_.param("ee_mass_left", ee_mass_left, 0.0); //FIXME to be estimated
    private_nh_.param("use_force_sensor_right", use_force_sensor_right_, false);
    private_nh_.param("use_force_sensor_left", use_force_sensor_left_, false);
    private_nh_.param("calibration_number", calibration_number, calibration_number);
    private_nh_.param("world_frame", world_frame, std::string("world"));


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
    traj_left_mutex.unlock();

    traj_right.joint_names.push_back("right_arm_a1_joint");
    traj_right.joint_names.push_back("right_arm_a2_joint");
    traj_right.joint_names.push_back("right_arm_e1_joint");
    traj_right.joint_names.push_back("right_arm_a3_joint");
    traj_right.joint_names.push_back("right_arm_a4_joint");
    traj_right.joint_names.push_back("right_arm_a5_joint");
    traj_right.joint_names.push_back("right_arm_a6_joint");

    traj_right.points.push_back(point);
    traj_right_mutex.unlock();

    sub_right = nh.subscribe("/kuka_command_right",1,&ArmsManager::callback_right,this);
    sub_left = nh.subscribe("/kuka_command_left",1,&ArmsManager::callback_left,this);
    sub_right_aux = nh.subscribe("/right_arm/command_aux",1,&ArmsManager::callback_right_aux,this);
    sub_left_aux = nh.subscribe("/left_arm/command_aux",1,&ArmsManager::callback_left_aux,this);
    sub_command_force_right_aux = nh.subscribe("/right_arm/command_force_aux",1,&ArmsManager::callback_command_force_right_aux,this);
    sub_command_force_left_aux = nh.subscribe("/left_arm/command_force_aux",1,&ArmsManager::callback_command_force_left_aux,this);
    sub_left_aux = nh.subscribe("/left_arm/command_aux",1,&ArmsManager::callback_left_aux,this);
    pub_command_right = nh.advertise<trajectory_msgs::JointTrajectory>("/right_arm/joint_trajectory_controller/command", 10);
    pub_command_left = nh.advertise<trajectory_msgs::JointTrajectory>("/left_arm/joint_trajectory_controller/command", 10);
    sub_left_state = nh.subscribe("/left_arm/joint_states",1,&ArmsManager::state_callback_left,this);
    sub_right_state = nh.subscribe("/right_arm/joint_states",1,&ArmsManager::state_callback_right,this);
    sub_emergency_right = nh.subscribe("/right_arm/emergency_event",1,&ArmsManager::emergency_callback,this);
    sub_emergency_right = nh.subscribe("/left_arm/emergency_event",1,&ArmsManager::emergency_callback,this);

    //Subscribe to force sensor if used    sub_emergency_right = nh.subscribe("/right_arm/emergency_event",1,&ArmsManager::emergency_callback_right,this);

    if (use_force_sensor_right_)
    {
      pub_feedback_right  = nh.advertise<shared_msgs::FeedbackTrajectory>("/arms_manager/feedback_force_right", 10);

      right_force_sub_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh, "/my_sensor_right/ft_sensor_hw/my_sensor_right", 20);
      right_force_tf_filter_ = new tf::MessageFilter<geometry_msgs::WrenchStamped>(*right_force_sub_, tf_listener_, world_frame, 20);
      right_force_tf_filter_->registerCallback(boost::bind(&ArmsManager::FTsensor_callback_right, this, _1));
    }

    if (use_force_sensor_left_)
    {
      pub_feedback_left  = nh.advertise<shared_msgs::FeedbackTrajectory>("/arms_manager/feedback_force_left", 10);

      left_force_sub_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh, "/my_sensor_left/ft_sensor_hw/my_sensor_left", 20);
      left_force_tf_filter_ = new tf::MessageFilter<geometry_msgs::WrenchStamped>(*left_force_sub_, tf_listener_, world_frame, 20);
      left_force_tf_filter_->registerCallback(boost::bind(&ArmsManager::FTsensor_callback_left, this, _1));
    }

    bias_force_right_[0] = bias_force_right_[1] = bias_force_right_[2] = 0;
    bias_force_left_[0] = bias_force_left_[1] = bias_force_left_[2] = 0;
    force_flag_right.store(shared_msgs::FeedbackTrajectory::NOT_SENSED);
    force_flag_left.store(shared_msgs::FeedbackTrajectory::NOT_SENSED);

    //Dynamic reconfiguration for force thresholds
    configFun = boost::bind(&ArmsManager::config_callback, this, _1, _2);
    server.setCallback(configFun);
    
    //Initialize force flags and ths
    for (int i = 0; i < 6; ++i)
    {
      force_ths[i].store(1.0);
      sensitivity_axis_right[i].store(shared_msgs::CommandTrajectory::SENSED_ERROR);
      sensitivity_axis_left[i].store(shared_msgs::CommandTrajectory::SENSED_ERROR);
    }
    
    //Initialize stopped arm to false
    right_arm_stopped.store(false);
    left_arm_stopped.store(false);
}


ArmsManager::~ArmsManager()
{
    sensor_thread_stopped.store(true);
    sensor_thread->detach();
    if (sensor_thread->joinable()) sensor_thread->join();
    delete sensor_thread;
}

void ArmsManager::config_callback(kuka_interface_pkg::config_toolConfig &config, uint32_t level)
{
    ROS_INFO_STREAM("Received new config parameters.");

    force_ths[0].store(config.Fx);
    force_ths[1].store(config.Fy);
    force_ths[2].store(config.Fz);
    force_ths[3].store(config.Mx);
    force_ths[4].store(config.My);
    force_ths[5].store(config.Mz);

    ROS_INFO_STREAM("Fx: " << config.Fx << ", Fy: " << config.Fy << ", Fz: " << config.Fz << ", Mx: " << config.Mx << ", My: " << config.My << ", Mz: " << config.Mz);
}

void ArmsManager::sensor_thread_callback()
{
    ros::Rate f(100);
    while(ros::ok() && !sensor_thread_stopped.load())
    {
      ros::spinOnce();
      f.sleep();
    }
}

void ArmsManager::init()
{
    ROS_INFO_STREAM("Initialization.");
    
    ros::Rate f(10);
    
    if (use_left_arm_)
    {
      while(!active_left)
      {
	ROS_INFO("Waiting for left arm ... ");
      	ros::spinOnce();
	f.sleep();
      }
    }
    
    if (use_right_arm_)
    {
      while(!active_right)
      {
	ROS_INFO("Waiting for right arm ... ");
      	ros::spinOnce();
	f.sleep();
      }
    }

    ROS_INFO_STREAM("Robots are active.");
    
    //Start a separate thread for reading sensors for controlling the arms with a fixed frequency
    if (use_force_sensor_left_||use_force_sensor_right_)
    {
	sensor_thread_stopped.store(false);
	sensor_thread = new boost::thread(&ArmsManager::sensor_thread_callback, this);
    }

    float expected_force_period = 1.0;
    float timeout = (float)calibration_number*expected_force_period;

    if (use_force_sensor_right_)
    {
    	//wait for calibration
    	ROS_INFO_STREAM("Started calibration right. Expected " <<  calibration_number << " samples.");
    	ros::Time start_time = ros::Time::now();
	while(calibration_counter_right_ < calibration_number)
	{
	  while (calibration_counter_right_ <= calibration_number && (ros::Time::now()-start_time).toSec() < timeout)
	      f.sleep();
	  
	  if (calibration_counter_right_ < calibration_number)
	  {
	      ROS_ERROR_STREAM("Error in calibrating the right sensor. Please, check the connection.");
	      start_time = ros::Time::now();
	  }
	}
    }

    if (use_force_sensor_left_)
    {
    	//wait for calibration
    	ROS_INFO_STREAM("Started calibration left. Expected " <<  calibration_number << " samples.");
    	ros::Time start_time = ros::Time::now();
	while (calibration_counter_left_ < calibration_number)
	{
	  while (calibration_counter_left_ <= calibration_number && (ros::Time::now()-start_time).toSec() < timeout)
	      f.sleep();
	  
	  if (calibration_counter_left_ < calibration_number)
	  {
	    ROS_ERROR_STREAM("Error in calibrating the left sensor. Please, check the connection.");
	    start_time = ros::Time::now(); //reset the timer
	  }
	}
    }
}

void ArmsManager::callback_left(const geometry_msgs::Pose& msg)
{
    traj_left_mutex.lock();
    store_reference(msg,traj_left);
    traj_left_mutex.unlock();
}

void ArmsManager::callback_right(const geometry_msgs::Pose& msg)
{
    traj_right_mutex.lock();
    store_reference(msg,traj_right);
    traj_right_mutex.unlock();
}

void ArmsManager::callback_left_aux(const trajectory_msgs::JointTrajectory& msg)
{
    if(!left_arm_stopped.load())
    {
      traj_left_mutex.lock();
      traj_left = msg;
      traj_left_mutex.unlock();
    }
}

void ArmsManager::callback_right_aux(const trajectory_msgs::JointTrajectory& msg)
{
    if(!right_arm_stopped.load())
    {
      traj_right_mutex.lock();
      traj_right = msg;
      traj_right_mutex.unlock();
    }
}

void ArmsManager::state_callback_left(const sensor_msgs::JointState& msg)
{
    if(active_left) return;

    traj_left_mutex.lock();
    traj_left.points.at(0).positions.at(0) = (msg.position.at(0));
    traj_left.points.at(0).positions.at(1) = (msg.position.at(1));
    traj_left.points.at(0).positions.at(2) = (msg.position.at(6));
    traj_left.points.at(0).positions.at(3) = (msg.position.at(2));
    traj_left.points.at(0).positions.at(4) = (msg.position.at(3));
    traj_left.points.at(0).positions.at(5) = (msg.position.at(4));
    traj_left.points.at(0).positions.at(6) = (msg.position.at(5));
    traj_left_mutex.unlock();

    active_left=true;
}

void ArmsManager::state_callback_right(const sensor_msgs::JointState& msg)
{
    if(active_right) return;

    traj_right_mutex.lock();
    traj_right.points.at(0).positions.at(0) = msg.position.at(0);
    traj_right.points.at(0).positions.at(1) = msg.position.at(1);
    traj_right.points.at(0).positions.at(2) = msg.position.at(6);
    traj_right.points.at(0).positions.at(3) = msg.position.at(2);
    traj_right.points.at(0).positions.at(4) = msg.position.at(3);
    traj_right.points.at(0).positions.at(5) = msg.position.at(4);
    traj_right.points.at(0).positions.at(6) = msg.position.at(5);
    traj_right_mutex.unlock();

    active_right=true;
}

void ArmsManager::callback_command_force_left_aux(const shared_msgs::CommandTrajectory& msg)
{
  sensitivity_axis_left[0].store(msg.fx_neg);
  sensitivity_axis_left[1].store(msg.fx_pos);
  sensitivity_axis_left[2].store(msg.fy_neg);
  sensitivity_axis_left[3].store(msg.fy_pos);
  sensitivity_axis_left[4].store(msg.fz_neg);
  sensitivity_axis_left[5].store(msg.fz_pos);
}

void ArmsManager::callback_command_force_right_aux(const shared_msgs::CommandTrajectory& msg)
{
  sensitivity_axis_right[0].store(msg.fx_neg);
  sensitivity_axis_right[1].store(msg.fx_pos);
  sensitivity_axis_right[2].store(msg.fy_neg);
  sensitivity_axis_right[3].store(msg.fy_pos);
  sensitivity_axis_right[4].store(msg.fz_neg);
  sensitivity_axis_right[5].store(msg.fz_pos);
}


bool ArmsManager::gravityCompensation(float mass, geometry_msgs::WrenchStamped msg, tf::Vector3& f)
{
	tf::Vector3 weight_force_world_frame(0.0,0.0,-9.81*mass);

	//get the current transform between world and sensor frame
	tf::StampedTransform transform;
	try
	{
	  tf_listener_.waitForTransform(world_frame, msg.header.frame_id, msg.header.stamp, ros::Duration(0.1));
	  tf_listener_.lookupTransform(world_frame, msg.header.frame_id, msg.header.stamp, transform);
	}
	catch (const std::exception& e)
	{
	  ROS_WARN_STREAM("error in arms manager " << e.what());
	  return false;
	}

	tf::Matrix3x3 R_world2sensor = transform.getBasis().transpose();

	tf::Vector3 weight_force_sensor_frame;
	weight_force_sensor_frame[0] = R_world2sensor[0][2]*weight_force_world_frame[2];
	weight_force_sensor_frame[1] = R_world2sensor[1][2]*weight_force_world_frame[2];
	weight_force_sensor_frame[2] = R_world2sensor[2][2]*weight_force_world_frame[2];

	tf::Vector3 meas(msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z);
	f = meas - weight_force_sensor_frame;

	return true;
}

void ArmsManager::FTsensor_callback_left(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	ROS_DEBUG_STREAM("FTsensor_callback_left");

	tf::Vector3 meas;
	bool compensation_ok;
	compensation_ok = gravityCompensation(ee_mass_left, *msg, meas);
	if (!compensation_ok)
		return;

	ROS_DEBUG_STREAM("force compensed: [" << meas[0] << "," << meas[1] << "," << meas[2] << "]");

	if (calibration_counter_left_ <= calibration_number)
	{
		//calibrate
		if (calibration_counter_left_ <= calibration_number)
		{
		    bias_force_left_ = bias_force_left_*calibration_counter_left_ + meas;
		    calibration_counter_left_++;
		    bias_force_left_ /= calibration_counter_left_;
		    ROS_INFO_STREAM("#" << calibration_counter_left_ << ": actual estimation " << bias_force_left_[0] << "," << bias_force_left_[1] << "," << bias_force_left_[2] <<"]");
		}

		if (calibration_counter_left_ == calibration_number)
		    ROS_INFO_STREAM("Calibrated left sensor. Estimated bias: [" << bias_force_left_[0] << "," << bias_force_left_[1] << "," << bias_force_left_[2] <<"]");
	}
	else
	{
		meas = meas - bias_force_left_;
		ROS_DEBUG_STREAM("force unbiased and compensed: [" << meas[0] << "," << meas[1] << "," << meas[2] << "]");
		
		//FIXME To be optimized by using functions and standard maps. To be trusted!!
		bool stop_left = false;
		bool stop_right = false;
		int force_flag = shared_msgs::CommandTrajectory::NOT_SENSED;
		
		for (int i = 0; i < 2; ++i)
		{
		  //if the axis should not be sensed, continue
		  if(sensitivity_axis_left[2*i].load() != shared_msgs::CommandTrajectory::NOT_SENSED)
		  {
		    //check negative axis
		    if ( meas[i] < -force_ths[i].load() )
		    {
		      if (sensitivity_axis_left[2*i].load() == shared_msgs::CommandTrajectory::SENSED_TRANSITION)
		      {
			//detected a desired contact. State transition.
			force_flag = (force_flag ==  shared_msgs::FeedbackTrajectory::SENSED_ERROR) ? force_flag : shared_msgs::FeedbackTrajectory::SENSED_TRANSITION;
			
			//stop just the left arm
			stop_left = true;
		      }
		      else
		      {
			//undesired contact
			force_flag = shared_msgs::FeedbackTrajectory::SENSED_ERROR;
			
			//stop both the arm
			stop_left = true;
			stop_right = true;
			
			break;
		      }
		    }
		  }
		  
		  if(sensitivity_axis_left[2*i+1].load() != shared_msgs::CommandTrajectory::NOT_SENSED)
		  {
		    //check positive axis
		    if ( meas[i] > force_ths[i].load() )
		    {
		      if (sensitivity_axis_left[2*i+1].load() == shared_msgs::CommandTrajectory::SENSED_TRANSITION)
		      {
			//detected a desired contact. State transition.
			force_flag = (force_flag ==  shared_msgs::FeedbackTrajectory::SENSED_ERROR) ? force_flag : shared_msgs::FeedbackTrajectory::SENSED_TRANSITION;
			
			//stop just the left arm
			stop_left = true;
		      }
		      else
		      {
			//undesired contact
			force_flag = shared_msgs::FeedbackTrajectory::SENSED_ERROR;
			
			//stop both the arm
			stop_left = true;
			stop_right = true;
			
			break;
		      }
		    }
		  }
		}
		
		//Update the flags and stop the arms eventually
		force_flag_left.store(force_flag);
		left_arm_stopped.store(stop_left);
		if (force_flag == shared_msgs::FeedbackTrajectory::SENSED_ERROR)
		  right_arm_stopped.store(stop_right); //if stopped due to an error 
	}
}

void ArmsManager::FTsensor_callback_right(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	ROS_DEBUG_STREAM("FTsensor_callback_right");

	tf::Vector3 meas;
	bool compensation_ok;
	compensation_ok = gravityCompensation(ee_mass_right,*msg,meas);
	if (!compensation_ok)
		return;

	ROS_DEBUG_STREAM("force compensed: ["<<meas[0]<<","<<meas[1]<<","<<meas[2]<<"]");

	if (calibration_counter_right_ <= calibration_number)
	{
		//calibrate
		if (calibration_counter_right_ <= calibration_number)
		{
		    bias_force_right_ = bias_force_right_*calibration_counter_right_ + meas;
		    calibration_counter_right_++;
		    bias_force_right_ /= calibration_counter_right_;
		    ROS_INFO_STREAM("#" << calibration_counter_right_ << ": actual estimation " << bias_force_right_[0] << "," << bias_force_right_[1] << "," << bias_force_right_[2] <<"]");
		}
		if (calibration_counter_right_ == calibration_number)
		    ROS_INFO_STREAM("Calibrated right sensor. Estimated bias: [" << bias_force_right_[0] << "," << bias_force_right_[1] << "," << bias_force_right_[2] <<"]");
	}
	else
	{
		meas = meas - bias_force_right_;
		ROS_DEBUG_STREAM("force unbiased and compensed: ["<<meas[0]<<","<<meas[1]<<","<<meas[2]<<"]");
		
		bool stop_left = false;
		bool stop_right = false;
		int force_flag = shared_msgs::CommandTrajectory::NOT_SENSED;
		
		for (int i = 0; i < 2; ++i)
		{
		  //if the axis should not be sensed, continue
		  if(sensitivity_axis_right[2*i].load() != shared_msgs::CommandTrajectory::NOT_SENSED)
		  {
		    //check negative axis
		    if ( meas[i] < -force_ths[i].load() )
		    {
		      if (sensitivity_axis_right[2*i].load() == shared_msgs::CommandTrajectory::SENSED_TRANSITION)
		      {
			//detected a desired contact. State transition.
			force_flag = (force_flag ==  shared_msgs::FeedbackTrajectory::SENSED_ERROR) ? force_flag : shared_msgs::FeedbackTrajectory::SENSED_TRANSITION;
			
			//stop just the right arm
			stop_right = true;
		      }
		      else
		      {
			//undesired contact
			force_flag = shared_msgs::FeedbackTrajectory::SENSED_ERROR;
			
			//stop both the arm
			stop_left = true;
			stop_right = true;
			
			break;
		      }
		    }
		  }
		  
		  if(sensitivity_axis_right[2*i+1].load() != shared_msgs::CommandTrajectory::NOT_SENSED)
		  {
		    //check positive axis
		    if ( meas[i] > force_ths[i].load() )
		    {
		      if (sensitivity_axis_right[2*i+1].load() == shared_msgs::CommandTrajectory::SENSED_TRANSITION)
		      {
			//detected a desired contact. State transition.
			force_flag = (force_flag ==  shared_msgs::FeedbackTrajectory::SENSED_ERROR) ? force_flag : shared_msgs::FeedbackTrajectory::SENSED_TRANSITION;
			
			//stop just the left arm
			stop_right = true;
		      }
		      else
		      {
			//undesired contact
			force_flag = shared_msgs::FeedbackTrajectory::SENSED_ERROR;
			
			//stop both the arm
			stop_left = true;
			stop_right = true;
			
			break;
		      }
		    }
		  }
		}
		
		//Update the flags and stop the arms eventually
		force_flag_right.store(force_flag);
		right_arm_stopped.store(stop_left);
		if (force_flag == shared_msgs::FeedbackTrajectory::SENSED_ERROR)
		  left_arm_stopped.store(stop_left); //if stopped due to an error 
	}
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

void ArmsManager::emergency_callback(const std_msgs::Bool& msg)
{
  //stop both the arms
  left_arm_stopped.store(msg.data);
  right_arm_stopped.store(msg.data);
  
  //FIXME communicate to matlab the event separately
  force_flag_left.store(shared_msgs::FeedbackTrajectory::SENSED_ERROR);
  force_flag_right.store(shared_msgs::FeedbackTrajectory::SENSED_ERROR);
}

void ArmsManager::run()
{
  ros::Rate f(100);

  while(ros::ok())
  {
      //publish commands to arms
      traj_left_mutex.lock();
      pub_command_left.publish(traj_left);
      traj_left_mutex.unlock();

      traj_right_mutex.lock();
      pub_command_right.publish(traj_right);
      traj_right_mutex.unlock();

      //publish force flags
      if (use_force_sensor_right_)
      {
	shared_msgs::FeedbackTrajectory msg_right;
	msg_right.time = ros::Time::now();
	msg_right.value = force_flag_right.load();
	pub_feedback_right.publish(msg_right);
      }

      if (use_force_sensor_left_)
      {
	shared_msgs::FeedbackTrajectory msg_left;
	msg_left.time = ros::Time::now();
	msg_left.value = force_flag_left.load();
	pub_feedback_left.publish(msg_left);
      }

      if (!use_force_sensor_right_ && !use_force_sensor_left_)
	      ros::spinOnce();

      f.sleep();
  }

}
