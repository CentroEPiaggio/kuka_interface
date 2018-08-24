rosshutdown
rosinit('http://192.168.0.150:11311')

%% definition of pub sub and msgs
% pub_trajectory_right = rospublisher('/right_arm/joint_trajectory_controller/command_aux', 'trajectory_msgs/JointTrajectory');
% msg_trajectory_right = rosmessage(pub_trajectory_right);
% 
% pub_trajectory_left = rospublisher('/left_arm/joint_trajectory_controller/command_aux', 'trajectory_msgs/JointTrajectory');
% msg_trajectory_left = rosmessage(pub_trajectory_left);
% 
% msg_trajectory_right.JointNames = {'right_arm_a1_joint', 'right_arm_a2_joint', 'right_arm_e1_joint', 'right_arm_a3_joint', 'right_arm_a4_joint', 'right_arm_a5_joint', 'right_arm_a6_joint'};
% joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% joint_send.Positions = zeros(1,7); %number of joints
% joint_send.Velocities = zeros(1,7);
% 
% msg_trajectory_left.JointNames = {'left_arm_a1_joint', 'left_arm_a2_joint', 'left_arm_e1_joint', 'left_arm_a3_joint', 'left_arm_a4_joint', 'left_arm_a5_joint', 'left_arm_a6_joint'};
% joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% joint_send.Positions = zeros(1,7); %number of joints
% joint_send.Velocities = zeros(1,7);
% joint_send.TimeFromStart.Sec = 0;
% joint_send.TimeFromStart.Nsec = 5000000;
% 
% msg_trajectory_right.Points = joint_send;       
% msg_trajectory_right.Header.Seq = 0;
% 
% msg_trajectory_left.Points = joint_send;       
% msg_trajectory_left.Header.Seq = 0;

%% 
%home joints positions
load('q_0_right')
load('q_0_left')

%matrix of numbers of steps for each state's trajectory. Column 1 : right
%arm, Column 2: left arm, rows are the states.
t_prova = [800,800;800,800];
%to detect when the robot has reached last point of its trajectory
pos_threshold = 0.1;