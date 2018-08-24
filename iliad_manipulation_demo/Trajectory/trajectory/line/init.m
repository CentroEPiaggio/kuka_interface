rosshutdown
rosinit('http://192.168.0.150:11311')

%% definition of pub sub and msgs
% pub_trajectory_right = rospublisher('/right_arm/joint_trajectory_controller/command_aux', 'trajectory_msgs/JointTrajectory');
% msg_trajectory_right = rosmessage(pub_trajectory_right);
% 
% msg_trajectory_right.JointNames = {'right_arm_a1_joint', 'right_arm_a2_joint', 'right_arm_e1_joint', 'right_arm_a3_joint', 'right_arm_a4_joint', 'right_arm_a5_joint', 'right_arm_a6_joint'};
% joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% joint_send.Positions = zeros(1,7); %number of joints
% joint_send.Velocities = zeros(1,7);
% 
% joint_send.TimeFromStart.Sec = 0;
% joint_send.TimeFromStart.Nsec = 5000000;
% 
% msg_trajectory_right.Points = joint_send;       
% msg_trajectory_right.Header.Seq = 0;

%%
load('q_0_right')
disp = 0.2;
x_home =[1.5223, 0.8010, 1.4997]';

wp1_pos = x_home;
wp1_rot = [0, -pi/2, 0]';

wp2_pos = wp1_pos + [disp,0,0]';
wp2_rot = [0, -pi/2, 0]';

wp3_pos = wp1_pos;
wp3_rot = [0, -pi/2, 0]';
wp = [[wp1_pos;wp1_rot],[wp2_pos;wp2_rot], [wp3_pos; wp3_rot]];
t_prova = [800,800,800];
pos_threshold = 0.1;