% clear all
% clcS

% left_home;
% right_home;

% q_out_right_home = [2.5165; 1.8225; -0.7225; 2.0591; -0.3041; 0.9122; -2.1925];
% q_out_left_home = [-0.1418; -1.2722; -0.0166; -2.0250; 0.4926; 0.2960; -2.5104];

load('q_out_right_home');
load('q_out_left_home.mat'); 

%% publish

rosshutdown
rosinit

pub_trajectory = rospublisher('/left_arm/joint_trajectory_controller/command', 'trajectory_msgs/JointTrajectory');
msg_trajectory = rosmessage(pub_trajectory);
pub_trajectory_right = rospublisher('/right_arm/joint_trajectory_controller/command', 'trajectory_msgs/JointTrajectory');
msg_trajectory_right = rosmessage(pub_trajectory_right);

msg_trajectory.JointNames = {'left_arm_a1_joint', 'left_arm_a2_joint', 'left_arm_e1_joint', 'left_arm_a3_joint', 'left_arm_a4_joint', 'left_arm_a5_joint', 'left_arm_a6_joint'};
msg_trajectory_right.JointNames = {'right_arm_a1_joint', 'right_arm_a2_joint', 'right_arm_e1_joint', 'right_arm_a3_joint', 'right_arm_a4_joint', 'right_arm_a5_joint', 'right_arm_a6_joint'};

joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
joint_send.Positions = zeros(1,7);
joint_send.Velocities = zeros(1,7);
% joint_send.TimeFromStart = rosduration(0);
joint_send.TimeFromStart.Sec = 0;
joint_send.TimeFromStart.Nsec = 5000000;

msg_trajectory.Points = joint_send;
msg_trajectory_right.Points = joint_send;
msg_trajectory.Header.Seq = 0;
msg_trajectory_right.Header.Seq = 0;

%% mano
pub_hand = rospublisher('/right_hand/joint_trajectory_controller/command', 'trajectory_msgs/JointTrajectory');
msg_hand = rosmessage(pub_hand);

msg_hand.JointNames = {'right_hand_synergy_joint'};

hand_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
hand_send.Positions = 0;
hand_send.Velocities = 0;
% joint_send.TimeFromStart = rosduration(0);
hand_send.TimeFromStart.Sec = 0;
hand_send.TimeFromStart.Nsec = 50000000;

msg_hand.Points = hand_send;
msg_trajectory_right.Points = joint_send;
msg_hand.Header.Seq = 0;

for j = 1 : 2000
    msg_trajectory.Header.Seq = j;
%     msg_trajectory.Points.Positions = q_out_left(:,j)';
    msg_trajectory.Points.Positions = q_out_left_home(:,j)';
%     msg_trajectory.Points.Velocities = qd_out(:,j)';
    send(pub_trajectory,msg_trajectory);
%     rosrate(1000);
    msg_trajectory_right.Header.Seq = j;
%     msg_trajectory_right.Points.Positions = q_out_right2(:,j)';
    msg_trajectory_right.Points.Positions = q_out_right_home(:,j)';
%     msg_trajectory_right.Points.Velocities = qd_out(:,j)';
    send(pub_trajectory_right,msg_trajectory_right);
    msg_hand.Header.Seq = j;
    msg_hand.Points.Positions = 0;
    send(pub_hand,msg_hand);
    pause(0.0001);
end

rosshutdown
