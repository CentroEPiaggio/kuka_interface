%{
===========================================================================
This script executes the bag1
===========================================================================
%}

%% set folders path

RP_project_paths

%% clean workspace

close all
clear all
clc
    
%% main script name

script_name = mfilename;

%% initialize KUKA arms

robot_ID = 'TWO_ARMS_r';

TWO_ARMS_transformations

VITO_geometry_and_kinematics

%% starting condition

q_0_r = [0.1; -1.47; -0.1; -pi/4; 0.1; pi/4; -0.1];
q_0 = q_0_r;

qd_0 = zeros(7,1);

T_b_DH0 = T_b_DH0r;
T_DH7_ee = T_DH7r_eer;

%% init test

% algorithm parameters

    N = 16;                  	% tasks number
    %N = 2;                  	% tasks number

    Ts = 0.0005;                	% sampling time

% step parameters
    
    DPI_lambda_max = 0.1*10^4; 	% damping for pinv
    DPI_epsilon = 0.1;          % bound for pinv

    beta_pos = 0.001;
    beta_vel = 0.1;

    lambda = 0.9;               % damping lambda for task N+1
    
    % for bags vito (not fast circle): 0.3 0.1
    kp = 0.8;
    ko = 2.8;                   % orientation error gain
    K = [ones(1,14), kp, ko];  	% error gain vector
    %K = [kp, ko];               % error gain vector

% whole parameters vector
param_vect = [DPI_lambda_max, DPI_epsilon, beta_pos, beta_vel, lambda, K];
    
%% tasks definition

    xj2_max = kuka_jmax(1);    	% max constraint
    xj2_min = kuka_jmin(1);   	% min constraint
    
    xj3_max = kuka_jmax(2);    	% max constraint
    xj3_min = kuka_jmin(2);    	% min constraint
    
    xj4_max = kuka_jmax(3);  	% max constraint
    xj4_min = kuka_jmin(3);    	% min constraint
    
    xj5_max = kuka_jmax(4);    	% max constraint
    xj5_min = kuka_jmin(4);    	% min constraint

    xj6_max = kuka_jmax(5);   	% max constraint
    xj6_min = kuka_jmin(5);   	% min constraint
    
    xj7_max = kuka_jmax(6);    	% max constraint
    xj7_min = kuka_jmin(6);     % min constraint
    
    xee_max = kuka_jmax(7);    	% max constraint
    xee_min = kuka_jmin(7);    	% min constraint


    bag = rosbag('new_sliding.bag');

    TWO_ARMS_read_bag1n2; 
   
% find home position for setting up
    
    % compute using direct kinematics         
        q_sym = sym('q', [length(q_0) 1]);
        DH_table_num = double(subs(DH_table_sym, q_sym, q_0));
    
        [~, Tee_home] = direct_kinematics_DH(DH_table_num);

        % use pre and post transformations
        Tee_home = T_b_DH0 * Tee_home * T_DH7_ee;
        
        x_home = Tee_home(1:3,4);
        R_home = Tee_home(1:3,1:3);
        
% go to a first intermediate point
    
    T_near2 = 500;                 % ex: 200 - relative
    t_near2 = T_near2;              % absolute
    
    % with displacement
    endeff_dim = 0.2;
    displ_x = 0.2;
    displ_y = 0.3;
    displ_z = -0;
    
    x_near2 = p_b_eer_b(:,1);
    x_near2(1) = x_near2(1) + displ_x; 
    x_near2(2) = x_near2(2) + displ_y; 
    x_near2(3) = x_near2(3) + displ_z + 0.1;      % go 20 cm higher above bag
    
    R_near2 = R_b_eer_b(:,:,1);
 
    %x_pos_near2 = generate_line_points(x_near1, x_near2, T_near2);
    x_pos_near2 = NaN(3, t_near2);      % init for speed
    for i = 1 : t_near2
        x_pos_near2(:,i) = x_near2;
    end
    
    x_or_ee_des = NaN(3, 3, t_near2);   % init for speed 
    for i = 1 : t_near2
        x_or_ee_des(:,:,i) = R_near2;
    end

% go to first pose

    T_pose = 150;                   % relative
    t_pose = t_near2 + T_pose;   	% absolute
    
    x_pose = p_b_eer_b(:,1);
    x_pose(1) = x_pose(1) + displ_x;
    x_pose(2) = x_pose(2) + displ_y;
    x_pose(3) = x_pose(3) + displ_z;
    R_pose = R_b_eer_b(:,:,1);

    x_pos_pose = generate_line_points(x_near2, x_pose, T_pose);

    for i = t_near2+1 : t_pose
        x_or_ee_des(:,:,i) = R_pose;
    end
    
% execute bag trajectory
    
    T_bag = endeff_dim * 1000;     	% relative
    t_bag = t_pose + T_bag;         % absolute
    
    x_posebag = x_pose;
    x_posebag(1) = x_posebag(1) - endeff_dim;
    R_posebag = R_b_eer_b(:,:,1);
    
    x_pos_bag = generate_line_points(x_pose, x_posebag, T_bag);
    
    for i = t_pose+1 : t_bag
        x_or_ee_des(:,:,i) = R_posebag;
    end

% go back home

    T_back = 300;                   % relative
    t_back = t_bag + T_back;        % absolute
    
    x_back = x_home;
    R_back = R_b_eer_b(:,:,1);

    x_pos_back = generate_line_points(x_pos_bag(:,end), x_back, T_back);

    for i = t_bag+1 : t_back
        x_or_ee_des(:,:,i) = R_back;
    end

% whole trajectory  

    x_pos_ee_des = [x_pos_near2, x_pos_pose, x_pos_bag, x_pos_back];
    
    iter_num = t_back;

% all N tasks
    x_des = cell(N, iter_num);  % init for speed
    for k = 1 : iter_num
        x_des(:,k) = {  xee_max; xee_min; xj7_max; xj7_min; xj6_max; ...
                        xj6_min; xj5_max; xj5_min; xj4_max; xj4_min; ...
                        xj3_max; xj3_min; xj2_max; xj2_min; ...
                        x_pos_ee_des(:,k); x_or_ee_des(:,:,k)};
    end

% variables for RP algorithm
    
    % flag showing if p is a task or a constraint  
    unil_constr = [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 0, 0];
    
    % constraint value (NaN when not present)
    x_cons = [  xee_max, xee_min, xj7_max, xj7_min, xj6_max, xj6_min, ...
                xj5_max, xj5_min, xj4_max, xj4_min, xj3_max, xj3_min, ...
                xj2_max, xj2_min, NaN, NaN];
 
%% algorithm

% define function handles of J and T for the fast version
J_and_T_hand = def_JT_handle(robot_ID);

% execute algorithm
[q_out, qd_out, e_out] = reverse_priority_pos_or_7j( 	N, Ts, iter_num, ...
                                                        J_and_T_hand, ...
                                                        q_0, qd_0, x_des, ...
                                                        unil_constr, ...
                                                        x_cons, param_vect);
                                                    
        plot_errors(e_out, iter_num);
                                                    
        DH_table_num_out1 = double(subs(DH_table_sym, q_sym, q_out(:,iter_num)));     %tabella DH
    
        [~, Tee_out] = direct_kinematics_DH(DH_table_num_out1);

        % use pre and post transformations
        Tee_out = T_b_DH0 * Tee_out * T_DH7_ee;
        
        x_out = Tee_out(1:3,4);       %posizione ee calcolata con cinematica diretta
        R_out = Tee_out(1:3,1:3);

plot_q
plot_qd
