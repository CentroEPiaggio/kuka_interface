%% clean workspace

% close all
% clear all
% clc

robot_ID = 'TWO_ARMS_l';

TWO_ARMS_transformations

VITO_geometry_and_kinematics

q_0_left = [0.0; 0.0; -0.3; -0.3; 0.0; 0.0; -0.1];
qd_0_left = zeros(7, 1);

T_b_DH0 = T_b_DH0l;
T_DH7_ee = T_DH7l_eel;

%% set parameters algorithm

N = 17;
Ts = 0.1;

% step parameters
    
    DPI_lambda_max = 0.1*10^4; 	% damping for pinv
    DPI_epsilon = 0.1;          % bound for pinv

    beta_pos = 0.01;
    beta_vel = 0.01;

    lambda = 0.9;               % damping lambda for task N+1
    
    % for bags vito (not fast circle): 0.3 0.1
    kp = 0.8;
    ko = 0.8;                   % orientation error gain
%     k3 = 1;
    K = [ones(1,14), kp, ko, 1];  	% error gain vector

% whole parameters vector
param_vect = [DPI_lambda_max, DPI_epsilon, beta_pos, beta_vel, lambda, K];
        
%% task definition

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

    z_link_3 = 0.72;
    
    % find home position for setting up
    
    % compute using direct kinematics         
        q_sym = sym('q', [length(q_0_left) 1]);
        DH_table_num = double(subs(DH_table_sym, q_sym, q_0_left));
    
        [~, Tee_home] = direct_kinematics_DH(DH_table_num);

        % use pre and post transformations
        Tee_home = T_b_DH0 * Tee_home * T_DH7_ee;
        
        x_home = Tee_home(1:3,4);
        R_home = Tee_home(1:3,1:3);
        
        t_prova = 2000;

        x_final = [0.95; 0.801; 0.580];

%         x_final = [0.882; 1.200; 0.7006];

        x_intermediate = [x_home(1)-0.15, x_home(2), x_home(3)+0.05];
        
        x_pos = generate_line_points(x_home, x_final, t_prova);
        
        x_data = [x_home'; x_intermediate; x_final'];
        
        t_samples = 0 : 2;
        t = 0 : 0.001 : (2-0.001);
        
        ppx = spline(t_samples, x_data(:,1), t);
        ppy = spline(t_samples, x_data(:,2), t);
        ppz = spline(t_samples, x_data(:,3), t);
        
        for i = 1:size(t,2)
    
            traj(:,i) = [ppx(i);ppy(i);ppz(i)];
    
        end
        
        R = plot3(  x_data(:,1),x_data(:,2),x_data(:,3),'o',...
                traj(1,:),traj(2,:),traj(3,:),...
                'linewidth',2,'Color','k');
            
            ZYX = [0, 0, 0];     %angoli riferiti a posizione orizzontale della piastra
            R_off = eul2rotm([0, 0, 0], 'ZYX');
            R_pre = eul2rotm([-pi/2, 0, -pi/2], 'ZYX');
            R2 = eul2rotm([pi, 0, 0], 'ZYX');
            R1 = eul2rotm([0, 0, pi/6], 'ZYX');
        
            R_prova = R_pre*eul2rotm(ZYX, 'ZYX')*R1*R2*R_off;
            
            theta_traj = generate_line_points([0, 0, -pi/6], [0, 0, -pi/6], 1000);
            theta_traj = [theta_traj, generate_line_points([0, 0, -pi/6], ZYX, 1000)];
            
        for j = 1 : t_prova
            x_or_ee_des(:, :, j) = R_pre*eul2rotm(theta_traj(:,j)', 'ZYX')*R1*R2*R_off;
            x_pos_ee_des(:, :, j) = traj(:,j);
        end
        
        iter_num = t_prova;
        
        x_des = cell(N, iter_num);  % init for speed
        for k = 1 : iter_num
            x_des(:,k) = {  xee_max; xee_min; xj7_max; xj7_min; xj6_max; ...
                        xj6_min; xj5_max; xj5_min; xj4_max; xj4_min; ...
                        xj3_max; xj3_min; xj2_max; xj2_min; ...
                        x_pos_ee_des(:,k); x_or_ee_des(:,:,k); z_link_3};
        end        
        
        % variables for RP algorithm
    
    % flag showing if p is a task or a constraint  
    unil_constr = [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 0, 0, 1];
    
    % constraint value (NaN when not present)
    x_cons = [  xee_max, xee_min, xj7_max, xj7_min, xj6_max, xj6_min, ...
                xj5_max, xj5_min, xj4_max, xj4_min, xj3_max, xj3_min, ...
                xj2_max, xj2_min, NaN, NaN, z_link_3];
 
%% algorithm
J_and_T_hand = def_JT_handle(robot_ID);

% execute algorithm
[q_out_left_home, qd_out_left, e_out_left] = reverse_priority_7j_z(N, Ts, iter_num, ...
                                                        J_and_T_hand, ...
                                                        q_0_left, qd_0_left, x_des, ...
                                                        unil_constr, ...
                                                        x_cons, param_vect);
                                        
        plot_errors(e_out_left, iter_num);
                                                    
        DH_table_num_out1 = double(subs(DH_table_sym, q_sym, q_out_left_home(:,iter_num)));     %tabella DH
    
        [~, Tee_out] = direct_kinematics_DH(DH_table_num_out1);

        % use pre and post transformations
        Tee_out = T_b_DH0 * Tee_out * T_DH7_ee;
        
        x_out_left = Tee_out(1:3,4);       %posizione ee calcolata con cinematica diretta

        
        %%
        q_0_left = q_out_left_home(:, iter_num);

        savefile = 'q_out_left_home.mat';
        save(savefile,'q_out_left_home');
        
        savefile = 'q_0_left.mat';
        save(savefile,'q_0_left');
