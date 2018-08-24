%% clean workspace

% close all
% clear all
% clc

robot_ID = 'TWO_ARMS_r';

TWO_ARMS_transformations

VITO_geometry_and_kinematics

q_0 = [0.0; -0.3; 0.0; -0.3; 0.0; -0.7; 0.0];
qd_0 = zeros(7, 1);

T_b_DH0 = T_b_DH0r;
T_DH7_ee = T_DH7r_eer;

%% set parameters algorithm

N = 17;
Ts = 0.1;

% step parameters
    
    DPI_lambda_max = 0.1*10^4; 	% damping for pinv
    DPI_epsilon = 0.1;          % bound for pinv

    beta_pos = 0.1;
    beta_vel = 0.1;

    lambda = 0.9;               % damping lambda for task N+1
    
    % for bags vito (not fast circle): 0.3 0.1
    kp = 0.5;
    ko = 20.0;                   % orientation error gain
%     k3 = 1;
    K = [ones(1,14), kp, ko, 0.05];  	% error gain vector

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

    z_link_3_max = 1.73;
    
    % find home position for setting up
    
    %% define trajectory and orientation       
        q_sym = sym('q', [length(q_0) 1]);
        DH_table_num = double(subs(DH_table_sym, q_sym, q_0));
    
        [~, Tee_home] = direct_kinematics_DH(DH_table_num);

        % use pre and post transformations
        Tee_home = T_b_DH0 * Tee_home * T_DH7_ee;
        
        x_home = Tee_home(1:3,4);
        R_home = Tee_home(1:3,1:3);
        
        t_prova = 2000;

        x_final = [1.522; x_home(2); 1.5];
        
        x_intermediate = [x_home(1)+0.03, x_home(2), x_home(3)+0.3];
        
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
            
            ZYX = [0, 0, 0];        %angoli riferiti a terna link 7
        
        R2 = eul2rotm([pi, 0, 0], 'ZYX');
        R_prova = eul2rotm(ZYX, 'ZYX');
        R_pre = eul2rotm([pi, -pi/2, 0], 'ZYX');
%         R_prova = eye(3);

        R_prova = R_pre*eul2rotm(ZYX, 'ZYX')*R2;

        theta = generate_line_points([0, -pi/3, 0], [0, -2*pi/180, 0], 2000);
        
        for j = 1 : t_prova
            x_or_ee_des(:, :, j) = R_pre*eul2rotm(theta(:,i)', 'ZYX')*R2;
            x_pos_ee_des(:, :, j) = traj(:,j);
        end
        
        iter_num = t_prova;
        
        x_des = cell(N, iter_num);  % init for speed
        for k = 1 : iter_num
            x_des(:,k) = {  xee_max; xee_min; xj7_max; xj7_min; xj6_max; ...
                        xj6_min; xj5_max; xj5_min; xj4_max; xj4_min; ...
                        xj3_max; xj3_min; xj2_max; xj2_min; ...
                        x_pos_ee_des(:,k); x_or_ee_des(:,:,k); z_link_3_max};
        end        
        
        % variables for RP algorithm
    
    % flag showing if p is a task or a constraint  
    unil_constr = [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 0, 0, 2];
    
    % constraint value (NaN when not present)
    x_cons = [  xee_max, xee_min, xj7_max, xj7_min, xj6_max, xj6_min, ...
                xj5_max, xj5_min, xj4_max, xj4_min, xj3_max, xj3_min, ...
                xj2_max, xj2_min, NaN, NaN, z_link_3_max];
 
%% algorithm
J_and_T_hand = def_JT_handle(robot_ID);

% execute algorithm
[q_out_right_home, qd_out_right, e_out_right] = reverse_priority_7j_z(N, Ts, iter_num, ...
                                                        J_and_T_hand, ...
                                                        q_0, qd_0, x_des, ...
                                                        unil_constr, ...
                                                        x_cons, param_vect);
            %%                            
        plot_errors(e_out_right, iter_num);
        
        q_0_right = q_out_right_home(:,iter_num);
                                                    
        savefile = 'q_0_right.mat';
        save(savefile,'q_0_right');
        
        savefile = 'q_out_right_home.mat';
        save(savefile,'q_out_right_home');