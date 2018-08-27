function [q_out_right] = traj_line_0(t_prova, q_0_right_, wp2_pos, wp2_rot)
%% clean workspace
q_0_right = zeros(1,7);
for i = 1 : 7
    q_0_right(i) = q_0_right_(i);
end

robot_ID = 'TWO_ARMS_r';

%% q
qd_0 = zeros(7, 1);
  
%%   Initialize KUKA transformation
% T_b_0 = T_b_DH0 expressed in b

% from base to 0right = DH_0
    
        ypr_b_0r = [0, -0.7854, pi];

        R_b_0r = eul2rotm(ypr_b_0r, 'ZYX');
        R_b_0r = R_b_0r;
        
        p_b_0r = [  0.77; ...
                    0.801; ...
                    1.607];

        T_b_0r = [[R_b_0r, p_b_0r]; [0 0 0 1]]; 
        T_b_DH0r = T_b_0r;

    % from base to 0left = DH_0
    
        ypr_b_0l = [pi, 0.5236, pi];

        R_b_0l = eul2rotm(ypr_b_0l, 'ZYX');

        p_b_0l = [  0.57; ...
                    0.801; ...
                    1.307];
                
        T_b_0l = [[R_b_0l, p_b_0l]; [0 0 0 1]];
        T_b_DH0l = T_b_0l;

% T_DH7_ee & T_ee_DH7

    % from DH7 to ee_right --> Rz(pi)
        T_DH7r_eer = [[[-1 0 0; 0 -1 0; 0 0 1], zeros(3,1)]; [0 0 0 1]];
        T_eer_DH7r = inv(T_DH7r_eer);

        T_DH7l_eel = [[[-1 0 0; 0 -1 0; 0 0 1], zeros(3,1)]; [0 0 0 1]];
        T_eel_DH7l = inv(T_DH7l_eel);
        

%% parameters for reverse priority algorithm
N = 16;
Ts = 0.1;
DPI_lambda_max = 0.1*10^4; 	% damping for pinv
DPI_epsilon = 0.1;          % bound for pinv

beta_pos = 0.01;
beta_vel = 0.1;

lambda = 0.9;    

kp = 0.8;
ko = 0.5;                   % orientation error gain
K = [ones(1,14), kp, ko];  	% error gain vector
T_b_DH0 = T_b_DH0r;
T_DH7_ee = T_DH7r_eer;

%% whole parameters vector
param_vect = [DPI_lambda_max, DPI_epsilon, beta_pos, beta_vel, lambda, K];
    
%task definition
kuka_jmax = [170, 120, 170, 120, 170, 120, 170] * 2*pi / 360;   % official
kuka_jmin = -kuka_jmax;                                         % official

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
    
%% compute using direct kinematics     
DH_table_num = KUKA_LWR_DHtable(q_0_right);
[~, Tee_home] = direct_kinematics_DH(DH_table_num);

%% use pre and post transformations
Tee_home = T_b_DH0 * Tee_home * T_DH7_ee;
R_home = Tee_home(1:3,1:3);

%% transformations
R2 = eul2rotm([pi, 0, 0], 'ZYX');
R_pre = eul2rotm([pi, -pi/2, 0], 'ZYX'); 
ZYX_0 = [0,0,0];
ZYX_1 = wp2_rot;
theta_traj = generate_line_points(ZYX_0, ZYX_1, t_prova);
traj = generate_line_points(Tee_home(1:3,4), wp2_pos, t_prova);

for i = 1:t_prova
    x_or_ee_des(:, :, i) = R_pre*eul2rotm(theta_traj(:, i)', 'ZYX')*R2;
end
iter_num = t_prova;
 
x_des = cell(N, iter_num);  % init for speed
        for k = 1 : iter_num
            x_des(:,k) = {  xee_max; xee_min; xj7_max; xj7_min; xj6_max; ...
                        xj6_min; xj5_max; xj5_min; xj4_max; xj4_min; ...
                        xj3_max; xj3_min; xj2_max; xj2_min; ...
                        traj(:,k); x_or_ee_des(:,:,k)};
        end        
        unil_constr = [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 0, 0];

% constraint value (NaN when not present)
x_cons = [  xee_max, xee_min, xj7_max, xj7_min, xj6_max, xj6_min, ...
            xj5_max, xj5_min, xj4_max, xj4_min, xj3_max, xj3_min, ...
            xj2_max, xj2_min, NaN, NaN];

% define function handles of J and T for the fast version
J_and_T_hand = def_JT_handle(robot_ID);

%coder.extrinsic('reverse_priority_pos_or_7j')
% execute algorithm
[q_out_right, qd_out_right, e_out_right] = reverse_priority_pos_or_7j( N, Ts, iter_num, ...
                                                        J_and_T_hand, ...
                                                        q_0_right, qd_0, x_des, ...
                                                        unil_constr, ...
                                                        x_cons, param_vect);
end

