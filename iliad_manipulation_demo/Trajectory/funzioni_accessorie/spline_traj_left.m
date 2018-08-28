function q_out = spline_traj_left(t_prova, q_0_right_, wp2_pos, wp2_rot)

%t_prova = number of samples of the trajectory.
%q_0_right= iniital position
%wp2_pos = list of waypoints coordinates
%wp_rot = desired attitude in home ee frame
%this function creates a joint position trajectory starting from q_0_right.
%the cartesian trtajectory passes by all positions in wp_pos. The cartesian attitude is a
%generate_line_trajectory between the first attitude (corresponding to the
%ee orientation when the robot is in q_0_right) and the attitude described
%by wp_rot

%number of waypoints
wp_num = length(wp2_pos)/3;
t_samples = 0 : wp_num;
step = t_samples(end)/t_prova;
t = 0 : step : (wp_num-step);
                
        %%
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

Tee_home = T_b_DH0 * Tee_home * T_DH7_ee;
R_home = Tee_home(1:3,1:3);

%% transformations
R_off = eul2rotm([0, 0, 0], 'ZYX'); % offset attorno z
R_pre = eul2rotm([-pi/2, 0, -pi/2], 'ZYX');
R2 = eul2rotm([pi, 0, 0], 'ZYX');
R1 = eul2rotm([0, 0, pi/6], 'ZYX');

ZYX_0 = rotm2eul(R_home);
ZYX_1 = wp2_rot(end-2:end);
R_1 = R_pre*eul2rotm(ZYX_1)*R1*R2;
ZYX_1 = eul2rotm(R_1);
theta_traj = generate_line_points(ZYX_0, ZYX_1, t_prova);
x_home_row = Tee_home(1:3,4)';

x_data = x_home_row;
for j = 1:2:wp_num*3-2
    x_data = [x_data;wp2_pos(j:j+2)'];
end

ppx = spline(t_samples, x_data(:,1), t);
ppy = spline(t_samples, x_data(:,2), t);
ppz = spline(t_samples, x_data(:,3), t);

for i = 1:t_prova
       
    traj(:,i) = [ppx(i);ppy(i);ppz(i)];
    x_or_ee_des(:, :, i) = eul2rotm(theta_traj(:, i)', 'ZYX');
        
end

 iter_num_1 = t_prova;
        
        x_des = cell(N, iter_num_1);  % init for speed
        for k = 1 : iter_num_1
            x_des(:,k) = {  xee_max; xee_min; xj7_max; xj7_min; xj6_max; ...
                        xj6_min; xj5_max; xj5_min; xj4_max; xj4_min; ...
                        xj3_max; xj3_min; xj2_max; xj2_min; ...
                        traj(:,k); x_or_ee_des(:,:,k)};
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
[q_out, qd_out_right_1, e_out_right_1] = reverse_priority_pos_or_7j( 	N, Ts, iter_num_1, ...
                                                        J_and_T_hand, ...
                                                        q_0_right, qd_0, x_des, ...
                                                        unil_constr, ...
                                                        x_cons, param_vect);
                                               