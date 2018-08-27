function wp = generate_waypoint(q, grid, disp, absolute)
%%OUTPUT: wp is a vector of 6 numbers, 3 position cordinates and 3 euler
%%angles
%% INPUTS: q is the actual joint positions vector
          % grid is a vectpr of 6 nubers, of the i-th element of grid
          %is equal to 1, than the corresponding position coordinate/euler angle of wp 
          %has to be computed incrementally, otherwise, the absolute
          %position/euler angle
          %contained in absolute has to be considered.

q_0_right = q';
robot_ID = 'TWO_ARMS_r';
  
%%   Initialize KUKA transformation
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
        
T_b_DH0 = T_b_DH0r;
T_DH7_ee = T_DH7r_eer;

    
%% compute using direct kinematics 
if(sum(grid) > 0) % at least one displacement to be used
    DH_table_num = KUKA_LWR_DHtable(q_0_right);
    [~, Tee] = direct_kinematics_DH(DH_table_num);

    %% use pre and post transformations
    Tee = T_b_DH0 * Tee * T_DH7_ee;
    R_act = Tee(1:3,1:3);
    rot_act = rotm2eul(R_act)';
    pos_act = Tee(1:3,4);
    act = [pos_act;rot_act];
end
% wp is the vector of way point position and euler angles
wp = zeros(6,1);
for i = 1:6
    if(grid(i)) % if "use displacements" enabled
        wp(i) = act(i) + disp(i);
    else
        wp(i) = absolute(i);
    end
end


