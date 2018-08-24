%{
===========================================================================
This script initializes the VITO robot direct and differential kinematics
===========================================================================
%}

% init KUKA arms geometry parameters and direct kinematics
KUKA_LWR_geometry_and_direct_kinematics;

% use pre and post transformations
for j = 1:j_num    
    Tj_sym_vito_right{j} = T_b_DH0r * Tj_sym{j};
    Tj_sym_vito_left{j} = T_b_DH0l * Tj_sym{j};
end
Tee_sym_vito_right  = T_b_DH0r * Tee_sym * T_DH7r_eer;
Tee_sym_vito_left   = T_b_DH0l * Tee_sym * T_DH7l_eel;


%% ------------------------------------------------------------------------
% symbolic jacobians

    % right ee position and orientation
    Jp_ee_sym_vito_right = jacobian(Tee_sym_vito_right(1:3,4), q_sym(1:j_num));
    Jog_ee_sym_vito_right = [Tj_sym_vito_right{1}(1:3,3), Tj_sym_vito_right{2}(1:3,3), Tj_sym_vito_right{3}(1:3,3), Tj_sym_vito_right{4}(1:3,3), Tj_sym_vito_right{5}(1:3,3), Tj_sym_vito_right{6}(1:3,3), Tj_sym_vito_right{7}(1:3,3)];  

    % left ee position and orientation
    Jp_ee_sym_vito_left = jacobian(Tee_sym_vito_left(1:3,4), q_sym(1:j_num));
    Jog_ee_sym_vito_left = [Tj_sym_vito_left{1}(1:3,3), Tj_sym_vito_left{2}(1:3,3), Tj_sym_vito_left{3}(1:3,3), Tj_sym_vito_left{4}(1:3,3), Tj_sym_vito_left{5}(1:3,3), Tj_sym_vito_left{6}(1:3,3), Tj_sym_vito_left{7}(1:3,3)];  
        
