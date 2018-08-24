function [T_b_DH0r, T_DH7r_eer] = TWO_ARMS_USED()
%{
===========================================================================
This script initializes KUKA ARMS transformations between base, arms and ee
===========================================================================
%}
 
%% T_b_0 = T_b_DH0 expressed in b

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

%% T_DH7_ee & T_ee_DH7

    % from DH7 to ee_right --> Rz(pi)
        T_DH7r_eer = [[[-1 0 0; 0 -1 0; 0 0 1], zeros(3,1)]; [0 0 0 1]];
        T_eer_DH7r = inv(T_DH7r_eer);

    % from DH7 to ee_left --> Rz(pi)

%         ypr_ee_left = [0.5236, 0.0, 0.0];
% 
%         R_7_eel = eul2rotm(ypr_ee_left, 'ZYX');
%                 
%         T_DH7l_eel = [[R_7_eel, zeros(3,1)]; [0 0 0 1]];

        T_DH7l_eel = [[[-1 0 0; 0 -1 0; 0 0 1], zeros(3,1)]; [0 0 0 1]];
        T_eel_DH7l = inv(T_DH7l_eel);
        

end

