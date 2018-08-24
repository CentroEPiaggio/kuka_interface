function [initial_pos_right, initial_joint_vector_right ] = get_initial_pos_right()

TWO_ARMS_transformations

VITO_geometry_and_kinematics

%rosshutdown
%rosinit

sub_initial_config_right = rossubscriber('/right_arm/joint_states');
    pause(0.01)
    msg2_left = receive(sub_initial_config_right,5);
    initial_joint_vector_right = msg2_left.Position;
    %be carful: *_arm_e1 is the third vector and not the end effector
    dummy_initial_joint_vector_right = initial_joint_vector_right;
    initial_joint_vector_right(3,1) = dummy_initial_joint_vector_right(7,1);
    for u = 4 : 7
        initial_joint_vector_right (u,1) = dummy_initial_joint_vector_right(u-1,1);
    end
    
DH_table_num_right = double(subs(DH_table_sym, q_sym, initial_joint_vector_right));     %tabella DH
    
        [~, Tee_right] = direct_kinematics_DH(DH_table_num_right);

        % use pre and post transformations
        Tee_right = T_b_DH0r * Tee_right * T_DH7r_eer;
        
        initial_pos_right = Tee_right(1:3,4);       %posizione ee calcolata con cinematica diretta
        initial_pos_right = initial_pos_right';
     
%rosshutdown
end
     
       