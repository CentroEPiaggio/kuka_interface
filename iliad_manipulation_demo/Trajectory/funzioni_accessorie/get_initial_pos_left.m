function [initial_pos_left, initial_joint_vector_left ] = get_initial_pos_left()

TWO_ARMS_transformations

VITO_geometry_and_kinematics

%rosshutdown
%rosinit

sub_initial_config_left = rossubscriber('/left_arm/joint_states');
    pause(1)
    msg2_left = receive(sub_initial_config_left,5);
    initial_joint_vector_left = msg2_left.Position;
    dummy_initial_joint_vector_left = initial_joint_vector_left;
    initial_joint_vector_left(3,1) = dummy_initial_joint_vector_left(7,1);
    for u = 4 : 7
        initial_joint_vector_left (u,1) = dummy_initial_joint_vector_left(u-1,1);
    end
    
    DH_table_num_left = double(subs(DH_table_sym, q_sym, initial_joint_vector_left));     %tabella DH
    
        [~, Tee_left] = direct_kinematics_DH(DH_table_num_left);

        % use pre and post transformations
        Tee_left = T_b_DH0l * Tee_left * T_DH7l_eel;
        
        initial_pos_left = Tee_left(1:3,4);       %posizione ee calcolata con cinematica diretta
        initial_pos_left = initial_pos_left';
     
%rosshutdown
end
