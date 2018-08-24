function J_and_T_hand = def_JT_handle(robot_ID)
%{
===========================================================================
    This function initialize the J and T handle cell array

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

	robot_ID            string        	robot name

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------

	J_and_T_hand        {4 x 1}     	cell arry with function handles

===========================================================================
%}

    J_and_T_hand = cell(6,1);

    J_and_T_hand{1} = str2func(strcat('Jee_', robot_ID, '_pos_function'));
    J_and_T_hand{2} = str2func(strcat('Jee_', robot_ID, '_or_function'));
    J_and_T_hand{3} = str2func(strcat('Tee_', robot_ID, '_pos_function'));
    J_and_T_hand{4} = str2func(strcat('Tee_', robot_ID, '_or_function'));
    J_and_T_hand{5} = str2func(strcat('J_link3_', robot_ID, '_pos_function'));
    J_and_T_hand{6} = str2func(strcat('J_link3', robot_ID, '_or_function'));
    J_and_T_hand{7} = str2func(strcat('T_link3_', robot_ID, '_pos_function'));
    J_and_T_hand{8} = str2func(strcat('T_link3', robot_ID, '_or_function'));
    
end