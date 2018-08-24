function eo = eo_using_quat(R_des, R_cur)
%{
===========================================================================
	This function computes the orientation error, for a robot task 
    execution, using quaternions

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

    R_des       	[3 x 3]         	matrix that gives the desired
                                        orientation

    R_cur        	[3 x 3]         	matrix that gives the current
                                        orientation
    
---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------

	eo              [3 x 1]             column with the orientation error

===========================================================================
%}

    Q_des = (dcm2quat(transpose(R_des))).';     % column
    Q_cur = (dcm2quat(transpose(R_cur))).';   	% column
    
    % Note: transpose due to opposite dcm definition between 
    % robotics and Matlab
    
    eo = Q_cur(1)*Q_des(2:4) - Q_des(1)*Q_cur(2:4) - cross(Q_des(2:4), Q_cur(2:4));
 
end
