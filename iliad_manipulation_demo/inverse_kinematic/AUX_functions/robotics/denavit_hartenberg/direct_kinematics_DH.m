function [Tj, Tee] = direct_kinematics_DH(DH_table)
%{
===========================================================================
	This function computes the direct kinematics for the 3D robot, either
    numeric or symbolic depending on the input

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

    DH_table	[j_num x 4]  	DH parameters table. It can be either
                                numeric or symbolic

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------
    
	Tj          {1 x j_num]    	cell array containing rototranslation 
                                matrices for all joints. Each element is:                    
    
        Tj{i}       [4 x 4]         matrix with joint i orientation 
                                    (rotation matrix) and position 
                                    (3 components vector)  
                                                          
	Tee        	[4 x 4]         matrix containing ee rototranslation  
  
===========================================================================
%}

    j_num = size(DH_table, 1);
    
    % extract parameters columns
        d = DH_table(:,1);
        th = DH_table(:,2);
        a = DH_table(:,3);
        ph = DH_table(:,4);    
        
    % compute transformations
              
        Tj{1} = eye(4);     % should be considered T0: global base frame
          
        for i = 2 : j_num     
        	Tj{i} = Tj{i-1} * T_DH(d(i-1), th(i-1), a(i-1), ph(i-1));
        end
        
        % (j_num+1) is the ee
        Tee = Tj{j_num} * T_DH(d(j_num), th(j_num), a(j_num), ph(j_num));
      
end
    