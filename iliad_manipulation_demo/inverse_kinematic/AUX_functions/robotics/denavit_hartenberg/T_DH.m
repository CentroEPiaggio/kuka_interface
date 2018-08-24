function T = T_DH(d, th, a, ph)
%{
===========================================================================
	This function computes the rototranslation matrix, given the 
    DH parameters

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

	d       [1 x 1]         	DH parameter d

	th      [1 x 1]         	DH parameter theta

	a       [1 x 1]         	DH parameter a

	ph      [1 x 1]         	DH parameter alpha

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------
    
	T       [4 x 4]          	matrix that gives orientation (rotation 
                                matrix) and position (3 components vector)                                
 
===========================================================================
%}

    T = [cos(th)    -cos(ph)*sin(th)    sin(ph)*sin(th)      a*cos(th)  ; ...
         sin(th)     cos(ph)*cos(th)    -sin(ph)*cos(th)     a*sin(th)  ; ...
           0           sin(ph)             cos(ph)             d      	; ...
           0              0                   0                1       ];

end