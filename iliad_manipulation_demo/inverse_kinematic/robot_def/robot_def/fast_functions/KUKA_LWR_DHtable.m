function DH_table = KUKA_LWR_DHtable(q)
%{
===========================================================================
This script initialize the KUKA LWR geometry params
===========================================================================
%}

l0  = 0.11;   	% m
l1  = 0.2005; 	% m
l2  = 0.20;    	% m
l3  = 0.20;  	% m
l4  = 0.20;    	% m
l5  = 0.19;    	% m
lee = 0.078;  	% m

l = [(l0 + l1), (l2 + l3), (l4 + l5), lee];

% DH parameters
    d = [l(1); 0; l(2); 0; l(3); 0; l(4)];
    a = [0; 0; 0; 0; 0; 0; 0];
    th = [q(1); q(2); q(3); q(4); q(5); q(6); q(7)];
    ph = [pi/2; -pi/2; -pi/2; pi/2; pi/2; -pi/2; 0];
        
% DH table
DH_table = [d, th, a, ph];
      
end

