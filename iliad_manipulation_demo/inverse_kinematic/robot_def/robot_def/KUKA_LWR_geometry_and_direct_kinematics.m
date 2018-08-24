%{
===========================================================================
This script initialize the KUKA LWR geometry params and direct kinematics
===========================================================================
%}

j_num = 7;

l0  = 0.11;   	% m
l1  = 0.2005; 	% m
l2  = 0.20;    	% m
l3  = 0.20;  	% m
l4  = 0.20;    	% m
l5  = 0.19;    	% m
lee = 0.078;  	% m

l = [(l0 + l1), (l2 + l3), (l4 + l5), lee];

lb = 0.1;       % m, base

kuka_jmax = [170, 120, 170, 120, 170, 120, 170] * 2*pi / 360;   % official
% kuka_jmax = [2.97, 2.09, 2.97, 2.09, 2.97, 2.09, 2.97];
kuka_jmin = -kuka_jmax;                                         % official

% symbolic joint angles
q_sym = sym('q', [j_num 1]);

% symbolic DH parameters
    d = [l(1); 0; l(2); 0; l(3); 0; l(4)];
    th = [q_sym(1); q_sym(2); q_sym(3); q_sym(4); q_sym(5); q_sym(6);  q_sym(7)];
    a = [0; 0; 0; 0; 0; 0; 0];
    ph = [pi/2; -pi/2; -pi/2; pi/2; pi/2; -pi/2; 0];
        
% symbolic DH table
DH_table_sym = [d, th, a, ph];

[Tj_sym, Tee_sym] = direct_kinematics_DH(DH_table_sym);
        
