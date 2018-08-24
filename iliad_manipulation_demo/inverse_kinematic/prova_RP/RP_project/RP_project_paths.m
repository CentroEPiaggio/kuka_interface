%{
===========================================================================
This script contians RP project's folders paths
===========================================================================
%}

folders_name = {'AUX_functions';
                'RP_functions';
                'RP_project';
                'RP_specific';
                'robot_def'};

sub_flag = [    1;
                1;
                0;
                0;
                1]; 

addpath(genpath('AUX_functions/matlab/paths/'));
%run('../AUX_functions/matlab/paths/set_folders')
set_folders