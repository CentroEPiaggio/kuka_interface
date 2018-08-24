%{
===========================================================================
This script sets the project's folders paths
===========================================================================
%}      

run_level = 3;

dots = '../';

for i = 1:length(folders_name)
    
    % evaluate folder level
    back_path = '';
    for j = 1:run_level
        back_path = strcat(dots, back_path);
    end
    
    % determinate folder path
    folder_path = strcat(back_path, folders_name{i});
    
    % set path
    if sub_flag(i)
        addpath(genpath(folder_path));
    else
        addpath(folder_path);
    end
    
end