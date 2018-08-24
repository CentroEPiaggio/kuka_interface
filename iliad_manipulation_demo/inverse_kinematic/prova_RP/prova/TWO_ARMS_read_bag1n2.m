%{
===========================================================================
This script extracts trajectory data from the .bag file
===========================================================================
%}

% convert the bag in timeseries
tims = timeseries(bag);

% convert the timeseries in a matrix
tims_mat = tims.Data.';

% extract position and orientation
    p_b_eer_b = tims_mat(1:3,:);  
    
   	% too far for correct execution (avoid singularity)
    p_b_eer_b(1,:) = p_b_eer_b(1,:) - 0.3; 
    
    Q_b_eer_b(1,:) = tims_mat(7,:);         % Re part is the last
    Q_b_eer_b(2:4,:) = tims_mat(4:6,:);     % Im part are the first three

% convert quaternions in rotation matrices
for i = 1:size(tims_mat, 2)
    R_b_eer_b(:,:,i) = quat2rotm(Q_b_eer_b(:,i).');
end

% T_b_ee (desired pose from base to ee_right) expressed in b
for i = 1:size(tims_mat, 2)
    T_b_eer_b(:,:,i) = [[R_b_eer_b(:,:,i), p_b_eer_b(:,i)]; [0 0 0 1]];  
end

%% give a look

% position
    figure
    plot(p_b_eer_b.')
    
% orientation YPR angles
    YPR_b_eer_b = quat2eul(Q_b_eer_b.');
    figure
    plot(YPR_b_eer_b)
%{    
% error
    for i = 1801:2199
        err_p(:,i) = e_out{15,i};
    end
    for i = 1801:2199
        err_o(:,i) = e_out{16,i};
    end
    figure
    plot(err_p)
%}
% show bag trajectory
    figure
    hold on
    plot3([p_b_eer_b(1,:)],[p_b_eer_b(2,:)],[p_b_eer_b(3,:)])
    plot3([p_b_eer_b(1,1)],[p_b_eer_b(2,1)],[p_b_eer_b(3,1)], 'marker', 'o', 'markersize', 5)
%    plot_frame(T_b_eer_b(:,:,1), 0.2)
    axis([0, 2, 0, 2, 0, 2]);
    axis square
    view([-90, 50]);
    hold off
