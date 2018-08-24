function x_left = get_x_left(p_x, ZYX)

        R_off = eul2rotm([0, 0, 0], 'ZYX');
        R_pre = eul2rotm([-pi/2, 0, -pi/2], 'ZYX');
        R2 = eul2rotm([pi, 0, 0], 'ZYX');
        R1 = eul2rotm([0, 0, pi/6], 'ZYX');

        d = [0; 0.1140; 0.2795];    % inserire vettore urdf di velvet fixed       
%         d = [-0.062; 0.2305; 0.1844]; 
        R_pre = eul2rotm([-pi/2, 0, -pi/2], 'ZYX');     
                
        x_left_t = p_x' - R_pre*eul2rotm(ZYX, 'ZYX')*R1*R_off*d;
        x_left = x_left_t';
end