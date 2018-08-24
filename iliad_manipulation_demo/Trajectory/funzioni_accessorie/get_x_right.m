function x_right = get_x_right(p_x, ZYX)

        d = [0; 0; 0.217];

        R_pre = eul2rotm([pi, -pi/2, 0], 'ZYX');    
                
        x_right_t = p_x' - R_pre*eul2rotm(ZYX, 'ZYX')*d;
        x_right = x_right_t';
end