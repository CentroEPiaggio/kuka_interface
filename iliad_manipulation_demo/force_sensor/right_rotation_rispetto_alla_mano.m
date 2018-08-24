function [Fee] = right_rotation_rispetto_alla_mano(Fb)

    R = eul2rotm([32*pi/180 + pi/2, 0, 0], 'ZYX');
    
    Fee = R*Fb

end