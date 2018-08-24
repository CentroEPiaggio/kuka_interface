function h = unilateral_constr_activation_check(unil_constr, x_cur, x_cons, xd_lower, beta_pos, beta_vel)
%{
===========================================================================
	This function check for activation or deactivation of the unilateral 
    constraints 

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

    unil_constr     [1 x 1]   	flag with:
                                    1 if max unilateral constraint
                                    2 if min unilateral constraint

    x_cur        	[1 x 1]     current task generalized position

    x_cons          [1 x 1]     current constraint generalized position

    xd_lower      	[1 x 1]     current generalized velocity contribution
                                coming from lower priority tasks, which 
                                have already been processed during the 
                                current algorithm step

    beta_pos        [1 x 1]     generalized position activation buffer,
                                defining the smooth transition from 
                                activation to deactivation and vice versa

    beta_vel        [1 x 1]     generalized position activation buffer,
                                defining the smooth transition from
                                activation to deactivation and vice versa

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------

	h               [1 x 1]     unilateral constraint activation parameter  

===========================================================================
%}

    if unil_constr == 1          % max unilateral constraint

        if x_cur < (x_cons - beta_pos)                              % inactive
            h_pos = 0;                                             
        elseif (x_cons - beta_pos) <= x_cur < x_cons                % smooth variation
            h_pos = smooth_function(((x_cur - x_cons) + beta_pos) / beta_pos);    
        else                                                        % active
            h_pos = 1;                                             
        end
        if xd_lower < -beta_vel
            h_vel = 0;
        elseif -beta_vel <= xd_lower < 0
            h_vel = smooth_function((xd_lower + beta_vel) / beta_vel);
        else 
            h_vel = 1;
        end

    elseif unil_constr == 2      % min unilateral constraint  

        if x_cur > (x_cons + beta_pos)                              % inactive
            h_pos = 0;                                             
        elseif x_cons < x_cur && x_cur <= (x_cons + beta_pos)       % smooth variation
            h_pos = smooth_function(((x_cur - x_cons) - beta_pos) / -beta_pos);    
        else                                                    	% active
            h_pos = 1;                                             
        end
        if xd_lower > beta_vel
            h_vel = 0;
        elseif xd_lower <= beta_vel && xd_lower > 0
            h_vel = smooth_function((xd_lower - beta_vel) / -beta_vel);
        else 
            h_vel = 1;
        end

    else
        error('Bad constraint number');
    end

    h = h_pos * h_vel;
 
end
