function [qd_new, e_new] = reverse_priority_step(N, qd_prev, x_des_cur, xd_des_cur, unil_constr, x_cons_cur, param_vect, J, x_cur)
%{
===========================================================================
	This function executes one single step of the reverse priority 
    algorithm, which computes the joint velocities necessary 
    for multiple tasks execution. 
    The tasks, which can be unilateral or bilateral constraints, are 
    specified with different priorities and processed in reverse order, 
    starting from the lower priority task. 

    The task execution error is also returned, for plots and analysis.

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

    N               [1 x 1]   	tasks number

	qd_prev       [j_num x 1]  	vector with previous step joint velocity

    x_des_cur       {N x 1}   	cell array with current desired tasks 
                                position

    xd_des_cur      {N x 1}    	cell array with current desired tasks 
                                velocity

	unil_constr     [1 x N]     vector which elements are:
                                    0 if the corresponding x_des is a task 
                                    1 if it's a max unilateral constraint
                                    2 if it's a min unilateral constraint

	x_cons_cur      [1 x N]     vector with the current limits for max and 
                                min constraints, depending on unil_constr

	param_vect    [1 x (N+5)]   vector with algorithm parameters:

        DPI_lambda_max  [1 x 1]     damping for damped pseudo inverse

        DPI_epsilon     [1 x 1]   	bound for damped pseudo inverse

        beta_pos        [1 x 1]   	position buffer for un. constr.

        beta_vel        [1 x 1]    	velocity buffer for un. constr.

        lambda          [1 x 1]    	damping lambda for task N+1

        K               [1 x N]   	error gain vector

	J               {1 x N}   	cell array with jacobians associated with 
                                the tasks

	x_cur           {1 x N}   	cell array with current tasks position

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------
    
	qd_new        [j_num x 1]   vector with new computed joint velocity
    
    e_new           {N x 1}   	cell array with new error

===========================================================================
%}
    
    % extract parameters from param_vect
       
        DPI_lambda_max = param_vect(1);     % damping for damped pseudo inverse   
        DPI_epsilon = param_vect(2);        % bound for damped pseudo inverse  
        beta_pos = param_vect(3);           % position buffer for un. constr.  
        beta_vel = param_vect(4);           % velocity buffer for un. constr.  
        lambda = param_vect(5);             % damping lambda for task N+1
        K = param_vect(6:6+N-1);            % error gain vector 


    % algorithm step

        % init vectors
        Jra{N+1} = [];
        T{N} = [];

        for p = N:-1:1     % e.g.: 3 2 1 

            % augmented jacobian
            Jra{p} = [J{p}; Jra{p+1}];
            
            % pseudoinverse
            Jra_pinv{p} = damped_pseudo_inverse(Jra{p}, ...
                                                DPI_lambda_max, ...
                                                DPI_epsilon);
            
            % rank-update
            T{p} = rank_update(J{p}, Jra_pinv{p});
           
            % error computation and xd computation
                if unil_constr(p)
                    xd{p} = 0;
                else
                    % compute orientation error, using quaternions
                    if(size(x_des_cur{p}) == [3,3])     % it's a matrix (R)                                   
                        e{p,1} = eo_using_quat(x_des_cur{p}, x_cur{p});
                    % compute position error
                    else
                        e{p,1} = x_des_cur{p} - x_cur{p};
                    end

                    % compute tasks vel
                    xd{p} = xd_des_cur{p} + K(p)*e{p};
                end
                
        end
        

        % (N+1)-th task: min vel variation from previous step
        qd{N+1} = lambda * qd_prev; 
       
        
        for p = N:-1:1     % e.g.: 3 2 1 
            
            if unil_constr(p)   	% p is a unilateral constraint
                
                % xd given by lower priority tasks, required 
                % by unilateral_constr_activation_check
                xd_lower{p} = J{p}*qd{p+1};     
                
                h(p) = unilateral_constr_activation_check(  unil_constr(p), ...
                                                            x_cur{p}, ...
                                                            x_cons_cur(p), ...
                                                            xd_lower{p}, ...
                                                            beta_pos, ...
                                                            beta_vel);
                
            else                    % p is a task                       
                h(p) = 1;       
            end
            
            
            qd{p} = qd{p+1} + h(p) * T{p}*pinv(J{p}*T{p}) * (xd{p} - J{p}*qd{p+1});
           
        end
  
        % the real vel. is qd{1}
        qd_new = qd{1};
        
        e_new = e;
   
end