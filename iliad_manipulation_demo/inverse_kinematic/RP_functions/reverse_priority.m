function [q, qd, e] = reverse_priority(N, Ts, iter_num, q_0, qd_0, x_des, unil_constr, x_cons, param_vect, Jsym, xsym)
%{
===========================================================================
	This function executes the reverse priority algorithm, which computes 
    the joint velocities necessary for multiple tasks execution. 
    The tasks, which can be unilateral or bilateral constraints, are 
    specified with different priorities and processed in reverse order,
    starting from the lower priority task. 

    The joint angles are computed with an integration.
    The task execution error is also returned, for plots and analysis.

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

    N               [1 x 1]         tasks number

    Ts              [1 x 1]         sampling time

    iter_num        [1 x 1]         max number of iterations

    q_0           [j_num x 1]       vector with starting joint angles

    qd_0          [j_num x 1]       vector with starting joint velocities

    x_des        {N x iter_num}     cell arry with desired position or/and
                                    orientation tasks 

	unil_constr     [1 x N]         vector which elements are:
                                        0 if the corresponding x_des is a task 
                                        1 if it's a max unilateral constraint
                                        2 if it's a min unilateral constraint

	x_cons          [m x N]         vector with the limits for max and min
                    m >= 1          constraints, depending on unil_constr

	param_vect    [1 x (N+5)]       vector with parameters used in the 
                                    algorithm step, please see 
                                    reverse_priority_step.m for details
   
    Jsym            {1 x N}         cell array with symbolic jacobians

	xsym            {1 x N}         cell array with symbolic tasks 
  
---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------

	q          [j_num x iter_num]  	matrix with joint angles
    
	qd         [j_num x iter_num]	matrix with joint velocities

    e            {N x iter_num}     cell array with task errors   
 
===========================================================================
%}

    % user message
    disp('Reverse priority algorithm initialization');

    % initialization (i.e. k = 1)
    q(:,1) = q_0;
  	qd(:,1) = qd_0;            

    e = {};              	% not defined for k = 1, because iterations 
                        	% start from k = 2 for convenience
    
    j_num = length(q_0);
   
    for k = 2 : iter_num
        
        % user message
        disp(strcat('step #', num2str(k)));
        disp('... init');
        
        % required symbolic variables definition
        q_sym = sym('q', [j_num 1]);
  
        for p = 1:N 
            % numeric jacobians
            J{p} = double(subs(Jsym{p}, q_sym, q(:,k-1)));
        
            % actual x
            x{p,k} = double(subs(xsym{p}, q_sym, q(:,k-1)));
        end
        
        x_cur = x(:,k);                     % x: cell array 
        
        x_des_cur = x_des(:,k);         
        x_des_prev = x_des(:,k-1); 
        
        qd_prev = qd(:,k-1);
              
        if k <= length(x_cons(:,1))
            x_cons_cur = x_cons(k,:);       % select the k-th step row
        else
            x_cons_cur = x_cons(end,:);     % works also for time-invariant
        end
        
        % compute tasks desired vel (for trajectory tracking)
        for p = N:-1:1
            
            % if x_des is a matrix (R), xd_des_cur will be an ang. vel. w:
         	if size(x_des_cur{p}) == [3,3]      % it's a matrix (R)
                if x_des_cur{p} == zeros(3)     % const R
                    xd_des_cur{p} = [0; 0; 0];  % angular velocity is 0
                else 
                    xd_des_cur{p} = (x_des_cur{p} - x_des_prev{p}) / Ts;    % Rdot
                    w_ee_hat = xd_des_cur{p} * x_des_prev{p}.';             % Rdot * R.'
                    w_ee = skew_2_vect(w_ee_hat);
                    xd_des_cur{p} = w_ee;
                end
            
            % all the other cases    
            else
                xd_des_cur{p} = (x_des_cur{p} - x_des_prev{p}) / Ts;
            end
            
        end
        
        % user message
        disp('... start computation');

        [qd_new, e_new] = reverse_priority_step(N, qd_prev, x_des_prev, ...
                                                xd_des_cur, unil_constr, ...
                                                x_cons_cur, param_vect, ...
                                                J, x_cur);

        % user message
        disp('... done step');
        
        % append to vectors
        qd = [qd, qd_new];
        e = [e, e_new];              
        
        % append new q, given by integration
        q = [q, q(:,k-1) + qd_new*Ts];
        
    end
    
    % user message    
    disp('Reverse priority algorithm complete');
    
end