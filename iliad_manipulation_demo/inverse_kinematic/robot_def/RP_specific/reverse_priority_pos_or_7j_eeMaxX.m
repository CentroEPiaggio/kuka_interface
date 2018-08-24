function [q, qd, e] = reverse_priority_pos_or_7j_eeMaxX(N, Ts, iter_num, J_and_T_hand, q_0, qd_0, x_des, unil_constr, x_cons, param_vect)
%{
===========================================================================
	Exactly as the general reverse_priority.m, but enhanced in performance. 
    On the other hand, this can be used only when all the following tasks 
    are present:

    - ee space constraint (x)
    - all 7 joints limits constraints
    - position task
    - orientation task

    J_and_T_hand is a new input containing the function handles for J and
    T.
    For further informations, see reverse_priority.m
===========================================================================
%}

    % user message
    disp('Reverse priority algorithm initialization');

    % initialization (i.e. k = 1)
    q(:,1) = q_0;
  	qd(:,1) = qd_0;            

    % ---------------------------------------------------------------------
    % specific part

        J{1} = [ 0, 0, 0, 0, 0, 0, 1];    	% ee max unilateral constraint
        J{2} = [ 0, 0, 0, 0, 0, 0, 1];    	% ee min unilateral constraint 
        J{3} = [ 0, 0, 0, 0, 0, 1, 0];    	% joint 7 max unilateral constraint
        J{4} = [ 0, 0, 0, 0, 0, 1, 0];    	% joint 7 min unilateral constraint 
        J{5} = [ 0, 0, 0, 0, 1, 0, 0];    	% joint 6 max unilateral constraint
        J{6} = [ 0, 0, 0, 0, 1, 0, 0];     	% joint 6 min unilateral constraint 
        J{7} = [ 0, 0, 0, 1, 0, 0, 0];    	% joint 5 max unilateral constraint
        J{8} = [ 0, 0, 0, 1, 0, 0, 0];     	% joint 5 min unilateral constraint 
        J{9} = [ 0, 0, 1, 0, 0, 0, 0];    	% joint 4 max unilateral constraint
        J{10} = [ 0, 0, 1, 0, 0, 0, 0];   	% joint 4 min unilateral constraint 
        J{11} = [ 0, 1, 0, 0, 0, 0, 0];    	% joint 3 max unilateral constraint
        J{12} = [ 0, 1, 0, 0, 0, 0, 0];   	% joint 3 min unilateral constraint 
        J{13} = [ 1, 0, 0, 0, 0, 0, 0];   	% joint 2 max unilateral constraint
        J{14} = [ 1, 0, 0, 0, 0, 0, 0];  	% joint 2 min unilateral constraint            
    
        % (qdMAX) error init for variable gain 
        e = cell(17,1);              	
        e{16,1} = (x_des{16,1} - J_and_T_hand{3}(q_0));  
        
    % ---------------------------------------------------------------------
            
    for k = 2:1:iter_num
        
        % user message
        disp(strcat('step #', num2str(k)));
        disp('... init');
        
        % -----------------------------------------------------------------
        % specific part        
        
            q1 = q(1, k-1);
            q2 = q(2, k-1);
            q3 = q(3, k-1);
            q4 = q(4, k-1);
            q5 = q(5, k-1);
            q6 = q(6, k-1);
            q7 = q(7, k-1);

            % numeric jacobian
            J{17} = J_and_T_hand{2}([q1, q2, q3, q4, q5, q6, q7]);  
            J{16} = J_and_T_hand{1}([q1, q2, q3, q4, q5, q6, q7]);  
           
            Jtemp = J_and_T_hand{1}([q1, q2, q3, q4, q5, q6, q7]);
            J{15} = Jtemp(1,:);
            
            % actual x
            x{17,k} = J_and_T_hand{4}([q1, q2, q3, q4, q5, q6, q7]);
            x{16,k} = J_and_T_hand{3}([q1, q2, q3, q4, q5, q6, q7]); 
            
            xtemp = J_and_T_hand{3}([q1, q2, q3, q4, q5, q6, q7]); 
            x{15,k} = xtemp(1); 

            x{13,k} = q1;
            x{14,k} = q1;
            x{1,k} = q7;
            x{2,k} = q7;
            x{3,k} = q6;
            x{4,k} = q6;
            x{5,k} = q5;
            x{6,k} = q5;
            x{7,k} = q4;
            x{8,k} = q4;
            x{9,k} = q3;
            x{10,k} = q3;
            x{11,k} = q2;
            x{12,k} = q2;
            
        % -----------------------------------------------------------------
        
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
        
        % -----------------------------------------------------------------
        % specific part
        
            % (qdMAX) scale gain by position error
            param_vect2 = param_vect;
            if norm(e{16,k-1},2) < 0.01
                param_vect2(6:6+N-1) = 10 * param_vect(6:6+N-1);
            end
        
        % -----------------------------------------------------------------
        
        % user message
        disp('... starting computation');
        tic

        [qd_new, e_new] = reverse_priority_step(N, qd_prev, x_des_prev, ...
                                                xd_des_cur, unil_constr, ...
                                                x_cons_cur, param_vect2, ...
                                                J, x_cur);

        toc
        % user message
        disp('... done');
        
        % append to vectors
        qd = [qd, qd_new];
        e = [e, e_new];              
        
        % append new q, given by integration
        q = [q, q(:,k-1) + qd_new*Ts];
        
    end
    
    % user message    
    disp('Reverse priority algorithm complete');
    
end