function M_pinv = damped_pseudo_inverse(M, lambda_max, epsilon)
%{
===========================================================================
	This function computes the damped pseudo inverse

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

    M              [n x m]              matrix

	lambda_max     [1 x 1]              damping coefficient that represents 
                                        the damping 'strength'
                                        e.g. 0.1

    epsilon        [1 x 1]              damping threshold
                                        e.g. 0.1
    
---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------

	M_pinv         [m x n]              pseudo-inverse

===========================================================================
%}

    [U, S_mat, V] = svd(M);
    sing_vals = nonzeros(S_mat);

    lambda_quad = 0;

    if sing_vals(end) < epsilon
        lambda_quad = ( 1 - (sing_vals(end) / epsilon)^2 ) * lambda_max^2;
    end

    S = zeros(size(M));

    i = 1;
    while i <= length(sing_vals)
        S(i,i) = (sing_vals(i)) / (sing_vals(i)^2 + lambda_quad);
        i = i + 1;
    end

    M_pinv = V * S' * U';
 
end
