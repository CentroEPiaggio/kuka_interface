function [points] = generate_line_points(A, B, N)   
%{
===========================================================================
	This function returns N points of R3, linearly placed between A and B

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

	A               [3 x 1]  	vector with first R3 point

    B               [3 x 1]    	vector with second R3 point

    N               [1 x 1] 	number of points

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------
    
	points          [3 x N]     matrix with N points

===========================================================================
%}

    tA = 0;
    tB = N;
    points = NaN(3,N);    % init for speed
    
    points(1,1) = A(1);
    points(2,1) = A(2);
    points(3,1) = A(3);
    
    for t = (tA+2):1:(tB-1)
        points(1,t) = A(1) + (t - tA) * (B(1) - A(1)) / (tB - tA);
        points(2,t) = A(2) + (t - tA) * (B(2) - A(2)) / (tB - tA);
        points(3,t) = A(3) + (t - tA) * (B(3) - A(3)) / (tB - tA);
    end
    
        t = 0:1/(N-1):1;

    points(1,N) = B(1);
    points(2,N) = B(2);
    points(3,N) = B(3);
    
    for k=1:N
        for j=1:3
        points(j,k) = A(j)+t(k)*(B(j)-A(j));
        end
    end
end