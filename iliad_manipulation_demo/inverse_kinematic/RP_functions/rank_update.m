function T = rank_update(J, Jra_pinv)
%{
===========================================================================
	This function executes the rank-update procedure

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

	J                                   matrix to be updated, which gives
                                        the output dimension

    Jra_pinv                            matrix from from which the columns 
                                        are extracted, giving the search 
                                        dimension

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------

	T                                   updated matrix

===========================================================================
%}

    J_dim = size(J, 1);
    Pinv_dim = size(Jra_pinv, 2);
    
    % first column
        % init i = 1, index for T columns
        % init j = 1, index for Pinv columns
        T = Jra_pinv(:,1);      % always l.i.
        i = 2;               	% i.e.: i = i+1;
        j = 2;                	% i.e.: j = j+1;
   
    % other columns
        while i <= J_dim
            while j <= Pinv_dim
                tj = Jra_pinv(:,j);     % possible new l.i. column
                j = j+1;
                if rank([T, tj]) == i	% lin. ind.
                    T = [T, tj];        % append column 
                    i = i+1;
                    break
                end
            end
        end

    if size(T, 2) < J_dim
         error('There are not enough lin. ind. columns in T');
    end
    
end   