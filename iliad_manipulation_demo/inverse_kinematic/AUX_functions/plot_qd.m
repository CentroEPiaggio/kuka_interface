%{
===========================================================================
This script plots the joint velocities qd
===========================================================================
%}

j_num = length(q_out(:,2));         % joints number

figure

n_sub = ceil(j_num/2);
m_sub = 2;

for i = 1:j_num
    
    subplot(n_sub, m_sub, i)
    
    hold on
    plot(qd_out(i,:))
    hold off
      
    title(strcat('qd', num2str(i)));
%   axis([0, iter_num, -pi, pi])
    
end
