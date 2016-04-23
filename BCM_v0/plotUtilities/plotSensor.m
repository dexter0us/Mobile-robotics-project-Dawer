% Function to plot robot's sensor range
% 
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ========================================================================
function plotSensor(h, posn, rad, C_dep, C_span)

mag1 = C_dep + rad;
p = rotate(mag1,0,(posn(3)- C_span/2)) + [posn(1) posn(2)];
plot([posn(1) p(1)], [posn(2) p(2)], '-k');
p = rotate(mag1,0,(posn(3)- C_span/2 + C_span)) + [posn(1) posn(2)];
plot([posn(1) p(1)], [posn(2) p(2)], '-k');
arc1([posn(1), posn(2)],posn(3),mag1,C_span,100,'k');