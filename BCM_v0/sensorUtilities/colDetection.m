% Collision Detection Function
% 
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% =======================================================================
function status = colDetection(posn, rad, course)
% angs = 0:pi/10:2*pi;
% y = posn(1)+rad;
% x = posn(2)+rad;
angs = 0:pi/10:2*pi;
y = posn(1) + (rad)*sin(angs);
x = posn(2) + (rad)*cos(angs);

for i = 1:length(y)
    if(course(round(x(i)),round(y(i))) == 0)
        status = 1;
        return;
    end
end
status = 0;
