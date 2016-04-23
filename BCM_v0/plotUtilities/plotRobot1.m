% Function to plot robot in the arena with axes
% 
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ========================================================================
function plotRobot1(h,posn,SF,clr)

x       = posn(1);
y       = posn(2);
theta   = posn(3);
P       = pose2stance(x,y,theta,SF);

% hold on

% robot platform
rad1 = SF*0.7;
rad2 = SF*0.25;
circle1([x,y],rad1,10000,clr);
circle1([x,y],rad2,100,'-y');


%wheels
X2 = P(5:8,1);
X2(5) = P(5,1);
Y2 = P(5:8,2);
Y2(5) = P(5,2);
fill(X2,Y2,'-k');

X3 = P(9:12,1);
X3(5) = P(9,1);
Y3 = P(9:12,2);
Y3(5) = P(9,2);
fill(X3,Y3,'-k');

%axes
axesLength = SF*1.3;
p = rotate(axesLength,0,theta) + [x y];
plot([x p(1)], [y p(2)], '-b');

%arrow in x-axis
p1 = rotate(axesLength,0,theta) + [x y];
p2 = rotate(SF*(1.3-0.2),SF*0.1,theta) + [x y];
plot([p1(1) p2(1)], [p1(2) p2(2)], '-b');

p1 = rotate(axesLength,0,theta) + [x y];
p2 = rotate(SF*(1.3-0.2),-0.1*SF,theta) + [x y];
plot([p1(1) p2(1)], [p1(2) p2(2)], '-b');