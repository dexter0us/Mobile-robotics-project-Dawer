% Function to simulate sensor arrangement on a robot
%
% Can be used for two arrangments
% 1. sonar/ultrasonic sensor sweeping across a span (C_span)
% 2. A number of photosensor on a periphery (C_Binstr_l)
%
% Also uses a grayscale image for computation
%
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ========================================================================
function [C_BinStr_o C_BinStr_t Dist Dist1 Dist2 th th_o th_g] = sensorModule1c(posn, rad, course, C_dep, C_span, C_BinStr_l);
global tag1
[dimy, dimx]    = size(course);
% C_BinStr_l      = 100;
% th_inc          = posn(3)+pi/6;
th_inc          = posn(3)- C_span/2;

sizeFactor      = size(1:C_BinStr_l);
C_BinStr_o      = ones(sizeFactor);     % assumes no obstacle in front (white)
C_BinStr_t      = ones(sizeFactor);     % assumes no target in front (white)
Dist            = C_dep*ones(sizeFactor);
Dist1           = 0.2*zeros(sizeFactor);
Dist2           = C_dep*ones(sizeFactor);
th              = zeros(sizeFactor);
th_g            = zeros(sizeFactor);
th_o            = zeros(sizeFactor);

for j = 1:C_BinStr_l
    Tin = 0;
    Oin = 0;
    for i = rad:(rad + C_dep)
        posn33      = th_inc;
        x           = posn(2)+ i*sin(posn33)
%         pause
        y           = posn(1)+ i*cos(posn33);
%         x(x < 20)         = 20;
%         x(x > dimx-20)    = dimx-20;
%         y(y < 20)         = 20;
%         y(y > dimy-20)    = dimy-20;
%         x(x < 2)         = 2;
%         x(x > dimx-2)    = dimx-2;
%         y(y < 2)         = 2;
%         y(y > dimy-2)    = dimy-2;
    if(course(round(x),round(y)) == 0)      % Obstacle (Black = 0)
            C_BinStr_o(j)   = 0;
            th_o(j) = th_inc;
            Dist(j) = i;
            Dist2(j) = 0;
            Oin = 1;
        end
        if (course(round(x),round(y)) == 76)    % Target (red->grey =76)
            C_BinStr_t(j)   = 0;
            th_g(j) = th_inc;
%             Dist1(j)= 1;
            Dist1(j)= i;            
            Tin = 1;
        end
        % if (C_BinStr_t(j)== 0)|(C_BinStr_o(j)==0)
        if (Tin ~= 0) | (Oin ~= 0)
            break
        end
    end
    th(j) = th_inc;
    [x1 y1] = pol2cart(th_inc,i);
    th_inc  = th_inc + C_span/C_BinStr_l;
    posn    = [posn(1) posn(2) posn33];
    % ===================================================================
    % Uncomment following section to plot the scan
    if tag1 == 0
        a2 = [posn(1), posn(1)+x1];
        b2 = [posn(2), posn(2)+y1];
        plot(a2,b2,'g:')
        drawnow;
        % pause(0.1);
    end
    % ===================================================================
end
tag1 = 1;