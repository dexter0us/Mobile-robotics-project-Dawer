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
function [C_BinStr_o C_BinStr_t Dist th th_g Dist1] = sensorModule1(posn, rad, course, C_dep, C_span, C_BinStr_l);
% global tag1
[dimy, dimx]    = size(course);
% C_BinStr_l      = 100;
% th_inc          = posn(3)+pi/6;
th_inc          = posn(3)- C_span/2;    %theta of inclination = (theeta - c sapn angle)/2

sizeFactor      = size(1:C_BinStr_l);
C_BinStr_o      = ones(sizeFactor);     % assumes no obstacle in front (white)
C_BinStr_t      = ones(sizeFactor);     % assumes no target in front (white)
Dist            = C_dep*ones(sizeFactor);
Dist1           = 0.2*zeros(sizeFactor);
th              = zeros(sizeFactor);
th_g            = zeros(sizeFactor);
tag1 = 0;

for j = 1:C_BinStr_l
    for i = rad:(rad + C_dep)
        posn33      = th_inc;
        x           = posn(2)+ i*sin(posn33);
        y           = posn(1)+ i*cos(posn33);
        x(x < 1)    = 1;
        x(x > dimx) = dimx;
        y(y < 1)    = 1;
        y(y > dimy) = dimy;
        if(course(round(x),round(y)) == 0)      % Obstacle (Black = 0)
            C_BinStr_o(j)   = 0;
            % if the sensor see the obstacle, then it will put that
            % specific entry of the obstacle vector as zero.
        end
        if (course(round(x),round(y)) == 76)    % Target (red->grey =76)
            %if the sensors see the target, then it will make that specific
            %entry of the target vector as zero. Also put the J'th entry
            %of the vector th_g equal to th_inc. also put that specific
            %entry of Dist1 vector as 1.
            C_BinStr_t(j)   = 0;
            th_g(j) = th_inc;   %th_g is only for the goal. when there will be a goal in the range, it will get that specific entry equal to th_inc otherwise equal to 0.
            Dist1(j)= 1;        % it is just the binary/flag vector tha just represents if the goal is within the range of that particular sensor. if the goal is within the ragne of sensor 5 then place 1 at 5th element of this vector.
        end
        % if (C_BinStr_t(j)== 0)|(C_BinStr_o(j)==0)
        if (C_BinStr_o(j)==0)
            Dist(j) = i;    % it is the value of the sensor. just like (analogRead(sensor_number)) in the case of Arduino. and this is vector containing the values of all 16 sensors.
            break
        end
        if (C_BinStr_t(j)== 0)
            break
        end
    end
    th(j) = th_inc;    % th contains the th_inc of all the inclination angles on the robot, and updates in every circle from zero.
    [x1 y1] = pol2cart(th_inc,i);
    th_inc  = th_inc + C_span/C_BinStr_l;
    posn    = [posn(1) posn(2) posn33];
    % ===================================================================
    % Uncomment following section to plot the scan
%     if tag1 == 0
%         a2 = [posn(1), posn(1)+x1];
%         b2 = [posn(2), posn(2)+y1];
%         plot(a2,b2,'g:')
%         drawnow;
%         % pause(0.1);
%     end
    % ===================================================================
end
tag1 = 1;