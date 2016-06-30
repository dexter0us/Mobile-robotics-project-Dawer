% Sample Navigation Strategy using 'objective function' approach
% You should write your own code, replacing this, according to the assigned
% navigation approach
% 
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ========================================================================
function [F_th1 f_OA f_GS f_GS1] = objFn_Heading(th, th_g, Dist, C_BinStr_l, C_dep, Dist1)

sFactor = 0.1;
nVectors = 50;
w_OA = 1;
w_GS = 1;

sizeFactor      = size(1:C_BinStr_l);
f_OA            = ones(sizeFactor);
f_GS            = zeros(sizeFactor);

for j = 1:C_BinStr_l
    if Dist(j)<C_dep
        f_OA(j) = Dist(j)/C_dep;
        plot(Dist(j));
    end
end

m = floor(sFactor*nVectors);        % window size for filter (general)
m1 = floor(sFactor*nVectors);     % window size for filter (goal seeking)
% m1 = m;

% Adj1 = th - th_g;
% f_GS = 1 - (abs(Adj1)/pi);
f_GS = Dist1;

%=====================================================================
% Alternative approach for goal seeking function
% th2 = npi2pi(th,'radians');
% th_g2 = npi2pi(th_g,'radians');
% f_GS1 = 1 - abs((th2-th_g2)/pi);
%=====================================================================
% Alternative approach for goal seeking function
% this function only gives the robot a direction. it will create a vector
% that will just contain values normalized accordingly. if the sensor has
% not seen anything, its value will be small, if the sensor has seen the
% goal, the value will be larger.

for j = 1:C_BinStr_l
    f_GS1a(j) = (1 - cos(th_g(j)-th(j)))/2;
end
f_GS1 = Dist1 + f_GS1a;
% F_th1 = f_GS1;
% F_th2 = f_GS1;
%=====================================================================
% The below steps are filtering the output by smoothing them.
% f_OA_fltd = medfilt1(f_OA,m);

f_OA_fltd = filtfilt(ones(1,m)/m,1,f_OA); % this step is applying the filter to the sensor.
f_GS_fltd = filtfilt(ones(1,m1)/m1,1,f_GS);
f_GS1_fltd = filtfilt(ones(1,m1)/m1,1,f_GS1);
% 

% F_th1 is the main vector that will determine that in which direction the
% robot needs to go.... the technique will either output the angle of
% inclinaiton or this vector that will contain all the values and then the
% direction will be decided on the basis of maximum of the output values.
F_th1 = (w_OA*f_OA_fltd) + (w_GS*f_GS_fltd);
% F_th2 = (1-w_OA*(1-f_OA_fltd)).*(1-w_GS*(1-f_GS_fltd));

% F_th1 = filtfilt(ones(1,m)/m,1,f_OA);
% F_th2 = filtfilt(ones(1,m1)/m1,1,f_GS);