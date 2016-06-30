% Test Program for Mobile Robot Navigation
% Using grayscale image for arena in sensor module  
%
% Written by: Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% Modified by: Muhammad Dawer Saeed.  2016-MS-MC-13.
% Technique used: Beam Curvature Method (BCM).
%
% University of Engineering and Technology
% ========================================================================
% ========================================================================
clear; close all;
global sBinStr_l sBinStr_o sBinStr_t Dist th th_g Dist1
global aa mi tag1 positionTrace orientationTrace FT mi1 mi2 ki ai thT th
addpath('plotUtilities','generalUtilities','arenaUtilities','sensorUtilities','navModules','scenarios') % adding these directories in the program. The names are the directories
%set(0,'DefaultFigureWindowStyle','docked')
set(0,'DefaultFigureWindowStyle','normal')  %doc style of the windows

% ========================================================================
% First execute the file named: 'expConfig' 
load('expConfig')   % load the expConfig file. This file decides the initial configurations on the robot/program. We need to run this file before running the program.
tF = 150;   % Final time. Here it is actually the number of steps.

% ========================================================================
% Plotting configuration

recallArena = 1;

% the arena names to try are as follows:
% 'S0_1a'*, 'S0_1b', 'S0_1c', 'S1_1a', 'S1_1b', 'S1_1c'*, 'S1_1d'*, 'S1_1e',
% 'S1_1f'*, 'S1_1g*', 'S2_2a', 'S2_2b'*, 'S2_2c'*, 'S2_2d', 'S2_2e', 'S2_3a',
% 'S2_3b'*, 'S2_3c', 'S2_3d', 'S2_3e'*,
% replace the following name one by one!

arenaName = 'S2_3d';   %Default: S2_3d

if recallArena == 1
    load(arenaName, 'arena', 'obstacles', 'targets')    %these variables will be loaded into the program so that we can use it.
    arenaSize = [arena(1,2) arena(1,3) arena(3,4) (arena(3,3)+arena(3,5))]; % 
    [arena obstacles targets] = recreateArena(arenaSize, arena, obstacles, targets);    %redraw the arena on matlab.
    [obstSr obstSc] = size(obstacles);  %Sr = Number of Rows (number of obstacles), Sc = Number of Columns (it is not acutally being used in our case, it was a part of simulator code's matrix values something....)
    [targSr targSc] = size(targets);    %get the values of the targets from the above loaded variables.
    nObst          = obstSr;    %number of obstacles
    nTarg          = targSr;    %number of targets to be rescued
    MM             = nTarg;     %here it is equal to the number of targets, but it will be used later on, may be.
end

% ========================================================================
% Initial plots

course = rgb2gray(imread(arenaName,'png')); %reading the image and converting it to the grayscale.
course = flipud(course);    %flipping it upside down. B/c the robot takes the lower left point as origin but in images it starts from the top left corner, so we are flipping it.
course = imresize(course, [600 840]);   %image resize to the corresponding resolution. 
h1 = imshow(course); axis image off; colormap gray; % some trivial commands to show the image.
hold on;

[ES] = energySource(esLocX,esLocY);     % not used in our case i guess. But we can use it to have some braitenbug experiments.

% ========================================================================

% Initial plots prior to start of algorithm
% this loop is just assigning the colors to the respective robots and
% plotting the corresponding things on the robot.
for kk = 1:nR         %nR is for more than one robots.
    plotRobot1(h1, icPalette(kk,:), SF, clrPalette(kk,2:4));    %icPallette =  initial conditioin pallette (defined in the expConfig.m file.)
    plotSensor(h1, icPalette(kk,:), rad, sDepPalette(kk), sSpanPalette(kk));
    [sBinStr_o sBinStr_t dist_o dist_t dist_o1 th th_o th_g]   = sensorModule1c(icPalette(kk,:), rad, course, sDepPalette(kk), sSpanPalette(kk), sBinStr_lPalette(kk));
    if kk == 1; tag1 = 0; end
end
% pause

% ========================================================================
% all of the information is mostly for the multi-robot information, and
% multi-robot scenario.
posn1       = icPalette(1,:);
posn        = posn1;
posn_p      = [posn(1)-2 posn(2)-2 posn(3)];

patchesT    = zeros(2*nTarg,200);
patchI      = 1;

jj          = 1;
ii          = 1;
nn          = 1;
ll          = 1;
% tInd        = 0;
tInd        = zeros(1,nTarg);
tagSearch   = 1;
tagRescue   = 0;
tSwitch     = [];
tag3        = 0;
tic

%this is the main loop function which repeats itself untill the robot find
%its targets, or if all the targets have been rescued, or the time in not
%yet expired.

while((nTarg ~= 0) | (tagSearch ==1) | (tagRescue == 1)) & (tS <= tF)

    posn            = icPalette(ll,:);  % the position of the robot
    clr             = clrPalette(ll,2:4);
    sDep            = sDepPalette(ll);  % sensors depth
    sSpan           = sSpanPalette(ll); %sensors span.
    sBinStr_l       = sBinStr_lPalette(ll); % The number of total sensors on the robot
    energy          = energyPalette(ll);
    ID              = idPalette(ll);    %id fo the specific robot
    orientation     = orienPalette(ll); % the orientation of the robot

    positionTrace   = [positionTrace posn'];    %keepting the track of the position, it is keepting the trace of the movement of robot in this matrix.

    % =====================================================================
    % Your code most probably change the following! 
    % =====================================================================
    % sBinStr_t
    % dist_o
    % sBinStr_o
    % th
    % th_g
    % dist_t
    
    [sBinStr_o sBinStr_t dist_o th th_g dist_t] = sensorModule1(posn, rad, course, sDep, sSpan, sBinStr_l);
    % we need to replace the following function according to our logic. and
    % place our own function into it. (Also if we need to add the velocity
    % function, it will be incorporated here as well.)

    [th_inc F_th1]= objFn_Heading_Dawer(th,th_g,Dist,Dist1,sBinStr_l,sDep,posn,posn1,obstacles,dist_o,targets);
    theta=(th_inc);
    
    %[F_th1 F_th2 f_OA f_OA_fltd f_GS f_GS_fltd f_GS1 f_GS1_fltd] = objFn_Heading(th, th_g, dist_o, sBinStr_l, sDep, dist_t); %obstacle avoid + goal seeking is happening here.
    %[F_th1 f_OA f_GS f_GS1] = objFn_Heading_D(th, th_g, dist_o, sBinStr_l, sDep, dist_t); %obstacle avoid + goal seeking is happening here.    
    %[thC thT] = max(F_th1); %maximum is being calculated here, and the thT is the position of that maximum value in the matrix.
    %figure(2);
    %plot(F_th1);
    %polar(th,F_th1);
    % [thC thT] = max(F_th2);
    %theta = (th(thT));  %this is the "thT"th vlue of the "th" vector.
        %kinematics function will be implemented here.
    rho1 = 10;  %steps of robot, to jump for 10 pixels.
    
    % end of changings generally.

    [EE negR kappa] = energyFa(posn, sBinStr_l, sBinStr_o, sBinStr_t, dist_o, sDep, energy, ES, negR, maxEnergy);

    % =====================================================================
    if (tagSearch == 1)
        [prx pry] = pol2cart(theta,rho1);   % here at this point the program is updating the new coordinates for the new forward position.
                                            % the theta is the angle of
                                            % inclinaiton, in which
                                            % direction the robot will move
                                            % forward. i.e in the goal
                                            % direction.
        posn_p = posn;
        posn = [posn(1)+prx posn(2)+pry theta];
        N_targP = nTarg;
        figure(1)
        plotRobot1(h1, posn, SF, clr)
        plotSensor(h1, posn, rad, sDepPalette(kk), sSpanPalette(kk))    % <---------- My edit
        % we might need to ask the robot to move kinematically here. (just for the display/show purpose.)
        drawnow;

        for i =1:nTarg
            diffX = abs(posn(1)-targets(i,2));
            diffY = abs(posn(2)-(targets(i,3)));
            diffVal = 50;
            if (diffX < diffVal) & (diffY < diffVal)
                if tInd(i) == 0
                    tInd(i) = i;
                    tagRescue = 1;
                    tag3 = i;
                    tagSearch = 0;
                    ID = 3;
                end
            end

            if (diffX < diffVal) & (diffY < diffVal)
                tag2 = 0;
                tagRescue = 1;
                tagSearch = 0;
                % jj = jj+1
                cInfo1 = get(h1, 'CData');
                abc = floor(targets(tInd(i),2)-8:(targets(tInd(i),2)+targets(tInd(i),4)+8));
                bcd = floor((targets(tInd(i),3)-8):(targets(tInd(i),3)+targets(tInd(i),5)+8));
                cInfo1(bcd,abc) = 0;
                course(bcd,abc)=255;
                ID = 3;
            end
        end
    end
    if (tagRescue == 1)

        [prx pry] = pol2cart(theta,rho1);
        posn_p = posn;
        posn = [posn(1)+prx posn(2)+pry theta];
        N_targP = nTarg;
        figure(1)
        plotRobot1(h1, posn, SF, clr)
        % drawnow;

        diffX = abs(posn(1)-targets(tInd(tag3),2));
        diffY = abs(posn(2)-(targets(tInd(tag3),3)));
        if (diffX < 50) & (diffY < 50)
            targets(tInd(tag3),1) = 0;
            cInfo = get(h1, 'CData');
            abc = floor(targets(tInd(tag3),2)-8:(targets(tInd(tag3),2)+targets(tInd(tag3),4)+8));
            bcd = floor((targets(tInd(tag3),3)-8):(targets(tInd(tag3),3)+targets(tInd(tag3),5)+8));
            cInfo(bcd,abc) = 200;
            set(h1, 'CData', cInfo);
            course(bcd,abc)=255;
            drawnow;
            targets(tInd(tag3),:) = [];
            nTarg = nTarg - 1;
            % tagRescue = 0;
            if any(tInd ~=0); tagRescue = 0; end
            if nTarg == 0; tagSearch = 0; else tagSearch = 1;end
            tInd(tag3) = [];
            patchesT(patchI,1:length(abc)) = abc;
            patchesT(patchI+1,1:length(bcd)) = bcd;
            patchI = patchI + 2;
            tSwitch(nn) = ii;
            nn = nn + 1;
            tagSearch = 1;
            tag3 = 0;
        end
    end
    % =======================================================================
    orientation         = posn_p(3);
    collision           = kappa(1,thT);
    energy              = EE(1,thT);
    icPalette(ll,:)     = posn;
    energyPalette(ll)   = energy;
    idPalette(ll)       = ID;
    orienPalette(ll)    = orientation;
    tarPer              = tagRescue;

    orientationTrace    = [orientationTrace orientation];
    collisionTrace      = [collisionTrace collision];
    energyTrace         = [energyTrace energy];
    idTrace             = [idTrace ID];
    idPalette(ll)       = ID;

    tS = tS + 1;
    ii = ii + dt;
    if nTarg == 0; break; end
end
%     [tS jj tag1 tag2 tInd tagSearch tagRescue]
thyme = toc

%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Results
TT = tF;
tt = ii;
mm = MM-nTarg;
HH = 1;             % start of simulation is the only human interaction
MS = 50*((mm/MM) + 1 - (tt/TT))/(HH*HH);
CC = nnz(collisionTrace);

Results = [thyme tt CC MS]'
Traces = [positionTrace' collisionTrace'  energyTrace' inflammationTrace' idTrace'];

p = profile('info');
name = arenaName;

% In the following line, provide the path of your 'Result' folder
% ... mine is as under
fullName = strcat('C:\Users\Raza\Desktop\RoboSim\Simulator\Results\',name); % we need to change this address according to my PC, so that it could save those values into that file.

save(fullName,'p','Results','Traces','tSwitch', 'patchesT');

%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

fig2 = figure(2);
fig3 = figure(3);

figure(2);
hold on;
subplot(2,1,1),
plot(energyTrace(1:end),'g'),
axis([0 length(energyTrace) 0 max(energyTrace)+5]);
ylabel('Energy Trace')

subplot(2,1,2),
plot(collisionTrace(1:end),'g')
axis([0 length(collisionTrace) -0.5 1.5]);
ylabel('Collision Trace')
xlabel('Simulation Steps')

% figure(3);
% subplot(3,3,1), polar(th,f_OA); title('f_{OA}');
% subplot(3,3,2), polar(th,f_OA_fltd);title('f_{OA} filtered');
% subplot(3,3,3), polar(th,f_GS);title('f_{GS}');
% subplot(3,3,4), polar(th,f_GS_fltd);title('f_{GS} filtered');
% subplot(3,3,5), polar(th,f_GS1);title('f_{GS1}');
% subplot(3,3,6), polar(th,f_GS1_fltd);title('f_{GS1} filtered');
% subplot(3,3,7), polar(th,dist_o);title('Distance');
% subplot(3,3,8), polar(th,F_th1);title('F - sum');
% subplot(3,3,9), polar(th,F_th2);title('F - product');