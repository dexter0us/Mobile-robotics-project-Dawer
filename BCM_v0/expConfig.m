% Experiment configuration
%
% Written by: Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ========================================================================
% initial conditions for the robot, it provides the x, y and the theeta
% position of the robot.
ic1         = [50 50 pi/2];
ic2         = [400 50 pi/2];
ic3         = [800 50 pi/2];
ic4         = [50 300 pi/2];
ic5         = [800 300 pi/2];
ic6         = [50 500 pi/2];
% ic6         = [50 500 0];
% ic7         = [400 520 pi/2];
ic7         = [400 550 0];
ic8         = [800 550 pi];
%----------------------------------------------------------------------------
rad         = 10;   % it is the 
SF          = 10;   % 

tag1        = 0;
tag2        = 1;
esLocX      = 150;            % Energy Source location
esLocY      = 450;
maxEnergy   = 3000;
energy      = maxEnergy;
negR        = 0;
thT         = 1;

colC        = 0;    % collision counter
numS        = 1;
tarPC       = 0;    % target persistence
fluC        = 0;    % fluctuation in trajectory
ID          = 1;
tS          = 1;
dt          = 1;

positionTrace       = [];
collisionTrace      = [];
energyTrace         = [];
orientationTrace    = [];
inflammationTrace   = [];
idTrace             = [];
% ---------------- end of defining the basic variables -----------------
nR                  = 1;
nRmax               = 7;    % maximum number of robots (for specifying colors only)
icPalette           = [ic8; ic2; ic3; ic4; ic5; ic6; ic7; ic8];
clrs                = [.2 .4 .4; .6 .4 .4; .2 .2 .4; .4 .4 .2; .6 .2 .4; .6 .4 .2; .4 .4 .4 ];
clrPalette          = [(1:nRmax)' clrs];
sDepPalette         = [250; 350; 250; 250; 250; 250; 250];  %sensor depth palette
sSpanPalette        = [2*pi/3; 2*pi/2; 2*pi/3; 2*pi/3; 2*pi/3; 2*pi/3; 2*pi/3]; % sensor span palette
sBinStr_lPalette    = [16; 30; 20; 16; 30; 20; 25];
energyPalette       = [maxEnergy; maxEnergy; maxEnergy; maxEnergy; maxEnergy; maxEnergy; maxEnergy];
idPalette           = [1; 1; 1; 1; 1; 1; 1];
incmpNames          = {'posn', 'clr', 'sDep', 'sSpan', 'sBinStr_l', 'energy'};
orienPalette        = icPalette(:,end);

save('expConfig') 