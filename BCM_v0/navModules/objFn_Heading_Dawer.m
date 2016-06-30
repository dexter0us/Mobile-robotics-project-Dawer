% Beam Curvature Method Navigation Strategy using 'objective function' approach
%  
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com

% Modified by: Muhammad Dawer Saeed.  2016-MS-MC-13.
% Technique used: Beam Curvature Method (BCM).

% University of Engineering and Technology
% ========================================================================
% This function takes in some parameters and then after processing them,
% gives out two things, 1) angle of inclination, at which the robot needs
% to be turned. 2) A vector of values which may be used later for some
% other purpose.
% ========================================================================
function [angle_incl F_th1] = objFn_Heading_Dawer (th, ...
    th_g,Dist,Dist1,C_BinStr_1,C_dep,posn,posn1,obstacles,dist_o, ...
    targets)

def_obst_dist = 40;
def_obst_dist_arr = [];
rho1 = [];
rho2 = [];
def_max_distance = 0;
alpha = 0.5;
beta = 0.5; %since "alpha+beta=1" is necessary.
beam_rating = [];
% ----------------- END OF DEFINE CONSTANTS -------------------------------
% ***********************************************************************
% we need to have an information of the targets pre-hand
ts=size(targets);
tv=ts(:,1);
for this_target=1:tv
   tar = targets(this_target,:);
   T_x = tar(2) + (tar(4)/2);
   T_y = tar(3) + (tar(5)/2);
   theta_goal=atan2((T_y-posn(2)),(T_x-posn(1)));    % angle of the goal.
   first_goal=abs(th-theta_goal);                      % looking for the first goal to go.
   [~,n]=min(first_goal);
   steering_angle = th(n);
end

% First of all we need to calculate the beam associated to the obstacles.
% ***********************************************************************
if (isempty(obstacles) == 1)    %if the array is empty? size will not work, array can have a size while being empty.
    for x=1:C_BinStr_1
        if (dist_o(x)<def_obs_dist) %<---------------------
            angel_incl = th(en) + pi/2; %<-------------------
        end
    end % end of for loop
else
    % apply beam to every obstacle
    no_of_obs = size(obstacles);
    for obs=1:no_of_obs(:,1)    %Loop for every entry of the obstacles. 
        this_obs = obstacles(obs,:);    %get the specific row and all the columns
        %fprintf('The obstacles row is: %d\r\n' , round(this_obs)); % print out the obastacles row
        
        % Now find the center of each obstacle: C = center
        C_x = this_obs(2)+(this_obs(4)/2);  % x coordinate of center of obstacle
        C_y = this_obs(3)+(this_obs(5)/2);  % y coordinate of center of obstacle
        % now find the angle theeta_Obs , theeta_simple and d_obs
        % first find the x,y coordinates of the obstacle wrt robot
        obs_x_wrt_robot = C_x-posn(2);  %<---------------
        obs_y_wrt_robot = C_y-posn(1);   %<---------------
        %now finding the theeta_obs and theeta_simple
        theeta_obs = atan(obs_x_wrt_robot/obs_y_wrt_robot);
            %but first we need to find the r_obs and d_obs
            d_obs = sqrt((obs_x_wrt_robot^2) + (obs_y_wrt_robot^2));
            r_obs = ((max(this_obs(4),this_obs(5)))/2) ;  %find the maximum of the two distances of the obstacle, make it half and it is the radius of our obstacle.
            fprintf('d_obs , r_obs: %d , %d\r\n' , round(d_obs), round(r_obs)); % print out the variable
            def_obst_dist_arr(obs) = d_obs;
        theeta_simple = asin(r_obs/d_obs);
        % Now to find the rhos:
        rho1(obs) = theeta_obs - theeta_simple;
        rho2(obs) = theeta_obs + theeta_simple;
    end % end of for loop
    [val, num]= max(def_obst_dist_arr);    %<-----------------
    def_max_dis = def_obst_dist_arr(num);
% ***********************************************************************
% Calculating the beam overlapping:

% ***********************************************************************
% Goal Heading Calculation:
d_safe = 50;
rho_safe = d_safe/def_max_dis;
rho1_safe = abs(max(rho2(obs)) + min(d_safe/def_max_dis, (rho2(obs)+(pi/2))));
rho2_safe = abs(min(rho1(obs)) - min(d_safe/def_max_dis, (rho2(obs)+(pi/2))));
rho1_lim = max(rho1(obs),rho1_safe);
rho2_lim = min(rho2(obs),rho2_safe);

% ***********************************************************************

% ***********************************************************************
% Selection of the Best Beam:
%     no_of_beams = size(def_obst_dist_arr);
%     for beam = 1:no_of_beams
%         if((rho1(beam)<phi) && (rho2(beam)>phi))
%             epsilon = 0;
%         
%         elseif(rho2(beam) <phi)
%             epsilon = phi - rho2(beam);
%         else
%             epsilon = rho1(beam)-phi;
%         end
%         beam_rating(beam) = alpha *...
%             ((d*cos(abs(epsilon)))/def_max_distance) - ...
%             beta*abs(epsilon/pi);
%     end
%     [chosen_beam_num chosen_beam_value] = max(beam_rating);
% ***********************************************************************
% Final Calculaiton of goal angle.

    if(rho2_lim >= rho1_lim)
        if(steering_angle >= rho1_lim && steering_angle<=rho2_lim)
            phi_goal=steering_angle
        elseif (steering_angle<=rho2_lim)   % steering_angle>=-inf
            phi_goal=rho1_lim
        elseif (steering_angle>=rho1_lim)   % steering_angle<=inf
            phi_goal=rho2_lim
        end
    end
    if (rho2_lim < rho1_lim)
        d_1_near = def_max_dis;
        d_2_near = def_max_dis;
        rho1_near = rho1(obs);
        rho2_near = rho2(obs);
        delta_rho1_safe = abs (rho1_safe - rho1_near);
        delta_rho2_safe = abs(rho2_safe-rho2_near);
        phi_goal = rho2_lim + ((rho1_lim - rho2_lim)*(alpha*(d_2_near/(d_1_near+d_2_near))+beta*(delta_rho1_safe/(delta_rho1_safe+delta_rho2_safe))));
    end
    % *********************************************************************
    % Now avoiding those Obstacles that calculated above using angle.
    angle_incl = phi_goal;
end % end of main if condition
F_th1 =0;
end % end of the funtion.