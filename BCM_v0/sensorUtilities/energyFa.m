% Function to calculate Energy on the basis of current and future states
%
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ========================================================================
function [EE neg_R kappa] = energyFa(posn, C_BinStr_l, C_BinStr_o, C_BinStr_t, Dist, C_dep, energy, ES, neg_R, Max_Energy)

Er      = 5;    % good move
Ef      = 25;   % food
Em      = 2;    % move
Ec      = 7;    % collision
Et      = 1;    % wait

sizeFactor     = size(1:C_BinStr_l);
dir            = ones(sizeFactor);
gam            = zeros(sizeFactor);
phi            = zeros(sizeFactor);
kappa          = zeros(sizeFactor);
mu             = ones(sizeFactor);  % all moves are possible
ind1           = floor([posn(1) posn(2)]);
for i = 1:C_BinStr_l
    %===========================================================
%     if (C_BinStr_o(i)~=dir(i)) && (C_BinStr_t(i)==dir(i))
%     if (C_BinStr_o(i)==dir(i)) && (C_BinStr_t(i)~=dir(i))     
    if (C_BinStr_o(i)==dir(i)) | (C_BinStr_t(i)~=dir(i))             
        gam(i) = 1;
    end
    %===========================================================
    if ES(ind1) > 0.1
        phi(i) = ES;
    end
    %===========================================================
%     if (Dist(i)/C_dep <= 0.2)      % Collision expected
    if (Dist(i)/C_dep <= 0.1)      % Collision expected        
        kappa(i) = 1;
    end
    %===========================================================
    delta_E     = Er*gam(i) + Ef*phi(i) - 2*Em*mu(i) - Ec*kappa(i);
%     delta_E     = Er*gam(i) + Ef*phi(i) - Em*mu(i) - Ec*kappa(i);
    energyPrev  = energy;
    EE(i)       = energyPrev - Et + delta_E;
    % Upper & Lower Bounds
    if EE(i) > Max_Energy
        EE(i) = Max_Energy;
    elseif EE(i) < 10 & neg_R == 0
        EE(i) = 20;
        neg_R = 1;
    end
end