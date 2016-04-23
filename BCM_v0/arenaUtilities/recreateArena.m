% Arena function to draw arena with existing arena parameters
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ========================================================================

function [arena obstacles targets] = recreateArena(Arena_Size, arena, obstacles, targets)

% Arena-boundary is defined in terms of polygons
Ax = Arena_Size(1);
Ay = Arena_Size(2);
Aw = Arena_Size(3);
Ah = Arena_Size(4);
Abw = 10;    % arena border/wall width
axis([Ax-2,Aw+2,Ay-2,Ah+2]);
hold on;
% ========================================================================
[g,h] = size(arena);
for j = 1:g
    rectangle('Position',arena(j,2:5),'FaceColor','k');
end
[k,l] = size(obstacles);
for m = 1:k
    rectangle('Position',obstacles(m,2:5),'FaceColor','k');
end
[n,o] = size(targets);
for p = 1:n
    rectangle('Position',targets(p,2:5),'Curvature',[1,1],'FaceColor','r','EdgeColor','none')
end