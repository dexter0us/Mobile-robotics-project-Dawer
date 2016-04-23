function P = rotate(x,y,theta)

R = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];

P = [x y];
P = (R*P')';