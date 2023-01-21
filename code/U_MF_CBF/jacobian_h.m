% Jacobian of the control barrier function
% h =||(p-obs)||^2 + mu(p-pobs)^Tpdot
% p referes to the point A of the robot
syms x y theta v w a ox oy mu r delta real
p = [x; y];
pdot = [v*cos(theta)-w*a*sin(theta);
        v*sin(theta)+w*a*cos(theta)];
pbar = [ox; oy];
d = sqrt((ox-x)^2+(oy-y)^2);
bear = atan2(oy-y,ox-x);
h = d-r-delta*cos(theta-bear);

hdot = jacobian(h,[x; y; theta]);
disp(simplify(hdot))

