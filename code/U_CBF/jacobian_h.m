syms x y theta v w a ox oy mu real
p = [x; y];
pdot = [v*cos(theta)-w*a*sin(theta);
        v*sin(theta)+w*a*cos(theta)];
pbar = [ox; oy]
h = (p-pbar)'*(p-pbar)+mu*(p-pbar)'*pdot

hdot = jacobian(h,[x; y; theta; v; w]);
hdot(1)

