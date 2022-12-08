% This file is going to extract the simulation
% variables from simulink
close all
clc
r = 0.5;          % radius obstacle
q1g = 1.9;     % pdes x
q2g = -0.8;     % pdes y
alpha = 0.01;     % barrier certificate param 
qO1 = [1;-0.5];    % obstacle position
qO2 = [2.5;-0.8];    % obstacle position

threshold_skip = 0.3;

obs = [qO1,qO2];
out = sim('DDICBF_MO');
t = out.tout;
q1 =  out.configuration_vector.Data(:,1);
q2 =  out.configuration_vector.Data(:,2);
%figure("Name","Phase Portrait")
plot(q1,q2,'k','LineWidth',3);
text(q1g,q2g,'$\bf{q_{goal}}$','Interpreter','latex','FontSize',20);
xlabel('$p_x$','Interpreter','latex','FontSize',20);
ylabel('$p_y$','Interpreter','latex','FontSize',20);
grid on;
[~,num_obs] = size(obs);
for i = 1:num_obs
    ob = obs(:,i);
    hold on;
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + ob(1);
    yunit = r * sin(th) + ob(2);
    col = rand(1,3);
    h = plot(xunit, yunit,'color', col ,'LineWidth',2);
    fill(xunit, yunit, col)
end
