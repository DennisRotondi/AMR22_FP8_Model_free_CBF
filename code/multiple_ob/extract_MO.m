% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation 

% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

clear 
close all
clc

qstart =[0,0];          % starting configuration
simTime = 50;           % simulation time
r = 0.5;                % radius obstacle
qg =[2.7, -1.8];        % desired configuration
alpha = 2;              % barrier certificate param 
qO1 = [1.5;0];          % 1st obstacle position
qO2 = [3;-1];           % 2nd obstacle position
qO3 = [2;-2];           % 3rd obstacle position
Kp = 1 ;                % proportional term
threshold_skip = 0.3;

obs = [qO1,qO2,qO3];    % obstacles
out = sim('DDICBF_MO');
t = out.tout;
q1 =  out.configuration_vector.Data(:,1);
q2 =  out.configuration_vector.Data(:,2);
figure("Name","Obstacle Avoidance through CBF")
plot(q1,q2,'color',rand(1,3),'LineWidth',4);
text(qstart(1)+0.1,qstart(2),'$\bf{q_{start}}$','Interpreter','latex','FontSize',20);
text(qg(1),qg(2),'$\bf{q_{goal}}$','Interpreter','latex','FontSize',20);
xlabel('position, $q_1$ (m)','Interpreter','latex');
ylabel('position, $q_2$ (m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
fontsize(gca,30,'points');
[~,num_obs] = size(obs);
for i = 1:num_obs
    ob = obs(:,i);
    hold on;
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + ob(1);
    yunit = r * sin(th) + ob(2);
    col = rand(1,3);
    h = plot(xunit, yunit,'color', 'k','LineWidth',6);
    fill(xunit, yunit, col)
end
plot(qstart(1),qstart(2),'marker','o','Color','red','MarkerSize',15,'MarkerFaceColor','red'); hold on;
plot(qg(1),qg(2),'marker','o','Color','green','MarkerSize',15,'MarkerFaceColor','green');
hold on