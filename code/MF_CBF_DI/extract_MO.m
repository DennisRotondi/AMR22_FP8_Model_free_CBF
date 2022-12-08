% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation 

% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

clear 
close all
clc
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 29 20]);

qstart =[0,0];          % starting configuration
qdotstart =[0,0];       % starting initial velocity
simTime = 50;           % simulation time
r = 0.5;                % radius obstacle
qg =[2.7, -1.8];        % desired configuration
alpha = 0.1;            % barrier certificate param 
qO1 = [1;-0.6];         % 1st obstacle position
qO2 = [3;-1];           % 2nd obstacle position
qO3 = [2;-1.5];         % 3rd obstacle position
qO4 = [0.8;-1.8];       % 4th obstacle position
Kp = 0.2 ;              % proportional term
Kd = 1 ;                % derivative term 
threshold_skip = 0.05;  % 0.05

obs = [qO1, qO2, qO3, qO4];    % obstacles

out = sim('DDICBF_MO');
t = out.tout;
q1 =  out.configuration_vector.Data(:,1);
q2 =  out.configuration_vector.Data(:,2);
figure("Name","Obstacle Avoidance through CBF")
plot(q1,q2,'Color','k','LineWidth',3);
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

input_norm =  out.input_norm.Data;
safe_velocity_norm =  out.safe_velocity_norm.Data;
velocity_norm = out.velocity_norm.Data;
desired_velocity_norm = out.desired_velocity_norm.Data;
time = out.safe_velocity_norm.time;
figure("Name","Obstacle Avoidance through CBF: Velocity")
plot(time,safe_velocity_norm,time,velocity_norm,time,desired_velocity_norm,'LineWidth',3);
legend('safe','actual','desired','Interpreter','latex','FontSize',30);
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('velocity, $\| \dot{q} \|$ (m/s)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,30,'points');

figure("Name","Obstacle Avoidance through CBF: Control effort")
plot(out.input_norm.time,input_norm,'Color',[0.6,0.15,0.9],'LineWidth',3);
legend("$\alpha$ = "+num2str(alpha),'Interpreter','latex','FontSize',30);
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('input, $ \| u \|$ (m/$s^2$)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,30,'points');