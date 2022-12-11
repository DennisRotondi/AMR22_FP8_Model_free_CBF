% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of Model-free unicycle

% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

clear 
close all
clc
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
q0 = [0 0 0 0 0];       % start variables x0 y0 theta0 v0 w0
qstart = [q0(1) ; q0(2)]; % initial cartesian position

% system parameters
m = 1;     % mass
Icm = 0.5; % inertia
a = 0.4;
params=[m,Icm,a];

% nominal controller gains
kp = 0.3;   % proportional
kd = 0.2;   % derivative
ContGainVec=[kp kd];

% reference values
x1d = 4; % xdes reference
y1d = -1; % ydes reference 
qg=[x1d ;y1d];

% simulation parameters
simTime = 150;               % time of simulation
% barrier parameters
alpha = 0.2;                % barrier certificate param 
mu = 0.03;
% obstaicle parameters
r1 = 0.75;                      % radius obstacle
qO1 = [1.7;0];                  % 1st obstacle position
qO2 = [1.5;-1.4];               % 2rd obstacle position
qO3 = [3;-1.5];                 % 3nd obstacle position
qO4 = [3;-0.19];                % 4th obstacle position
qO5 = [4;-0.4];                 % 5th obstacle position
r = [r1 0.45 r1 0.45 0.45];     % radius vector
threshold_skip = 0;             % 0.05
obs = [qO1 qO2 qO3];    % obstacles
conditional_delta = [0.05 0.01 0.05 0.05 0.05];

% Start simulation
out = sim('simulation_UCBF');
q1 =  out.configuration_vector.Data(1,:); % x(t)
q2 =  out.configuration_vector.Data(2,:); % y(t)
nom_input_norm =  out.u_nominal.Data;     % ||u(t)||
time = out.configuration_vector.Time;                          % t 


% In the configuration space
% plot in q1-q2
figure("Name","Obstacle Avoidance through CBF")
plot(q1,q2,'Color','k','LineWidth',4);
axis("equal");
text(qstart(1)+0.1,qstart(2),'$\bf{q_{start}}$','Interpreter','latex','FontSize',20);
text(qg(1),qg(2),'$\bf{q_{goal}}$','Interpreter','latex','FontSize',20);
xlabel('position, $q_1$ (m)','Interpreter','latex');
ylabel('position, $q_2$ (m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
fontsize(gca,30,'points');
[~,num_obs] = size(obs);

% plot obstacles 
for i = 1:num_obs
    ob = obs(:,i);
    hold on;
    th = 0:pi/50:2*pi;
    xunit = r(i) * cos(th) + ob(1);
    yunit = r(i) * sin(th) + ob(2);
    col = rand(1,3);
    h = plot(xunit, yunit,'color', 'k','LineWidth',6);
    hold on;
    fill(xunit, yunit, col)
    text(ob(1),ob(2),"$O_"+ num2str(i) +"$",'Interpreter','latex','FontSize',40,'Color','white');
    if conditional_delta(i) ~=0
        xunit_clearance = (r(i)+conditional_delta(i)) * cos(th) + ob(1);
        yunit_clearance = (r(i)+conditional_delta(i)) * sin(th) + ob(2);
        h = plot(xunit_clearance, yunit_clearance, '--','LineWidth',3);
    end
end
plot(qstart(1),qstart(2),'marker','o','Color','red','MarkerSize',15,'MarkerFaceColor','red'); hold on;
plot(qg(1),qg(2),'marker','o','Color','green','MarkerSize',20,'MarkerFaceColor','green');
plot(q1(end),q2(end),'marker','o','Color','blue','MarkerSize',15,'MarkerFaceColor','blue'); hold on;

% Plot Configuration Evolution
figure("Name","Obstacle Avoidance through CBF: Position Evolution")
plot(time,q1,time,q2,'LineWidth',4);
yline(qg(1),'-.','LineWidth',4)
yline(qg(2),'-.','LineWidth',4)
legend('$q_1(t)$','$q_2(t)$','Interpreter','latex','FontSize',30);
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('position, $q(t)$ (m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,30,'points');

% Plot Nominal Acceleration
figure("Name","Obstacle Avoidance through CBF: Velocity")
plot(time,nom_input_norm,'LineWidth',4);
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('torque, $\| \tau \|$ (N$\cdot$ m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,30,'points');

% % print CBF function
% figure("Name","Obstacle Avoidance through CBF: Control Barrier Function")
% plot(time,cbf(1,:),'Color',NMatlabBlue ,'LineWidth',4);
% yline(0,'LineWidth',4,'Color','red')
% xlabel('time, $t$ (s)','Interpreter','latex');
% ylabel('CBF, $h$ (m)','Interpreter','latex');
% fontname(gca,"Latin Modern Math")
% xlim([0,simTime])
% grid on;
% fontsize(gca,30,'points');