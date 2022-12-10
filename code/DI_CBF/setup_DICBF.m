% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of STANDARD CBF Method

% Variable of interest - - - - - - 
%  --- safe input (torque) 
%  --- configuration (px,py)
%  --- cbf hbar

% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

clear 
close all
clc
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);

qstart =[0,0];          % starting configuration
qdotstart =[0,0];           % starting initial velocity
simTime = 250;              % simulation time
r1 = 0.75;                  % radius obstacle
M = 5*eye(2);               % inertia matrix
qg =[4, -1];                % desired configuration
alpha = 0.1;                % barrier certificate param 
qO1 = [1.5;0];              % 1st obstacle position
qO3 = [3;-1.5];             % 2nd obstacle position
qO2 = [1.5;-1.4];           % 3rd obstacle position
qO4 = [3;-0.19];            % 4th obstacle position
qO5 = [4;-0.4];             % 5th obstacle position
r = [r1 0.45 r1 0.45 0.45];
Kp = 0.2;                       % proportional term
Kd = 1;                         % derivative term 
threshold_skip = 0;             % 0.05
obs = [qO1 qO2 qO3 qO4 qO5];    % obstacles
mu = 0.02;
conditional_delta = [0.1 0.1 0.05 0.5 0.05];

out = sim('simulation_DICBF');
t = out.tout;
q1 =  out.configuration_vector.Data(:,1);
q2 =  out.configuration_vector.Data(:,2);
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
plot(qg(1),qg(2),'marker','o','Color','green','MarkerSize',15,'MarkerFaceColor','green');


% import simData from simulink
safe_input_norm =  out.safe_u_norm.Data;
velocity_norm = out.velocity_norm.Data;
desired_input_norm = out.desired_input_norm.Data;
cbf = out.cbf.Data;
time = out.safe_u_norm.time;


% print position 
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




% print torques norms
figure("Name","Obstacle Avoidance through CBF: Velocity")
plot(time,safe_input_norm,time,desired_input_norm,'LineWidth',4);
legend('safe','desired','Interpreter','latex','FontSize',30);
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('torque, $\| \tau \|$ (N$\cdot$ m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,30,'points');

% import colors
NMatlabBlue     = [0        0.4470   0.7410];
NMatlabBordeaux = [0.6350   0.0780   0.1840];

% print safe Input norm
figure("Name","Obstacle Avoidance through CBF: Control effort")
plot(time,safe_input_norm,'Color',NMatlabBordeaux,'LineWidth',4);
legend("$\alpha$ = "+num2str(alpha),'Interpreter','latex','FontSize',30);
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('input, $ \| u \|$ (m/$s^2$)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,30,'points');

% print CBF function
figure("Name","Obstacle Avoidance through CBF: Control Barrier Function")
plot(time,cbf(1,:),'Color',NMatlabBlue ,'LineWidth',4);
yline(0,'LineWidth',4,'Color','red')
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('CBF, $h$ (m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,30,'points');