% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of STANDARD CBF Method on DI

% Variable of interest - - - - - - 
%  --- safe input (torque) 
%  --- configuration (px,py)
%  --- cbf hbar

clear 
close all
clc
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
%% robot initialization
% these are the robot parameters that remain fixed during the experiments
qstart =[0,0];              % starting configuration
qdotstart =[0,0];           % starting initial velocity
M = 1*eye(2);               % true inertia matrix
robot_radius = 0;           % radius of the robot 
a = 0;                      % displacement wrt robot center
%% simulation stuff
simTime = 50;              % simulation time
M2 = 1*eye(2);              % estimated inertia matrix ( may be different from original)
MAP = "map_2";
% the function creates an 4xn matrix where n is the numer of the obstacles,
% the first two rows are the center, then radious and clearance
obstacles = setup_environment(MAP);
%% control parameters
qg =[4, -1];                % desired configuration
alpha = 0.1;                % barrier certificate param 
Kp = 0.2;                   % proportional term
Kd = 1;                     % derivative term 
mu = 0.02;                  % cbf velocity weight
threshold_skip = 0;         % hysteresis mechanism
%% run simulation
out = sim('simulation_DICBF');
% import simData from simulink
time = out.cbf.Time;
q1 = out.configuration_vector.Data(:,1);
q2 = out.configuration_vector.Data(:,2);
cbf = out.cbf.Data;
% actual velocity of the robot
velocity_norm = out.actual_velocity_norm.Data;
% desired input produced from the nominal controller
desired_input_norm = out.desired_input_norm.Data;
% input after safe optimization (actual input)
safe_input_norm =  out.safe_u_norm.Data;

%% plots
fig1 = plot_map(obstacles);
plot_trajectory(q1,q2,qstart,qg);
fig2 = plot_cbf(cbf,time);
% 
% figure("Name","Obstacle Avoidance through CBF")

% axis("equal");



% [~,num_obs] = size(obs);
% 

% 
% 

% 
% 
% % import simData from simulink
% safe_input_norm =  out.safe_u_norm.Data;
% velocity_norm = out.velocity_norm.Data;
% desired_input_norm = out.desired_input_norm.Data;
% cbf = out.cbf.Data;
% time = out.safe_u_norm.time;
% 
% 
% % print position 
% figure("Name","Obstacle Avoidance through CBF: Position Evolution")
% plot(time,q1,time,q2,'LineWidth',4);
% yline(qg(1),'-.','LineWidth',4)
% yline(qg(2),'-.','LineWidth',4)
% legend('$q_1(t)$','$q_2(t)$','Interpreter','latex','FontSize',30);
% xlabel('time, $t$ (s)','Interpreter','latex');
% ylabel('position, $q(t)$ (m)','Interpreter','latex');
% fontname(gca,"Latin Modern Math")
% xlim([0,simTime])
% grid on;
% fontsize(gca,30,'points');
% 
% 
% 
% 
% % print torques norms
% figure("Name","Obstacle Avoidance through CBF: Velocity")
% plot(time,safe_input_norm,time,desired_input_norm,'LineWidth',4);
% legend('safe','desired','Interpreter','latex','FontSize',30);
% xlabel('time, $t$ (s)','Interpreter','latex');
% ylabel('torque, $\| \tau \|$ (N$\cdot$ m)','Interpreter','latex');
% fontname(gca,"Latin Modern Math")
% xlim([0,simTime])
% grid on;
% fontsize(gca,30,'points');
% 
% % import colors
% 
% NMatlabBordeaux = [0.6350   0.0780   0.1840];
% 
% % print safe Input norm
% figure("Name","Obstacle Avoidance through CBF: Control effort")
% plot(time,safe_input_norm,'Color',NMatlabBordeaux,'LineWidth',4);
% legend("$\alpha$ = "+num2str(alpha),'Interpreter','latex','FontSize',30);
% xlabel('time, $t$ (s)','Interpreter','latex');
% ylabel('input, $ \| u \|$ (m/$s^2$)','Interpreter','latex');
% fontname(gca,"Latin Modern Math")
% xlim([0,simTime])
% grid on;
% fontsize(gca,30,'points');
