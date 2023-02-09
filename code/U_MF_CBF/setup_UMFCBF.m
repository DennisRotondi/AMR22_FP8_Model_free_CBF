% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of MODEL FREE CBF Method on U

clear; close all; clc;
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
%% robot initialization
qstart = [-2 0 pi/2]';      % starting pose (x0 y0 theta0)
vwstart = [0 0]';          % starting velocities (w0,v0)
m = 5;                     % mass             
a = 0.1;                   % distance from the center 
robot_radius = 0.3;
Icm = 1/2*m*robot_radius^2;% inertia
params=[m,Icm,a];          % vector of parameters
M = [m 0; 0 Icm];
%% simulation stuff
% 1 for watching the robot movie
% 0 else
animation_mode = 0;                  
simTime = 30;              % time of simulation
MAP = "map_3";
obstacles = setup_environment(MAP,a,robot_radius);
[~, num_obs] = size(obstacles);
disturbance = true;        % if to simulate with disturbance in h or not
%% control parameters
qg=[6 ; -3];                 % qgoal
Kp = 0.3;                    % proportional
Kd = 10;                     % derivative
alpha = 1.9;                   % barrier certificate param 
delta = 3;                 % barrier parameter
threshold_skip = 0;          % hysteresis mechanism
k1 = sqrt(min(eig(M))/2)     % k1 bound defined as in the report
lambda = Kd/max(eig(M))      % stability parameter 
norm_d_inf = 1.6;
gamma = norm_d_inf/(2*k1*alpha) % gamma function
%% run simulation
out = sim('simulation_UMFCBF');           % from sim
% import simData from simulink
time = out.cbf.Time;
q1 =  out.configuration_vector.Data(:,1);
q2 =  out.configuration_vector.Data(:,2);
input_norm =  out.input_norm.Data;
cbf = squeeze(out.cbf.Data);
% TODO SPLIT VELOCITIES FOR PLOTS, NO NORMS
% safe_velocity_norm =  out.safe_velocity_norm.Data;
% velocity_norm = out.velocity_norm.Data;
% desired_velocity_norm = out.desired_velocity_norm.Data;

%% plots
if disturbance
    [fig1, colours] = plot_map(obstacles,gamma);
else
    [fig1, colours] = plot_map(obstacles);
end
plot_trajectory(q1,q2,qstart,qg)
fig2 = plot_cbf(cbf,time, true, colours);
fig3 = plot_evolution(q1,q2,qg,time);

% TODO
%fig5 = plot_comparison({nom_input_norm}, "nominal input", time, 'meters', "$\mu(q-q_O)^T\cdot\dot{q}$");