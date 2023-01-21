% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of MODEL FREE CBF Method on U

clear; close all; clc;
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
%% robot initialization
qstart = [-2 0.1 0]';      % starting pose (x0 y0 theta0)
vwstart = [0 0]';          % starting velocities (w0,v0)
m = 5;                 % mass              % inertia
a = 0.20;                  % distance from the center 
robot_radius = 0.3;
Icm = 1/2*m*robot_radius^2; 
params=[m,Icm,a];          % vector of parameters

%% simulation stuff
% 1 for watching the robot movie
% 0 else
animation_mode = 0;                  
simTime = 50;              % time of simulation
MAP = "map_4";
obstacles = setup_environment(MAP,a,robot_radius);
%% control parameters
qg=[4 ; -1];                 % qgoal
Kp = 2;                      % proportional
Kd = 100;                      % derivative
alpha = 0.2;                   % barrier certificate param 
threshold_skip = 0;          % 0.05
delta = 0.5;                 % barrier parameter
%% run simulation
out = sim('simulation_UMFCBF');           % from sim
% import simData from simulink
time = out.cbf.Time;
q1 =  out.configuration_vector.Data(:,1);
q2 =  out.configuration_vector.Data(:,2);
input_norm =  out.input_norm.Data;
cbf = out.cbf.Data;
% TODO SPLIT VELOCITIES FOR PLOTS, NO NORMS
% safe_velocity_norm =  out.safe_velocity_norm.Data;
% velocity_norm = out.velocity_norm.Data;
% desired_velocity_norm = out.desired_velocity_norm.Data;

%% plots
fig1 = plot_map(obstacles);
plot_trajectory(q1,q2,qstart,qg)
fig2 = plot_cbf(cbf,time);
fig3 = plot_evolution(q1,q2,qg,time);
% TODO
% fig5 = plot_comparison({nom_input_norm}, "nominal input", time, 'meters', "$\mu(q-q_O)^T\cdot\dot{q}$");