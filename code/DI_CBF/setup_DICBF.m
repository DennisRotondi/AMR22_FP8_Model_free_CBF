% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of STANDARD CBF Method on DI

clear; close all; clc;
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
%% robot initialization
% these are the robot parameters that remain fixed during the experiments
qstart = [0 ; 0];           % starting position
qdotstart = [0 ; 0];        % starting velocity
M = 1*eye(2);               % true inertia matrix
robot_radius = 0;           % radius of the robot 
a = 0;                      % displacement wrt robot center
%% simulation stuff
simTime = 100;              % simulation time
M2 = M;                     % estimated inertia matrix ( may be different from original)
MAP = "map_3";
% the function creates an 4xn matrix where n is the numer of the obstacles,
% the first two rows are the center, then radious and clearance
obstacles = setup_environment(MAP);
[~, num_obs] = size(obstacles);
%% control parameters  
qg = [5, -1];               % desired position
alpha = 0.2;                % barrier certificate param 
Kp = 0.2;                   % proportional term
Kd = 1;                     % derivative term 
mu = 0.8;                   % cbf velocity weight
%% run simulation
out = sim('simulation_DICBF');
% import simData from simulink
time = out.cbf.Time;
q1 = out.configuration_vector.Data(:,1);
q2 = out.configuration_vector.Data(:,2);
cbf = squeeze(out.cbf.Data);
% the term to guarantee relative degree = 1
%second_part_cbf = out.second_part_cbf.Data(1,:);
% actual velocity of the robot
velocity_norm = out.actual_velocity_norm.Data;
% desired input produced from the nominal controller
desired_input_norm = out.desired_input_norm.Data;
% input after safe optimization (actual input)
safe_input_norm =  out.safe_u_norm.Data;
%% plots
[fig1, colours] = plot_map(obstacles);
plot_trajectory(q1,q2,qstart,qg);
fig2 = plot_cbf(cbf,time, true, colours);
fig3 = plot_evolution(q1,q2,qg,time);
signals = {desired_input_norm, safe_input_norm};
name = "Torque signals";
sig_names = ["desired","safe"];
dimension = 'torque, $\| \tau \|$ (N$\cdot$ m)';
fig4 = plot_comparison(signals, name, time, dimension, sig_names);
%fig5 = plot_comparison({second_part_cbf}, "cbf second part", time, 'meters', "$\mu(q-q_O)^T\cdot\dot{q}$");
