% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of MODEL FREE CBF Method on DI

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
simTime = 50;               % simulation time
M2 = 1*eye(2);              % estimated inertia matrix ( may be different from original)
MAP = "map_2";
% the function creates an 4xn matrix where n is the numer of the obstacles,
% the first two rows are the center, then radius and clearance
obstacles = setup_environment(MAP);
%% control parameters
qg = [4, -1];               % desired position
alpha = 0.1;                % barrier certificate param 
Kp = 0.2;                   % proportional term
Kd = 1;                     % derivative term 
threshold_skip = 0;         % hysteresis mechanism
%% run simulation
out = sim('simulation_DIMFCBF');
% import simData from simulink
time = out.cbf.Time;
q1 =  out.configuration_vector.Data(:,1);
q2 =  out.configuration_vector.Data(:,2);
input_norm =  out.input_norm.Data;
safe_velocity_norm =  out.safe_velocity_norm.Data;
velocity_norm = out.velocity_norm.Data;
desired_velocity_norm = out.desired_velocity_norm.Data;
cbf = out.cbf.Data;
%% plots
fig1 = plot_map(obstacles);
plot_trajectory(q1,q2,qstart,qg);
fig2 = plot_cbf(cbf,time);
fig3 = plot_evolution(q1,q2,qg,time);
signals = {safe_velocity_norm, velocity_norm, desired_velocity_norm};
name = "Velocity ";
sig_names = ["safe","actual","desired"];
dimension = 'velocity, $\| \dot{q} \|$ (m/s)';
fig4 = plot_comparison(signals, name, time, dimension, sig_names);
name = "Control effort";
sig_names ="$ \alpha $ = "+num2str(alpha);
dimension = 'input, $ \| u \|$ (N$\cdot$ m)';
fig5 = plot_comparison({input_norm}, name, time, dimension, sig_names);
