% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of STANDARD CBF Method on U

clear; close all; clc;
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
%% robot initialization
qstart = [-2 0 0]';        % starting pose (x0 y0 theta0)
vwstart = [0 0]';          % starting velocities (w0,v0)
m = 1;                     % mass
Icm = 0.6;                 % inertia
a = 0.20;                  % distance from the center 
params=[m,Icm,a];          % vector of parameters
robot_radius = 0.3;
%% simulation stuff
% 1 for watching the robot movie
% 0 else
animation_mode = 0;                  
simTime = 50;              % time of simulation
MAP = "map_4";
obstacles = setup_environment(MAP,a,robot_radius);
%% control parameters
qg=[4 ;-1];                % qgoal
Kp = 0.2;                  % proportional
Kd = 1;                    % derivative
ContGainVec = [Kp Kd];       % gain vector
alpha = 0.8;                 % barrier certificate param 
mu = 0.3; % 0.1;          % parameter of the CBF
% mu < min(2/kd, 2kd/(kp+kd^2))
threshold_skip = 0;        % 0.05
%% run simulation
out = sim('simulation_UCBF');           % from sim
% import simData from simulink
time = out.cbf.Time;
q1 = out.configuration_vector.Data(1,:);
q2 = out.configuration_vector.Data(2,:);
theta =  out.theta.Data; 
cbf = out.cbf.Data;
% the term to guarantee relative degree = 1
second_part_cbf = out.second_part_cbf.Data(1,:);
% desired input produced from the nominal controller
desired_input_norm = out.desired_input_norm.Data;
% input after safe optimization (actual input)
safe_input_norm =  out.safe_u_norm.Data;
%% plots
fig1 = plot_map(obstacles);
plot_trajectory(q1,q2,qstart,qg)
fig2 = plot_cbf(cbf,time);
fig3 = plot_evolution(q1,q2,qg,time);
signals = {desired_input_norm, safe_input_norm};
name = "Torque signals";
sig_names = ["desired","safe"];
dimension = 'torque, $\| \tau \|$ (N$\cdot$ m)';
fig4 = plot_comparison(signals, name, time, dimension, sig_names);
fig5 = plot_comparison({second_part_cbf}, "cbf second part", time, 'meters', "$\mu(q-q_O)^T\cdot\dot{q}$");