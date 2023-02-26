% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of STANDARD CBF Method on U

clear; close all; clc;
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
%% robot initialization
qstart = [0 0 pi/2]';       % starting pose (x0 y0 theta0)
vwstart = [0 0]';          % starting velocities (w0,v0)
robot_radius = 0.3;
m = 5.3;                   % mass
Icm = 1/2*m*robot_radius^2;% inertia
a = 0.1;                   % distance from the center 
m2 = 3;
Icm2 = 1/2*m2*robot_radius^2;
params = [m2,Icm2,a];      % vector of parameters
%% simulation stuff
% 1 for watching the robot movie
% 0 else
animation_mode = 0;                  
simTime = 40;              % time of simulation
MAP = "map_3";
obstacles = setup_environment(MAP,a,robot_radius);
[~, num_obs] = size(obstacles);
%% control parameters
qg = [8 ; -3];             % qgoal
Kp = 0.2;                  % proportional
Kd = 1;                    % derivative
delta = 0.5;               % barrier parameter
ContGainVec = [Kp Kd];     % gain vector
alpha = 0.8;               % barrier certificate param 
mu = 0.3; % 0.1;           % parameter of the CBF
% mu < min(2/kd, 2kd/(kp+kd^2))
%% run simulation
out = sim('simulation_UCBF');           % from sim
% import simData from simulink
time = out.cbf.Time;
q1 = out.configuration_vector.Data(1,:);
q2 = out.configuration_vector.Data(2,:);
theta =  out.theta.Data; 
cbf = out.cbf.Data;
% the term to guarantee relative degree = 1
%second_part_cbf = out.second_part_cbf.Data(1,:);
% desired input produced from the nominal controller
% desired_input_norm = out.desired_input_norm.Data;
% input after safe optimization (actual input)
safe_input =  out.safe_input.Data;
desired_input = squeeze(out.desired_input.Data);
%% plots
[fig1, colours] = plot_map(obstacles);
plot_trajectory(q1,q2,qstart,qg)
fig2 = plot_cbf(cbf,time, true, colours);
fig3 = plot_evolution(q1,q2,qg,time);

signalsx = {safe_input(:,1), desired_input(1,:)};
name = "Control effort";
sig_names = ["actual","nominal"];
dimension = 'input, $u_1$ (N)';
fig4 = plot_comparison(signalsx, name, time, dimension, sig_names);
% ylim([-3 10])
xlim([0 35])
signalsy = {safe_input(:,2), desired_input(2,:)};
name = "Control effort2";
sig_names = ["actual","nominal"];
dimension = 'input, $u_2$ (N$\cdot$m)';
fig5 = plot_comparison(signalsy, name, time, dimension, sig_names);
xlim([0 35])
% dist con vel, cbf as distance measure; naive; multiple cbf 

list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for i = 1:length(index_interpreter)
    default_name = strrep(list_factory{index_interpreter(i)},'factory','default');
    set(groot, default_name,'latex');
end