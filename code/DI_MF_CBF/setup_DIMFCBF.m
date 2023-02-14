% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of MODEL FREE CBF Method on DI

clear; close all; clc;
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
%% robot initialization
% these are the robot parameters that remain fixed during the experiments
qstart = [0 ; 0];          % starting position
qdotstart = [0 ; 0];        % starting velocity
M = 1*eye(2);               % true inertia matrix
robot_radius = 0;           % radius of the robot 
a = 0;                      % displacement wrt robot center
%% simulation stuff
simTime = 40;               % simulation time
MAP = "map_2";
% the function creates an 4xn matrix where n is the numer of the obstacles,
% the first two rows are the center, then radius and clearance
obstacles = setup_environment(MAP);
disturbance = true;         % if to simulate with disturbance in h or not
%% control parameters
qg = [4, -1];               % desired position
Kp = 0.2*1;                 % proportional term
Kd = 1*1;                   % derivative term 
threshold_skip = 0;         % hysteresis mechanism
alpha = 0.99;               % barrier certificate param 
k1 = sqrt(min(eig(M))/2)    % k1 bound defined as in the report
lambda = Kd/max(eig(M))     % stability parameter 
norm_d_inf = 0.5;
gamma = norm_d_inf/(2*k1*alpha) % gamma function
%% run simulation
out = sim('simulation_DIMFCBF');
% import simData from simulink
time = out.cbf.Time;
q1 =  out.configuration_vector.Data(:,1);
q2 =  out.configuration_vector.Data(:,2);
input_norm =  out.input_norm.Data;
input = out.input.Data;
safe_velocity_norm =  out.safe_velocity_norm.Data;
velocity_norm = out.velocity_norm.Data;
desired_velocity_norm = out.desired_velocity_norm.Data;

nominal_velocity = squeeze(out.nominal_velocity.Data);
safe_velocity =  out.safe_velocity.Data;
tracked_velocity = out.tracked_velocity.Data;

cbf = out.cbf.Data;
%% plots
% todo: track velocities 
if disturbance
    fig1 = plot_map(obstacles,gamma);
else
    fig1 = plot_map(obstacles);
end
plot_trajectory(q1,q2,qstart,qg);
fig2 = plot_cbf(cbf,time);
yline(gamma,'LineWidth',4,'Color','blue','HandleVisibility','off')
fig3 = plot_evolution(q1,q2,qg,time);

signals = {safe_velocity_norm, velocity_norm, desired_velocity_norm};
name = "Velocity ";
sig_names = ["safe","actual","nominal"];
dimension = 'velocity, $\| \dot{q} \|$ (m/s)';
fig4 = plot_comparison(signals, name, time, dimension, sig_names);

signalsx = {safe_velocity(:,1), tracked_velocity(:,1), nominal_velocity(1,:)};
name = "Velocity on x";
sig_names = ["safe","actual","nominal"];
dimension = 'velocity, $\dot{q}_1$ (m/s)';
fig5 = plot_comparison(signalsx, name, time, dimension, sig_names);

signalsy = {safe_velocity(:,2), tracked_velocity(:,2), nominal_velocity(2,:)};
name = "Velocity on y";
sig_names = ["safe","actual","nominal"];
dimension = 'velocity, $\dot{q}_2$ (m/s)';
fig6 = plot_comparison(signalsy, name, time, dimension, sig_names);

name = "Control effort";
sig_names ='a';
dimension = 'input, $u_1$ (N)';
fig7 = plot_comparison({input(:,1)}, name, time, dimension, sig_names);
b = gca; legend(b,'off');
name = "Control effort2";
sig_names ='b';
dimension = 'input, $u_2$ (N)';
fig8 = plot_comparison({input(:,2)}, name, time, dimension, sig_names);
b = gca; legend(b,'off');

list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for i = 1:length(index_interpreter)
    default_name = strrep(list_factory{index_interpreter(i)},'factory','default');
    set(groot, default_name,'latex');
end