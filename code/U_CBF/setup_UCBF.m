% This file is going to extract the simulation
% variables from simulink and to initializate 
% the simulation of Model-free unicycle

% Control Barrier Function, Progect FP8- 2022
% Dennis Rotondi - Marco Montagna - Mirko Mizzoni

% The CBF considers the point A
% h = ||p-pobs||^2 - Robs - clearance
clear 
close all
clc
set(0, 'defaultFigureUnits', 'centimeters', 'defaultFigurePosition', [20 20 25 25]);
q0 = [-1 0 0 0 0];         % start variables x0 y0 theta0 v0 w0
qstart = [q0(1) ; q0(2)]; % initial cartesian position
RadiusRobot = 0.3;

% display mode - - - - - - - - - - - - - - - - - 

% 1 for watching the robot movie
% 0 else
animation_mode = 1; 
                   

% simulation parameters
simTime = 150;               % time of simulation

% reference values
x1d = 4;                     % xdes reference
y1d = -1;                    % ydes reference 
qg=[x1d ;y1d];               % qgoal

% system parameters
m = 1;                       % mass
Icm = 0.6;                   % inertia
a = 0.20;                    % distance from the center 
params=[m,Icm,a];            % vector of parameters

% nominal controller gains
kp = 0.6;                    % proportional
kd = 1;                      % derivative
ContGainVec=[kp kd];         % gain vector

% barrier parameters
alpha = 0.8;                 % barrier certificate param 
mu = 1;                    % parameter of the CBF

% obstacle parameters
r1 = 1;                      % radius obstacle
qO1 = [1.8;0.7];               % 1st obstacle position
qO2 = [3;-3];                % 2nd obstacle position
qO3 = [4.6;-3];              % 3rd obstacle position
qO4 = [5;0.5];               % 4th obstacle position
qO5 = [4;0];                 % 5th obstacle position
r = [r1 1.4 r1 0.45 0.45];   % radius vector
threshold_skip = 0;          % 0.05
obs = [qO1 qO2 qO4];         % obstacle vector
clearance = RadiusRobot+a+0.1;   % clearance

% Start simulation - - - - -  - - - - - - - - - - - -
out = sim('simulation_UCBF');             % from sim
q1 =  out.configuration_vector.Data(1,:); % x(t)
q2 =  out.configuration_vector.Data(2,:); % y(t)
theta =  out.theta.Data;                  % y(t)
nom_input_norm =  out.u_nominal.Data;     % ||u(t)||
time = out.configuration_vector.Time;     % time t 

% Plot trajectory of point A          
figure("Name","Obstacle Avoidance through CBF")
plot(q1,q2,'Color','#102542','LineWidth',4,'LineStyle','-.');
axis("equal");
xlabel('position, $q_1$ (m)','Interpreter','latex');
ylabel('position, $q_2$ (m)','Interpreter','latex');
[~,num_obs] = size(obs);
hold on;
% plot the obstacles 
for i = 1:num_obs
    ob = obs(:,i);
    th = 0:pi/50:2*pi;
    xunit = r(i) * cos(th) + ob(1);
    yunit = r(i) * sin(th) + ob(2);
    col = rand(1,3);
    plot(xunit, yunit,'color', 'k','LineWidth',9);
    fill(xunit, yunit, col);
    text(ob(1),ob(2),"$O_"+ num2str(i) +"$",'Interpreter','latex','FontSize',40,'Color','white');
    xunit_clearance = (r(i)+0.1+a) * cos(th) + ob(1);
    yunit_clearance = (r(i)+0.1+a) * sin(th) + ob(2);
    plot(xunit_clearance, yunit_clearance,'Color',col,'LineStyle','--','LineWidth',4);
    
end

% Plotting the qstart and qgoal
text(qstart(1)+0.01,qstart(2)+0.23,'$\bf{q_{start}}$','Interpreter','latex','FontSize',20);
text(qg(1)+0.15,qg(2),'$\bf{q_{goal}}$','Interpreter','latex','FontSize',20);
fontname(gca,"Latin Modern Math")
fontsize(gca,35,'points');
% to note that the actual robot coordinates
% are : x = x - acos(theta) ; y = y-asin(theta

% Roomba Settings for Plotting
xunit = RadiusRobot * cos(th) + q1(1)-a*cos(theta(1));
yunit = RadiusRobot * sin(th) + q2(1)-a*sin(theta(1));
plot(qstart(1),qstart(2),'marker','o','Color','red','MarkerSize',18,'MarkerFaceColor','red'); 
plot(qg(1),qg(2),'marker','o','Color','green','MarkerSize',18,'MarkerFaceColor','green');
plot(q1(end),q2(end),'marker','o','Color','blue','MarkerSize',18,'MarkerFaceColor','blue'); 
if animation_mode==1
    col_roomba = [1; 0.5; 0; 0.75];
    m=plot(q1(i),q2(i),'marker','o','Color','green','MarkerSize',3,'MarkerFaceColor','green');
    f = fill(xunit,yunit,[0.16,0.62,0.57],'FaceAlpha',0.4);
    f.LineWidth = 3;
    f.EdgeColor = [0.5 0.2 0.55];
    for i=1:length(time)
        delete(f);
        delete(m);
        q=[q1(i);q2(i)];
        if norm(q-qg)<1e-1
            break % tolerance setting
        end
        xunit = RadiusRobot * cos(th) + q1(i)-a*cos(theta(i));
        yunit = RadiusRobot * sin(th) + q2(i)-a*sin(theta(i));
        f = fill(xunit,yunit,[0.16,0.62,0.57],'FaceAlpha',0.9);
        f.LineWidth = 3;
        f.EdgeColor = [0.1882 0.1882 0.1882];
        m = plot(q1(i),q2(i),'marker','o','Color','k','MarkerSize',5,'MarkerFaceColor','green');
        drawnow;
        title("time:"+num2str(time(i)));
%         if (abs(time(i)-20) < 1e-2)
%             pause;
%         end
    end
end
hold off;

% Plot Configuration Evolution
figure("Name","Obstacle Avoidance through CBF: Position Evolution")
h =plot(time,q1,'r',time,q2,'b','LineWidth',4);
yline(qg(1),'-.','LineWidth',4)
yline(qg(2),'-.','LineWidth',4)
legend('$q_1(t)$','$q_2(t)$','Interpreter','latex','FontSize',30);
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('position, $q(t)$ (m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,35,'points');

% Plot Nominal Acceleration
figure("Name","Obstacle Avoidance through CBF: Velocity")
plot(time,nom_input_norm,'Color','#102542','LineWidth',4);
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('torque, $\| \tau \|$ (N$\cdot$ m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,35,'points');
cbf = out.cbf.Data;
NMatlabBlue     = [0        0.4470   0.7410]; 

% print CBF function
figure("Name","Obstacle Avoidance through CBF: Control Barrier Function")
plot(time,cbf(1,:),'Color','#102542' ,'LineWidth',5);
yline(0,'LineWidth',4,'Color','red')
xlabel('time, $t$ (s)','Interpreter','latex');
ylabel('CBF, $h$ (m)','Interpreter','latex');
fontname(gca,"Latin Modern Math")
xlim([0,simTime])
grid on;
fontsize(gca,35,'points');
