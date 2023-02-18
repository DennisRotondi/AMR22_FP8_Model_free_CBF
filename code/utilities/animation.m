close all

step = 0.0333333333; % 30 FPS
time = [0:step:out.cbf.Time(end)];

timestamp = strrep(string(datetime('now'))," ","-");
v = VideoWriter("animations/sim-"+timestamp+".avi",'Motion JPEG AVI');
v.Quality = 90;
framerate = round(1/step);
v.FrameRate = framerate;

%if consider unycicle
uni = a>0;
mf = exist('gamma','var');
gg = 0;
rem = 0;
min_dist=0.05;
if uni
    rem = robot_radius;
end
if mf 
    gg = gamma;
    q1 = q1';
    q2 = q2';
end
if ~uni && ~mf
    q1 = q1';
    q2 = q2';
    cbf = squeeze(cbf)';
    out.cbf.Time = out.cbf.Time';
end

% resample time series in order to have fixed step samples
q1a = resample(squeeze(q1),out.cbf.Time,1/step);
q2a = resample(squeeze(q2),out.cbf.Time,1/step);
cbfa = resample(squeeze(cbf),out.cbf.Time',1/step,'Dimension',2);
in1 = resample(squeeze(safe_input(:,1)),out.cbf.Time, 1/step);
in2 = resample(squeeze(safe_input(:,2)),out.cbf.Time, 1/step);
if uni
    thetaa = resample(squeeze(theta), out.cbf.Time,1/step);
end

visibility = 'on';
fig = figure('Units','pixels','Position', [0 0 1280 1024],'visible',visibility);
t = tiledlayout(3,4,'TileSpacing','Compact','Padding','Compact');
spax = nexttile([3 3]);
set(gca,'Color',"#F7F7FF")
colours = plot_mapan(obstacles,gg, rem);
plot(qstart(1),qstart(2),'marker','o','Color','red','MarkerSize',18,'MarkerFaceColor','red'); 
plot(qg(1),qg(2),'marker','o','Color','green','MarkerSize',18,'MarkerFaceColor','green');

if ~uni
    xlim([0,4.5])
    ylim([-3,1.5]);
else
    xlim([-0.5+qstart(1),qg(1)+1]);
    ylim([-5,5]);
end

for i=1:length(time) 
    q1i = q1a(1,i);
    q2i = q2a(1,i);
    if norm([q1i;q2i]-[q1(1,end);q2(1,end)]) < min_dist
        break;
    end
    disp(i)
end
time = time(1,1:i);

q1a = q1a(1:i);
q2a = q2a(1:i);
if uni
    thetaa = thetaa(1:i);
end
cbfa = cbfa(:,1:i);
in1 = in1(1:i);
in2 = in2(1:i);

open(v);
disp("Start saving video...")
for i=1:length(time) 
    q1i = q1a(i);
    q2i = q2a(i);
    if uni
        thetai = thetaa(i);
    else
        thetai = 0;
    end
    nexttile(1,[3 3])
    plot_trajan(q1i,q2i,thetai, uni);

    nexttile(4)
    plot_cbfan(cbfa, time,i, uni, colours)
    if mf
        yline(gamma,'LineWidth',1,'Color','blue','HandleVisibility','off');
    end
    nexttile(8)
    plot_single(in1, time, i, 'input, $u_1$ (N)');
     
    nexttile(12)
    if uni
        plot_single(in2, time, i,'input, $u_2$ (N$\cdot$m)');
    else
        plot_single(in2, time, i,'input, $u_2$ (N)');
    end
    xlabel('time, $t$ (s)','Interpreter','latex');
    if i>2
        frame = getframe(fig);
        writeVideo(v,frame);
    end
    drawnow;
end

close(v);
close(fig);
disp("...video saved")

function [colours] = plot_mapan(obstacles, gamma, rem)
    if nargin < 2
        gamma = 0;
        rem = 0;
    end
    % plot obstacles 
    [~, num_obs] = size(obstacles);
    axis("equal");
    xlabel('position, $q_1$ (m)','Interpreter','latex', "FontSize",30);
    ylabel('position, $q_2$ (m)','Interpreter','latex', "FontSize",30);
    hold on;
    colours = zeros(num_obs,3);
    for i = 1:num_obs
        ob = obstacles(:,i);
        ob(3) = ob(3)-rem;
        th = 0:pi/50:2*pi;
        col = rand(1,3);
        colours(i,:) = col;
        if ob(4)+gamma ~= 0
            xunit_clearance = (ob(3)+ob(4)+gamma) * cos(th) + ob(1);
            yunit_clearance = (ob(3)+ob(4)+gamma) * sin(th) + ob(2);
            plot(xunit_clearance, yunit_clearance, '--','LineWidth',3, 'Color', col);
        end
    end
    for i = 1:num_obs
        ob = obstacles(:,i);
        ob(3) = ob(3)-rem;
        th = 0:pi/50:2*pi;
        xunit = ob(3) * cos(th) + ob(1);
        yunit = ob(3) * sin(th) + ob(2);
        col=colours(i,:);
        plot(xunit, yunit,'color', 'k','LineWidth',6);
        fill(xunit, yunit, col)
        text(ob(1)-0.3,ob(2),"$O_"+ num2str(i) +"$",'Interpreter','latex','FontSize',40,'Color','white');
    end
    set(gca,'FontSize',35);
    set(gca,'FontName',"Latin Modern Math");
    box on;
    ax = gca;
    ax.LineWidth = 2;
end

function plot_cbfan(cbf,time,j, multiplecbf, colours)
    if nargin<3
        multiplecbf = false;
        colours = "#000000";
    end
    % print CBF function
    [n, ~] = size(cbf);
    hold on;
    ii = max(1,j-3);
    if multiplecbf
        for i=1:n
            hold on;
%             plot(time(i),cbf(i,j),'marker','o','Color',colours(i,:),'MarkerSize',2,'MarkerFaceColor',colours(i,:));
            plot(time(ii:j),cbf(i,ii:j),'Color',colours(i,:),'LineWidth',2,"DisplayName","$h_"+i+"$");
            hold on;
        end
    else
        plot(time(ii:j),cbf(1,ii:j),'Color',"#000000",'LineWidth',2);
        hold on;
    end
    yline(0,'LineWidth',1,'Color','red', 'HandleVisibility','off')
    if j < 2
        xlim([0,time(end)])
        ylim([min(min(cbf,[],'all'),0), max(cbf,[],'all')])
        ylabel('CBF, $h$ (m)','Interpreter','latex');
        grid on;
        set(gca,'FontSize',18);
        set(gca,'FontName',"Latin Modern Math");
        box on;
        ax = gca;
        ax.LineWidth = 1.5;
        t = time(end)+4;
        ax.YLabel.Position=ax.YLabel.Position+[-ax.YLabel.Position(1)+t,0,0];
    end

end

function plot_single(signals, time, i, dimension)
    hold on
%     plot(time(i),signals(i),'marker','o','Color','k','MarkerSize',2,'MarkerFaceColor','k');
    ii = max(1,i-3);
    plot(time(ii:i),signals(ii:i),'LineWidth',2,'Color',"#102542");

    ylabel(dimension,'Interpreter','latex');
    if i<2
        xlim([0,time(end)])
        ylim([min(min(signals,[],'all'),0), max(signals,[],'all')])
        grid on;
        set(gca,'FontSize',18);
        set(gca,'FontName',"Latin Modern Math");
        box on;
        ax = gca;
        ax.LineWidth = 1.5;
        t = time(end)+4;
        ax.YLabel.Position=ax.YLabel.Position+[-ax.YLabel.Position(1)+t,0,0];
    end
end

function plot_trajan(q1,q2,theta, uni)
    persistent ff mm
    if uni
        robot_radius=0.3;
        a = 0.1;
    end
    if uni && isempty(ff)
        th = 0:pi/50:2*pi;
        xunit = robot_radius * cos(th) + q1-a*cos(theta);
        yunit = robot_radius * sin(th) + q2-a*sin(theta);

        ff = fill(xunit,yunit,[0.16,0.62,0.57],'FaceAlpha',0.9);
        ff.LineWidth = 3;
        ff.EdgeColor = [0.1882 0.1882 0.1882];
        mm = plot(q1,q2,'marker','o','Color','k','MarkerSize',5,'MarkerFaceColor','green');
    end
    plot(q1,q2,'marker','o','Color','blue','MarkerSize',8,'MarkerFaceColor','blue');
    box on;
    ax = gca;
    ax.LineWidth = 2;
    if uni
        delete(ff);
        delete(mm);
        th = 0:pi/50:2*pi;
        xunit = robot_radius * cos(th) + q1-a*cos(theta);
        yunit = robot_radius * sin(th) + q2-a*sin(theta);

        ff = fill(xunit,yunit,[0.16,0.62,0.57],'FaceAlpha',0.9);
        ff.LineWidth = 3;
        ff.EdgeColor = [0.1882 0.1882 0.1882];
        mm = plot(q1,q2,'marker','o','Color','k','MarkerSize',5,'MarkerFaceColor','green');
    end
end