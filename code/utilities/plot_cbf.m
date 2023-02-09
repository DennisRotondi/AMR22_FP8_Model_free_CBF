function fig = plot_cbf(cbf,time, multiplecbf, colours)
    if nargin<3
        multiplecbf = false;
        colours = "#000000";
    end

    % print CBF function
    fig = figure("Name","Obstacle Avoidance through CBF: Control Barrier Function");
    [n, ~] = size(cbf);
    if multiplecbf
        for i=1:n
            hold on;
            plot(time,cbf(i,:),'Color',colours(i,:),'LineWidth',4,"DisplayName","h obstacle "+i);
            hold on;
        end
        legend('Location','northeast');
    else
        plot(time,cbf,'Color',colours,'LineWidth',4);
        hold on;
    end
    yline(0,'LineWidth',4,'Color','red', 'HandleVisibility','off')
    xlabel('time, $t$ (s)','Interpreter','latex');
    ylabel('CBF, $h$ (m)','Interpreter','latex');
    xlim([0,time(end)])
    grid on;
    set(gca,'FontSize',35);
    set(gca,'FontName',"Latin Modern Math");
end

