function fig = plot_cbf(cbf,time)
    % print CBF function
    fig = figure("Name","Obstacle Avoidance through CBF: Control Barrier Function");
    plot(time,cbf(1,:),'Color','#102542','LineWidth',4);
    yline(0,'LineWidth',4,'Color','red')
    xlabel('time, $t$ (s)','Interpreter','latex');
    ylabel('CBF, $h$ (m)','Interpreter','latex');
    xlim([0,time(end)])
    grid on;
    set(gca,'FontSize',35);
    set(gca,'FontName',"Latin Modern Math");
end

