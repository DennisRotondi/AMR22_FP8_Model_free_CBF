function fig = plot_cbf(cbf,time)
    % print CBF function
    fig = figure("Name","Obstacle Avoidance through CBF: Control Barrier Function");
    NMatlabBlue     = [0        0.4470   0.7410];
    plot(time,cbf(1,:),'Color',NMatlabBlue ,'LineWidth',4);
    yline(0,'LineWidth',4,'Color','red')
    xlabel('time, $t$ (s)','Interpreter','latex');
    ylabel('CBF, $h$ (m)','Interpreter','latex');
    xlim([0,time(end)])
    grid on;
    set(gca,'FontSize',30);
    set(gca,'FontName',"Latin Modern Math");
end

