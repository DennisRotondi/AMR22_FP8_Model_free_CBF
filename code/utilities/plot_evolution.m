function fig = plot_evolution(q1,q2,qg,time)
    fig=figure("Name","Obstacle Avoidance through CBF: Position Evolution");
    plot(time,q1,time,q2,'LineWidth',4);
    yline(qg(1),'-.','LineWidth',4)
    yline(qg(2),'-.','LineWidth',4)
    legend('$q_1(t)$','$q_2(t)$','Interpreter','latex','FontSize',35);
    xlabel('time, $t$ (s)','Interpreter','latex');
    ylabel('position, $q(t)$ (m)','Interpreter','latex');
    xlim([0,time(end)])
    grid on;
    set(gca,'FontSize',35);
    set(gca,'FontName',"Latin Modern Math");
    box on
end

