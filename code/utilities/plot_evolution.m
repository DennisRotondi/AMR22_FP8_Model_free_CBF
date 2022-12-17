function fig = plot_evolution(time,q1,q2,qg,simTime)
    fig=figure("Name","Obstacle Avoidance through CBF: Position Evolution");
    plot(time,q1,time,q2,'LineWidth',4);
    yline(qg(1),'-.','LineWidth',4)
    yline(qg(2),'-.','LineWidth',4)
    legend('$q_1(t)$','$q_2(t)$','Interpreter','latex','FontSize',30);
    xlabel('time, $t$ (s)','Interpreter','latex');
    ylabel('position, $q(t)$ (m)','Interpreter','latex');
    fontname(gca,"Latin Modern Math")
    xlim([0,simTime])
    grid on;
    fontsize(gca,35,'points');
end

