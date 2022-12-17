function plot_trajectory(q1,q2,qs,qg)
    plot(q1,q2,'Color','k','LineWidth',4);
    plot(qs(1),qs(2),'marker','o','Color','red','MarkerSize',15,'MarkerFaceColor','red'); hold on;
    plot(qg(1),qg(2),'marker','o','Color','green','MarkerSize',15,'MarkerFaceColor','green');
    text(qs(1)+0.1,qs(2),'$\bf{q_{start}}$','Interpreter','latex','FontSize',30);
    text(qg(1),qg(2)-0.1,'$\bf{q_{goal}}$','Interpreter','latex','FontSize',30);
    plot(q1(end),q2(end),'marker','o','Color','blue','MarkerSize',10,'MarkerFaceColor','blue');
end

