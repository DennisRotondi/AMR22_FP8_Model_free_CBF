function [fig, colours] = plot_map(obstacles, gamma)
    if nargin < 2
        gamma = 0;
    end
    % plot obstacles 
    [~, num_obs] = size(obstacles);
    fig = figure("Name","Obstacle Avoidance through CBF: Trajectory evolution");
    colours = zeros(num_obs,3);
    axis("equal");
    xlabel('position, $q_1$ (m)','Interpreter','latex', "FontSize",30);
    ylabel('position, $q_2$ (m)','Interpreter','latex', "FontSize",30);
    hold on;
    for i = 1:num_obs
        ob = obstacles(:,i);
        th = 0:pi/50:2*pi;
        xunit = ob(3) * cos(th) + ob(1);
        yunit = ob(3) * sin(th) + ob(2);
        col = rand(1,3);
        colours(i,:) = col;
        plot(xunit, yunit,'color', 'k','LineWidth',6);
        fill(xunit, yunit, col)
        text(ob(1)-0.3,ob(2),"$O_"+ num2str(i) +"$",'Interpreter','latex','FontSize',40,'Color','white');
        if ob(4)+gamma ~= 0
            xunit_clearance = (ob(3)+ob(4)+gamma) * cos(th) + ob(1);
            yunit_clearance = (ob(3)+ob(4)+gamma) * sin(th) + ob(2);
            plot(xunit_clearance, yunit_clearance, '--','LineWidth',3, 'Color', col);
        end
    end
    set(gca,'FontSize',35);
    set(gca,'FontName',"Latin Modern Math");
end

