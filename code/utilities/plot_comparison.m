function fig = plot_comparison(signals, name, time, dimension, sig_names)
    % print torques norms
    fig = figure("Name","Obstacle Avoidance through CBF: "+ name);
    [~, num_sig] = size(signals);
    color = ["#102542", "#23CE6B","#D11149","#EA638C"];
    hold on
    for i=1:num_sig
        plot(time,cell2mat(signals(i)),'LineWidth',4, 'DisplayName',sig_names(i),'Color',color(i));
    end
    legend('Interpreter','latex','FontSize',35);
    xlabel('time, $t$ (s)','Interpreter','latex');
    ylabel(dimension,'Interpreter','latex');
    xlim([0,time(end)]);
    grid on;
    set(gca,'FontSize',35);
    set(gca,'FontName',"Latin Modern Math");
end

