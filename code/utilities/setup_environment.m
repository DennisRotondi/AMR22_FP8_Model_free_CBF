function obstacles = setup_environment(map_name)
    % this function return the "map" i.e. the distrbution of 
    % the obstacles according to the chosen map_name
    qO1 = [1.5;0];                  % 1st obstacle position
    qO2 = [3;-1.5];               % 2rd obstacle position
    qO3 = [3;-1.5];                 % 3nd obstacle position
    qO4 = [3;-0.19];                % 4th obstacle position
    qO5 = [4;-0.4];                 % 5th obstacle position
    obs = [qO1 qO2 qO3 qO4 qO5];    % obstacles center
    r = [0.75 0.75 0.45 0.45 0.45]; % obstacles radius
    clearance = [0 0 0.05 0.05 0.05];
    obstacles = [];

    if map_name == "map_2"
        list_idx = [1,2];
    end
    if map_name == "map_3"
        list_idx = [1,2,3];
    end
    for i=list_idx
        obss = [obs(:,i); r(i); clearance(i)];
        obstacles = [obstacles, obss];
    end
end

