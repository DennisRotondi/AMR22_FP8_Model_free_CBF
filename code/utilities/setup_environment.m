function obstacles = setup_environment(map_name, a, robot_radius)
    if nargin < 3
        a = 0; 
        robot_radius = 0;
    end
    % this function return the "map" i.e. the distrbution of 
    % the obstacles according to the chosen map_name
    qO1 = [1.5;0];                  % 1st obstacle position
    qO2 = [3;-1.5];                 % 2nd obstacle position
    qO3 = [3;-1.5];                 % 3rd obstacle position
    qO4 = [3;-0.19];                % 4th obstacle position
    qO5 = [4;-0.4];                 % ... and so on
    qO6 = [1.8;0.7];                
    qO7 = [3;-3];                 
    qO8 = [4.6;-3];               
    qO9 = [5;0.5];                
    qO10 = [4;0];                 
    obs = [qO1 qO2 qO3 qO4 qO5 qO6 qO7 qO8 qO9 qO10];    % obstacles center
    r = [0.75 0.75 0.45 0.45 0.45 1 1.4 1 0.75 0.45];    % obstacles radius
    clr = robot_radius+a;
    clearance = [0 0 0.05 0.05 0.05 0.1 0.1 0.07 0.1 0.05];
    obstacles = [];
    if map_name == "map_2"
        list_idx = [1,2];
    elseif map_name == "map_3"
        list_idx = [1,2,3];
    elseif map_name == "map_4"
        list_idx = [6,7,9];
    end
    for i=list_idx
        obss = [obs(:,i); r(i); clearance(i)+clr];
        obstacles = [obstacles, obss];
    end
end

