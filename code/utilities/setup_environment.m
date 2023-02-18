function obstacles = setup_environment(map_name, a, robot_radius)
    if nargin < 3
        a = 0; 
        robot_radius = 0;
    end
    % this function return the "map" i.e. the distrbution of 
    % the obstacles according to the chosen map_name
    qO1 = [1.5;0];                  % 1st obstacle position
    qO2 = [3;-1.5];                 % 2nd obstacle position
    qO3 = [3;1];                    % 3rd obstacle position
    qO4 = [3.5;0.5];                % 4th obstacle position
    qO5 = [3;-1.5];                 % ... and so on
    qO6 = [6;-1];                
    qO7 = [3;-1];                 
    qO8 = [5;-1];               
    qO9 = [1;-1];                
    qO10 = [6;-2.5];                 
    obs = [qO1 qO2 qO3 qO4 qO5 qO6 qO7 qO8 qO9 qO10];    % obstacles center
    r = [0.75 0.75 0.20 0.45 0.45 0.75 0.45 0.45 0.20 0.30];    % obstacles radius
    clr = robot_radius+a;
    clearance = [0 0 0 0 0 0 0 0 0 0];
    obstacles = [];
    if map_name == "map_2"
        list_idx = [1,2];
    elseif map_name == "map_3"
        list_idx = [4,5,6];
    elseif map_name == "map_4"
        list_idx = [9,4,5,8,10];
    elseif map_name == "map_5"
        list_idx = [3,5];
    end
    for i=list_idx
        obss = [obs(:,i); r(i)+clr; clearance(i)];
        obstacles = [obstacles, obss];
    end
end

