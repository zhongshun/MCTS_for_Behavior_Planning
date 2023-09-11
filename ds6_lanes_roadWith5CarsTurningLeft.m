function [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = ds6_lanes_roadWith5CarsTurningLeft()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.14 (R2023a) and Automated Driving Toolbox 3.7 (R2023a).
% Generated on: 09-Aug-2023 13:43:52

% Construct a drivingScenario object.
scenario = drivingScenario("SampleTime", 1.0, "StopTime", 15.0);
actorWaypoints = cell(0);
allStatus = cell(0);
egoWaypoints = [-23.2079542936621 -5.86580669860298 0;
    -6.0 -5.6 0;
    21.5 -5.6 0;
    96.7 -6 0];
roadConfigs = cell(0);

% Add all road segments
roadCenters = [-68.9 -0.5 0;
    100.2 0 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('DoubleSolid', 'Color', [0.9 0.9 0.3])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(6, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');
tempRoad = struct('roadCenters', roadCenters, 'marking', marking, ...
    'laneSpecification', laneSpecification);
roadConfigs{numel(roadConfigs) + 1} = tempRoad;


roadCenters = [8.1 70.8 0;
    7.2 -79.3 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('DoubleSolid', 'Color', [0.96 0.98 0.3])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(6, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road1');
tempRoad = struct('roadCenters', roadCenters, 'marking', marking, ...
    'laneSpecification', laneSpecification);
roadConfigs{numel(roadConfigs) + 1} = tempRoad;


% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-23.2079542936621 -5.86580669860298 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
% speed = [5;5;5;5;5;5];
% trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [24 1.3 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [24 1.3 0;
    8.3 -1.9 0;
    -1.1 -16.3 0;
    -1.9 -28.9 0;
    -2.3 -77.8 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [10;10;10;10;10];
yaw =  [180;NaN;NaN;NaN;NaN];
waittime = [0;0;0;0;0];
car1Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car1Status;
trajectory(car1, waypoints, speed, waittime, 'Yaw', yaw);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [31 1.4 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [31 1.4 0;
    19.1 1.5 0;
    4.2 -6.1 0;
    2.3 -19.1 0;
    2.2 -34.4 0;
    1.9 -67.2 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [10;10;10;10;10;10];
yaw =  [180;NaN;NaN;NaN;NaN;NaN];
waittime = [0;0;0;0;0;0];
car2Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car2Status;
trajectory(car2, waypoints, speed, 'Yaw', yaw);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [49.8 1.7 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [49.8 1.7 0;
    16.2 0.3 0;
    8.1 -1.7 0;
    0.2 -10.5 0;
    -1.9 -22.4 0;
    -2 -66.9 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [5;15;10;10;10;15];
yaw =  [180;NaN;NaN;NaN;NaN;NaN];
waittime = [0;0;0;0;0;0;];
car3Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car3Status;
trajectory(car3, waypoints, speed, 'Yaw', yaw);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [63.4 1.5 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');
waypoints = [63.4 1.5 0;
    36.4 1.4 0;
    17.3 -0.3 0;
    7.2 -8.5 0;
    5.6 -18.4 0;
    5.8 -62.3 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [5;10;15;15;15;15];
yaw =  [180;NaN;NaN;NaN;NaN;NaN];
waittime = [0;0;0;0;0;0];
car4Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car4Status;
trajectory(car4, waypoints, speed, 'Yaw', yaw);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [73.9 1.7 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');
waypoints = [73.9 1.7 0;
    55 1.9 0;
    28.9 2.1 0;
    8.5 -5 0;
    5.7 -17.5 0;
    5.7 -53 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [5;7;8;10;10;10];
waittime = [0;0;0;0;0;0];
yaw = [NaN;NaN;NaN;NaN;NaN;NaN];
car5Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car5Status;
trajectory(car5, waypoints, speed, waittime);

