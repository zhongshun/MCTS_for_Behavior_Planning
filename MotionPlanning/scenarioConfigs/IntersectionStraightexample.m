function [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionStraightexample()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.14 (R2023a) and Automated Driving Toolbox 3.7 (R2023a).
% Generated on: 17-Aug-2023 14:06:18

% Construct a drivingScenario object.
scenario = drivingScenario("SampleTime",1.0, "StopTime",15.0);
actorWaypoints = cell(0);
allStatus = cell(0);
roadConfigs = cell(0);
egoWaypoints = [-56.4081537864988 -6 0;
    21.5 -6 0;
    96.7 -6 0];

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
    'Position', [-56.4081537864988 -5.96299137340629 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-53.5 -2.2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [-53.5 -2.2 0;
    -42.5 -2.9 0;
    -34.8 -5.3 0;
    -26 -5.9 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [10;10;10;0];
waittime = [0;0;0;5];
yaw = [NaN;NaN;NaN;NaN];
car1Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car1Status;
trajectory(car1, waypoints, speed, waittime);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-32.9987369256917 -5.54979212024746 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [-32.9987369256917 -5.54979212024746 0;
    -16 -4 0;
    -4.7 -2.9 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [10;5;0];
yaw =  [0;NaN;0];
waittime = [0;0;5];
car2Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car2Status;
trajectory(car2, waypoints, speed, waittime, 'Yaw', yaw);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-39.0846292643625 -9.31544670639672 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [-39.0846292643625 -9.31544670639672 0;
    -16.4 -8 0;
    -5.4 -6.1 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [5;5;5];
yaw =  [0;NaN;0];
waittime = [0;0;0];
car3Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car3Status;
trajectory(car3, waypoints, speed, 'Yaw', yaw);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-30.6968875133277 -9.09537017357411 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');
waypoints = [-30.6968875133277 -9.09537017357411 0;
    -13 -9.1 0;
    -2 -9.2 0;
    2.6 -17.3 0;
    2.0 -30.5 0;
    1.5 -43.1 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [5;5;5;5;5;5];
yaw =  [NaN;NaN;NaN;NaN;NaN;NaN];
waittime = [0;0;0;0;0;0];
car4Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car4Status;
trajectory(car4, waypoints, speed, 'Yaw', yaw);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-23.8 -2.3 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');
waypoints = [-23.8 -2.3 0;
    -4.2 -2.5 0;
    9.1 0.5 0;
    13.2 15 0;
    13.3 36.3 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [10;10;10;10;10];
yaw = [NaN;NaN;NaN;NaN;NaN];
waittime = [0;0;0;0;0;];
car5Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car5Status;
trajectory(car5, waypoints, speed);

