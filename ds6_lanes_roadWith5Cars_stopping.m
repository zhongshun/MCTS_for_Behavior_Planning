function [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus,roadConfigs] = ds6_lanes_roadWith5Cars_stopping()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.14 (R2023a) and Automated Driving Toolbox 3.7 (R2023a).
% Generated on: 17-Aug-2023 10:21:00

% Construct a drivingScenario object.
scenario = drivingScenario("SampleTime", 1.0, "StopTime",15.0);
actorWaypoints = cell(0);
allStatus = cell(0);
egoWaypoints = [-47.408062700446 -5.96329158427874 0;
    -4.8 -5.7 0;
    21.5 -5.6 0;
    52.4 -5.8 0;
    74.8 -5.6 0;
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
    'Position', [-47.408062700446 -5.96329158427874 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-41.4 -2.2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [-41.4 -2.2 0;
    -7 -2 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [10;0];
waittime = [0;5];
yaw = [NaN;NaN];
car1Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car1Status;
trajectory(car1, waypoints, speed, waittime);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-34.7086462016492 -6.07793025731412 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [-34.7086462016492 -6.07793025731412 0;
    -6.3 -5.8 0];
speed = [10;0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
waittime = [0;6];
yaw = [NaN;NaN];
car2Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car2Status;
trajectory(car2, waypoints, speed, waittime);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-39.0846292643625 -9.31544670639672 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [-39.0846292643625 -9.31544670639672 0;
    -6.4 -8.9 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [5;5];
waittime = [0;0];
yaw = [NaN;NaN];
car3Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car3Status;
trajectory(car3, waypoints, speed);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-30.6968875133277 -9.09537017357411 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');
waypoints = [-30.6968875133277 -9.09537017357411 0;
    -4.8 -8.8 0;
    3.4 -11 0;
    5.5 -17.7 0;
    5.3 -34.9 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [5;5;5;5;5];
waittime = [0;0;0;0;0];
yaw = [NaN;NaN;NaN;NaN;NaN];
car4Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car4Status;
trajectory(car4, waypoints, speed);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-23.8 -2.3 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');
waypoints = [-23.8 -2.3 0;
    -6.1 -2.2 0;
    10.1 2.4 0;
    12.8 10.6 0;
    13.6 31.1 0];
actorWaypoints{numel(actorWaypoints) + 1} = waypoints;
speed = [10;10;5;5;10];
waittime = [0;0;0;0;0];
yaw = [NaN;NaN;NaN;NaN;NaN];
car5Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car5Status;
trajectory(car5, waypoints, speed, waittime);

