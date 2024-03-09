clear;
% Initially state
addpath('scenarioConfigs');

% VERY IMPORTANT IMFORMATION !!!
% To simulate each scenario that we provided below, just wrap the comments
% of each line. 
% E.g, defaultly, now the program is simulating the scenario of
% Intersection, Go Straight example2.
% After selection the right scenario, just click run and wait until two
% graphs come out, they will show the direct details about this simulation.
% Hint: As the special traits of the MCTS, you may get different result for
% a single scenario, just try to run more times to explore them!
quantResult = cell(1, 100);
for r = 1: 100

% This is for Intersection, Go Straight example2.
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionGoStraightexample2();

% This is for Intersection, Unprotected Straight Cross example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionUnprotectedStraightCrossexample();

% This is for Intersection, Blocked by Stationary Objects example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionBlockedbyStationaryObjectsexample();

% This is for Intersection, Go Straight example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionGoStraightexample();

% This is for Intersection, Blocked example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionBlockedexample();

% This is for Intersection, Unprotected Right Turn example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionUnprotectedRightTurnexample();

% This is for Intersection, Straight example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionStraightexample();

% This is for Intersection, Unprotected Left Turn example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionUnprotectedLeftTurnexample();

% This is for Highway Exit (HE) example
[scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = HighwayExitHEexample();

% This is for Large Curvature example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = LargeCurvatureexample();

% This is for Shanghai Roundabout example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = ShanghaiRoundabout();

% This is for Shanghai Pudong South Intersection Unprotected left turn
% example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = ShanghaiPudongSouthIntersection();

% This is for Shanghai Lujiazui Tunnel example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] =  ShanghaiLujiazuiTunnel();

% This is for Shanghai Centural Avenue Turn Right example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = ShanghaiCenturalAvenue();

% This is for Shanghai Centural Avenue Go Straight example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = ShanghaiCenturalAvenueGoStraight();

% Above are all the scenarios of the program.


temp_test = struct("success", 0, "collision", 0, "Round", r, 'Time', 0);
MCTS_time = [];
% This is for giving the egoCar's initial position.
% setStartEgoState(egoWaypoints, velocity, acceleration)
startEgoState = setStartEgoState(egoWaypoints, 5, 0, egoVehicle.Yaw);

% Create a reference path using waypoints
egorefPath = referencePathFrenet(egoWaypoints);
% refPath = [x y theta kappa dkappa s]
connector = trajectoryGeneratorFrenet(egorefPath,'TimeResolution',1.0);
pathPoints = closestPoint(egorefPath, egorefPath.Waypoints(:,1:2)); % [x y theta kappa dkappa s]
roadS = pathPoints(:,end);

% Set destination position.
DestinationS = egorefPath.PathLength;
SuccessPointS = egorefPath.SegmentParameters(end, end);
% DestinationS = egorefPath.SegmentParameters(end, end);

% Moving egoVehicle into the initial state of the scenario.
helperMoveEgoVehicleToState(egoVehicle, startEgoState);

egoFrenetState = global2frenet(egorefPath, startEgoState);

% Initialize basic configs
TIME = 0;
max_iter = 50;
accMax = 5;
limitJerk = 15;
speedlimit = 20;
MaxTimeHorizon = 2.0;
MaxRolloutHorizon = 2.0;
TimeResolution = 1.0;
root.visits = 1;
root.time = TIME;
root.state = startEgoState; % [x y theta kappa speed acc]
root.children = 0;
root.index = 1;
root.score = 0;
root.parent = 0;
root.UCB = inf;
root.egoFrenetState = egoFrenetState;% [s ds dss l dl dll]
root.laneChangingProperties = struct('LeftChange', 0, 'RightChange', 0, 'Change', false);


AllPath = {startEgoState};
AllTree = {0};
% plot(scenario,'Waypoints','off','RoadCenters','off');
% chasePlot(egoVehicle,"ViewHeight",10.0, "ViewPitch",20);


% compute predicted positions for detected cars
predictedActTrajectories = packUpActorVehicleTrajactory(actorWaypoints, allStatus);
predictedActPositions = cell(0);
for i = 1:numel(predictedActTrajectories)
    if numel(predictedActTrajectories{i}) == 3
        predictedActPositions{i} = predictedActTrajectories{i};
        continue;
    end
    time = 0;
    trajectory = predictedActTrajectories{i};
    predictedActPositions{i} = getActorCurrPosition(trajectory, time);
    time = time + 0.2;
    while time <= predictedActTrajectories{i}.TimeOfArrival(end)
        currentActPosition = getActorCurrPosition(trajectory, time);
        predictedActPositions{i} = [predictedActPositions{i}; currentActPosition];
        time = time + 0.2;
    end
    if time > predictedActTrajectories{i}.TimeOfArrival(end)
        currentActPosition = getActorCurrPosition(trajectory, predictedActTrajectories{i}.TimeOfArrival(end));
        predictedActPositions{i} = [predictedActPositions{i}; currentActPosition];
    end

end


while scenario.SimulationTime < scenario.StopTime && root.egoFrenetState(1) < DestinationS
    
    % This is to detect all actor Vehicles.
    profiles = actorProfiles(scenario);

    % This is to detect the lane that the egoVehicle is driving on.
    lbdry =  laneBoundaries(egoVehicle);
    roadWidth = abs(lbdry(1).LateralOffset - lbdry(2).LateralOffset);


    curr_node = root;
    Tree = {};
    Tree{1} = root;

    %PLANNING SIMULATION
    tic;
    while (Tree{1}.visits < max_iter)
        curr_node = selection(Tree{1}, Tree);


        if curr_node.time < MaxTimeHorizon
            if numel(curr_node.children) == 1
                if curr_node.visits == 0
                    % rollout
                    cost = roll_out(curr_node, MaxRolloutHorizon, TimeResolution, predictedActPositions, accMax, speedlimit, egorefPath, DestinationS, egoVehicle, profiles, scenario);
                    % back propagate
                    Tree = back_propagation(curr_node, cost, Tree);
                    Tree = updateUCB(Tree{1}, Tree);
                else
                    % expand
                    Tree = expand(Tree, curr_node, TimeResolution, accMax, speedlimit, egorefPath, predictedActPositions, egoVehicle, profiles, lbdry, roadWidth, scenario);
                    if numel(Tree) == 1
                        Tree{1}.visits = max_iter;
                    end
                end
                curr_node = Tree{1};
            end
        else
            % extra rollout for Maxtimehorizon
            cost = roll_out(curr_node, MaxRolloutHorizon, TimeResolution, predictedActPositions, accMax, speedlimit, egorefPath, DestinationS, egoVehicle, profiles, scenario);
            Tree = back_propagation(curr_node, cost, Tree);
            Tree = updateUCB(Tree{1}, Tree);
            curr_node = Tree{1};
        end
    end
    % 1 is the index of the root node.
    root = Tree{1};
    flagCollision = false;
    if numel(Tree{1}.children) > 1

        expectedNode = Tree{root.children(2)};
        for i = 2:numel(root.children)
            if Tree{root.children(i)}.UCB >= expectedNode.UCB
                expectedNode = Tree{root.children(i)};
            end
        end
        expectedTrajectory = expectedNode.egoFrenetState;
    else
        stop = true;
        expectedNode = struct('state', root.state, 'time', root.time + TimeResolution, ...
            'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', root.index, 'UCB', inf, 'egoFrenetState', root.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', root.laneChangingProperties);

        emergencyAcc = -(2 * root.egoFrenetState(2) / TimeResolution) - root.egoFrenetState(3);
        % Let's set the maximum deceleration to be 8m/s^2
        if emergencyAcc <= -8
            emergencyAcc = -8;
            stop = false;
        end
        emergencyJerkS = (emergencyAcc - root.egoFrenetState(3)) / TimeResolution;
        [displacementEmergencyS, deltaSpeedEmergencyS, displacementEmergencyL, deltaSpeedEmergencyL] = getDisplacement(root, emergencyJerkS, 0, TimeResolution);

        expectedNode.egoFrenetState = root.egoFrenetState + [displacementEmergencyS, deltaSpeedEmergencyS, emergencyJerkS * TimeResolution, displacementEmergencyL, deltaSpeedEmergencyL, 0];
        expectedNode.state = frenet2global(egorefPath, expectedNode.egoFrenetState);

        disp("Tried emergency break.")
        if stop
            disp("There's obstacles forward, the car has stopped.")
            % disp(checkCollision(root, expectedNode, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, egorefPath))
            expectedNode.egoFrenetState(3) = 0;
            expectedNode.state = frenet2global(egorefPath, expectedNode.egoFrenetState);
        else
            if checkCollision(root, expectedNode, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, egorefPath)
                disp("Collision is inevitable.");
                flagCollision = true;
                temp_test.collision = 1;
                plot(scenario);
                break;
            end
        end

    end
    MCTS_time(numel(MCTS_time) + 1) = toc;
    AllPath{numel(AllPath) + 1} = expectedNode;
    AllTree{numel(AllTree) + 1} = Tree;
    wp = [root.state(1:2) 0; expectedNode.state(1:2) 0];
    if wp(1) ~= wp(2)
        speed = [root.state(5); expectedNode.state(5)];
        waypath = referencePathFrenet(wp);
        yaw = [0; 0];
        % trajectory(egoVehicle, wp, speed, 'Yaw', yaw);
        % show(waypath);
        helperMoveEgoVehicleToState(egoVehicle, expectedNode.state)
    else
        helperMoveEgoVehicleToState(egoVehicle, expectedNode.state)
    end
    


    % reset root properties for next iteration
    root.visits = 1;
    % root.time = TIME;
    root.state = expectedNode.state; % [x y theta kappa speed acc]
    root.children = 0;
    root.index = 1;
    root.score = 0;
    root.parent = 0;
    root.UCB = inf;
    root.egoFrenetState = expectedNode.egoFrenetState;% [s ds dss l dl dll]
    root.laneChangingProperties = expectedNode.laneChangingProperties;
    advance(scenario)
    if root.egoFrenetState(1, 1) >= SuccessPointS
        temp_test.success = 1;
        disp("Round " + r + ": Success");
        break;
    end
end

% displayScenario(AllPath, actorWaypoints, profiles, allStatus, roadConfigs);
temp_test.Time = MCTS_time;
quantResult{r} = temp_test;

end

plotQuantitative(quantResult, max_iter);



function Tree_ = expand(Tree, node, TimeResolution, accMax, speedlimit, refPath, predictedActPositions, egoVehicle, profiles, lbdry, roadWidth, scenario)
% This is the situation for the car to do a sudden break.
Tree_ = Tree;
newNode5 = struct('state', node.state, 'time', node.time + TimeResolution, ...
    'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, ...
    'parent', node.index, 'parentaCC', node.state(1, end), 'parentV', node.state(1, end -1), 'parentX', node.state(1, 1), 'parentY', node.state(1, 2), ...
    'UCB', inf, 'egoFrenetState', node.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);

emergencyAcc = -(2 * node.egoFrenetState(2) / TimeResolution) - node.egoFrenetState(3);
% Let's set the maximum deceleration to be 8m/s^2
if emergencyAcc <= -8
    emergencyAcc = -8;
end
emergencyJerkS = (emergencyAcc - node.egoFrenetState(3)) / TimeResolution;
[displacementEmergencyS, deltaSpeedEmergencyS, displacementEmergencyL, deltaSpeedEmergencyL] = getDisplacement(node, emergencyJerkS, 0, TimeResolution);
if displacementEmergencyS < 0
    displacementEmergencyS = 0;
end
newNode5.egoFrenetState = node.egoFrenetState + [displacementEmergencyS, deltaSpeedEmergencyS, emergencyJerkS * TimeResolution, displacementEmergencyL, deltaSpeedEmergencyL, 0];
newNode5.egoFrenetState(3) = 0;
newNode5.state = frenet2global(refPath, newNode5.egoFrenetState);
if ~checkCollision(node, newNode5, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath) && node.egoFrenetState(2) > 0
    Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode5.index;
    Tree{numel(Tree) + 1} = newNode5;
end

% accMax is the upper bound for egoVehicle

Jerk3 = -node.egoFrenetState(3) / TimeResolution;
% Set the acceleration all to zero;

% The situation for egoVehicle to keep its constant speed
newNode3 = struct('state', node.state, 'time', node.time + TimeResolution, ...
    'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, ...
    'parent', node.index, 'parentaCC', node.state(1, end), 'parentV', node.state(1, end -1), 'parentX', node.state(1, 1), 'parentY', node.state(1, 2), ...
    'UCB', inf, 'egoFrenetState', node.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);

newNode3.laneChangingProperties.Change = false;

[displacementS3, deltaSpeedS3, displacementL3, deltaSpeedL3] = getDisplacement(node, Jerk3, 0, TimeResolution);


newNode3.egoFrenetState = newNode3.egoFrenetState + [displacementS3, deltaSpeedS3 , Jerk3 * TimeResolution, displacementL3, deltaSpeedL3, 0];

newNode3.state = frenet2global(refPath, newNode3.egoFrenetState);



if ~checkCollision(node, newNode3, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath) && newNode3.egoFrenetState(2) <= speedlimit && newNode3.egoFrenetState(2) >= 0
    Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode3.index;
    Tree{numel(Tree) + 1} = newNode3;
end

% Lane changing only happens when the egoVehicle has a constant speed

for i = 1:numel(lbdry)
    % check whether the lanes are dashed
    if (lbdry(i).BoundaryType == 2 || lbdry(i).BoundaryType == 4) && node.state(5) >= 1 && (node.egoFrenetState(1) >= refPath.SegmentParameters(end, end)  || node.egoFrenetState(1) <= refPath.SegmentParameters(2, end))
        if lbdry(i).LateralOffset > 0
            % change to the left-side lane

            newNode4 = struct('state', newNode3.state, 'time', node.time + TimeResolution, ...
                'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, ...
                'parent', node.index, 'parentaCC', node.state(1, end), 'parentV', node.state(1, end -1), 'parentX', node.state(1, 1), 'parentY', node.state(1, 2), ...
                'UCB', inf, 'egoFrenetState', newNode3.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);
            deltaL = lbdry(i).LateralOffset + 0.5 * roadWidth;
            % deltaL represents to the distance that the car moves
            % laterally
            newNode4.laneChangingProperties.LeftChange = newNode4.laneChangingProperties.LeftChange + 1;
            newNode4.laneChangingProperties.Change = true;
            newNode4.egoFrenetState = newNode4.egoFrenetState + [0 0 0 deltaL 0 0];
            newNode4.state = frenet2global(refPath, newNode4.egoFrenetState);
            % check whether there's a collision while changing the lane
            if ~checkCollision(node, newNode4, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath) && newNode4.egoFrenetState(2) <= speedlimit && newNode4.egoFrenetState(2) >= 0
                Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode4.index;
                Tree{numel(Tree) + 1} = newNode4;
            end

        elseif lbdry(i).LateralOffset < 0
            % change to the right-side lane
            newNode4 = struct('state', newNode3.state, 'time', node.time + TimeResolution, ...
                'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, ...
                'parent', node.index, 'parentaCC', node.state(1, end), 'parentV', node.state(1, end -1), 'parentX', node.state(1, 1), 'parentY', node.state(1, 2),...
                'UCB', inf, 'egoFrenetState', newNode3.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);
            deltaL = lbdry(i).LateralOffset - 0.5 * roadWidth;
            newNode4.laneChangingProperties.RightChange = newNode4.laneChangingProperties.RightChange + 1;
            newNode4.laneChangingProperties.Change = true;
            newNode4.egoFrenetState = newNode4.egoFrenetState + [0 0 0 deltaL 0 0];
            newNode4.state = frenet2global(refPath, newNode4.egoFrenetState);
            % check whether there's a collision while changing the lane
            if ~checkCollision(node, newNode4, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath) && newNode4.egoFrenetState(2) <= speedlimit && newNode4.egoFrenetState(2) >= 0
                Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode4.index;
                Tree{numel(Tree) + 1} = newNode4;
            end

        end
    end
end

% Then we expand the situations with acceleration
for nextAcc = 1:accMax

    % Slowing down, we ignore the situation of going backward, so we only
    % expand a node with negative acc if and only if the speed is bigger
    % than 1m/s.
    if node.egoFrenetState(2) > 1
        jerk1 = (-nextAcc - node.egoFrenetState(3)) / TimeResolution;
        newNode1 = struct('state', node.state, 'time', node.time + TimeResolution, ...
            'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, ...
            'parent', node.index, 'parentaCC', node.state(1, end), 'parentV', node.state(1, end -1), 'parentX', node.state(1, 1), 'parentY', node.state(1, 2),...
            'UCB', inf, 'egoFrenetState', node.egoFrenetState,'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);

        [displacementS1, deltaSpeedS1, displacementL1, deltaSpeedL1] = getDisplacement(node, jerk1, 0, TimeResolution);
        newNode1.laneChangingProperties.Change = false;
        newNode1.egoFrenetState = newNode1.egoFrenetState + [displacementS1, deltaSpeedS1, jerk1 * TimeResolution, displacementL1, deltaSpeedL1, 0];
        newNode1.state = frenet2global(refPath, newNode1.egoFrenetState);
        feasible = newNode1.egoFrenetState(2) + (nextAcc + 3) * TimeResolution * 0.5;
        if ~checkCollision(node, newNode1, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath) && newNode1.egoFrenetState(2) > 0 && feasible >= 0
            Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode1.index;
            Tree{numel(Tree) + 1} = newNode1;
        end
    end

    % acceleration section
    jerk2 = (nextAcc - node.egoFrenetState(3)) / TimeResolution;
    newNode2 = struct('state', node.state, 'time', node.time + TimeResolution, ...
        'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, ...
        'parent', node.index, 'parentaCC', node.state(1, end), 'parentV', node.state(1, end -1), 'parentX', node.state(1, 1), 'parentY', node.state(1, 2),...
        'UCB', inf, 'egoFrenetState', node.egoFrenetState,'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);

    [displacementS2, deltaSpeedS2, displacementL2, deltaSpeedL2] = getDisplacement(node, jerk2, 0, TimeResolution);

    newNode2.laneChangingProperties.Change = false;

    newNode2.egoFrenetState = newNode2.egoFrenetState + [displacementS2, deltaSpeedS2, jerk2 * TimeResolution, displacementL2, deltaSpeedL2, 0];
    newNode2.state = frenet2global(refPath, newNode2.egoFrenetState);
    if ~checkCollision(node, newNode2, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath) && newNode2.egoFrenetState(3) <= 3 && newNode2.egoFrenetState(2) < speedlimit && newNode2.egoFrenetState(2) > 0 && displacementS2 >= 0
        Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode2.index;
        Tree{numel(Tree) + 1} = newNode2;
    end


end
if numel(Tree) == numel(Tree_)
    Tree = back_propagation(Tree{node.index}, 0, Tree);
    Tree_ = Tree;
else
    Tree_ = Tree;

end

end

function newNode = selection(node, Tree)
% choose the best node with the biggest UCB score
newNode = Tree{node.index};
while numel(newNode.children) ~= 1

    bestChild = Tree{newNode.children(2)};
    for i=2:length(newNode.children)
        if Tree{newNode.children(i)}.UCB >= bestChild.UCB
            bestChild = Tree{newNode.children(i)};
        end
    end
    newNode = bestChild;
end
end

function cost = roll_out(node, MaxTimeHorizon, TimeResolution, predicted, accMax, speedlimit, refPath, checkPoint, egoVehicle, profiles, scenario)
% This function process the simulation.
cost = 0;  % initial cost.
currNode = node;
% termninal state is when currTime > MaxTimeHorizon or has resulted in a
% collision

while currNode.time < MaxTimeHorizon
    randomNum = rand();
    randomAcc = randi([1, accMax]);

    if randomNum <= 0.1
        % This situation is for slowing down, so randomAcc is a negative
        % number.

        % Compute Jerk first
        deltaAcc = -randomAcc - currNode.egoFrenetState(3);
        Jerk = deltaAcc / TimeResolution;

        % As a sudden break may happen, so the limit jerk for slowing down
        % is allowed.

        % [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(node, deltaAccelerationS, deltaAccelerationL, TimeResolution)
        [displacementS1, deltaSpeedS1, displacementL1, deltaSpeedL1] = getDisplacement (currNode, Jerk, 0, TimeResolution);


        % parentaCC stands for the acceleration of the parent node
        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', 0, 'visits', 0, 'score', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState, 'laneChangingProperties', currNode.laneChangingProperties);
        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS1, deltaSpeedS1 , Jerk * TimeResolution, displacementL1, deltaSpeedL1, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        if checkCollision(currNode, newNode, predicted, egoVehicle, profiles, TimeResolution, scenario, refPath)
            cost = -10000;
            break;
        end
        currNode = newNode;
    elseif randomNum >= 0.9
        % This situation is for speeding up

        % If Acc is out of limit, then let restrain the acc to be a proper
        % number under the limit.
        if randomAcc >= 3
            % So currAcc = currNode.egoFrenetState(3)) - limitJerk * TimeResolution;
            randomAcc = 3;
        end

        % compute jerk.
        deltaAcc = randomAcc - currNode.egoFrenetState(3);
        Jerk = deltaAcc / TimeResolution;

        [displacementS2, deltaSpeedS2, displacementL2, deltaSpeedL2] = getDisplacement(currNode, Jerk, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', 0, 'visits', 0, 'score', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState, 'laneChangingProperties', currNode.laneChangingProperties);
        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS2, deltaSpeedS2 , Jerk * TimeResolution, displacementL2, deltaSpeedL2, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        if checkCollision(currNode, newNode, predicted, egoVehicle, profiles, TimeResolution, scenario, refPath)

            cost = -10000;
            break;
        end
        currNode = newNode;
    else
        % This situation is for keeping speed

        deltaAcc = -currNode.egoFrenetState(3);
        Jerk = deltaAcc / TimeResolution;

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', 0, 'visits', 0, 'score', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState, 'laneChangingProperties', currNode.laneChangingProperties);

        [displacementS3, deltaSpeedS3, displacementL3, deltaSpeedL3] = getDisplacement(currNode, Jerk, 0, TimeResolution);


        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS3, deltaSpeedS3, Jerk * TimeResolution, displacementL3, deltaSpeedL3, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        if checkCollision(currNode, newNode, predicted, egoVehicle, profiles, TimeResolution, scenario, refPath)
            cost = -10000;
            break;
        end
        currNode = newNode;
    end
end

% special case for one time simulation for the terminal node.
if currNode.time >= MaxTimeHorizon
    randomNum = rand();
    randomAcc = randi([1, accMax]);
    if randomNum <= 0.1
        % This situation is for slowing down, so randomAcc is a negative
        % number.

        % Compute Jerk first
        deltaAcc = -randomAcc - currNode.egoFrenetState(3);
        Jerk = deltaAcc / TimeResolution;
        % As a sudden break may happen, so the limit jerk for slowing down
        % is allowed.

        % [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(node, deltaAccelerationS, deltaAccelerationL, TimeResolution)
        [displacementS1, deltaSpeedS1, displacementL1, deltaSpeedL1] = getDisplacement (currNode, Jerk, 0, TimeResolution);


        % parentaCC stands for the acceleration of the parent node
        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', 0, 'visits', 0, 'score', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState, 'laneChangingProperties', currNode.laneChangingProperties);
        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS1, deltaSpeedS1 , Jerk * TimeResolution, displacementL1, deltaSpeedL1, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        if checkCollision(currNode, newNode, predicted, egoVehicle, profiles, TimeResolution, scenario, refPath)
            cost = -10000;
        end
        currNode = newNode;
    elseif randomNum >= 0.9
        % This situation is for speeding up

        % If Acc is out of limit, then let restrain the acc to be a proper
        % number under the limit.
        if randomAcc >= 3
            % So currAcc = currNode.egoFrenetState(3)) - limitJerk * TimeResolution;
            randomAcc = 3;
        end

        % compute jerk.
        deltaAcc = randomAcc - currNode.egoFrenetState(3);
        Jerk = deltaAcc / TimeResolution;
        [displacementS2, deltaSpeedS2, displacementL2, deltaSpeedL2] = getDisplacement(currNode, Jerk, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', 0, 'visits', 0, 'score', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState, 'laneChangingProperties', currNode.laneChangingProperties);
        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS2, deltaSpeedS2 , Jerk * TimeResolution, displacementL2, deltaSpeedL2, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        if checkCollision(currNode, newNode, predicted, egoVehicle, profiles, TimeResolution, scenario, refPath)

            cost = -10000;
        end
        currNode = newNode;
    else
        % This situation is for keeping speed

        deltaAcc = -currNode.egoFrenetState(3);
        Jerk = deltaAcc / TimeResolution;

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', 0, 'visits', 0, 'score', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState, 'laneChangingProperties', currNode.laneChangingProperties);

        [displacementS3, deltaSpeedS3, displacementL3, deltaSpeedL3] = getDisplacement(currNode, Jerk, 0, TimeResolution);


        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS3, deltaSpeedS3, Jerk * TimeResolution, displacementL3, deltaSpeedL3, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        if checkCollision(currNode, newNode, predicted, egoVehicle, profiles, TimeResolution, scenario, refPath)
            cost = -10000;
        end
        currNode = newNode;
    end
end

aggregate_cost = costFunction(node, newNode, checkPoint, predicted, MaxTimeHorizon, TimeResolution, egoVehicle, speedlimit, profiles, scenario);
% disp("final_efficiency_cost:" + cost(2));
% disp("final_reasonable_cost:" + cost(3));
% disp("final_similarity_cost:" + cost(4));
% disp("finial_lane_change_cost:" + cost(5));
% disp("final_safety_cost:" + cost(6));
% disp("cost: " + cost(1));
cost = cost + aggregate_cost(1);
end

function tree_ = back_propagation(node, score, tree)
% update every node's UCB in the MCTS tree
while node.parent ~= 0
    tree{node.index}.score = node.score + score;
    tree{node.index}.visits = node.visits + 1;
    node = tree{node.parent};
end
tree{node.index}.score = node.score + score;
tree{node.index}.visits = node.visits + 1;
tree_ = tree;
end

function tree_ = updateUCB(node, tree)
% this function updates the ucb of all nodes in the tree, using a bfs
% starting from the root
queue = {node};
while ~isempty(queue)
    currNode = queue{1};
    queue(1) = [];
    if currNode.visits == 0
        tree{currNode.index}.UCB = inf;
        tree{currNode.index}.avgScore = inf;
    else
        tree{currNode.index}.UCB = currNode.score / currNode.visits + 5 * sqrt(log(tree{1}.visits) / currNode.visits);
        tree{currNode.index}.avgScore = tree{currNode.index}.score / tree{currNode.index}.visits;
    end
    if numel(currNode.children) ~= 1
        for i = 1:(numel(currNode.children) - 1)
            queue{numel(queue) + 1} = tree{currNode.children(i + 1)};
        end
    end

end
tree_ = tree;

end


function flag = checkCollision(node, nextNode, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath)
% using AABB method to check the collision of the vehicles
flag = false;
% disp(node.state);
% disp(nextNode.state);
egoVehicleTraj = packUpEgoVehicleTrajactory(node, nextNode, TimeResolution, refPath);
currTime = scenario.SimulationTime + node.time;
index = int32(currTime / 0.2) + 1;
for i = 1:numel(predictedActPositions)
    % compute x, y distance between egoVehicle and actorCars

    % Get the config of the actorCar
    objCarDim = [profiles(i + 1).Length, profiles(i + 1).Width];

    egoCarDim = [egoVehicle.Length, egoVehicle.Width];
    for j = 2:numel(egoVehicleTraj(:, 1))
        if index + j - 1 <= numel(predictedActPositions{i}(:, 1))
            xdistance = abs(egoVehicleTraj(j, 1) - predictedActPositions{i}(index + j - 1, 1)) - 0.5 * (objCarDim(1) * abs(cosd(predictedActPositions{i}(index + j - 1, 3))) + objCarDim(2) * abs(sind(predictedActPositions{i}(index + j - 1, 3)))) - 0.5 * (egoCarDim(1) * abs(cos(egoVehicleTraj(j, 3))) + egoCarDim(2) * abs(sin(egoVehicleTraj(j, 3))));
            ydistance = abs(egoVehicleTraj(j, 2) - predictedActPositions{i}(index + j - 1, 2)) - 0.5 * (objCarDim(2) * abs(cosd(predictedActPositions{i}(index + j - 1, 3))) + objCarDim(1) * abs(sind(predictedActPositions{i}(index + j - 1, 3)))) - 0.5 * (egoCarDim(2) * abs(cos(egoVehicleTraj(j, 3))) + egoCarDim(1) * abs(sin(egoVehicleTraj(j, 3))));
        else
            xdistance = abs(egoVehicleTraj(j, 1) - predictedActPositions{i}(end, 1)) - 0.5 * (objCarDim(1) * abs(cosd(predictedActPositions{i}(end, 3))) + objCarDim(2) * abs(sind(predictedActPositions{i}(end, 3)))) - 0.5 * (egoCarDim(1) *  abs(cos(egoVehicleTraj(j, 3))) + egoCarDim(2) * abs(sin(egoVehicleTraj(j, 3))));
            ydistance = abs(egoVehicleTraj(j, 2) - predictedActPositions{i}(end, 2)) - 0.5 * (objCarDim(2) * abs(cosd(predictedActPositions{i}(end, 3))) + objCarDim(1) * abs(sind(predictedActPositions{i}(end, 3)))) - 0.5 * (egoCarDim(2) *  abs(cos(egoVehicleTraj(j, 3))) + egoCarDim(1) * abs(sin(egoVehicleTraj(j, 3))));
        end
        if xdistance <= 0 && ydistance <= 0
            flag = true;
            break
        end
    end
end
end

function cost = costFunction(node, nextNode, checkPoint, predictedActPositions, MaxTimeHorizon, TimeResolution, egoVehicle, speedlimit, profiles, scenario)

if abs(node.egoFrenetState(4)) >= 0.5
    is_lane_change = 1;
else 
    is_lane_change = 0;
end
% smoothness
% jerk_smoothness_cost = calculate_lon_jerk_smoothness(node, nextNode, TimeResolution);
% theta_smoothness_cost = calculate_lat_theta_smoothness(node, nextNode, TimeResolution);
% kappa_smoothness_cost = calculate_lat_kappa_smoothness(node, nextNode, TimeResolution);

% efficiency
speed_efficiency_cost = calculate_speed_efficiency(node, nextNode, speedlimit);
travel_efficiency_cost = calculate_travel_efficiency(node, nextNode, checkPoint);
% reliability
deviation_cost = calculate_deviation_cost;

% lane-changing
lane_cost = calculate_lane_cost(node, nextNode, is_lane_change);
lane_change_cost = is_lane_change;

% similarity
lon_similarity_cost = calculate_longitudinal_similarity(node, nextNode);
% lateral_similarity = calculate_lateral_similarity(planning_result, last_planning_result, index_traj);

% safety
safety_cost = calculateSafetyCost_(nextNode, predictedActPositions, egoVehicle, profiles, scenario);

% w_smoothness = 50.0;
w_efficiency = 115.0;
w_reasonable = 70.0;
w_similarity = 50;
w_safety = 100.0;
w_lane_change = 136.0;
final_efficiency_cost = w_efficiency * (travel_efficiency_cost + speed_efficiency_cost);
final_reasonable_cost = w_reasonable * (deviation_cost + lane_cost);
final_similarity_cost = w_similarity * (lon_similarity_cost);
% final_lon_smoothness_cost = w_smoothness * jerk_smoothness_cost;
% final_lat_smoothness_cost = w_smoothness * (theta_smoothness_cost + kappa_smoothness_cost);
finial_lane_change_cost = w_lane_change * lane_change_cost;
if node.state(1, end - 1) <= 3
    % when the penalty is > 2000, the egoVehicle would try to change lane
    % or it would just stop
    final_efficiency_cost = final_efficiency_cost + 2000;
end
final_safety_cost = w_safety * safety_cost; 
cost = [-(final_efficiency_cost + final_reasonable_cost + final_similarity_cost + finial_lane_change_cost + final_safety_cost), -final_efficiency_cost, -final_reasonable_cost, -final_similarity_cost, -finial_lane_change_cost, -final_safety_cost];
end

function jerk_smoothness_cost = calculate_lon_jerk_smoothness(node, nextNode, TimeResolution)
    jerk_smoothness = (nextNode.egoFrenetState(1, 3) - node.egoFrenetState(1,3)) / TimeResolution;
    k_jerk_smoothness = 0.01;
    jerk_smoothness_deviation = 450.0;
    jerk_smoothness_cost = 1.0 / (1.0 + k_jerk_smoothness * exp(jerk_smoothness_deviation - jerk_smoothness));
end

function theta_smoothness_cost = calculate_lat_theta_smoothness(node, nextNode, TimeResolution)
    theta_smoothness = (nextNode.state(1, 3) - node.state(1,3)) / TimeResolution;
    k_theta_smoothness = 0.01;
    theta_smoothness_deviation = 450.0;
    theta_smoothness_cost = 1.0 / (1.0 + k_theta_smoothness * exp(theta_smoothness_deviation - theta_smoothness));
end

function kappa_smoothness_cost = calculate_lat_kappa_smoothness(node, nextNode, TimeResolution)
    kappa_smoothness = (nextNode.state(1, 3) - node.state(1,3)) / TimeResolution;
    k_kappa_smoothness = 0.01;
    kappa_smoothness_deviation = 450.0;
    kappa_smoothness_cost = 1.0 / (1.0 + k_kappa_smoothness * exp(kappa_smoothness_deviation - kappa_smoothness));
end

function speed_efficiency_cost = calculate_speed_efficiency(node, nextNode, speedlimit)
% k_speed = 2.0;
distance_s = nextNode.egoFrenetState(1) - node.egoFrenetState(1);
total_time = nextNode.time - node.time;
average_speed = distance_s / total_time;
speed_target = speedlimit;
% other way for calculation
% speed_efficiency_cost = 1.0 / (1.0 + exp(k_speed * (average_speed - speed_target)));
k_quad = 1 / speedlimit;
speed_efficiency_cost = k_quad * abs(-(average_speed - speed_target));
end

function travel_efficiency_cost = calculate_travel_efficiency(node, nextNode, checkPoint)
% k_distance = 0.15;
distance_s = nextNode.egoFrenetState(1) - node.egoFrenetState(1);
target_distance = checkPoint - node.egoFrenetState(1);
% other way for calculation
% travel_efficiency_cost = 1.0 / (1.0 + exp(k_distance * (distance_s - target_distance)));
k_quad = 1 / target_distance;
travel_efficiency_cost = max(k_quad * -(distance_s - target_distance), 0);

end

function deviation_cost = calculate_deviation_cost
deviation_cost = 0;
end

function lane_cost = calculate_lane_cost(node, nextNode, is_lane_change)
% epsilon = 0.01;
% max_deviation = 0.0;
lane_cost = 0.0;
if is_lane_change
    lane_cost = 1.0;
    return
end
end

function lon_similarity_cost = calculate_longitudinal_similarity(node, nextNode)
lon_similarity_diff = 0.0;
w_v = 2.0;
w_a = 5.0;
v_diff = max(node.parentV - node.state(1, end - 1), 0);
a_diff = max(node.parentaCC - node.state(1, end), 0);
lon_similarity_diff = lon_similarity_diff + w_v * abs(v_diff) + w_a * abs(a_diff);

k_lon_similarity = 0.01;
lon_similarity_deviation = 400;
lon_similarity_cost = 1.0 / (1.0 + exp(k_lon_similarity * (lon_similarity_deviation - lon_similarity_diff)));

end

function safety_cost = calculateSafetyCost_(nextNode, predictedActPositions, egoVehicle, profiles, scenario)
% min_safety_dist = 100.0;
% current_dist = 100.0;
currTime = scenario.SimulationTime + nextNode.time;
index = int32(currTime / 0.2) + 1;
k_safety = 1.0;
safety_deviation = 1.5;
safety_cost = 0.0;

for i = 1:numel(predictedActPositions)

    if index <= numel(predictedActPositions{i}(:, 1))
        predicted = predictedActPositions{i}(index, :);
    else
        predicted = predictedActPositions{i}(end, :);
    end

    % Get the config of the actorCar
    objCarDim = [profiles(i).Length, profiles(i).Width];

    egoCarDim = [egoVehicle.Length, egoVehicle.Width];

    xdistance = abs(nextNode.state(1) - predicted(1)) - 0.5 * (abs(objCarDim(1) * cosd(predicted(3))) + abs(objCarDim(2) * sind(predicted(3)))) - 0.5 * (abs(egoCarDim(1) * cosd(nextNode.state(3))) + abs(egoCarDim(2) * sin(nextNode.state(3))));
    ydistance = abs(nextNode.state(2) - predicted(2)) - 0.5 * (abs(objCarDim(2) * cosd(predicted(3))) + abs(objCarDim(1) * sind(predicted(3)))) - 0.5 * (abs(egoCarDim(2) * sind(nextNode.state(3))) + abs(egoCarDim(1) * sin(nextNode.state(3))));

    if ydistance <= 0
        current_dist = xdistance;
        safety_cost = safety_cost + 1.0 / (1.0 + exp(k_safety * (current_dist - safety_deviation)));
        % if current_dist < min_safety_dist
        %     min_safety_dist = current_dist;
        % end
        
    end
end
% safety_cost =1.0 / (1.0 + exp(k_safety * (min_safety_dist - safety_deviation)));

end
