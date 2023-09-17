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


% This is for Intersection, Go Straight example2.
[scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = IntersectionGoStraightexample2();

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
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = HighwayExitHEexample();

% This is for Large Curvature example
% [scenario, egoVehicle, egoWaypoints, actorWaypoints, allStatus, roadConfigs] = LargeCurvatureexample();

% Above are all the scenarios of the program.


% This is for giving the egoCar's initial position.
% setStartEgoState(egoWaypoints, velocity, acceleration)
startEgoState = setStartEgoState(egoWaypoints, 5, 0);

% Create a reference path using waypoints
egorefPath = referencePathFrenet(egoWaypoints);
% refPath = [x y theta kappa dkappa s]
connector = trajectoryGeneratorFrenet(egorefPath,'TimeResolution',1.0);
pathPoints = closestPoint(egorefPath, egorefPath.Waypoints(:,1:2)); % [x y theta kappa dkappa s]
roadS = pathPoints(:,end);

% Set destination position.
DestinationS = egorefPath.PathLength;
% DestinationS = egorefPath.SegmentParameters(end, end);

% Moving egoVehicle into the initial state of the scenario.
helperMoveEgoVehicleToState(egoVehicle, startEgoState);

egoFrenetState = global2frenet(egorefPath, startEgoState);

% Initialize basic configs
TIME = 0;
max_iter = 3000;
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
                break;
            end
        end

    end
    % TotalTime(numel(TotalTime) + 1) = toc;
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
end

displayScenario(AllPath, actorWaypoints, profiles, allStatus, roadConfigs);



function Tree_ = expand(Tree, node, TimeResolution, accMax, speedlimit, refPath, predictedActPositions, egoVehicle, profiles, lbdry, roadWidth, scenario)
% This is the situation for the car to do a sudden break.
Tree_ = Tree;
newNode5 = struct('state', node.state, 'time', node.time + TimeResolution, ...
    'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'parentaCC', node.state(1, end), 'UCB', inf, 'egoFrenetState', node.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);

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
    'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'parentaCC', node.state(1, end), 'UCB', inf, 'egoFrenetState', node.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);

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
                'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'parentaCC', node.state(1, end),'UCB', inf, 'egoFrenetState', newNode3.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);
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
                'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'parentaCC', node.state(1, end), 'UCB', inf, 'egoFrenetState', newNode3.egoFrenetState, 'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);
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
            'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'parentaCC', node.state(1, end), 'UCB', inf, 'egoFrenetState', node.egoFrenetState,'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);

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
        'children', 0, 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'parentaCC', node.state(1, end), 'UCB', inf, 'egoFrenetState', node.egoFrenetState,'avgScore', 0, 'laneChangingProperties', node.laneChangingProperties);

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

    if randomNum <= 0.2
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
    elseif randomNum >= 0.8
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

cost2 = costFunction(node, newNode, checkPoint, predicted, MaxTimeHorizon, TimeResolution, egoVehicle, speedlimit, profiles, scenario);
cost = cost + cost2(1);
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
AccLevel = 0.8 * nextNode.egoFrenetState(3) + 0.2 * node.egoFrenetState(3);
comfort = (nextNode.egoFrenetState(3) + node.egoFrenetState(3)) / 2;

% jerk calculation
jerk = (nextNode.egoFrenetState(3) - node.egoFrenetState(3)) / (nextNode.time - node.time);

cost_comfort = calculateComfortCost(jerk, comfort, node);
cost_pass = calculatePassibilityCost(nextNode, checkPoint, MaxTimeHorizon);
cost_safety = calculateSafetyCost(nextNode, predictedActPositions, egoVehicle, profiles, scenario);
cost_lane_changing = calculateLaneChangingCost(node);
cost_is_break_to_stop = calculateBreakToStop(nextNode, MaxTimeHorizon);

% stimulate the car to moveforward.
if nextNode.egoFrenetState(2) < speedlimit
    % the standard level for cost_stimulation is 0.0, e.g.
    % speed ==  speedlimit, acc == 0
    cost_stimulation = 10 * (speedlimit - nextNode.egoFrenetState(2)) ^ 2 + 5 * (3 - AccLevel);

elseif nextNode.egoFrenetState(2) > speedlimit
    expectAcc = (nextNode.egoFrenetState(2) - speedlimit) / TimeResolution;
    cost_stimulation = abs(nextNode.egoFrenetState(3) - expectAcc);

else
    cost_stimulation = 0.0;
end

% calculate cost

cost = [-(cost_comfort + cost_safety + cost_pass + cost_stimulation + cost_lane_changing + cost_is_break_to_stop), -cost_comfort, -cost_safety, -cost_pass, -cost_stimulation, - cost_lane_changing, - cost_is_break_to_stop];

end


function cost_comfort = calculateComfortCost(jerk, comfort, node)

cost_comfort_jerk = 2 / (1 + exp(-jerk));
cost_comfort_acc = 0;
cost_comfort_alter = 0;
acc = node.egoFrenetState(3);

if comfort < -2
    cost_comfort_acc = - 20 * comfort;
end

if acc * node.parentaCC < 0
    cost_comfort_alter = 25.0;
end

cost_comfort = cost_comfort_acc + cost_comfort_jerk + cost_comfort_alter;

% The comfort reward function is a sigmoid-like function.


end


function cost_pass = calculatePassibilityCost(node, checkPoint, MaxTimeHorizon)

% The reward should be higher if the vehicle is able to pass the DestinationS more smoothly
% For those node's time < MaxTimeHorizon, just let the reward be 0.
if node.time >= MaxTimeHorizon

    if node.egoFrenetState(1) > checkPoint
        cost_pass = 0.0;
    else
        cost_pass = 2.0 + abs(node.egoFrenetState(1) - checkPoint);
    end
else
    cost_pass = 0.0;
end
end


function cost_safety = calculateSafetyCost(nextNode, predictedActPositions, egoVehicle, profiles, scenario)

SAFE_DISTANCE = 5;
Emergency_Distance = 1;
cost_safety = 0;
speed = nextNode.egoFrenetState(2);
acc = nextNode.egoFrenetState(3);
currTime = scenario.SimulationTime + nextNode.time;
index = int32(currTime / 0.2) + 1;

% Design a Piecewise function so that it returns a logorithmn decrease
% when the distance is smaller than SAFE_DISTANCE

for i = 1:numel(predictedActPositions)

    if index <= numel(predictedActPositions{i}(1, :))
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

        if xdistance <= Emergency_Distance && xdistance >= 0
            % The growth of the cost function on this situation follows a linear trends
            % with a slope of -10.
            cost_safety_temp = -10 * (xdistance - Emergency_Distance) + 3.0 + 20 * speed ^2 + 10 * acc^3;

        elseif xdistance > Emergency_Distance && xdistance <= SAFE_DISTANCE

            cost_safety_temp = 1.0 * (xdistance - SAFE_DISTANCE) / (Emergency_Distance - SAFE_DISTANCE)  + 2.0 + 3 * speed ^2 + acc^3;


        else
            % Normally give a no more than 2.0 reward for the action that
            % keeps distance.
            cost_safety_temp = max(-1/100 * (xdistance - SAFE_DISTANCE) + 2.0, 0.0);
        end
    else
        cost_safety_temp = 0;
    end

    cost_safety = cost_safety + cost_safety_temp;
end

end

function cost_laneChanging = calculateLaneChangingCost(node)
% This function calculate the cost of the egoVehicle's action of chaning
% lanes
if abs(node.egoFrenetState(4)) >= 0.5
    cost_laneChanging = 150.0;
else
    cost_laneChanging = 0.0;
end
end

function cost_is_break_to_stop = calculateBreakToStop(node, MaxTimeHorizon)
% This function gives a panalty cost if the egoVehicle's speed is not
% moving
if node.time >= MaxTimeHorizon && node.state(5) < 2
    cost_is_break_to_stop = 200.0;
else
    cost_is_break_to_stop = 0.0;
end
end
