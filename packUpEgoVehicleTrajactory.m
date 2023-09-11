function egoVehicleTraj = packUpEgoVehicleTrajactory(node, nextNode, TimeResolution, refPath)
waypoints = [node.state(1:3); nextNode.state(1:3)];
jerk = (nextNode.state(end) - node.state(end)) / TimeResolution;
if any(waypoints(1,:) ~= waypoints(2,:))
    newRefPath = referencePathFrenet(waypoints);
    currNode = node;
    egoVehicleTraj = currNode.state;
    for i = 2: int32(TimeResolution / 0.2) + 1
        currFrenetState = global2frenet(newRefPath, currNode.state);
        currNode.egoFrenetState = currFrenetState;
        [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(currNode, jerk, 0, 0.2);
        nextglobState = newRefPath.interpolate(currFrenetState(1) + displacementS);
        nextFrenetState = currFrenetState + [displacementS, deltaSpeedS , jerk * 0.2, double(displacementL / int32(TimeResolution / 0.2)), deltaSpeedL, 0];
        nextAcc = sqrt(nextFrenetState(3)^2 + nextFrenetState(6)^2);
        nextSpeed = sqrt(nextFrenetState(2)^2 + nextFrenetState(5)^2);
        nextglobState(5) = nextSpeed;
        nextglobState(6) = nextAcc;
        nextFrenetState = global2frenet(refPath, nextglobState);
        egoVehicleTraj = [egoVehicleTraj; nextglobState];
        currNode.state = nextglobState;
        currNode.egoFrenetState = nextFrenetState;
        % currFrenetState = global2frenet(refPath, currNode.state);
        % [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(currNode, jerk, 0, 0.2);
        % nextglobState = refPath.interpolate(currFrenetState(1) + displacementS);
        % nextFrenetState = currFrenetState + [displacementS, deltaSpeedS , jerk * 0.2, double(displacementL / int32(TimeResolution / 0.2)), deltaSpeedL, 0];
        % nextAcc = sqrt(nextFrenetState(3)^2 + nextFrenetState(6)^2);
        % nextSpeed = sqrt(nextFrenetState(2)^2 + nextFrenetState(5)^2);
        % nextglobState(5) = nextSpeed;
        % nextglobState(6) = nextAcc;
        % nextFrenetState = global2frenet(refPath, nextglobState);
        % egoVehicleTraj = [egoVehicleTraj; nextglobState];
        % currNode.state = nextglobState;
        % currNode.egoFrenetState = nextFrenetState;
    end
else
    egoVehicleTraj  = repmat(node.state, int32(TimeResolution / 0.2));
end

end
