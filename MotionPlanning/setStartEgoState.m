function startEgoState = setStartEgoState(egoWaypoints, initialVelocity, initialAcc, yaw)
% set the starter state for the egoVehicle.
startEgoState = [egoWaypoints(1, 1:2) deg2rad(yaw) 0 initialVelocity initialAcc];
end

