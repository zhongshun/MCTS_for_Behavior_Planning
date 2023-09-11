function startEgoState = setStartEgoState(egoWaypoints, initialVelocity, initialAcc)
% set the starter state for the egoVehicle.
startEgoState = [egoWaypoints(1, 1:3) 0 initialVelocity initialAcc];
end

