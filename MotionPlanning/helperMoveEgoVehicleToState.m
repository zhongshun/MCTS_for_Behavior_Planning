function helperMoveEgoVehicleToState(egoVehicle, currentEgoState)
% Move ego vehicle in scenario to a state calculated by the planner
%
% egoVehicle - driving.scenario.Actor in the scenario
% currentEgoState - [x y theta kappa speed acc]

% Set 2-D Position
egoVehicle.Position(1:2) = currentEgoState(1:2);

% Set 2-D Velocity (s*cos(yaw) s*sin(yaw))
egoVehicle.Velocity(1:2) = [cos(currentEgoState(3)) sin(currentEgoState(3))]*currentEgoState(5);

% Set Yaw in degrees
egoVehicle.Yaw = currentEgoState(3)*180/pi;
% egoVehicle.Yaw = currentEgoState(3);

% Set angular velocity in Z (yaw rate) as v/r
egoVehicle.AngularVelocity(3) = currentEgoState(4)*currentEgoState(5);

end
