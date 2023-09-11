function currentActPosition = getActorCurrPosition(trajectory, sampleTime)
% This function is to get the current position the certain actorVehicle.
[position, orientation,~,~,~] = lookupPose(trajectory, sampleTime);
eulerAnglesDegrees = eulerd(orientation,'ZYX','frame');
currentActPosition = [position(1:2) eulerAnglesDegrees(1)];
end

