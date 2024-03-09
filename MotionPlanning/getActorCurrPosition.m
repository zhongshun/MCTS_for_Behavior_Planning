function currentActPosition = getActorCurrPosition(trajectory, sampleTime)
% This function is to get the current position the certain actorVehicle.
[position, orientation,~,~,~] = lookupPose(trajectory, sampleTime);
eulerAnglesDegrees = eulerd(orientation,'ZYX','frame');
currentActPosition = [position(1:2) eulerAnglesDegrees(1)];
% set up sigma
x_sigma = 0.05;
y_sigma = 0.05;

% generate noise
x_noise = x_sigma * randn(1);
y_noise = y_sigma * randn(1);

% add noise on current position
currentActPosition = currentActPosition + [x_noise y_noise 0];
end

