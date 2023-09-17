function [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(node, jerkS, deltaL, TimeResolution)
% deltaSpeed = (2 * node.acc + deltaAcc) * 1/2 * TimeResolution, so we
% consider to use integral.
% (currSpeed + deltaSpeed) * TimeResolution = distance
% distance = currSpeed * TimeResolution + 1/2 * node.acc *
% TimeResolution^2 + 1/6 * deltaAcc) * TimeResolution ^3

deltaT = TimeResolution;
deltaSpeedS = node.egoFrenetState(3) * TimeResolution + 1/2 * jerkS * TimeResolution^2;
displacementS = node.egoFrenetState(2) * deltaT + 1/2 * node.egoFrenetState(3) * TimeResolution^2 + 1/6 * jerkS * TimeResolution^3;
deltaSpeedL = 0;
displacementL = deltaL;
end

