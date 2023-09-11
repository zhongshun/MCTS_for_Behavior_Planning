function predictedActTrajectory = packUpActorVehicleTrajactory(actorWaypoints, allStatus)
% This function packups trajectories for all the actorVehicles.
predictedActTrajectory = cell(0);
for i=1:numel(actorWaypoints)
    if numel(actorWaypoints{i}(:, 1)) == 1
        predictedActTrajectory{i} = actorWaypoints{i};
    else
        GroundSpeed = allStatus{i}.speed;
        WaitTime = allStatus{i}.waittime;
        if WaitTime(end) ~= 0
            WaitTime(end) = 0;
        end
        actPose = waypointTrajectory(actorWaypoints{i}, GroundSpeed=GroundSpeed,WaitTime=WaitTime, AutoBank=false, AutoPitch=true);
        predictedActTrajectory{i} = actPose;
    end
end