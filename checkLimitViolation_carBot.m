function outOfLimits = checkLimitViolation_carBot(data, pose)
% Returns true if robot violates limits, false otherwise
% INPUT:
% data - Map and robot data
% pose - pose of robot [x,y,theta] => center of robot & angle
% OUTPUT: 
% inLimits - True if robot violates limits, false otherwise
    outOfLimits = false;
    if ((pose(1) <= data.robotradius) ||...
        (pose(1) >= data.mapsize(2) - data.robotradius) || ...
        (pose(2) <= data.robotradius) || ...
        (pose(2) >= data.mapsize(1) - data.robotradius))
            outOfLimits = true;
    end
end