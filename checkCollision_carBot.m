function isColliding = checkCollision_carBot(data, pose)
% Returns true if robot is colliding with obstacles, false otherwise
% Assumes that the state is within feasible limits
% INPUT:
% data - Map and robot data
% pose - pose of robot [x,y,theta] => center of robot & angle
% OUTPUT: 
% isColliding - True if robot is in collision, false otherwise
    if data.inflatedmap(round(pose(2)), round(pose(1)))
        isColliding = true;
    else
        isColliding = false;
    end
end