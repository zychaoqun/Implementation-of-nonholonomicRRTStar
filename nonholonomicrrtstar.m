%function[fpath] = nonholonomicrrt(datafile, start, goal, seed, ...
%                                  deltastep, numcontrolsamples,...
%                                  maxlinearvel, maxsteerangle)
%('nonholrrtdata.mat', [100,40,pi], [150,350,0], 100, 5)
% Run a RRT planning test on the given map for a car-like robot which is
% represented as a circle to make collision checking efficient.
% Car control is [linearvel, angularvel]
% INPUTS:
% datafile          - Obstacle and robot data (obstacle map & inflated map, robot radius)
% robotStart        - Start position of the robot - center of car & angle ([x,y,theta] - car robot)
% targetStart       - Goal position of the robot - center of car & angle ([x,y,theta] - car robot)
% seed              - Deterministic seed
% deltaStep         - (Optional) Approx. number of cells by which to extend the graph in
%                     direction of the nearest-neigbour on the graph 
%                     (Default: 5)            
% numcontrolsamples - (Optional) Number of control samples for finding a feasible path
%                     that grows the tree towards the sampled state from its NN
%                     (Default: 25)
% maxlinearvel      - (Optional) Maximum linear velocity for the car (control limit)
%                     (Default: 20)
% maxsteerangle     - (Optional) Maximum steering angle for the car (control limit)
%                     (Default: 0.6)
% OUTPUT:
% fpath             - Final collision free planned path (N x 3, where N = path length & each row is of the form [x,y,theta])

clc
clear all

datafile='nonholrrtdata.mat';
deltastep=5;
start=[100,40,pi];
goal=[150,350,0];
seed=100;
rng(seed); % Seed the random number generator

% Get the data
%%%% DO NOT MODIFY THIS VARIABLE
data = load(datafile);

% Display map, start and goal positions
fg = figure(1); hold on;
image(255 - data.envmap*255); colormap gray;
plotCarBot(data, start, 'r');
plotCarBot(data, goal, 'g');
xlim([1 data.mapsize(2)]);
ylim([1 data.mapsize(1)]);
axis square;
title('Map for non-holonomic RRT');

% Check that the start & goal are within bounds
if(checkLimitViolation_carBot(data,start) || checkLimitViolation_carBot(data,goal))
    error('Robot start and goal state must be within the map limits');
end

% Check that the start & goal are not in collision
if(checkCollision_carBot(data,start) || checkCollision_carBot(data,goal))
    error('Robot start and goal state must not be in collision');
end

%% READ -> STUDENT TODO: Run planner and get results
%% PLEASE READ!!!!!!!!!!!!!!!!!!
%ct = 0;
%tempV=double([start -1]);
%RRTree=[tempV b_];
%RRTree=[node];
%RRTree=double([start -1 0 0 0 0]); %[start ID vel,ang,stepTime,totalTime]
start1.pose=double(start);
start1.preId=0;
%start1.n2node=[];
start1.cost=0;

RRTree=[start1];
stepsize=1; % size of each step of the RRT
disTh=5; % nodes closer than this threshold are taken as almost the same
failedAttempts=0;
numcontrolsamples=25;
x_new=[start1];
I_=0;
Timecost=0;
% 
index=10;
indexArray=[];
handle=[];
while(1)
    
% get sample
    if rand > 0,
        xrand(1:2)= rand(1,2) .* size(data.envmap); % random sample
    else
        xrand(1:2)= goal(1:2); % sample taken as goal to bias tree generation to goal
    end
    xrand(3)=2*pi*(rand*2-1);
    
% find nearest
[index,indexArray,minCost]=findNearest(RRTree,xrand);
CostTemp=40000;
xnewPose=[];
isValid=true;
Dist_=-1;
% choose connector
if minCost > 10
   rate=(xrand(2)-RRTree(index).pose(2))/(xrand(1)-RRTree(index).pose(1));
   theta=atan2(xrand(2)-RRTree(index).pose(2),xrand(1)-RRTree(index).pose(1));
   xrand(1)=RRTree(index).pose(1)+10*cos(theta);
   xrand(2)=RRTree(index).pose(2)+10*sin(theta);
end

for i=1:size(indexArray,2)
        
        xnear_=RRTree(indexArray(i)).pose;
        Cost_=RRTree(indexArray(i)).cost;
%       [path,isValid]=steering(xnear_,xrand,data);  
        isValid=isValidPath(xnear_(1:2),xrand(1:2),data);
        if isValid==false
            continue;
        end
        Dist_=distanceCost(xnear_(1:2),xrand(1:2));
        if Cost_+Dist_ < CostTemp
           preID=indexArray(i);
           pathCost=Cost_+Dist_;
           CostTemp=Cost_+Dist_;
        end       
    end    
% add the node
if Dist_==-1
   continue;
end

xnew.pose=xrand;
xnew.preId=preID;
xnew.cost=pathCost;
RRTree = [RRTree;xnew]; % add node

handle(length(RRTree))=plot([RRTree(preID).pose(1),xnew.pose(1)],[RRTree(preID).pose(2),xnew.pose(2)],'r');
hold on

% rewire    
for i=1:size(indexArray,2)
    if indexArray(i)==preID
        continue;
    end
    Cost_=RRTree(indexArray(i)).cost;
    %[path,isValid]=steering(xnew.pose,RRTree(indexArray(i)).pose,data);
    isValid=isValidPath(xnew.pose(1:2),RRTree(indexArray(i)).pose(1:2),data);
    if isValid==false
        continue;
    end   
    Dist_=distanceCost(xnew.pose(1:2),RRTree(indexArray(i)).pose(1:2));
    if xnew.cost+Dist_ < Cost_
        RRTree(indexArray(i)).preId=length(RRTree);
        RRTree(indexArray(i)).cost=xnew.cost+Dist_;
        delete(handle(indexArray(i)));
        handle(indexArray(i))=plot([xnew.pose(1),RRTree(indexArray(i)).pose(1)],[xnew.pose(2),RRTree(indexArray(i)).pose(2)],'r');
    end
end

pause(0.1)

end

% figure show
%figure(2)
% delete handle
% if length(RRTree)>=2
%     for i=2:length(RRTree)
%         handle=plot([RRTree(i).pose(1),RRTree(RRTree(i).preId).pose(1)],[RRTree(i).pose(2),RRTree(RRTree(i).preId).pose(2)],'--r');
%         hold on
%     end
% end
% path=[];
% prev=I_;
% while prev>0
%     path=[RRTree(prev,1:3) RRTree(prev,5:7);path];
%     prev=RRTree(prev,4);
% end
% line(path(:,1),path(:,2));
% %%      
% x = path(1,1:3);  
% for i=1:size(path,1)
%     dt1 = 0.1; % Step by 0.1 seconds
%     rollout1 = x;
%     L1 = 7.5; % Car length
%     linearvel1=path(i,4);
%     steerangl1=path(i,5);
%     for i=1:path(i,6)
%         x(1) = x(1) + linearvel1 * cos(x(3)) * dt1;
%         x(2) = x(2) + linearvel1 * sin(x(3)) * dt1;
% %        x(3) = x(3) + (linearvel1/L1) * tan(steerangl1) * dt1;
%         x(3) = x(3) + steerangl1 * dt1;
%         rollout1 = [rollout1; x];   % maintain history
%     end
%     if size(rollout1,1)>1
% 
%         plot(rollout1(:,1),rollout1(:,2),'r','LineWidth',3,'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5])
%         hold on
%         drawnow
%     end
%     for i=1:size(rollout1,1)
%         plotCarBot(data, rollout1(i,:), 'k'); % Do not plot start/goal
%         pause(0.05);
%     end
% 
% end

%% Nothing to do here - Display path
% Check that the path is within bounds/collision free

% for k = 1:size(fpath,1)
%     pt = fpath(k,:);
%     if checkLimitViolation_carBot(data, pt)
%         error('State (%f, %f, %f) on final path is out of bounds!',pt(1), pt(2), pt(3));
%     end
%     if checkCollision_carBot(data, pt)
%         error('State (%f, %f, %f) on final path is in collision!',pt(1), pt(2), pt(3));
%     end
% end

% Display path
% figure(fg);
% for k = 2:size(fpath,1)-1
%     plotCarBot(data, fpath(k,:), 'k'); % Do not plot start/goal
%     pause(0.05);
% end

%% NOTHING TO DO FROM THIS PART ONWARDS
function [xnew, isValid, rollout,time] = simulate_carBot(data, xnear, xrand, deltastep, linearvel, steerangl)
% Simulates a given control from the nearest state on the graph to the
% random sample.
% INPUTS:
% data      - Map and Robot data
% xnear     - Nearest point on the current graph to the random sample
% xrand     - Random sample
% deltastep - Number of steps to forward propagate (grow the graph)
% linearvel - Linear velocity of the car (control)
% steerangl - Steering angle of the car (control)
% OUTPUTS:
% xnew      - Potential new point to add to the graph for the current control
% isValid   - Flag to indicate if the rollout is valid (within limits & no
%             collisions). If isValid is false, you should discard this xnew/rollout.
% rollout   - Actual path found by the system from xnear->xnew

    % Set vehicle constants
    dt = 0.1; % Step by 0.1 seconds
    L = 7.5; % Car length
    
    % Simulate forward from xnear using the controls (linearvel, steerangl)
    % to generate the rollout
    x = xnear;
    rollout = x;
    for i=1:deltastep
        x(1) = x(1) + linearvel * cos(x(3)) * dt;
        x(2) = x(2) + linearvel * sin(x(3)) * dt;
%        x(3) = x(3) + (linearvel/L) * tan(steerangl) * dt;
        x(3) = x(3) + steerangl * dt;
        rollout = [rollout; x];   % maintain history
    end
    
    % Find the closest point to xrand on the rollout
    % This is xnew. Discard the rest of the rollout
    dst = distance(rollout, xrand);
    [A, I]=min(distanceCost(rollout(:,1:2),xrand(1:2)) ,[],1);   
    %[~, id] = min(dst, [], 1);
    xnew = rollout(I(1), :); 
    time=I(1)-1;
    % rollout(id+1:end,:) = []; % Do not need this part now
    % Check for collision on the path:
    isValid = true;
    for i=1:size(rollout,1)
        % For each point, check if it is in violation of the limits or if
        % it is in collision. If either, return isValid = false
        if(checkLimitViolation_carBot(data, rollout(i,:)) || ...
           checkCollision_carBot(data, rollout(i,:)))
            isValid = false;
            break;
        end
    end
    
    % In case the rollout is not valid, set xnew to empty
    if ~isValid
        xnew = [];
        return;
    end
    
end

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

function H = plotcircle(center, radius, NOP, cl)
    THETA=linspace(0,2*pi,NOP);
    RHO=ones(1,NOP)*radius;
    [X,Y] = pol2cart(THETA,RHO);
    X=X+center(1);
    Y=Y+center(2);
    H = fill(X,Y,cl);
end

function plotCarBot(data, pose, cl)
% Plots the car robot as a box
% INPUT:
% data - Map and robot data
% pose - pose of robot [x,y,theta] => center of robot & angle
% cl   - color of the box outline
    if nargin < 3; cl = 'w'; end; 
    
    % Plot a circle for the car
    H = plotcircle(pose(1:2), data.robotradius, 50, cl);
    set(H, 'Facecolor', 'w', 'FaceAlpha', 0.6);
    
    % Now plot a line for the direction of the car
    ed = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))] * [data.robotradius*1.5; 0];
    plot([pose(1), pose(1)+ed(1)], [pose(2), pose(2)+ed(2)], 'b-', 'Linewidth', 3);
end

function d = angdiff(th1, th2)
% Returns angle difference in -180 to 180
    d = th1-th2;
    d = mod(d+pi, 2*pi) - pi;
    d = d*(180/pi); % convert to degrees
end