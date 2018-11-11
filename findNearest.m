function [index,indexArray,minCost]= findNearest(RRT,xrand)
V=10000;
index=-1;
indexArray=[];
% find the nearest neighbor
for i=1:size(RRT)
    X=distanceCost(RRT(i).pose(1:2),xrand(1:2));
    if X< V
        index=i;
        V=X;
    end
end
minCost=V;
% find the neighbor within a circle
V=V+10;
for i=1:size(RRT)
    Y=distanceCost(RRT(i).pose(1:2),xrand(1:2));
    if Y<V
       indexArray=[indexArray,i];
    end
end
    
end