function I=findNNwithinCircle(RRTree,x_new)
Radius=20;
I=[];
for i=1:size(RRTree,1)
    dist=distanceCost(RRTree(i,1:2),x_new);
    if dist< Radius
       I=[I,i];  
    end
end