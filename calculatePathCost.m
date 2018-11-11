function cost=calculatePathCost(array)
cost=0;
for i=1:length(array)-1
   cost=cost+distanceCost(array(1:2,i)',array(1:2,i+1)');
end

end