function isValid=isValidPath(start,goal,data)
isValid=true;
start1=start;
goal1=goal;
[x,y]=bresenham(start1(1),start1(2),goal1(1),goal1(2));
X=[x';y'];
for i=1:length(x)
    if(checkLimitViolation_carBot(data, X(:,i)) ||  checkCollision_carBot(data, X(:,i)))
        isValid = false;
        return;
    end
end

end