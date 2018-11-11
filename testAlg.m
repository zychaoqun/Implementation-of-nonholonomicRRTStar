x1=[1,1,0];% start point
x2=[3,5,pi/2];% target point
d=distanceCost(x1(1:2),x2(1:2));
dy=x2(2)-x1(2);
dx=x2(1)-x1(1);
phi=atan2(dy/d,dx/d);

x=[];
Kp1=0.1;
Kp2=0.05;
Kp3=0.05;
dt1=0.01;
% controller
x=x1;
figure
while distanceCost(x1(1:2),x2(1:2))>0.001 || abs(x1(3)-x2(3))>0.01
    U1=Kp1* distanceCost(x1(1:2),x2(1:2));
    U2=Kp2* (x2(3)-x1(3))+Kp3*(phi-x1(3));
    x(1) = x(1) + U1 * cos(x(3)) * dt1;
    x(2) = x(2) + U1 * sin(x(3)) * dt1;
    x(3) = x(3) + U2 * dt1;
  
    plot([x1(1),x(1)],[x1(2),x(2)])
    drawnow
    hold on  
    x1=x;
    
    d=distanceCost(x1(1:2),x2(1:2));
dy=x2(2)-x1(2);
dx=x2(1)-x1(1);
phi=atan2(dy/d,dx/d);

end
