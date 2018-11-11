function RRTree=rewire(RRTree,x_new,data)
numcontrolsamples=25;
maxlinearvel = 60;
maxsteerangle = 0.6;
I_r=findNNwithinCircle(RRTree,x_new);
x_near=x_new;
deltastep=5;
for i=1:size(I_r,1)
    x_rand=RRTree(I_r,1:2);
    for i = 1:numcontrolsamples
        linearvel= maxlinearvel*rand;
        steerangl= maxsteerangle*(rand*2-1);
        [xnew, isValid, rollout,time] = simulate_carBot(data, xnear, xrand, deltastep, linearvel, steerangl);
        if isValid
            dist=distanceCost(xnew,xnear);
            anddiff= xnew-RRTree(I_r,3);
            if dist<distance2xrand & angdiff < 
                x_new=xnew;
                rollout_=rollout;
                lvel=linearvel;
                lang=steerangl;
                ltime=time;
                ltotaltime=time+Timecost;
                %               plot(rollout(:,1),rollout(:,2));
                %               hold on
                %               drawnow
            end
        end
    end
end