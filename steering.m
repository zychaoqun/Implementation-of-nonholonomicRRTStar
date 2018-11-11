%   Demo Script  to test  POSQ controller for differential drive robots
%   generates the trajectory which brings a differential drive robot
%   with wheelbase B from the start pose xinit to the goal pose xgoal.
%   Need to set the proper direction (dir = 1 forward direction), the
%   integration time step (deltaT), the initial time (initT). Visualization
%   is done by using drawrobot in librobotics 
%   (see http://srl.informatik.uni-freiburg.de/downloads)
%   See also posqstep and posqintegr
%
% Copyright (c) 2014, Luigi Palmieri, Social Robotics Laboratory,
% University of Freiburg
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
% 
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
% 
% 2. Redistributions in binary form must reproduce the above copyright
% notice, this list of conditions and the following disclaimer in the
% documentation and/or other materials provided with the distribution.
% 
% 3. Neither the name of the copyright holder nor the names of its
% contributors may be used to endorse or promote products derived from this
% software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
% IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
% THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
% CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

%=============================
% Define the problem to solve
%=============================
function [path,isValid]=steering(start,goal,data)
isValid=true;
% xinit=[100;40;3.1416];
% xgoal=[111.34;169.80;4.33];
xinit=start;
xgoal=goal;
dir=1;
deltaT=0.1;
b=30;
initT=0;

%=============================
% Call Integration of the POSQ
%=============================
[xvec, speedvec, vel, inct,valid] = posqintegr(xinit, xgoal,dir, deltaT, b,initT);
%=============================
% Visualization
%=============================
steerpath=xvec;
if size(xvec,2)>100
    steerpath=xvec(:,1:100);
end
path=steerpath;

% For each point, check if it is in violation of the limits or if
% it is in collision. If either, return isValid = false
for i=1:length(path)
    if(checkLimitViolation_carBot(data, steerpath(1:2,i)) || ...
            checkCollision_carBot(data, steerpath(1:2,i)))
        isValid = false;
        return;
    end
end
figure(1)


% if(exist('drawrobot.m')>0)
%  hold on,title('Robot trajectory')
%     for i=1:1:length(steerpath)
%         drawrobot(steerpath(:,i),'r',2);
%         %plot([steerpath(1,i),steerpath(1,i+1)],[steerpath(2,i),steerpath(2,i+1)])
%         drawnow
%        % pause(0.05)
%     end
% else
%     disp('drawrobot not found!!! ')
end



