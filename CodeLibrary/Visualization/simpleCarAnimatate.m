function simpleCarAnimatate(t, z, param,roadDim, boundaryFunc, playbackSpeed)
%function simpleCarAnimatate(t, z, param,roadDim, boundaryFunc, playbackSpeed)
%
%This function will animate the car driving around.
%
%INPUTS:
%   t = [1,nGrid]=vector of time
%   z = [7, nGrid] = [x; y; q1; q2; v; fd; u2] = state of the car at the
%                                                times
%        x = horizontal position
%        y = vertical position
%        q1 = absolute angle (zero for angle to left)
%        q2 = steering angle
%        v = velocity in forward direction
%        fd = force from motor
%        u2 = desired steering angle
%   param = struct with constant scalar parameters for the car:
%       .l = distance from front wheel to back wheel
%       .a = distance from cm to front wheel
%       .b = distance from cm to back wheel
%       .w = width of car
%   roadDim=[+x, -x, +y, -y]=location of walls relative to 0,0
%   boundaryFunc=y=f(x)=curve that car will always be above
%   playbackSpeed = speed of animation

if nargin < 6
    playbackSpeed = 0.5;
end

if nargin < 5
    P.plotFunc = @(t, zu)simpleCarDraw( zu, param,roadDim);
else
    %evaluate the boundary function once and pass the grid of points to the
    %plotting function
    grid=linspace(roadDim(2),roadDim(1),100);
    parkedGrid=boundaryFunc(grid);
    P.plotFunc = @(t, zu)simpleCarDraw( zu, param,roadDim,grid,parkedGrid);
end   

P.speed = playbackSpeed;
P.figNum = gcf();
animate(t, z, P);

end