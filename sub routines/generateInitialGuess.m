function [initialGuess] = generateInitialGuess(zBegin,zFinal, nGrid, duration)
%function [initialGuess] = generateInitialGuess(zBegin,zFinal, nGrid, duration)
%
%This function generates the initial guess for the trajectory optimizaion
%of a car. The spline is a cubic spline with derivative zero at the end
%points. The spline goes through 3 points. The end points are the final and
%initial state. The middle point is the average, of the final and initial
%state with the steering angle and car angle changed. This helps the
%simulation overcome the non-holonomic nature of the car.
%
%INPUTS:
%   zBegin = [7, 1] = [x; y; q1; q2; v; fd; u2] = initial state
%        x = horizontal position
%        y = vertical position
%        q1 = absolute angle (zero for angle to left)
%        q2 = steering angle
%        v = velocity in forward direction
%        fd = force from motor
%        u2 = desired steering angle
%   zFinal=[7, 1] = final state
%   nGrid=number of grid points for simulation
%   duration=length of time for simulation
%   
%   OUTPUTS:
%       initialGuess=struct containing the initial guess
%         .time = [1, nGrid] = time grid for the transcription (constant)
%         .state = [7, nGrid] = guess for state at gridpoints
%         .control = [2, nGrid] = guess for control at gridpoints
   

if nargin <5
    timeGrid=linspace(0, duration, nGrid);
end

%calculate middle point
zMiddle= (zBegin+zFinal)/2;

%find change in position
displacement=zFinal-zBegin;
%atan of displacement to rotate car
zMiddle(3)=atan2(displacement(2),displacement(1));
%steering angle of middle is change in car rotation
zMiddle(4)=zFinal(3)-zMiddle(3);
%time vector for initial guess
initialGuess.time = timeGrid;
%fit the spline
ppState = spline([0, duration/2, duration], [zeros(7,1),zBegin,zMiddle, zFinal,zeros(7,1)]);
%evaluate spline
stateGuess = ppval(ppState,initialGuess.time);
initialGuess.state = stateGuess;
%initial control is 0
initialGuess.control = zeros(2, length(timeGrid));         

end