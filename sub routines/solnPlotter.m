function[] = solnPlotter(soln,carParam,problemParam, useBoundaryFunc, figureNum)
%function[] = solnPlotter(soln,carParam,problemParam, useBoundaryFunc, figureNum)
%
%This function will plot the solutions for the car problems and create an
%animation.
%
%INPUTS:
%   soln=solution struct for the problem. See simpleCarSubProblem for
%       details
%   carParam=parameters that define car. See simpleCarSubProblem for details
%   problemParam=parameters that define the problem. See
%               simpleCarSubProblem for details
%   useBoundaryFunc=boolean=true if the boundary function should be plotted
%   figureNum=int=starting number for plotting the figures

%if figure num is not provided start at 8010;
if nargin<5
    figureNum=8010;
end

%crate the figure and parse the results
figure(figureNum); clf;
t = soln.grid.time;
x = soln.grid.state(1,:);
y = soln.grid.state(2,:);
q1 = soln.grid.state(3,:);
q2 = soln.grid.state(4,:);
v = soln.grid.state(5,:);
fd = soln.grid.state(6,:);

u1 = soln.grid.control(1,:);
u2 = soln.grid.control(2,:);
tBnd = t([1, end]);
bndMarker = 'ks';
%bndSize = 8;
lineWidth = 2;

% X,Y
subplot(3,2,1); hold on;
plot(t, x, 'LineWidth', lineWidth)
plot(t, y, 'LineWidth', lineWidth)
plot(tBnd, [problemParam.zBegin(2); problemParam.zFinal(2)], ...
    bndMarker, 'LineWidth', lineWidth);
plot(tBnd, [problemParam.zBegin(1); problemParam.zFinal(1)], ...
    bndMarker, 'LineWidth', lineWidth);
legend('X Pos', 'Y Pos')
xlabel('time (s)');
ylabel('Position (m)');
title('Simple Car BVP  --  Min. Effort')

%q1 and q2
subplot(3,2,3); hold on;
plot(t, q1, 'LineWidth', lineWidth)
plot(t, q2, 'LineWidth', lineWidth)
plot(tBnd, [problemParam.zBegin(4); problemParam.zFinal(4)], ...
    bndMarker, 'LineWidth', lineWidth);
plot(tBnd, [problemParam.zBegin(3); problemParam.zFinal(3)], ...
    bndMarker, 'LineWidth', lineWidth);
legend('Car Heading', 'Steering Angle')
xlabel('time (s)');
ylabel('Angle (rad)');

%velocity
subplot(3,2,5); hold on;
plot(t, v, 'LineWidth', lineWidth)
plot(tBnd, [problemParam.zBegin(5); problemParam.zFinal(5)], ...
    bndMarker, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Velocity (m/s)');

%force
subplot(3,2,2); hold on;
plot(t, fd, 'LineWidth', lineWidth)
plot(tBnd, [problemParam.zBegin(6); problemParam.zFinal(6)], ...
    bndMarker, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Force (N)');

% derivative of force
subplot(3,2,4); hold on;
plot(tBnd, [0,0], 'k--');
plot(t,u1, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Force Rate (N/s)');
title(sprintf('ObjVal: %4.4f', soln.info.objVal));

%rate of change of desired steering angle
subplot(3,2,6); hold on;
plot(tBnd, [0,0], 'k--');
plot(t,u2, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Rate of Change of Desired Steering Angle (rad/s)');

%quiver plot of trajectory
% figure(figureNum+10); clf;
% quiver(x,y,0.25*v.*cos(q1),0.25*v.*sin(q1), 'LineWidth', lineWidth)

figure (figureNum+10);clf;
subplot(2,1,1);
plot(soln.error(1,:),'LineWidth',lineWidth);
title('Error for State 1')
subplot(2,1,2);
bar(max(soln.errorGrid,[],1),'LineWidth',lineWidth);
title('Max Error for Each Segment')

%animation
figure (figureNum+20); clf;
d=0.55*carParam.l;
%set limits of graph so square and big enough
lim=[min([x,y,problemParam.roadDim])-d,max([x,y,problemParam.roadDim])+d];
xlim(lim)
ylim(lim)
if useBoundaryFunc
    simpleCarAnimatate(soln.grid.time, soln.grid.state, carParam,problemParam.roadDim, problemParam.boundaryFunc)
else
    simpleCarAnimatate(soln.grid.time, soln.grid.state, carParam,problemParam.roadDim)
end



end