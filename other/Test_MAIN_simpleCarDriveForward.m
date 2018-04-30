% MAIN  --  Simple Car Drive Forward
%
% Optimal trajectory for a car driving forward
% the trapezoid method for direct collocation.
%
% Demo:   adjust the parameters!
%
%    --> Can you find a set of parameters that does swings backward before
%    the final swing-up?
%
%   --> Adjust ALL of the parameters below and see what happens
% 

%TODO
% work on dealing with car not point
% update comments
% parallel parking/dot avoidance

run('CodeLibrary/addToPath.m');
clc; clear;

% Duration and number of grid points:
nGrid = 50;
duration = 10.0;

initialPos = [0  0 0];
finalPos  = [0.08 0  0];

tightRoadDim=[.14, -.14, .125, -.1];
tighterRoadDim=[.1, -.1, .8, -.8];
looseRoadDim=[.175, -.175, .2, -.2];
veryLooseRoadDim=[1.175, -1.175, 1.2, -1.2];


problemParam.steeringBounds.q2_b=45*pi/180;
problemParam.steeringBounds.u2_b=pi/2;

% Boundary states:
problemParam.zBegin = [initialPos, 0 , 0, 0, 0]';
problemParam.zFinal = [finalPos, 0 ,0,0, 0]';


%carParameters
carParam = struct('w',0.05','l', 0.1, 'a', 0.05, 'b', 0.05 , 'm', 0.2 , 'J', 3.333*10^-4,'tau',1/3);

%get initialGuess
initialGuess = generateInitialGuess(problemParam.zBegin,problemParam.zFinal, nGrid, duration)   ;  

% Set the options for FMINCON
problemParam.nlpOpt = optimoptions('fmincon');
problemParam.nlpOpt.Display = 'iter';
problemParam.nlpOpt.OptimalityTolerance = 1e-6;
problemParam.nlpOpt.ConstraintTolerance = 1e-10;
problemParam.nlpOpt.MaxFunctionEvaluations = 1e5;

problemParam.isTimeDVar=true;
problemParam.bndCstLevel='none';

problemParam.alpha=1e2;
problemParam.beta=0.25;


% Call the optimization:
[soln1, nextGuess] =  simpleCarSubNarrowRoadProblem(carParam, problemParam,initialGuess);

problemParam.bndCstLevel='dot';
problemParam.roadDim=looseRoadDim;
problemParam.nlpOpt.MaxFunctionEvaluations = 2e5;

[soln2, nextGuess] =  simpleCarSubNarrowRoadProblem(carParam, problemParam,nextGuess);

problemParam.roadDim=tightRoadDim;

[soln3, nextGuess] =  simpleCarSubNarrowRoadProblem(carParam, problemParam,nextGuess);

problemParam.roadDim=tighterRoadDim;

[soln4, nextGuess] =  simpleCarSubNarrowRoadProblem(carParam, problemParam,nextGuess);


problemParam.bndCstLevel='rectangle';
problemParam.roadDim=tightRoadDim;


[soln5, nextGuess] =  simpleCarSubNarrowRoadProblem(carParam, problemParam,nextGuess);

problemparam.isTimeDVar=true;


[soln, ~] =  simpleCarSubNarrowRoadProblem(carParam, problemParam,nextGuess);

soln1.info.objVal
soln2.info.objVal
soln3.info.objVal
soln4.info.objVal
soln5.info.objVal


%% Make some plots:
figure(8010); clf;
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
bndSize = 8;
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

subplot(3,2,5); hold on;
plot(t, v, 'LineWidth', lineWidth)
plot(tBnd, [problemParam.zBegin(5); problemParam.zFinal(5)], ...
    bndMarker, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Velocity (m/s)');

subplot(3,2,2); hold on;
plot(t, fd, 'LineWidth', lineWidth)
plot(tBnd, [problemParam.zBegin(6); problemParam.zFinal(6)], ...
    bndMarker, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Force (N)');

% Force
subplot(3,2,4); hold on;
plot(tBnd, [0,0], 'k--');
plot(t,u1, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Force Rate (N/s)');
title(sprintf('ObjVal: %4.4f', soln.info.objVal));

subplot(3,2,6); hold on;
plot(tBnd, [0,0], 'k--');
plot(t,u2, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Rate of Change of Desired Steering Angle (rad/s)');

figure(8020); clf;
quiver(x,y,0.25*v.*cos(q1),0.25*v.*sin(q1), 'LineWidth', lineWidth)

figure (8030); clf;
d=0.55*carParam.l;
lim=[min([x,y])-d,max([x,y])+d];
xlim(lim)
ylim(lim)
simpleCarAnimatate(soln.grid.time, soln.grid.state, carParam,problemParam.roadDim)
