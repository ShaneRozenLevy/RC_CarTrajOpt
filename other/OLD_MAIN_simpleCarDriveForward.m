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
%   talk to Matt about order of problems

run('CodeLibrary/addToPath.m');
clc; clear;

% Duration and number of grid points:
nGrid = 60;
duration = 4.0;

initialPos = [0  0 0];
finalPos  = [1 5  0];

q2_b=45*pi/180;
%q2_b=Inf;

u2_b=pi/2;
%u2_b=Inf;

problem.bounds.time.lb=duration;
problem.bounds.time.ub=duration;

%problem.bounds.time.lb=0;
%problem.bounds.time.ub=Inf;

% Boundary states:
zBegin = [initialPos, 0 , 0, 0]';
zFinal = [finalPos, 0 ,0,0]';


% Dynamics function:
param = struct('l', 0.1, 'a', 0.05, 'b', 0.05 , 'm', 0.2 , 'J', 3.333*10^-4,'tau',1/3);
problem.func.dynamics = @(t, z, u)( simpleCarDynamics(z, u, param) );

% Path integral:  (minimize the integral of actuation-squared)
problem.func.pathObj = @(t, z, u)(u(1,:).^2+0.5*u(2,:).^2+0);

%Path constraint
problem.func.pthCst = @(t,z,u)deal([],[]);

% Boundary constraint:
problem.func.bndCst = @(t0, tF, z0, zF)( ...
                        deal([], [z0 - zBegin; zF - zFinal]) );

%adding limit to steering angle

q2_ub=q2_b;
q2_lb=-q2_b;


u2_ub=u2_b;
u2_lb=-u2_b;

problem.bounds.state.ub=[Inf, Inf, Inf, q2_ub, Inf, Inf];
problem.bounds.state.lb=[-Inf, -Inf, -Inf, q2_lb, -Inf, -Inf];
problem.bounds.control.ub=[Inf, u2_ub];
problem.bounds.control.lb=[-Inf, u2_lb];


%Initial guess: (cubic segment, matching boundary conditions)

% zMiddle= (zBegin+zFinal)/2;
% displacement=zFinal-zBegin;
% zMiddle(3)=atan2(displacement(2),displacement(1));
% problem.guess.time = linspace(0, duration, nGrid);
% ppState = spline([0, duration/2, duration], [zeros(6,1),zBegin,zMiddle, zFinal,zeros(6,1)]);
% stateGuess = ppval(ppState, problem.guess.time);
% problem.guess.state = stateGuess;
% problem.guess.control = zeros(2, nGrid);    
problem.guess = generateInitialGuess( zBegin,zFinal, nGrid, duration);


% Set the options for FMINCON
problem.nlpOpt = optimoptions('fmincon');
problem.nlpOpt.Display = 'iter';
problem.nlpOpt.OptimalityTolerance = 1e-6;
problem.nlpOpt.ConstraintTolerance = 1e-10;
problem.nlpOpt.MaxFunctionEvaluations = 1e6;

% Call the optimization:
soln = dirColBvpTrap(problem);

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
plot(tBnd, [zBegin(2); zFinal(2)], ...
    bndMarker, 'LineWidth', lineWidth);
plot(tBnd, [zBegin(1); zFinal(1)], ...
    bndMarker, 'LineWidth', lineWidth);
legend('X Pos', 'Y Pos')
xlabel('time (s)');
ylabel('Position (m)');
title('Simple Car BVP  --  Min. Effort')

subplot(3,2,3); hold on;
plot(t, q1, 'LineWidth', lineWidth)

plot(t, q2, 'LineWidth', lineWidth)
plot(tBnd, [zBegin(4); zFinal(4)], ...
    bndMarker, 'LineWidth', lineWidth);
plot(tBnd, [zBegin(3); zFinal(3)], ...
    bndMarker, 'LineWidth', lineWidth);
legend('Car Heading', 'Steering Angle')
xlabel('time (s)');
ylabel('Angle (rad)');

subplot(3,2,5); hold on;
plot(t, v, 'LineWidth', lineWidth)
plot(tBnd, [zBegin(5); zFinal(5)], ...
    bndMarker, 'LineWidth', lineWidth);
xlabel('time (s)');
ylabel('Velocity (m/s)');

subplot(3,2,2); hold on;
plot(t, fd, 'LineWidth', lineWidth)
plot(tBnd, [zBegin(6); zFinal(6)], ...
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
ylabel('Desired Steering Angle (rad)');

figure(8020); clf;
quiver(x,y,0.25*v.*cos(q1),0.25*v.*sin(q1), 'LineWidth', lineWidth)


