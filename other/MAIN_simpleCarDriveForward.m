% MAIN  --  Simple Car Drive Forward
%
% Optimal trajectory for a car driving forward
% the trapezoid method for direct collocation.
%


run('CodeLibrary/addToPath.m');
clc; clear;

% Duration and number of grid points:
nGrid = 50;
duration = 6.0;

%car is making a narrow u turn
initialPos = [0  0 0];
finalPos  = [0.8 0.005  0];

tightRoadDim=[.8, -0.2, .1, -.1];


%bounds on steering and desired steering angle
problemParam.steeringBounds.q2_b=45*pi/180;
problemParam.steeringBounds.u2_b=pi/2;
problemParam.velocityBound=Inf;

% Boundary states:
problemParam.zBegin = [initialPos, 0 , 0, 0, 0]';
problemParam.zFinal = [finalPos, 0 ,0,0, 0]';

problemParam.useBoundaryFunc=false;


%carParameters
carParam = struct('w',0.05','l', 0.1, 'a', 0.05, 'b', 0.05 , 'm', 0.2 , 'J', 3.333*10^-4,'tau',1/3);

%get initialGuess
initialGuess = generateInitialGuess(problemParam.zBegin,problemParam.zFinal, nGrid, duration)   ;  

% Set the options for FMINCON
problemParam.nlpOpt = optimoptions('fmincon');
problemParam.nlpOpt.Display = 'iter';
problemParam.nlpOpt.OptimalityTolerance = 1e-6;
problemParam.nlpOpt.ConstraintTolerance = 1e-10;
problemParam.nlpOpt.MaxFunctionEvaluations = 2e5;

%set parameters for problem
problemParam.isTimeDVar=false;
problemParam.bndCstLevel='dot';
problemParam.roadDim=tightRoadDim;

%objective function constants
problemParam.alpha=1e1;%time
problemParam.beta=0.25;%steering angle


% Call the optimization:
[soln, nextGuess] =  simpleCarSubProblem(carParam, problemParam,initialGuess);

solnPlotter(soln,carParam,problemParam, false, 8010)
