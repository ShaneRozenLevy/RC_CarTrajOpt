% MAIN  --  Simple Car Drive Forward
%
% Optimal trajectory for a car turning around on a narrow road.
% the trapezoid method for direct collocation. This code uses trapezoidal
% direct collocation to derive the optimal trajectory for the car to
% turn around on a narrow road. This script should automatically add all 
% dependencies to your path assuming you have not moved around the 
% directories. 
%
%Dependencies:
%   autoGen_simpleCarDynamics.m
%   simpleCarDynamics.m
%   ppDer.m
%   ppSpline1.m
%   ppSpine2a.m
%   dirColBvpTrap.m
%   lagrangepoly.m
%   animate.m
%   simpleCarAnimatate.m
%   simplaceCarDraw.m
%   addToPath.m
%   boundaryTest.m
%   generateInitialGuess.m
%   simpleCarSubProblem.m
%   solnPlotter.m



run('CodeLibrary/addToPath.m');
addpath(genpath(fileparts(mfilename('fullpath'))));
clc; clear;

% Duration and number of grid points:
nGrid = 70;
duration = 6.0;

%car is making a narrow u turn
initialPos = [0  0 0];
finalPos  = [0 0.085  pi];

%lane width 0.075, median 0.01

roadDim3=[.15, -.15, .125, -.043];
roadDim2=[.16, -.16, .13, -.05];
roadDim1=[.17, -.17, .15, -.06];
veryLooseRoadDim=[1.175, -1.175, 1.2, -1.2];

%bounds on steering and desired steering angle
problemParam.steeringBounds.q2_b=40*pi/180;
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
problemParam.isTimeDVar=true;
problemParam.bndCstLevel='none';

%objective function constants
problemParam.alpha=2e1;%time
problemParam.beta=0.2;%steering angle


problemParam.nlpOpt.MaxFunctionEvaluations = 4e5;
problemParam.isTimeDVar=true;
problemParam.roadDim=roadDim1;
problemParam.bndCstLevel='rectangle';

% Call the optimization:
[soln1, nextGuess] =  simpleCarSubProblem(carParam, problemParam,initialGuess);

problemParam.roadDim=roadDim2;

[soln2, nextGuess] =  simpleCarSubProblem(carParam, problemParam,nextGuess);

problemParam.roadDim=roadDim3;

[soln, nextGuess] =  simpleCarSubProblem(carParam, problemParam,nextGuess);

solnPlotter(soln,carParam,problemParam, false, 8010)
 
