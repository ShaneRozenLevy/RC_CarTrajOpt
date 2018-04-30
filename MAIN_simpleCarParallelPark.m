% MAIN  --  Simple Car Parallel Park
%
% Optimal trajectory for a car parallel parking
% the trapezoid method for direct collocation. This code uses trapezoidal
% direct collocation to derive the optimal trajectory for the car to
% parallel park. This script should automatically add all dependencies to
% your path assuming you have not moved around the directories. 
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
nGrid = 75;
duration = 6.0;

%lane width 0.075
%median 0.01

%car is making a narrow u turn
initialPos = [0 0 0];
finalPos  = [.12 -0.06  0];

tightRoadDim=[.3, -.06, .05, -.095];
tighterRoadDim=[.2, -.01, .04, -.07];

carStartY=-0.045;
carEndX=0.04;
carStartX=0.19;


%bounds on steering and desired steering angle
problemParam.steeringBounds.q2_b=40*pi/180;
problemParam.steeringBounds.u2_b=pi/2;

%bounds on velocity
problemParam.velocityBound=.3;

% Boundary states:
problemParam.zBegin = [initialPos, 0 , 0, 0, 0]';
problemParam.zFinal = [finalPos, 0 ,0,0, 0]';


%carParameters
carParam = struct('w',0.05','l', 0.1, 'a', 0.05, 'b', 0.05 , 'm', 0.2 , 'J', 3.333*10^-4,'tau',1/3);

%get initialGuess
initialGuess = generateInitialGuess(problemParam.zBegin,problemParam.zFinal, nGrid, duration);  

% Set the options for FMINCON
problemParam.nlpOpt = optimoptions('fmincon');
problemParam.nlpOpt.Display = 'iter';
problemParam.nlpOpt.OptimalityTolerance = 1e-6;
problemParam.nlpOpt.ConstraintTolerance = 1e-10;

%set parameters for problem
problemParam.isTimeDVar=true;
problemParam.roadDim=tightRoadDim;
problemParam.boundaryFunc=boundaryTest('spline',problemParam.roadDim,carStartY,carEndX,carStartX);
problemParam.nlpOpt.MaxFunctionEvaluations = 4e5;
problemParam.useBoundaryFunc=true;
problemParam.bndCstLevel='none';

%objective function constants
problemParam.alpha=1e1;%time
problemParam.beta=2e0;%steering angle



% Call the optimization:
[soln1, nextGuess] =  simpleCarSubProblem(carParam, problemParam,initialGuess);

problemParam.bndCstLevel='dot';
[soln2, nextGuess] =  simpleCarSubProblem(carParam, problemParam,nextGuess);

problemParam.bndCstLevel='rectangle';
[soln3, nextGuess] =  simpleCarSubProblem(carParam, problemParam,nextGuess);


carEndX=0.05;
carStartX=0.18;
carStartY=-0.03;


problemParam.boundaryFunc=boundaryTest('spline',problemParam.roadDim,carStartY,carEndX,carStartX);

[soln, nextGuess] =  simpleCarSubProblem(carParam, problemParam,nextGuess);

carStartY=-0.045;
carEndX=0.04;
carStartX=0.19;
problemParam.boundaryFunc=boundaryTest('spline',problemParam.roadDim,carStartY,carEndX,carStartX);

solnPlotter(soln,carParam,problemParam, true, 8010)


