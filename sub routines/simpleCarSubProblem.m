function [soln,finalGuess] =  simpleCarSubProblem(carParam, problemParam,initialGuess)
%function [soln, finalGuess] =  simpleCarSubProblem(carParam, problemParam,initialGuess)
%
%This function solves a sub problem for a car. carParam sets the parameters
%for the car. problemParam is the parameters for what sort of problem to
%solve.
%
%INPUTS:
%   carParam = struct with constant scalar parameters:
%       .l = distance from front wheel to back wheel
%       .a = distance from cm to front wheel
%       .b = distance from cm to back wheel
%       .m = mass
%       .J = moment of inertia about cm
%       .tau=time constant of servo
%       .w  = width of car
%   problemParam
%       .roadDim=[+x, -x, +y, -y]=location of walls relative to 0,0
%       .bndCstLevel=level of path constraints
%                   ='none'=no path constraints
%                   ='dot'= car is a dot
%                   ='rectangle'=car is rectangle
%       .useBoundaryFunc=boolean=true if the
%       .boundaryFunc=y=f(x)=function of curve for a boundary
%                            ensures points of car always above curve
%       .isTimeDVar=boolean=true if time a decision variable
%       .zBegin=[6,1]
%       .zFinal=[6,1]
%       .steeringBounds
%           .u2_b=bound of u2
%           .q2_b=bound of q2
%       .nlpOpt=nonlinear programming options
%   initialGuess=struct=initialGuess for the problem
%               .time=[1,nGrid]=time vector for initial guess
%               .state=[nState,nGrid]=vector of state for initial guess
%               .control[nControl,nGrid]=vector of control for initial
%                   guess
%   soln=struct with fields:
%     .grid = solution at the grid-points
%         .time = [1, nTime]
%         .state = [nState, nTime] = state at each grid point
%         .control = [nControl, nTime] = control at each grid point
%         .dState = [nState, nTime] = derivative of state at grid points
%     .spline = method-consistent spline interpolation of the trajectory
%          .state = matlab PP struct = contains state trajectory
%          .control = matlab PP struct = contains control trajectory
%     .info = information about the optimization run
%         .nlpTime = time (seconds) spent in fmincon
%         .exitFlag = fmincon exit flag
%         .objVal = value of the objective function
%         .[all fields in the fmincon "output" struct]
%     .errorGrid=the error for each of the segment
%     .error=the error at all points in time for the trajectory

% Duration of initial guess:
duration = initialGuess.time(end);

%adding limit to steering angle
q2_b=problemParam.steeringBounds.q2_b;
u2_b=problemParam.steeringBounds.u2_b;
q2_ub=q2_b;
q2_lb=-q2_b;
u2_ub=u2_b;
u2_lb=-u2_b;
problem.bounds.state.ub=[Inf, Inf, Inf, q2_ub, problemParam.velocityBound, Inf, u2_ub];
problem.bounds.state.lb=[-Inf, -Inf, -Inf, q2_lb,  -problemParam.velocityBound, -Inf, u2_lb];
problem.bounds.control.ub=[Inf, Inf];
problem.bounds.control.lb=[-Inf, -Inf];

%setup if time is a decision variable or not
if(~problemParam.isTimeDVar)
    problem.bounds.time.lb=duration;
    problem.bounds.time.ub=duration;
else
    problem.bounds.time.lb=0;
    problem.bounds.time.ub=Inf;
end

% Dynamics function:
param = carParam;
problem.func.dynamics = @(t, z, u)( simpleCarDynamics(z, u, param) );

% Path integral:  (minimize the integral of actuation-squared and time)
problem.func.pathObj = @(t, z, u)( u(1,:).^2+u(2,:).^2*problemParam.beta+problemParam.alpha);

%Path constraint. Switching depending on problem param
if ~problemParam.useBoundaryFunc
    switch problemParam.bndCstLevel
        case 'none'
            problem.func.pthCst = @(t,z,u)deal([],[]);
        case 'dot'
            problem.func.pthCst = @(t,z,u)deal(dotPathCst(z, problemParam.roadDim),[]);
        case 'rectangle'
            problem.func. pthCst = @(t,z,u)deal(rectPathCst(z, problemParam.roadDim,carParam),[]);
    end
else
    switch problemParam.bndCstLevel
        case 'none'
            problem.func.pthCst = @(t,z,u)deal([],[]);
        case 'dot'
            problem.func.pthCst = @(t,z,u)deal([dotPathCst(z, problemParam.roadDim),...
                dotBoundaryPathCst(z,problemParam.boundaryFunc)],[]);
        case 'rectangle'
            problem.func.pthCst = @(t,z,u)deal([rectBoundaryPathCst(z,...
                problemParam.boundaryFunc,carParam),rectPathCst(z,...
                problemParam.roadDim,carParam)],[]);
    end
end

% Boundary constraint:
problem.func.bndCst = @(t0, tF, z0, zF)( ...
                        deal([], [z0 - problemParam.zBegin; zF - problemParam.zFinal]) );
                    
%get nlp options                    
problem.nlpOpt=problemParam.nlpOpt;

%copy over guess
problem.guess=initialGuess;

%run trajectory optimization
soln = dirColBvpTrap(problem);

%setup guess for next problem
finalGuess.state=soln.grid.state;
finalGuess.time=soln.grid.time;
finalGuess.control=soln.grid.control;
end

function c =dotPathCst(zGrid, roadDim)
%function c =dotPathCst(zGrid, roadDim)
%
%This function implements the path constraint to ensure that a point
%described in zGrid stays in the bounds of roadDim.
%
%INPUTS:
%   zGrid=[2,n]=[x;y]=location of point
%   roadDim==[+x, -x, +y, -y]=location of walls relative to 0,0
%
%OUTPUTS:
%   c=[1,4*n]=vector that when driven to <0 will mean point is inside road
%       dim at all points in n.

%parse zGrid
xGrid=zGrid(1,:);
yGrid=zGrid(2,:);

%check each of the walls
c1=xGrid-roadDim(1);
c2=roadDim(2)-xGrid;
c3=yGrid-roadDim(3);
c4=roadDim(4)-yGrid;

%concatinate results
c=[c1,c2,c3,c4];
end

function c =rectPathCst(zGrid, roadDim, param)
%function c =rectPathCst(zGrid, roadDim, param)
%
%This function implements the path constraints for a rectangular car. It
%finds the location of each of the corners and ensures they stay inside
%roadDim.
%
%INPUTS:
%   zGrid=[3,n]=[x;y;q1]=location of point
%   roadDim==[+x, -x, +y, -y]=location of walls relative to 0,0
%   param = struct with constant scalar parameters:
%       .l = distance from front wheel to back wheel
%       .a = distance from cm to front wheel
%       .b = distance from cm to back wheel
%       .w  = width of car
%
%OUTPUTS:
%   c=[1,16*n]=vector that when driven to <0 will mean point is inside road
%       dim at all points in n.

%location of car at all points in time
cm=zGrid(1:2,:);

%rotate body frame displacement from center of car to tire to world frame.
c1vec=[cos(zGrid(3,:))*param.a-sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.a+cos(zGrid(3,:))*param.w/2];

c2vec=[cos(zGrid(3,:))*param.a+sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.a-cos(zGrid(3,:))*param.w/2];

c3vec=[cos(zGrid(3,:))*param.b-sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.b+cos(zGrid(3,:))*param.w/2];

c4vec=[cos(zGrid(3,:))*param.b+sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.b-cos(zGrid(3,:))*param.w/2];

%add to location of car in world frame
c1vec=c1vec+cm;
c2vec=c2vec+cm;
c3vec=-c3vec+cm;
c4vec=-c4vec+cm;

%do path constraint all at once
c=dotPathCst([c1vec,c2vec,c3vec,c4vec], roadDim);
end

function c =dotBoundaryPathCst(zGrid, boundaryFunc)
%function c =dotBoundaryPathCst(zGrid, boundaryFunc)
%
%This function implements the path constraint to ensure that a point
%described in zGrid stays above boundaryFunc
%
%INPUTS:
%   zGrid=[2,n]=[x;y]=location of point
%   boundaryFunc=y=f(x)=function of curve for a boundary
%                       ensures points of car always above curve%
%OUTPUTS:
%   c=[1,n]=vector that when driven to <0 will mean point is inside road
%       dim at all points in n.

%parse zGrid
xGrid=zGrid(1,:);
yGrid=zGrid(2,:);

carGrid=boundaryFunc(xGrid);

c=carGrid-yGrid;
end


function c =rectBoundaryPathCst(zGrid, boundaryFunc, param)
%function c =rectBoundaryPathCst(zGrid, boundaryFunc, param)
%
%This function implements the path constraints for a rectangular car. It
%finds the location of each of the corners and a few other points on the 
%side and ensure they stay above boundaryFunc.
%
%INPUTS:
%   zGrid=[3,n]=[x;y;q1]=location of point
%   boundaryFunc=y=f(x)=function of curve for a boundary
%                       ensures points of car always above curve%
%   param = struct with constant scalar parameters:
%       .l = distance from front wheel to back wheel
%       .a = distance from cm to front wheel
%       .b = distance from cm to back wheel
%       .w  = width of car

%
%OUTPUTS:
%   c=[1,10*n]=vector that when driven to <0 will mean point is inside road
%       dim at all points in n.

%location of car at all points in time
cm=zGrid(1:2,:);

%rotate body frame displacement from center of car to tire to world frame.
c1vec=[cos(zGrid(3,:))*param.a-sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.a+cos(zGrid(3,:))*param.w/2];

c2vec=[cos(zGrid(3,:))*param.a+sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.a-cos(zGrid(3,:))*param.w/2];

c3vec=[cos(zGrid(3,:))*param.b-sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.b+cos(zGrid(3,:))*param.w/2];

c4vec=[cos(zGrid(3,:))*param.b+sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.b-cos(zGrid(3,:))*param.w/2];

c5vec=[-sin(zGrid(3,:))*param.w/2;...
    cos(zGrid(3,:))*param.w/2];

c6vec=[sin(zGrid(3,:))*param.w/2;...
    -cos(zGrid(3,:))*param.w/2];

c7vec=[cos(zGrid(3,:))*param.a/2-sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.a+cos(zGrid(3,:))*param.w/2];

c8vec=[cos(zGrid(3,:))*param.a/2+sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.a-cos(zGrid(3,:))*param.w/2];

c9vec=[cos(zGrid(3,:))*param.b/2-sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.b+cos(zGrid(3,:))*param.w/2];

c10vec=[cos(zGrid(3,:))*param.b/2+sin(zGrid(3,:))*param.w/2;...
    sin(zGrid(3,:))*param.b-cos(zGrid(3,:))*param.w/2];

%add vectors to center of mass
c1vec=c1vec+cm;
c2vec=c2vec+cm;
c3vec=-c3vec+cm;
c4vec=-c4vec+cm;
c5vec=c5vec+cm;
c6vec=c6vec+cm;
c7vec=c7vec+cm;
c8vec=c8vec+cm;
c9vec=-c9vec+cm;
c10vec=-c10vec+cm;

%do path constraint all at once
c=dotBoundaryPathCst([c1vec,c2vec,c3vec,c4vec,c5vec,c6vec,c7vec,c8vec,c9vec,c10vec], boundaryFunc);
end
