function soln = dirColBvpTrap(problem)
% soln = dirColBvpTrap(problem)
%
% This function computes the solution to a simple trajectory optimization
% problem using the trapezoid method for direct collocation.
%
% minimize: J = integral(pathObj(t, x, u))
%
% subject to:
%
%       dynamics:  dx = dynamics(t, x, u)
%
%       boundary constraints:  [c, ceq] = bndCst(x0, xF)
%
% given: time grid and initialization for state and control
%
%
% INPUT: "problem" -- struct with fields:
%
%     func -- struct for user-defined functions, passed as function handles
%
%         Input Notes:  square braces show size:  [a,b] = size()
%                 t = [1, nTime] = time vector (grid points)
%                 z = [nState, nTime] = state vector at each grid point
%                 u = [nControl, nTime] = control vector at each grid point
%                 x0 = [nState, 1] = initial state
%                 xF = [nState, 1] = final state
%
%         dz = dynamics(t, z, u)
%                 dz = [nState, nTime] = dz/dt = derivative of state wrt time
%
%         dObj = pathObj(t, z, u)
%                 dObj = [1, nTime] = integrand from the cost function
%
%         [c, ceq] = bndCst(t0, tF, x0, xF)
%                 c = column vector of inequality constraints  ( c <= 0 )
%                 ceq = column vector of equality constraints ( c == 0 )
%
%         [c, ceq] = pthCst(t,z,u)
%                 c = column vector of inequality constraints  ( c <= 0 )
%                 ceq = column vector of equality constraints ( c == 0 )               
%
%     guess - struct with an initial guess at the trajectory
%
%         .time = [1, nGrid] = time grid for the transcription (constant)
%         .state = [nState, nGrid] = guess for state at gridpoints
%         .control = [nControl, nGrid] = guess for control at gridpoints
%
%     nlpOpt = solver options object, created by:
%           >> nlpOpt = optimset('fmincon')
%       Useful options (set using "." operator, eg: nlpOpt.Display = 'off')
%           --> Display = {'iter', 'final', 'off'}
%           --> OptimalityTolerance = {1e-3, 1e-6}
%           --> ConstraintTolerance = {1e-3, 1e-5, 1e-10}
%
%   bounds
%           .lb = lower bounds on z
%           .ub = upper bounds on z
%
%
%   OUTPUT: "soln"  --  struct with fields:
%
%     .grid = solution at the grid-points
%         .time = [1, nTime]
%         .state = [nState, nTime] = state at each grid point
%         .control = [nControl, nTime] = control at each grid point
%         .dState = [nState, nTime] = derivative of state at grid points
%
%     .spline = method-consistent spline interpolation of the trajectory
%          .state = matlab PP struct = contains state trajectory
%          .control = matlab PP struct = contains control trajectory
%
%     .info = information about the optimization run
%         .nlpTime = time (seconds) spent in fmincon
%         .exitFlag = fmincon exit flag
%         .objVal = value of the objective function
%         .[all fields in the fmincon "output" struct]
%
%     .errorGrid=the error for each of the segment
%     .error=the error at all points in time for the trajectory
%
%
% NOTES:
%
%   guess.time is used as the time grid for the transcription
%
%CHANGE LOG:
%   4/19/2018 Line99: added handeling for lb and ub on states
%             Line306:added handeling for nonlinear path constraint
%   4/24/2018 Added error calculation to end of the main function

% Get problem description:
D = getProblemDescription(problem);

% Objective function:  (path objective)
P.objective = @(decVars)( objectiveFunction(decVars, problem, D) );

T=problem.guess.time(:,end);

% Initial guess:
P.x0 = packDecVars(problem.guess.state, problem.guess.control, D,T);

% No linear constraints:
P.Aineq = [];
P.bineq = [];
P.Aeq = [];
P.beq = [];

% No limits on the state or control
P.lb =[problem.bounds.time.lb,repmat([problem.bounds.state.lb,problem.bounds.control.lb],1,D.nGrid)];
P.ub =[problem.bounds.time.ub,repmat([problem.bounds.state.ub,problem.bounds.control.ub],1,D.nGrid)];

% Final setup
P.options = problem.nlpOpt;
P.solver = 'fmincon';

% System dynamics: (nonlinear constraint)
P.nonlcon = @(decVars)( nonlinearConstraint(decVars, problem, D) );

% Solve the optimization problem
startTime = tic;
[decVarSoln, objVal, exitFlag, info] = fmincon(P);
info.nlpTime = toc(startTime);
info.objVal = objVal;
info.exitFlag = exitFlag;

% Unpack the grid:
[grid.state, grid.control,T] = unpackDecVars(decVarSoln, D);

grid.time = D.tGrid*T;

% Create the interpolating splines:
grid.dState = problem.func.dynamics(grid.time, grid.state, grid.control);
spline.state = ppSpline2a(grid.time, grid.state, grid.dState);
spline.dstate=ppDer(spline.state);
spline.control = ppSpline1(grid.time, grid.control);

% Pack up the output
soln.grid = grid;
soln.spline = spline;
soln.info = info;
soln.decVar=decVarSoln;

%calculate error for solution
dError=@(t)(abs(ppval(spline.dstate,t)-problem.func.dynamics(t,ppval(spline.state,t),ppval(spline.control,t))));
errorVec=zeros(D.nState,D.nGrid-1);
for section=1:D.nGrid-1
    errorVec(:,section)=integral(dError,grid.time(section),grid.time(section+1),'ArrayValued',true);
end
soln.errorGrid=errorVec;

smallTGrid=linspace(0,grid.time(end),1000);
soln.error=dError(smallTGrid);
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function D = getProblemDescription(problem)
% D = getProblemDescription(problem)
%
% This function checks the consistency of the inputs and then constructs
% the problem description, which is used throughout the code..
%
% INPUTS:
%   problem = input structure for dirColBvpTrap
%
% OUTPUTS:
%   D = struct with problem details
%       .nGrid = integer = number of grid points
%       .nState = integer = dimension of the state space
%       .nControl = integer = dimension of the control space
%       .tGrid = [1, nGrid] = time grid
%       .hSeg = [1, nGrid - 1] = duration of each segment
%       .hSegState = [nState, nGrid - 1] = duration of each segment
%       .iLow = [1, nGrid-1] = index for the lower edge of each segment
%       .iUpp = [1, nGrid-1] = index for the upper edge of each segment
%

% Problem dimensions:
D.nGrid = length(problem.guess.time);
[D.nState, nGridState] = size(problem.guess.state);
if nGridState ~= D.nGrid
   error('problem.guess.state is an invalid size!');
end
[D.nControl, nGridControl] = size(problem.guess.control);
if nGridControl ~= D.nGrid
   error('problem.guess.control is an invalid size!');
end
D.nDecVar = (D.nControl + D.nState) * D.nGrid+1;

T=problem.guess.time(end);
% Set up the time grid:
D.tGrid = problem.guess.time/T;
D.hSeg = diff(D.tGrid);
D.hSegState = ones(D.nState, 1) * D.hSeg;

% Index grids:
D.iLow = 1:(D.nGrid - 1);
D.iUpp = 2:D.nGrid;

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function decVars = packDecVars(zGrid, uGrid, D, T)
% decVars = packDecVars(xGrid, uGrid, D, T)
%
% Converts from the easy-to-use format of xGrid and uGrid into the
% decision variable column vector that is required by fmincon
%
% INPUTS:
%   zGrid = [nState, nGrid] = state at the grid points
%   uGrid = [nControl, nGrid] = control at the grid points
%   D = problem description struct
%   T = length of time of the solution
%
% OUTPUTS:
%   decVars = [nDecVar, 1] = column vector of decision variables
%              nDecVar = (nState + nControl)*nGrid
%

decVars =[T; reshape([zGrid; uGrid], D.nDecVar-1, 1)];

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [zGrid, uGrid, T] = unpackDecVars(decVars, D)
% [zGrid, uGrid, T] = unpackDecVars(decVars, D)
%
% Converts from the decision variable column vector that is required by
% fmincon into the easy-to-use state and control grids.
%
% INPUTS:
%   decVars = [nDecVar, 1] = column vector of decision variables
%              nDecVar = (nState + nControl)*nGrid
%   D = problem description struct
%
% OUTPUTS:
%   zGrid = [nState, nGrid] = state at the grid points
%   uGrid = [nControl, nGrid] = control at the grid points
%   T     = length of time of simulations
%

% Reshape into a square matrix
T=decVars(1);
decVars=decVars(2:end);
zTmp = reshape(decVars, D.nState + D.nControl, D.nGrid);

% Figure out the indicies for state and control:
idxState = 1:D.nState;
idxControl = idxState(end) + (1:D.nControl);

% pull out the state and control components
zGrid = zTmp(idxState, :);
uGrid = zTmp(idxControl, :);

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function J = objectiveFunction(decVars, problem, D)
% J = objectiveFunction(decVars, problem, D)
%
% This function computes the integral objective function.
%
% INPUTS:
%   decVars = vector of decision variables
%   problem = problem struct: see dirColBvpTrap()
%   D = problem description struct: see getProblemDescription()
%
% OUTPUTS:
%   J = scalar = value of the path integral objective
%
% NOTES:
%   The path integral is computed using the trapezoid rule for quadrature.
%

%unpack decision variables
[xGrid, uGrid, T] = unpackDecVars(decVars, D);

%asses path objective and get the low and high points
g=problem.func.pathObj(D.tGrid,xGrid,uGrid);
gk=g(:,D.iLow);
gk1=g(:,D.iUpp);

%do the integral
J=0.5*dot(D.hSeg*T,(gk+gk1));
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [c, ceq] = nonlinearConstraint(decVars, problem, D)
% [c, ceq] = nonlinearConstraint(decVars, problem, D)
%
% This function computes the nonlinear constraints that are passed to
% FMINCON. This function should handle both the boundary constraints and
% the system dynamics.
%
% INPUTS:
%
% INPUTS:
%   decVars = vector of decision variables
%   problem = problem struct: see dirColBvpTrap()
%   D = problem description struct: see getProblemDescription()
%
% OUTPUTS:
%   c = column vector of inequality constraints
%   ceq = column vector of equality constraints
%

%unpack decision variables
[zGrid, uGrid,T] = unpackDecVars(decVars, D);

%evaluate the dynamics at all points and divide into the high and low sets
f=problem.func.dynamics(D.tGrid*T,zGrid,uGrid);
fk=f(:,D.iLow);
fk1=f(:,D.iUpp);

%do the trapazoid rule
deltaZ=1/2*D.hSeg*T.*(fk+fk1);
%reshape the constraint
dynCon=reshape(-zGrid(:,D.iUpp)+deltaZ+zGrid(:,D.iLow),(D.nGrid-1)*D.nState,1);

t0=D.tGrid(:,1)*T;
tF=D.tGrid(:,end)*T;
z0=zGrid(:,1);
zF=zGrid(:,end);

%concatinate dynamics w/ boundary constraint
[c1, ceq1] = problem.func.bndCst(t0, tF, z0, zF);
[c2, ceq2] = problem.func.pthCst(D.tGrid*T,zGrid,uGrid);


ceq=[ceq1;ceq2;dynCon];
c=[c1;c2];

end
