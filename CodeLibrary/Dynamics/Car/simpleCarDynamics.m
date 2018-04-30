function dz =simpleCarDynamics(z, u, param)
%function dz =simpleCarDynamics(z, u, param)
%
% This function computes the equations of motion for a simple car
%
% INPUTS:
%   z = [6, n] = [x; y; q1; q2; v; fd] = state
%        x = horizontal position
%        y = vertical position
%        q1 = absolute angle (zero for angle to left)
%        q2 = steering angle
%        v = velocity in forward direction
%        fd = force from motor
%        u2 = desired steering angle
%   u = [2, n] = [u1; u2] = control
%       u1 = derivative of force from motor
%       u3 = derivative of rate of change of steering angle
%   param = struct with constant scalar parameters:
%       .l = distance from front wheel to back wheel
%       .a = distance from cm to front wheel
%       .b = distance from cm to back wheel
%       .m = mass
%       .J = moment of inertia about cm
%       .tau=time constant of servo
%
% OUTPUTS:
%   dz = [6, n] = [dx; dy; dq1; dq2; dv; dfd; du2] = state derivative
%

% Unpack the state:
x  = z(1,:);
y  = z(2,:);
q1 = z(3,:);
q2 = z(4,:);
v  = z(5,:);
fd = z(6,:);
u2 = z(7,:);


% Unpack the control:
u1 = u(1, :);
u3 = u(2, :);

% Unpack the parameters:
m = param.m;
l = param.l;
a = param.a;
b = param.b;
J = param.J;
tau=param.tau;


% Call the automatically generated dynamics function:
[dx,dy,dq1,dq2,dv,dfd, du2] = autoGen_simpleCarDynamics(x,y,q1,q2,v,fd,u2,u1,u3,l,a,b,m,J,tau);

% Pack up the derivative of the state:
dz = [dx;dy;dq1;dq2;dv;dfd;du2];

end
