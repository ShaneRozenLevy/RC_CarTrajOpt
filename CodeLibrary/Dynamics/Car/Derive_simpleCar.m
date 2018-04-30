%This code generates the dynamics and linear dynamics for simplified car.
%model.
%dynamics are from http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.2.6994&rep=rep1&type=pdf

clear;clc;

% constant parameters
l = sym('l','real');
a = sym('a','real');
b = sym('b','real');
m = sym('m','real');
J = sym('J','real');
tau=sym('tau','real');



% states:
x = sym('x','real');  % absolute angle of link one
y = sym('y','real');  % absolute angle of linke two
q1 = sym('q1','real');
q2 = sym('q2','real');
v = sym('v','real');
fd = sym('fd','real');
u2 = sym('u2','real');


% controls:
u3 = sym('u3','real');
u1 = sym('u1','real');


gama=cos(q2).^2.*(l.^2.*m+(b.^2.*m+J).*(tan(q2).^2));

%dynamics
dx=(cos(q1)-b.*tan(q2)./l.*sin(q1)).*v;
dy=(sin(q1)+b.*tan(q2)./l.*cos(q1)).*v;
dq1=tan(q2)./l.*v;
dq2=-q2/tau+u2/tau;
dv=v.*(b.^2*m+J).*tan(q2)*dq2./gama+l.^2.*cos(q2)^2.*fd./gama;
dfd=u1;
du2=u3;

z=[x,y,q1,q2,v,fd u2]';
dz=[dx,dy,dq1,dq2,dv,dfd du2]';

%get linear dynamics
A = jacobian(dz, z);
B = jacobian(dz, [u1; u3]); 

% Write the dynamics function (equations of motion)
matlabFunction(dx, dy, dq1, dq2, dv, dfd, du2, ...
               'File', 'autoGen_simpleCarDynamics.m', ...
               'Outputs', {'dx','dy','dq1','dq2','dv','dfd','du2'}, ...
               'Optimize', true, ...
               'Vars',{'x','y','q1','q2','v','fd' ,'u2'...
                       'u1', 'u3', ...
                       'l', 'a','b','m','J','tau'});
                         
% Write the linearized dynamics function (equations of motion)
matlabFunction(A, B, ...
               'File', 'autoGen_simpleCarLinDyn.m', ...
               'Outputs', {'A','B'}, ...
               'Optimize', true, ...
               'Vars',{'x','y','q1','q2','v','fd' ,'u2'...
                       'u1', 'u3', ...
                       'l', 'a','b','m','J','tau'});

