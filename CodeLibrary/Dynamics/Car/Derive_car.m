%This code generates the dynamics and linear dynamics 
%dynamics are from http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.2.6994&rep=rep1&type=pdf

clear;clc;

% constant parameters
l = sym('l','real');
a = sym('a','real');
b = sym('b','real');
m = sym('m','real');
J = sym('J','real');
K = sym('K','real');
R = sym('R','real');
bm = sym('bm','real');
La = sym('La','real');
Nw = sym('Nw','real');
Nm = sym('Nm','real');
tau=sym('tau','real');
c=sym('c','real');


% states:
x = sym('x','real');  % absolute angle of link one
y = sym('y','real');  % absolute angle of linke two
q1 = sym('q1','real');
q2 = sym('q2','real');
v = sym('v','real');
fd = sym('fd','real');
u1 = sym('u1','real');


% controls:
u2 = sym('u2','real');
u3 = sym('u3','real');


gama=cos(q2).^2.*(l.^2.*m+(b.^2.*m+J).*(tan(q2).^2));

%dynamics
dx=(cos(q1)-b.*tan(q2)./l.*sin(q1)).*v;
dy=(sin(q1)+b.*tan(q2)./l.*cos(q1)).*v;
dq1=tan(q2)./l.*v;
dq2=1./tau*q2+c.*u2;
dv=v.*(b.^2*m+J).*tan(q2)*dq2./gama+l.^2.*cos(q2)^2.*fd./gama;

