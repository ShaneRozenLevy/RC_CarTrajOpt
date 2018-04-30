function simpleCarDraw(z, param, roadDim, grid,parkedGrid)
%function simpleCarDraw(z, param, roadDim, grid,parkedGrid)
%
%This function will draw the car and the boundaries of the road. Note if no
%road boundaries are given, they are not drawn.
%
%INPUTS:
%   z = [7, 1] = [x; y; q1; q2; v; fd; u2] = state of the car
%        x = horizontal position
%        y = vertical position
%        q1 = absolute angle (zero for angle to left)
%        q2 = steering angle
%        v = velocity in forward direction
%        fd = force from motor
%        u2 = desired steering angle
%   param = struct with constant scalar parameters for the car:
%       .l = distance from front wheel to back wheel
%       .a = distance from cm to front wheel
%       .b = distance from cm to back wheel
%       .w = width of car
%   roadDim=[+x, -x, +y, -y]=location of walls relative to 0,0
%   grid=[1,n]=grid of x coordinates where the boundary func has been
%              evaluated
%   parkedGrid=[1,n]=grid of y coordinates where boundary func has been
%                    evaluated


if nargin < 3
    roadDim=[10000, -10000, 10000, -10000];
end

%location of car center of mass
cm=[z(1),z(2)]';

%rotation matricies
R1=[cos(z(3)) -sin(z(3)); sin(z(3)) cos(z(3))];
R2=[cos(z(4)) -sin(z(4)); sin(z(4)) cos(z(4))];

%rotation matricies could be used here to calculated the location of the
%corners in world frame, but I used the code I used for calculating the
%path constraints to debug my code. 
c1vec=[cos(z(3,:))*param.a-sin(z(3,:))*param.w/2;...
    sin(z(3,:))*param.a+cos(z(3,:))*param.w/2];

c2vec=[cos(z(3,:))*param.a+sin(z(3,:))*param.w/2;...
    sin(z(3,:))*param.a-cos(z(3,:))*param.w/2];

c3vec=[cos(z(3,:))*param.b-sin(z(3,:))*param.w/2;...
    sin(z(3,:))*param.b+cos(z(3,:))*param.w/2];

c4vec=[cos(z(3,:))*param.b+sin(z(3,:))*param.w/2;...
    sin(z(3,:))*param.b-cos(z(3,:))*param.w/2];

%everything is beige
patch([-10000, 10000, 10000, -10000],[10000, 10000, -10000, -10000],[0.9290, 0.6940, 0.1250])

%the road is white
patch([roadDim(2), roadDim(1), roadDim(1), roadDim(2)],...
    [roadDim(3),roadDim(3),roadDim(4),roadDim(4)],'w')

%if the boundary func was provided. Use patch to draw it.
if nargin>3
    patch([-10000,grid,10000],[-10000,parkedGrid,-10000],[0.9290, 0.6940, 0.1250])
end

%draw body of car
body=[(c1vec+cm),(c2vec+cm),(-c3vec+cm),(-c4vec+cm)];
patch(body(1,:),body(2,:),'g')

%draw each of the tires. Only the front 2 are rotated
drawTire(body(:,1), R1*R2 ,param)

drawTire(body(:,2), R1*R2 ,param)

drawTire(body(:,3), R1 ,param)

drawTire(body(:,4), R1 ,param)

end

function drawTire(loc, R,param)
%function drawTire(loc, R,param)
%
%This function will draw a tire for a car given center of the tire, a
%rotation matrix and the car parameters.
%
%INPUTS:
%   param = struct with constant scalar parameters for the car:
%       .l = distance from front wheel to back wheel
%       .w = width of car

%location of each of the corners
tC1=(loc+R*[param.l,param.w/2]'/10);
tC2=(loc+R*[param.l,-param.w/2]'/10);
tC3=(loc+R*[-param.l,param.w/2]'/10);
tC4=(loc+R*[-param.l,-param.w/2]'/10);

%draw the tire
t=[tC1,tC2,tC4,tC3];
patch(t(1,:),t(2,:),'b')
end