function boundaryFunc= boundaryTest(method,roadDim,carStartY,carEndX,carStartX,R,epsilon,nPoints)
%function boundaryFunc= boundaryTest(method,roadDim,carStartY,carEndX,carStartX,R,epsilon,nPoints)
%
%This function will either use a lagrange polynomial or a 3rd order spline
%to model the boundary needed for parallel parking
%
%INPUTS:
%   method=string
%         ='spline'=third order spline
%         ='lagrange'=lagrange polynomial
%   roadDim=[+x, -x, +y, -y]=location of walls relative to 0,0
%   carStartY=the height at which the car starts
%   carEndX=the x position of the end of the first car
%   carStartX=the x position of the start of the second car
%   R=parameter for curve fitting. How tight the corners of the rectangle
%       are
%   Epsilon=parameter for curve fitting. Slope of cars
%   nPoints=number of points for each of the section
%
%OUTPUTS:
%   boundaryFunc=y=f(x)=function of curve for a boundary

negX=roadDim(2);
posX=roadDim(1);
negY=roadDim(4);

if nargin<6
    R=0.008;
end
if nargin <7
    epsilon=0;
end
if nargin <8
    nPoints=15;
end

%generate points that curve will be fitted through
x1=linspace(negX,carEndX-R,nPoints);
x2=carEndX;
x3=linspace(carEndX+R,carStartX-R,nPoints);
x4=carStartX;
x5=linspace(carStartX+R,posX,nPoints);

y1=linspace(carStartY,carStartY-epsilon,nPoints);
y2=(carStartY+negY)/2-epsilon;
y3=negY*ones(1,nPoints)-epsilon;
y4=(carStartY+negY)/2-epsilon;
y5=linspace(carStartY-epsilon,carStartY,nPoints);

%concatinate vectors
x=[x1,x2,x3,x4,x5];
y=[y1,y2,y3,y4,y5];

%fit spline throuhg points
boundarySpline= spline(x, [0,y,0]);

%if use spline return ppval of spine
%if use lagrange. evaluate spline at chebyshev points and fit lagrange
%polynomial. Then return polyval of polynomial
switch method
    case 'spline'
        boundaryFunc=@(xGrid)ppval(boundarySpline,xGrid);
    case 'lagrange'
        nChebPoints=25;
        chebPoints=1/2*(negX+posX)+1/2*(posX-negX)*cos((2*linspace(1,nChebPoints,nChebPoints)-1)/2/nChebPoints*pi);
        [P,~,~] = lagrangepoly(chebPoints,ppval(boundarySpline,chebPoints));
        boundaryFunc=@(xGrid)polyval(P,xGrid);
end

end

