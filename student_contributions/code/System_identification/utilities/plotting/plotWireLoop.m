function H = plotWireLoop(x,y,z,alpha,beta,gamma,radius,n,color,isDots)
% Points on circular wire centered around COM
theta = linspace(0,2*pi,n);
X = radius*cos(theta);
Y = radius*sin(theta);
Z = 0*X;

% Plotting
H = hgtransform;
plot3(X,Y,Z,'color',color,'LineWidth',1.5,'Parent',H);

if isDots
    plot3(X,Y,Z,'.','color',0.5*color,'Parent',H);
end

% Rotation & Translation
updatePositionOfObject(H,x,y,z,alpha,beta,gamma);