function H = plotWireFrame(x,y,z,alpha,beta,gamma,radius,len,nAxial,nRadial,color,isDense,isSwitching,isEdge)
% All radius and length combinations
[RADII,LEN] = meshgrid(linspace(0,radius,nRadial),linspace(-len/2,len/2,nAxial));
RADII = RADII(:);
LEN = LEN(:);

% Plotting
H = hgtransform;

offset = 0;
theta = linspace(0,2*pi,100);
for i = 1:length(RADII)
    if isDense || (LEN(i) == -len/2 || LEN(i) == len/2 || RADII(i) == radius)
        X = RADII(i)*cos(theta);
        Y = RADII(i)*sin(theta);
        Z = LEN(i)*ones(size(X));
        if isEdge && ((LEN(i) == -len/2 && RADII(i) == radius) || (LEN(i) == len/2 && RADII(i) == radius)) % If on the edge
            plot3(X,Y,Z,'color','k','LineWidth',1.5,'Parent',H);
        else
            if mod(i+offset,2) == 0 || ~isSwitching
                plot3(X,Y,Z,'color',color,'LineWidth',1.5,'Parent',H);
            else
                plot3(X,Y,Z,'color',0.5*color,'LineWidth',1.5,'Parent',H);
            end
        end
    end
    if mod(i,nAxial) == 0
        offset = ~offset;
    end
end

% Rotation & Translation
updatePositionOfObject(H,x,y,z,alpha,beta,gamma);
