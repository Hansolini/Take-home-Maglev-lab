function H = plotMagnetRealistic(x,y,z,alpha,beta,gamma,params,modelName)
% Colors
gray = [0.5,0.5,0.5];

switch modelName
    case {'accurate','filament'}
        H = plotCylinder( ...
            x,y,z,alpha,beta,gamma, ...
            params.magnet.r, ...
            params.magnet.l, ...
            gray,0,1);
    otherwise % fast is default
        H = plotWireLoop( ...
            x,y,z,alpha,beta,gamma, ...
            params.magnet.r, ...
            params.magnet.n, ...
            gray,1);    
end
