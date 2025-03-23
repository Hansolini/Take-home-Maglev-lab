function c = cross2D(a, b)
%CROSS2D Cross product equivalent for 2D vectors around z-axis
% a, b must be 2x1 vectors: [a1; a2], [b1; b2]
% Returns 2D vector [a2*b2 - a1*b1; 0] (only z component matters if extending to 3D)
    c = [a(1)*b(2) - a(2)*b(1); 0];
end
