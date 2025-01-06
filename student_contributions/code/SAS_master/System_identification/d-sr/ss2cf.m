function [ac,bc,dc,t]=ss2cf(a,b,d);
% [ac,bc,dc,t]=ss2cf(a,b,d);
% Purpose
% Transform a state space model (a,b,d) into observable canonical form
% (ac,bc,dc).
% xh = a x + b u     -->          xch = ac xc + bc u
% y  = d x + e u     -->          y   = dc xc + e  u
%
% Note:
% The matrix (e) is not influenced by state coordinate transformation.

