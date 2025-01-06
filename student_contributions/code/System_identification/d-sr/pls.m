function [W,P,C,T,U,E,F]=pls(X,Y,a);
%PLS The Partial Least Squares (PLS) algorithm.
%    [W,P,C,T,U,E,F]=pls(X,Y,a);
%    PURPOSE:
%    Given data matrices X and Y. This algorithm compute matrices P, C, T,
%    E and F which defines the decomposition X = T P' + E and Y = T C' + F.
%    The PLS estimate of the regression matrix B, in the bilinear equation 
%    Y = X B + E is given by B=W*inv(P'*W)*C', where B is an (r x m) matrix
%    of regression coefficients.
%
%    ON INPUT:
%    X     - An (N x r) data matrix with input variables.
%    Y     - An (N x m) data matrix with output variables.
%    a     - Number of components (factors) to estimate, 1 < a <= r.
%
%    ON OUTPUT:
%    T,P,C - Matrices in the PLS decomposition X = T P' + E and Y = T C' + F.
%    W     - Orthonormal weighting matrix.
%    P     - Matrix with lin. independent columns, loading matrix for X.
%    C     - Weight matrix for Y.
%    T     - Matrix of orthogonal score vectors for X.
%    U     - Matrix of orthogonal score vectors for Y.
%    E, F  - The residuals in the PLS decomposition.
%    REMARK:
%    The matrix of regression coefficients is B=W inv(P' W) C'.
%    See also D-SR functions PLS2 and PLS3.
%-----------------------------------------------------------------------------
%                                       COPYRIGHT 1996, FANTOFT PROCESS
%                                       License belong to:
%                                       Product id: 10 0000
%-----------------------------------------------------------------------------

% Last update: Sun Nov 24 20:23:46 GMT 1996, David Di Ruscio.

