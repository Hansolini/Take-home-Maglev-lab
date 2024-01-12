function [A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta)
nStates     = length(xLp);
nInputs     = length(uLp);
nOutputs    = length(h(xLp,uLp));

A = zeros(nStates);
B = zeros(nStates,nInputs);
C = zeros(nOutputs,nStates);
D = zeros(nOutputs,nInputs);

for i = 1:nStates
    A(:,i) = (f(xLp+delta*(i==1:nStates)',uLp)-f(xLp-delta*(i==1:nStates)',uLp))/(2*delta);
end
A = round(A,5);

for i = 1:nInputs
    B(:,i) = (f(xLp,uLp+delta*(i==1:nInputs)')-f(xLp,uLp-delta*(i==1:nInputs)'))/(2*delta);
end
B = round(B,5);

for i = 1:nStates
    C(:,i) = (h(xLp+delta*(i==1:nStates)',uLp)-h(xLp-delta*(i==1:nStates)',uLp))/(2*delta);
end
C = round(C,5);

for i = 1:nInputs
    D(:,i) = (h(xLp,uLp+delta*(i==1:nInputs)')-h(xLp,uLp-delta*(i==1:nInputs)'))/(2*delta);
end
D = round(D,5);
