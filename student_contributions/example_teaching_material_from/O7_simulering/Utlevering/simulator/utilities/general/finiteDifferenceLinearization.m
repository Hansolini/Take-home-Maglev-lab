function [A,B,C,D] = finiteDifferenceLinearization(f,h,x_eq,u_eq,delta)
nStates     = length(x_eq);
nInputs     = length(u_eq);
nOutputs    = length(h(x_eq,u_eq));

A = zeros(nStates);
B = zeros(nStates,nInputs);
C = zeros(nOutputs,nStates);
D = zeros(nOutputs,nInputs);

for i = 1:nStates
    A(:,i) = (f(x_eq+delta*(i==1:nStates)',u_eq)-f(x_eq-delta*(i==1:nStates)',u_eq))/(2*delta);
end
A = round(A,5);

for i = 1:nInputs
    B(:,i) = (f(x_eq,u_eq+delta*(i==1:nInputs)')-f(x_eq,u_eq-delta*(i==1:nInputs)'))/(2*delta);
end
B = round(B,5);

for i = 1:nStates
    C(:,i) = (h(x_eq+delta*(i==1:nStates)',u_eq)-h(x_eq-delta*(i==1:nStates)',u_eq))/(2*delta);
end
C = round(C,5);

for i = 1:nInputs
    D(:,i) = (h(x_eq,u_eq+delta*(i==1:nInputs)')-h(x_eq,u_eq-delta*(i==1:nInputs)'))/(2*delta);
end
D = round(D,5);
