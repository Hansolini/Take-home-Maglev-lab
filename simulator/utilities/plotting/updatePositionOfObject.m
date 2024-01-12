function H = updatePositionOfObject(H,x,y,z,alpha,beta,gamma)
M = makehgtform('translate',[x,y,z],'xrotate',alpha,'yrotate',beta,'zrotate',gamma);
set(H,'Matrix',M);
