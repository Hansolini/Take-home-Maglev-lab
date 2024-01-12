function y = maglevSystemMeasurements(x,u,params,modelName)
[bx,by,bz] = computeFieldTotal(params.sensors.x,params.sensors.y,params.sensors.z,x,u,params,modelName);
y = reshape([bx(:)'; by(:)'; bz(:)'],3*numel(bx),1);
