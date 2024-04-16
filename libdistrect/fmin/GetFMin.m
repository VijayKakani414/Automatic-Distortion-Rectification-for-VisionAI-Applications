function result = GetFMin(I, lineGroups)

% find the distortion parameters
funToSolve = @(x)getDistParamError(x, I, lineGroups);
[kParams, error] = fminsearch(funToSolve, [0;0]);

result = [kParams; error];

end