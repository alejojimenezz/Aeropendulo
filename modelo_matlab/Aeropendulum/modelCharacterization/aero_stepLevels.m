function [V,T] = aero_stepLevels(v, x)
%AERO_STEPLEVELS  Ultimo dato de cada escalon de voltaje (el mas cercano al estado estable).
v   = round(v, 2);
idx = [find(diff(v) ~= 0); numel(v)];
V   = v(idx);
T   = x(idx);
V = V(:);  T = T(:);
end
