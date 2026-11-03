function th = simAero(q, tg, Vg, th0, P)
%SIMAERO  Simula el modelo no lineal en MATLAB puro (para identificar parametros).
%   q = [Tc, a, V0, n, b]     Tmot(V) = a*max(V-V0,0)^n
%   J*th'' = Tmot - mgl*sin(th) - b*th' - Tc*tanh(th'/eps)
Tc = q(1); a = q(2); V0 = q(3); n = q(4); b = q(5);
th = zeros(size(tg));  x = [th0; 0];  th(1) = rad2deg(x(1));

ic = unique([1; find(diff(Vg) ~= 0) + 1; numel(tg)]);   % reinicia el solver en cada escalon
for k = 1:numel(ic)-1
    i1 = ic(k);  i2 = ic(k+1);
    T  = a*max(Vg(i1) - V0, 0)^n;
    f  = @(~,s) [ s(2); (T - P.mgl*sin(s(1)) - b*s(2) - Tc*tanh(s(2)/P.epsilon))/P.J ];
    [~,xs] = ode15s(f, tg(i1:i2), x, odeset('MaxStep',0.05));
    if i2 - i1 == 1, xs = xs([1 end],:); end
    th(i1:i2) = rad2deg(xs(:,1));
    x = xs(end,:)';
end
end
